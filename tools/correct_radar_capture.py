#!/usr/bin/env python3
"""Apply a constant radar boresight correction to an SB DDS capture.

The equipment pipeline consumes valid radar L2 longitude/latitude in
``device_tracks`` mode, so correcting only the L1 azimuth is insufficient.
This tool therefore:

* subtracts the configured clockwise azimuth bias from every RadarTarget L1
  azimuth; and
* rotates every valid RadarTarget L2 longitude/latitude by the same amount
  about the configured radar station.

Altitude, range, elevation, timestamps, IDs, and every non-radar packet are
left byte-for-byte unchanged.  The output is a new, fully hashed
``sb-dds-capture-v1`` file; the source file is never modified.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import math
import struct
import time
import uuid
from collections import Counter
from pathlib import Path


PXSB = struct.Struct("<BBHHIHHHHQ")
RADAR_MAGIC = 0xA55AA55A
RADAR_TAG = 0x1A01
RADAR_FRAME_SIZE = 66
FRAME_HEADER = struct.Struct("<IBHHHIQ")

# Offsets relative to the start of the 66-byte dev_proto radar frame.
AZIMUTH_OFFSET = 31
LONGITUDE_OFFSET = 41
LATITUDE_OFFSET = 45


def line_bytes(value: dict) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")


def physical_azimuth_deg(frame: bytes, start: int) -> float:
    """Decode both protocol-compliant signed angles and delivered 0..360 data."""

    unsigned = struct.unpack_from("<H", frame, start + AZIMUTH_OFFSET)[0]
    if unsigned <= 36000:
        return unsigned / 100.0
    return struct.unpack_from("<h", frame, start + AZIMUTH_OFFSET)[0] / 100.0


def signed_cdeg(degrees: float) -> int:
    normalized = (degrees + 180.0) % 360.0 - 180.0
    return int(round(normalized * 100.0))


def frame_starts(packet: bytes):
    if len(packet) < PXSB.size:
        raise ValueError("RadarTarget packet is shorter than the PXSB header")
    header = PXSB.unpack_from(packet)
    # Delivered bridge builds disagree on whether msgLength includes the
    # 26-byte MsgHeader.  Preserve either accepted source convention.
    if header[0] != 0x7E or header[4] not in (len(packet), len(packet) - PXSB.size):
        raise ValueError("Invalid PXSB marker or payload length")

    offset = PXSB.size
    for _ in range(header[1]):
        if offset + 6 > len(packet):
            raise ValueError("Truncated PXSB unit header")
        unit_length, unit_tag = struct.unpack_from("<IH", packet, offset)
        base = offset + 6
        candidates = []
        for start in (base, base + 8):
            if start + unit_length <= len(packet):
                magic = struct.unpack_from("<I", packet, start)[0]
                if magic == RADAR_MAGIC:
                    candidates.append(start)
        if len(candidates) != 1:
            raise ValueError("Cannot uniquely locate the radar dev_proto frame")
        start = candidates[0]
        if unit_tag != RADAR_TAG or unit_length != RADAR_FRAME_SIZE:
            raise ValueError("RadarTarget topic contains an unexpected unit")
        magic, version, tag, _, length, _, _ = FRAME_HEADER.unpack_from(packet, start)
        if magic != RADAR_MAGIC or version != 1 or tag != RADAR_TAG or length != RADAR_FRAME_SIZE:
            raise ValueError("Unsupported radar dev_proto frame")
        if struct.unpack_from("<I", packet, start + unit_length - 4)[0] != 0:
            raise ValueError("Radar frame CRC is non-zero; unknown CRC cannot be regenerated")
        yield start
        offset = start + unit_length
    if offset != len(packet):
        raise ValueError("Unexpected bytes after the final PXSB unit")


def correct_packet(
    packet: bytes,
    *,
    bias_deg: float,
    station_lon: float,
    station_lat: float,
) -> tuple[bytes, list[dict]]:
    result = bytearray(packet)
    reports = []
    meters_per_deg_lat = 111320.0
    meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(station_lat))
    rotation = math.radians(bias_deg)
    cos_a, sin_a = math.cos(rotation), math.sin(rotation)

    for start in frame_starts(packet):
        track_id = struct.unpack_from("<I", packet, start + 23)[0]
        old_azimuth = physical_azimuth_deg(packet, start)
        new_azimuth = (old_azimuth - bias_deg + 180.0) % 360.0 - 180.0
        struct.pack_into("<h", result, start + AZIMUTH_OFFSET, signed_cdeg(new_azimuth))

        lon_e7 = struct.unpack_from("<i", packet, start + LONGITUDE_OFFSET)[0]
        lat_e7 = struct.unpack_from("<i", packet, start + LATITUDE_OFFSET)[0]
        old_lon, old_lat = lon_e7 * 1e-7, lat_e7 * 1e-7
        coordinate_changed = (
            math.isfinite(old_lon)
            and math.isfinite(old_lat)
            and abs(old_lon - station_lon) < 1.0
            and abs(old_lat - station_lat) < 1.0
        )
        new_lon, new_lat = old_lon, old_lat
        if coordinate_changed:
            east = (old_lon - station_lon) * meters_per_deg_lon
            north = (old_lat - station_lat) * meters_per_deg_lat

            # Bearing is north-zero/clockwise. Subtracting bias_deg from that
            # bearing is a positive mathematical rotation in (east, north).
            corrected_east = east * cos_a - north * sin_a
            corrected_north = east * sin_a + north * cos_a
            new_lon = station_lon + corrected_east / meters_per_deg_lon
            new_lat = station_lat + corrected_north / meters_per_deg_lat
            struct.pack_into(
                "<i", result, start + LONGITUDE_OFFSET, int(round(new_lon * 1e7))
            )
            struct.pack_into(
                "<i", result, start + LATITUDE_OFFSET, int(round(new_lat * 1e7))
            )

        reports.append(
            {
                "track_id": track_id,
                "old_azimuth_deg": old_azimuth,
                "new_azimuth_deg": new_azimuth,
                "old_lon": old_lon,
                "old_lat": old_lat,
                "new_lon": new_lon,
                "new_lat": new_lat,
                "coordinate_changed": coordinate_changed,
            }
        )
    return bytes(result), reports


def transform(source: Path, destination: Path, args) -> dict:
    if source.resolve() == destination.resolve():
        raise ValueError("Source and destination must be different files")
    if destination.exists():
        raise FileExistsError(f"Refusing to overwrite existing output: {destination}")

    digest = hashlib.sha256()
    counts = Counter()
    radar_reports = 0
    coordinates_changed = 0
    samples = 0
    original_footer = None

    with source.open("rb") as input_stream, destination.open("xb") as output_stream:
        try:
            first = input_stream.readline()
            header = json.loads(first)
            if header.get("type") != "header" or header.get("format") != "sb-dds-capture-v1":
                raise ValueError("Input is not an sb-dds-capture-v1 file")
            header["session_id"] = str(uuid.uuid4())
            header["created_at_ns"] = time.time_ns()
            header["source"] = {
                "kind": "derived_radar_calibration",
                "input": source.name,
            }
            header["calibration"] = {
                "radar_azimuth_bias_removed_deg": args.bias_deg,
                "radar_station_lla": [args.station_lon, args.station_lat, args.station_alt],
                "l2_horizontal_position": "rotated_about_radar_station",
                "altitude_changed": False,
                "other_topics_changed": False,
            }
            encoded = line_bytes(header)
            output_stream.write(encoded)
            digest.update(encoded)

            for raw_line in input_stream:
                row = json.loads(raw_line)
                if row.get("type") == "footer":
                    original_footer = row
                    break
                if row.get("type") != "sample":
                    raise ValueError("Unexpected record in capture")
                samples += 1
                topic = row.get("topic")
                counts[topic] += 1
                if row.get("seq") != samples:
                    raise ValueError("Input sample sequence is not contiguous")

                packet = base64.b64decode(row["payload_base64"], validate=True)
                if len(packet) != row.get("length") or hashlib.sha256(packet).hexdigest() != row.get("sha256"):
                    raise ValueError(f"Input sample {samples} failed payload validation")
                if topic == "RadarTarget":
                    packet, reports = correct_packet(
                        packet,
                        bias_deg=args.bias_deg,
                        station_lon=args.station_lon,
                        station_lat=args.station_lat,
                    )
                    radar_reports += len(reports)
                    coordinates_changed += sum(r["coordinate_changed"] for r in reports)
                    row["length"] = len(packet)
                    row["sha256"] = hashlib.sha256(packet).hexdigest()
                    row["payload_base64"] = base64.b64encode(packet).decode("ascii")

                encoded = line_bytes(row)
                output_stream.write(encoded)
                digest.update(encoded)

            if original_footer is None:
                raise ValueError("Input capture has no footer")
            if original_footer.get("records") != samples or original_footer.get("counts") != dict(counts):
                raise ValueError("Input footer count does not match its records")
            footer = {
                "type": "footer",
                "records": samples,
                "counts": dict(counts),
                "reason": "derived_radar_calibration",
                "data_sha256": digest.hexdigest(),
            }
            output_stream.write(line_bytes(footer))
        except BaseException:
            output_stream.close()
            destination.unlink(missing_ok=True)
            raise

    return {
        "source": str(source),
        "destination": str(destination),
        "samples": samples,
        "counts": dict(counts),
        "radar_reports": radar_reports,
        "coordinates_changed": coordinates_changed,
        "bias_removed_deg": args.bias_deg,
        "station_lla": [args.station_lon, args.station_lat, args.station_alt],
        "altitude_changed": False,
        "output_data_sha256": digest.hexdigest(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--bias-deg", type=float, default=15.32)
    parser.add_argument("--station-lon", type=float, default=120.140891503278)
    parser.add_argument("--station-lat", type=float, default=30.111739215983178)
    parser.add_argument("--station-alt", type=float, default=25.967412913025026)
    args = parser.parse_args()
    report = transform(args.source, args.destination, args)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
