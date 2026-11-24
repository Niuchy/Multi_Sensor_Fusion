"""PXSB/dev_proto decoding helpers for company simulation DDS packets."""

from __future__ import annotations

import importlib.util
import math
import struct
import sys
from functools import lru_cache
from pathlib import Path


PXSB_HEADER_FORMAT = "<BBHHIHHHHQ"
PXSB_HEADER_SIZE = struct.calcsize(PXSB_HEADER_FORMAT)
PXSB_UNIT_BASE_SIZE = 6
PXSB_UNIT_WITH_SOURCE_TIME_SIZE = 14

FRAME_MAGIC = {
    0x1A01: 0xA55AA55A,
    0x1B01: 0xB44BB44B,
    0x1C01: 0xC33CC33C,
    0x1D01: 0xD22DD22D,
}


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


@lru_cache(maxsize=1)
def _company_parser_module():
    parser_path = (
        workspace_root() / "anti-drone-pkg/src/bus/proto_receiver.py"
    )
    if not parser_path.exists():
        raise RuntimeError(f"Company protocol parser not found: {parser_path}")
    name = "_anti_drone_company_proto_receiver"
    spec = importlib.util.spec_from_file_location(name, parser_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load company protocol parser: {parser_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _looks_like_frame(packet: bytes, offset: int, unit_tag: int) -> bool:
    if offset + 4 > len(packet):
        return False
    expected = FRAME_MAGIC.get(unit_tag)
    magic = struct.unpack_from("<I", packet, offset)[0]
    return expected is None or magic == expected


def unpack_pxsb_units(packet: bytes):
    """Return ``(unit_tag, source_time_ms, dev_frame_bytes)`` tuples.

    The company simulator's batch publisher uses a 14-byte unit header
    (length/tag/source-time), while an older zy bridge emits the 6-byte base
    header.  Both encodings are accepted.
    """
    if len(packet) < PXSB_HEADER_SIZE:
        raise ValueError(f"PXSB packet too short: {len(packet)}")

    header = struct.unpack_from(PXSB_HEADER_FORMAT, packet, 0)
    start_tag, unit_count = header[0], header[1]
    if start_tag != 0x7E:
        raise ValueError(f"Invalid PXSB start tag: 0x{start_tag:02X}")

    offset = PXSB_HEADER_SIZE
    out = []
    for _ in range(unit_count):
        if offset + PXSB_UNIT_BASE_SIZE > len(packet):
            raise ValueError("Truncated PXSB UnitHeader")
        unit_len, unit_tag = struct.unpack_from("<IH", packet, offset)
        base = offset + PXSB_UNIT_BASE_SIZE

        if _looks_like_frame(packet, base + 8, unit_tag):
            source_time = struct.unpack_from("<Q", packet, base)[0]
            frame_offset = base + 8
        elif _looks_like_frame(packet, base, unit_tag):
            source_time = header[-1]
            frame_offset = base
        else:
            raise ValueError(
                f"Unit 0x{unit_tag:04X} has no valid dev_proto magic"
            )

        frame_end = frame_offset + unit_len
        if frame_end > len(packet):
            raise ValueError(
                f"Truncated unit 0x{unit_tag:04X}: "
                f"need {unit_len}, have {len(packet) - frame_offset}"
            )
        out.append((unit_tag, source_time, packet[frame_offset:frame_end]))
        offset = frame_end
    return out


def parse_company_packet(packet: bytes):
    parser = _company_parser_module()
    parsed = []
    for unit_tag, source_time, frame in unpack_pxsb_units(packet):
        if unit_tag not in FRAME_MAGIC:
            continue
        if len(frame) < 27:
            raise ValueError(f"设备帧过短: {len(frame)} bytes")
        magic, version, frame_type, data_type, length, device_id, timestamp = struct.unpack_from("<IBHHHIQ", frame)
        if magic != FRAME_MAGIC[unit_tag] or frame_type != unit_tag or length != len(frame):
            raise ValueError(f"设备帧头不一致: tag=0x{unit_tag:04X}, ftype=0x{frame_type:04X}, declared={length}, actual={len(frame)}")
        obj = parser.parse_frame(unit_tag, frame, len(frame))
        if obj is None:
            raise ValueError(f"设备帧体无法解析: tag=0x{unit_tag:04X}, length={len(frame)}")
        parsed.append((unit_tag, source_time, obj))
    return parsed


class Wgs84Enu:
    """Company-compatible small-area WGS84/ENU conversion."""

    METERS_PER_DEG_LAT = 111320.0

    def __init__(self, origin):
        self.lng = float(origin[0])
        self.lat = float(origin[1])
        self.alt = float(origin[2])
        self.meters_per_deg_lng = self.METERS_PER_DEG_LAT * math.cos(
            math.radians(self.lat)
        )

    def to_enu(self, lng_deg, lat_deg, alt_m):
        return (
            (float(lng_deg) - self.lng) * self.meters_per_deg_lng,
            (float(lat_deg) - self.lat) * self.METERS_PER_DEG_LAT,
            float(alt_m) - self.alt,
        )

    def valid(self, lng_deg, lat_deg):
        try:
            return (
                math.isfinite(float(lng_deg))
                and math.isfinite(float(lat_deg))
                and abs(float(lng_deg) - self.lng) < 1.0
                and abs(float(lat_deg) - self.lat) < 1.0
            )
        except (TypeError, ValueError):
            return False


def radians(degrees):
    value = math.radians(float(degrees))
    return (value + math.pi) % (2.0 * math.pi) - math.pi
