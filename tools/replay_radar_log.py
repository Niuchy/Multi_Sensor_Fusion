"""Replay decoded dds_receiver radar logs in memory (never publishes DDS).

The text dump has time-of-day, not a date or original wire bytes. This checks
normalization/tracking/fusion, not DDS discovery or byte-for-byte wire fidelity.
"""

import argparse
import json
import re
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
from company_input_node import CompanyInputNode
from dds_tracking_node import TrackingDdsNode
from dds_fusion_node import FusionDdsNode
from company_dds import Topics


class MemoryBus:
    def __init__(self):
        self.messages = {}

    def write(self, topic, payload, **kwargs):
        self.messages[topic] = payload

    def read(self, topic):
        return []

    def publish_official_fusion(self, *args, **kwargs):
        pass


def read_radar_log(path):
    text = Path(path).read_text(encoding="utf-8")
    rows = []
    day = 0
    previous = None
    for block in re.split(r"╔══ Radar #\d+", text)[1:]:
        def number(key):
            match = re.search(r"\b" + key + r"=(-?[\d.]+)", block)
            if not match:
                raise ValueError(f"Missing {key} in radar log entry")
            return float(match.group(1))

        def integer(key):
            match = re.search(r"\b" + key + r"=(0x[0-9A-Fa-f]+|\d+)", block)
            if not match:
                raise ValueError(f"Missing {key} in radar log entry")
            return int(match.group(1), 16 if match.group(1).startswith("0x") else 10)

        tag = re.search(r"\btag=(\S+)", block)
        target_type = re.search(r"\btype=(\S+)", block)
        tags = {"外推点": 0, "真实点": 1}
        types = {"未知": 0, "鸟": 1, "无人机": 2, "车辆": 3,
                 "人员": 4, "大型目标": 5, "其他": 6}
        if not tag or tag[1] not in tags or not target_type or target_type[1] not in types:
            raise ValueError("Unknown or missing radar tag / target type; refusing to invent values")

        stamp = re.search(r"ts=(\d+):(\d+):(\d+)\.(\d+)", block)
        if not stamp:
            raise ValueError("Missing radar timestamp")
        h, m, s, ms = map(int, stamp.groups())
        tod = ((h * 60 + m) * 60 + s) * 1000 + ms
        if previous is not None and tod < previous - 43_200_000:
            day += 86_400_000
        previous = tod
        rows.append(SimpleNamespace(
            device_id=int(number("dev")), track_id=int(number("track_id")),
            timestamp_ms=day + tod, range_m=number("dis"),
            azimuth_deg=number("azi"), elevation_deg=number("ele"),
            height_m=number("height"), radial_vel_ms=number("radial_vel"),
            lng_deg=number("lng"), lat_deg=number("lat"), alt_m=number("alt"),
            speed_ms=number("speed"), course_deg=number("course"),
            extrap_flag=tags[tag[1]], target_type=types[target_type[1]], confidence=number("conf"),
            source_address=integer("src"), destination_address=integer("dst"),
            message_id=integer("msgID"), protocol_version=integer("ver"),
            data_type=integer("dtype"), crc32=integer("crc32"),
        ))
    if not rows:
        raise ValueError("No Radar entries found")
    return rows


def replay(path, *, scenario=None, **input_options):
    rows = read_radar_log(path)
    bus = MemoryBus()
    input_node = CompanyInputNode(
        scenario or ROOT / "anti-drone-pkg/config/scenarios/multi_target.yaml",
        bus=bus, **input_options,
    )
    tracking = TrackingDdsNode(bus=bus)
    fusion = FusionDdsNode(bus=bus)
    result = []
    for row in rows:
        input_node.ingest(0x1A01, row.timestamp_ms, row)
        input_node.flush_due(force=True)
        raw = bus.messages[Topics.RAW_MEASUREMENTS.name]
        tracking.process_raw(raw)
        tracks = bus.messages[Topics.SENSOR_TRACKS.name]
        fusion.process_tracks(tracks)
        fused = bus.messages[Topics.FUSED_TRACKS.name]
        result.append({
            "timestamp_ms": row.timestamp_ms, "dt": raw["dt"],
            "radar_ids": [t["id"] for t in tracks["radar_tracks"]],
            "internal_hits": [getattr(t, "hits", None)
                              for trk in tracking.radar_trks.values()
                              for t in getattr(trk, "tracks", [])],
            "fused_ids": [t["id"] for t in fused["fused"]],
            "external_ids": [t["id"] for t in fused["external_tracks"]],
        })
    intervals = [(b.timestamp_ms - a.timestamp_ms) / 1000
                 for a, b in zip(rows, rows[1:])]
    return {
        "mode": "offline decoded-log replay; no DDS publication",
        "records": len(rows), "device_ids": sorted({r.device_id for r in rows}),
        "source_track_ids": sorted({r.track_id for r in rows}),
        "real_points": sum(r.extrap_flag == 1 for r in rows),
        "predicted_points": sum(r.extrap_flag == 0 for r in rows),
        "interval_min_s": min(intervals), "interval_max_s": max(intervals),
        "interval_mean_s": sum(intervals) / len(intervals),
        "frames_with_fused_output": sum(bool(r["fused_ids"]) for r in result),
        "frames_with_external_output": sum(bool(r["external_ids"]) for r in result),
        "frames": result,
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--device-tracks", action="store_true")
    parser.add_argument("--scenario", type=Path, help="Optional equipment/site YAML")
    args = parser.parse_args()
    options = {"radar_input_mode": "device_tracks", "azimuth_convention": "north_cw"} if args.device_tracks else {}
    print(json.dumps(replay(args.log, scenario=args.scenario, **options), ensure_ascii=False, indent=2))
