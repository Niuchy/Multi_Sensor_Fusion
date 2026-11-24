"""Inspect this demo's DDS topics.

Examples:
    python dds_inspect.py list
    python dds_inspect.py echo frame --once
    python dds_inspect.py hz fused_tracks
"""

import argparse
import json
import sys
import time

from company_dds import CompanyDdsBus, Topics


ALIASES = {
    "sensor_info": Topics.SENSOR_INFO,
    "info": Topics.SENSOR_INFO,
    "raw": Topics.RAW_MEASUREMENTS,
    "measurements": Topics.RAW_MEASUREMENTS,
    "sensor_tracks": Topics.SENSOR_TRACKS,
    "tracks": Topics.SENSOR_TRACKS,
    "fused_tracks": Topics.FUSED_TRACKS,
    "fused": Topics.FUSED_TRACKS,
    "frame": Topics.FUSED_FRAME,
    "control": Topics.CONTROL_COMMAND,
    "company_radar": Topics.COMPANY_RADAR,
    "company_esm": Topics.COMPANY_ESM,
    "company_eo": Topics.COMPANY_EO,
    "company_acoustic": Topics.COMPANY_ACOUSTIC,
    "diagnostics": Topics.DIAGNOSTICS,
    "official_fusion": Topics.OFFICIAL_FUSION,
}


def all_topic_specs():
    return [
        Topics.SENSOR_INFO,
        Topics.RAW_MEASUREMENTS,
        Topics.SENSOR_TRACKS,
        Topics.FUSED_TRACKS,
        Topics.FUSED_FRAME,
        Topics.CONTROL_COMMAND,
        Topics.COMPANY_RADAR,
        Topics.COMPANY_ESM,
        Topics.COMPANY_EO,
        Topics.COMPANY_ACOUSTIC,
        Topics.OFFICIAL_FUSION,
        Topics.DIAGNOSTICS,
    ]


def resolve_topic(name):
    if name in ALIASES:
        return ALIASES[name].name
    for spec in all_topic_specs():
        if name == spec.name:
            return spec.name
    raise SystemExit("unknown topic or alias: {}".format(name))


def cmd_list(_args):
    print("{:<28} {}".format("ALIAS", "DDS TOPIC"))
    print("{:<28} {}".format("-" * 12, "-" * 40))
    for alias, spec in sorted(ALIASES.items()):
        print("{:<28} {}".format(alias, spec.name))
    print("\nKnown project topics:")
    for spec in all_topic_specs():
        print("  {:<32} {}".format(spec.name, spec.description))


def make_bus():
    try:
        return CompanyDdsBus(source="dds_inspect")
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2)


def cmd_echo(args):
    topic = resolve_topic(args.topic)
    bus = make_bus()
    print("echo {}  (Ctrl-C to stop)".format(topic), file=sys.stderr)
    while True:
        if topic in {
            Topics.COMPANY_RADAR.name,
            Topics.COMPANY_ESM.name,
            Topics.COMPANY_EO.name,
            Topics.COMPANY_ACOUSTIC.name,
            Topics.OFFICIAL_FUSION.name,
        }:
            samples = [
                {
                    "binary_topic": topic,
                    "length": len(data),
                    "head_hex": data[:64].hex(" "),
                }
                for data in bus.read_bytes(topic)
            ]
        else:
            samples = bus.read(topic)
        for sample in samples:
            text = json.dumps(sample, ensure_ascii=False, indent=2)
            if args.max_chars and len(text) > args.max_chars:
                text = text[: args.max_chars] + "\n... <truncated>"
            print(text)
            if args.once:
                return
        time.sleep(args.period)


def cmd_hz(args):
    topic = resolve_topic(args.topic)
    bus = make_bus()
    count = 0
    start = time.time()
    last = start
    print("measuring {}  (Ctrl-C to stop)".format(topic), file=sys.stderr)
    while True:
        if topic in {
            Topics.COMPANY_RADAR.name,
            Topics.COMPANY_ESM.name,
            Topics.COMPANY_EO.name,
            Topics.COMPANY_ACOUSTIC.name,
            Topics.OFFICIAL_FUSION.name,
        }:
            count += len(bus.read_bytes(topic))
        else:
            count += len(bus.read(topic))
        now = time.time()
        if now - last >= args.window:
            elapsed = max(now - start, 1e-9)
            print("{:.2f} Hz over {:.1f}s ({} samples)".format(count / elapsed, elapsed, count))
            last = now
        time.sleep(args.period)


def main():
    parser = argparse.ArgumentParser(description="Inspect DDS topics for this demo.")
    sub = parser.add_subparsers(dest="cmd")

    p_list = sub.add_parser("list", help="list known project DDS topics")
    p_list.set_defaults(func=cmd_list)

    p_echo = sub.add_parser("echo", help="print samples from a DDS topic")
    p_echo.add_argument("topic", help="topic name or alias, e.g. frame/raw/fused_tracks")
    p_echo.add_argument("--once", action="store_true")
    p_echo.add_argument("--period", type=float, default=0.05)
    p_echo.add_argument("--max-chars", type=int, default=6000)
    p_echo.set_defaults(func=cmd_echo)

    p_hz = sub.add_parser("hz", help="estimate sample rate for a DDS topic")
    p_hz.add_argument("topic", help="topic name or alias, e.g. frame/raw/fused_tracks")
    p_hz.add_argument("--window", type=float, default=1.0)
    p_hz.add_argument("--period", type=float, default=0.02)
    p_hz.set_defaults(func=cmd_hz)

    args = parser.parse_args()
    if not hasattr(args, "func"):
        parser.print_help()
        raise SystemExit(1)
    args.func(args)


if __name__ == "__main__":
    main()
