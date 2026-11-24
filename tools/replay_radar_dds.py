"""Reconstruct decoded radar logs and replay ONCE through real company DDS.

No publication without --publish. Preserves logged values and time intervals;
rebases time-of-day timestamps to the current epoch because the date is absent.
The log is not a byte capture: unlogged envelope fields are zero, UnitHeader is
14 bytes, and no claim of byte-for-byte reproduction is made.
"""

import argparse
import asyncio
import hashlib
import inspect
import json
import struct
import sys
import time
import urllib.request
from pathlib import Path

from replay_radar_log import ROOT, read_radar_log
import company_dds
from company_protocol import PXSB_HEADER_FORMAT, parse_company_packet


def encode_radar_packet(row, timestamp_ms):
    """Pack dev_proto's 23-byte header, 39-byte body, 4-byte tail + PXSB."""
    frame = struct.pack("<IBHHHIQ", 0xA55AA55A, row.protocol_version,
                        0x1A01, row.data_type, 66, row.device_id, timestamp_ms)
    frame += struct.pack("<IIhhIhiiIIHBBB", row.track_id, round(row.range_m * 10),
        round(row.azimuth_deg * 100), round(row.elevation_deg * 100),
        round(row.height_m * 10), round(row.radial_vel_ms * 10),
        round(row.lng_deg * 1e7), round(row.lat_deg * 1e7),
        round(row.alt_m * 10), round(row.speed_ms * 10), round(row.course_deg * 100),
        row.extrap_flag, row.target_type, round(row.confidence))
    frame += struct.pack("<I", row.crc32)
    unit = struct.pack("<IHQ", len(frame), 0x1A01, timestamp_ms) + frame
    packet = struct.pack(PXSB_HEADER_FORMAT, 0x7E, 1, row.source_address,
        row.destination_address, len(unit), 0, row.message_id, 0, 0, timestamp_ms) + unit
    # Fail before publishing on invalid reconstruction.
    parsed = parse_company_packet(packet)[0][2]
    if parsed.track_id != row.track_id or parsed.extrap_flag != row.extrap_flag:
        raise ValueError("Radar packet round-trip mismatch")
    return packet


def make_schedule(rows, epoch_ms):
    first = rows[0].timestamp_ms
    if any(b.timestamp_ms < a.timestamp_ms for a, b in zip(rows, rows[1:])):
        raise ValueError("Log timestamps are out of order")
    return [(r.timestamp_ms - first, epoch_ms + r.timestamp_ms - first,
             encode_radar_packet(r, epoch_ms + r.timestamp_ms - first)) for r in rows]


def get_diagnostics(gateway):
    # Debug endpoints are explicit targets, not requests through a shell's proxy.
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with opener.open(gateway.rstrip("/") + "/api/diagnostics", timeout=4) as response:
        return json.load(response)


def websocket_options(connect):
    options = dict(open_timeout=5, close_timeout=2)
    # websockets 15+ discovers proxies automatically; 11-14 has no proxy kwarg.
    if "proxy" in inspect.signature(connect).parameters:
        options["proxy"] = None
    return options


async def check_connections(args):
    """Read-only HTTP/WS preflight: no DDS participant, writer, or test packets."""
    import websockets
    snapshot = get_diagnostics(args.gateway)
    urls = [args.gateway.replace("http", "ws", 1).rstrip("/") + "/ws", args.external_ws]
    for url in urls:
        try:
            async with websockets.connect(url, **websocket_options(websockets.connect)) as ws:
                pong = await ws.ping()
                await asyncio.wait_for(pong, timeout=3)
        except Exception as exc:
            raise RuntimeError(f"WebSocket 直连失败 {url}: {type(exc).__name__}: {exc}") from exc
    alive = {name: snapshot.get("nodes", {}).get(name, {}).get("alive", False)
             for name in ("input", "tracking", "fusion")}
    if not all(alive.values()):
        raise RuntimeError(f"融合节点心跳异常: {alive}")
    return {"nodes_alive": alive, "websocket_direct": urls, "dds_packets_sent": 0}


def packet_counts(snapshot):
    return {k: v["packets"] for k, v in snapshot["nodes"]["input"]["topics"].items()}


def inspect_official_packet(packet):
    """Validate PXSB envelope independently, without forgiving extra bytes."""
    if len(packet) < 41:
        return {"valid": False, "error": "packet too short", "actual_bytes": len(packet)}
    header = struct.unpack_from(PXSB_HEADER_FORMAT, packet)
    unit_length, unit_tag, unit_stamp = struct.unpack_from("<IHQ", packet, 26)
    valid = (header[0] == 0x7E and header[1] == 1 and unit_tag == 1
             and header[4] == len(packet)-26 == 14+unit_length
             and unit_stamp == header[-1])
    return {"valid": valid, "actual_bytes": len(packet), "declared_payload_bytes": header[4],
            "actual_payload_bytes": len(packet)-26, "unit_length": unit_length,
            "header_timestamp_ms": header[-1], "unit_timestamp_ms": unit_stamp,
            "target_count": packet[40] if valid else None}


async def run_live(args, rows, result):
    import websockets
    topics = company_dds.Topics
    company_dds._bridge_path = lambda: ROOT / "libcompany_dds_bridge.so"
    bus = company_dds.CompanyDdsBus(source="radar_log_replay_TEST")
    json_topics = [topics.RAW_MEASUREMENTS.name, topics.SENSOR_TRACKS.name,
                   topics.FUSED_TRACKS.name, topics.FUSED_FRAME.name]
    messages = {t: [] for t in json_topics}
    result["dds_messages"] = messages
    result["websocket_messages"] = {"main": [], "external": []}
    result["sent"] = []
    result["wire_received"] = []
    result["official_packets"] = []
    ready = {name: asyncio.get_running_loop().create_future() for name in ("main", "external")}

    async def watch_ws(name, url):
        try:
            async with websockets.connect(url, **websocket_options(websockets.connect)) as ws:
                ready[name].set_result(None)
                async for data in ws:
                    result["websocket_messages"][name].append(json.loads(data))
        except Exception as exc:
            error = RuntimeError(f"WebSocket 直连失败 {url}: {type(exc).__name__}: {exc}")
            if not ready[name].done():
                ready[name].set_exception(error)
            raise error from exc

    bus.writer(topics.COMPANY_RADAR.name)
    for topic in [topics.COMPANY_RADAR.name, topics.OFFICIAL_FUSION.name, *json_topics]:
        bus.reader(topic)
    tasks = [asyncio.create_task(watch_ws("main", args.gateway.replace("http", "ws", 1).rstrip("/") + "/ws")),
             asyncio.create_task(watch_ws("external", args.external_ws))]
    expected_packets = set()

    async def pump_until(deadline):
        while time.monotonic() < deadline:
            for task in tasks:
                if task.done():
                    task.result()
                    raise RuntimeError("WebSocket observer disconnected")
            for packet in bus.read_bytes(topics.COMPANY_RADAR.name):
                if packet not in expected_packets:
                    raise RuntimeError("Unexpected RadarTarget packet: possible live radar; replay stopped")
                result["wire_received"].append(hashlib.sha256(packet).hexdigest())
            for topic in json_topics:
                messages[topic].extend(bus.read(topic))
            for packet in bus.read_bytes(topics.OFFICIAL_FUSION.name):
                result["official_packets"].append(packet.hex())
            await asyncio.sleep(0.02)

    try:
        await asyncio.wait_for(asyncio.gather(*ready.values()), timeout=7)
        before = get_diagnostics(args.gateway)
        if not all(before["nodes"].get(n, {}).get("alive") for n in ("input", "tracking", "fusion")):
            raise RuntimeError("Input/tracking/fusion nodes must all be alive")
        input_node = before["nodes"]["input"]
        if input_node.get("radar_input_mode") != "device_tracks":
            raise RuntimeError("This L2 radar replay requires device_tracks mode")
        last = input_node.get("last_data_received_at")
        if last and before["server_time"] - last < 5:
            raise RuntimeError("Equipment data recently active; refusing to mix replay and live input")
        print("DDS discovery + quiet-input guard (4 seconds); no packets sent yet", flush=True)
        await pump_until(time.monotonic() + 4)
        baseline = get_diagnostics(args.gateway)
        if packet_counts(before) != packet_counts(baseline):
            raise RuntimeError("Equipment counters changed during quiet guard; replay cancelled")
        if before["nodes"]["input"]["pid"] != baseline["nodes"]["input"]["pid"]:
            raise RuntimeError("Input node restarted during quiet guard")
        result["baseline"] = baseline
        start_mono = time.monotonic()
        schedule = make_schedule(rows, int(time.time() * 1000))
        expected_packets.update(packet for _, _, packet in schedule)
        for index, (offset, stamp, packet) in enumerate(schedule):
            await pump_until(start_mono + offset / 1000)
            bus.write_bytes(topics.COMPANY_RADAR.name, packet)
            result["sent"].append({"index": index+1, "timestamp_ms": stamp,
                "offset_ms": offset, "actual_offset_ms": (time.monotonic()-start_mono)*1000,
                "message_id": rows[index].message_id, "packet_hex": packet.hex()})
            print(f"TEST RadarTarget {index+1}/{len(rows)} msgID={rows[index].message_id} "
                  f"dev={rows[index].device_id} target={rows[index].track_id} "
                  f"tag={rows[index].extrap_flag} bytes={len(packet)}", flush=True)
        print(f"Replay finished; observing silence/expiry for {args.settle_seconds:g}s", flush=True)
        await pump_until(time.monotonic() + args.settle_seconds)
        result["after"] = get_diagnostics(args.gateway)
        result["checks"] = verify_result(rows, result)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def verify_result(rows, result):
    topics = company_dds.Topics
    messages = result["dds_messages"]
    stamps = {s["timestamp_ms"] for s in result["sent"]}
    def report_frames(topic):
        return {m["source_timestamp_ms"]: m for m in messages[topic]
                if m.get("source_timestamp_ms") in stamps}
    stages = {topic: report_frames(topic) for topic in messages}
    fused = stages[topics.FUSED_TRACKS.name]
    ids = {t["id"] for m in fused.values() for t in m["external_tracks"]}
    external = [t for m in result["websocket_messages"]["external"] for t in m.get("data", [])]
    main_stamps = {m.get("source_timestamp_ms") for m in result["websocket_messages"]["main"] if m.get("type") == "frame"}
    before, after = result["baseline"]["nodes"]["input"], result["after"]["nodes"]["input"]
    a, b = before["topics"]["RadarTarget"], after["topics"]["RadarTarget"]
    all_predicted = all(r.extrap_flag == 0 for r in rows)
    official = [inspect_official_packet(bytes.fromhex(packet)) for packet in result["official_packets"]]
    result["official_packet_audit"] = official
    checks = {
        "sent_all": len(result["sent"]) == len(rows),
        "wire_all_received": len(result["wire_received"]) == len(rows),
        "input_received_all": b["packets"] - a["packets"] == len(rows),
        "input_parsed_all": b["parsed"] - a["parsed"] == len(rows),
        "no_parse_errors": b["errors"] == a["errors"],
        "no_duplicate_or_old_drops": all(after.get(k, 0) == before.get(k, 0) for k in ("duplicates", "dropped_old")),
        "same_input_process": before["pid"] == after["pid"],
        "dds_all_stages": all(set(frames) == stamps for frames in stages.values()),
        "external_track_every_report": all(len(m["external_tracks"]) > 0 for m in fused.values()),
        "stable_single_fusion_id": len(ids) == 1 if len({(r.device_id,r.track_id) for r in rows}) == 1 else bool(ids),
        "main_websocket_every_report": stamps <= main_stamps,
        "external_websocket_tracks": bool(external),
        "external_websocket_every_report": stamps <= {t.get("updateTime") for t in external},
        "expiry_cleared": result["after"]["nodes"]["fusion"].get("external_count") == 0,
        "external_delete": any(t.get("status") == 3 for t in external),
        "official_dds_envelope_valid": bool(official) and all(p["valid"] for p in official),
    }
    if all_predicted:
        checks.update({
            "predictions_never_confirmed": all(not m["fused"] and all(t["is_coasting"] and t.get("last_measurement_ms") is None for t in m["external_tracks"]) for m in fused.values()),
            "external_coasting_not_confirmed": any(t.get("status") == 2 for t in external) and not any(t.get("status") == 1 for t in external),
        })
    result["summary"] = {"sent": len(result["sent"]), "received": b["packets"]-a["packets"],
        "parsed": b["parsed"]-a["parsed"], "parse_errors": b["errors"]-a["errors"],
        "fusion_ids": sorted(ids), "dds_report_frames": {k:len(v) for k,v in stages.items()},
        "external_statuses": sorted({t.get("status") for t in external}),
        "official_invalid_packets": sum(not p["valid"] for p in official),
        "all_checks_passed": all(checks.values())}
    return checks


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log")
    parser.add_argument("--publish", action="store_true", help="Actually send to the equipment RadarTarget DDS topic once")
    parser.add_argument("--check-only", action="store_true", help="只检查 HTTP/WS 与节点心跳，不初始化或发送 DDS")
    parser.add_argument("--audit-evidence", type=Path, help="Recheck an existing capture without publishing DDS")
    parser.add_argument("--gateway", default="http://127.0.0.1:8000")
    parser.add_argument("--external-ws", default="ws://127.0.0.1:11409/ws/main")
    parser.add_argument("--settle-seconds", type=float, default=15)
    parser.add_argument("--report", type=Path, help="Write captured evidence JSON to a NEW file")
    args = parser.parse_args()
    if sum(bool(x) for x in (args.publish, args.audit_evidence, args.check_only)) > 1:
        parser.error("--publish, --check-only and --audit-evidence are mutually exclusive")
    if args.report and args.report.exists():
        parser.error("Report already exists; choose a new path")
    if args.settle_seconds < 14:
        parser.error("At least 14 seconds required to verify the 12-second expiry")
    rows = read_radar_log(args.log)
    if args.publish or args.check_only:
        # Direct invocation is supported too; the root wrapper usually did this.
        sys.path.insert(0, str(ROOT))
        from run_sb_fusion import ensure_native_runtime
        ensure_native_runtime(sys.argv[1:], entrypoint=Path(__file__).resolve())
    schedule = make_schedule(rows, 1_700_000_000_000)
    result = {"mode": "LIVE DDS TEST" if args.publish else "DRY RUN; NO DDS PUBLICATION",
        "log_sha256": hashlib.sha256(Path(args.log).read_bytes()).hexdigest(),
        "records": len(rows), "duration_s": schedule[-1][0]/1000,
        "timestamp_policy": "rebase to current epoch, preserve original intervals",
        "reconstruction": "106B PXSB + dev_proto; logged fields preserved; unlogged envelope fields zero; not original bytes"}
    if args.audit_evidence:
        with args.audit_evidence.open(encoding="utf-8") as stream:
            captured = json.load(stream)
        if captured["log_sha256"] != result["log_sha256"]:
            parser.error("Evidence does not match this log")
        result = captured
        result["mode"] = "OFFLINE AUDIT OF LIVE DDS CAPTURE; NO NEW PUBLICATION"
        result["checks"] = verify_result(rows, result)
    elif args.check_only:
        result["mode"] = "READ-ONLY CONNECTION CHECK; NO DDS PUBLICATION"
        try:
            result["connection_check"] = asyncio.run(check_connections(args))
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
    elif args.publish:
        try:
            asyncio.run(run_live(args, rows, result))
        except Exception as exc:
            result["error"] = f"{type(exc).__name__}: {exc}"
    if args.report:
        with args.report.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({k:v for k,v in result.items() if k not in {"dds_messages", "websocket_messages", "sent", "wire_received", "official_packets", "official_packet_audit", "baseline", "after"}}, ensure_ascii=False, indent=2))
    return 1 if "error" in result or ((args.publish or args.audit_evidence) and not all(result.get("checks", {"missing": False}).values())) else 0


if __name__ == "__main__":
    sys.exit(main())
