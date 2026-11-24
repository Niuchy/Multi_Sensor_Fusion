"""Forward fused tracks from company DDS to an external WebSocket backend."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import sys
import time
from pathlib import Path
from urllib.parse import urlsplit


ROOT = Path(__file__).resolve().parent
DDS_DIR = ROOT / "dds_swarm_fusion"
ROOT_BRIDGE = ROOT / "libcompany_dds_bridge.so"
if str(DDS_DIR) not in sys.path:
    sys.path.insert(0, str(DDS_DIR))

import company_dds  # noqa: E402


# Direct execution and root launcher both use the bridge built at workspace root.
company_dds._bridge_path = lambda: ROOT_BRIDGE
CompanyDdsBus = company_dds.CompanyDdsBus
Topics = company_dds.Topics


LOG = logging.getLogger("fused_tracks_ws_sender")

TARGET_UNKNOWN = 0
TARGET_TYPES = {
    "unknown": TARGET_UNKNOWN,
    "bird": 1,
    "drone": 2,
    "quad": 2,
    "fixed_wing": 2,
    "heli": 2,
    "vehicle": 3,
    "person": 4,
    "large_target": 5,
    "other": 6,
}

TRACK_CONFIRMED = 1
TRACK_COASTING = 2
TRACK_DELETED = 3


def validate_ws_url(value: str) -> str:
    parsed = urlsplit(value)
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname:
        raise argparse.ArgumentTypeError(
            "WebSocket地址必须是 ws:// 或 wss:// 开头的完整地址"
        )
    return value


def _finite_float(value, default=0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    return number if math.isfinite(number) else float(default)


def _optional_float(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _optional_text(value):
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _external_sensor_id(sensor_id) -> str | None:
    value = _optional_text(sensor_id)
    if value is None:
        return None
    if value == "rf_df":
        return "esm"
    if value.startswith("rf_df_"):
        return f"esm_{value[len('rf_df_') :]}"
    return value


def target_type_for(track) -> int:
    has_rf_identity = any(
        track.get(key) for key in ("rf_id", "rf_freq", "sn_id", "model")
    )
    if not bool(track.get("class_evidence", False)) and not has_rf_identity:
        return TARGET_UNKNOWN
    return TARGET_TYPES.get(str(track.get("cls") or "unknown"), TARGET_UNKNOWN)


def format_track(track, *, origin, timestamp_ms: int) -> dict:
    """Convert one ENU fused track to the colleague backend's JSON schema."""
    track_id = int(track["id"])
    try:
        ref_lng, ref_lat, ref_alt = [float(value) for value in origin[:3]]
    except (TypeError, ValueError, IndexError):
        ref_lng, ref_lat, ref_alt = 120.0, 30.0, 0.0

    x = _finite_float(track.get("x"))
    y = _finite_float(track.get("y"))
    z = _finite_float(track.get("z"))
    vx = _finite_float(track.get("vx"))
    vy = _finite_float(track.get("vy"))
    vz = _optional_float(track.get("vz"))
    confidence = min(1.0, max(0.0, _finite_float(track.get("conf"))))
    longitude_scale = 111320.0 * math.cos(math.radians(ref_lat))
    if abs(longitude_scale) < 1e-6:
        longitude_scale = 1e-6

    raw_sensor_ids = track.get("sensor_ids")
    if not raw_sensor_ids:
        raw_sensor_ids = track.get("sources", [])
    if isinstance(raw_sensor_ids, str):
        raw_sensor_ids = [raw_sensor_ids]
    sensor_ids = sorted({
        external
        for external in (
            _external_sensor_id(sensor_id) for sensor_id in raw_sensor_ids
        )
        if external
    })

    age_ticks = max(0, int(_finite_float(track.get("age_ticks"), 0.0)))
    rf_freq = _optional_float(track.get("rf_freq"))
    if rf_freq is not None and rf_freq <= 0:
        rf_freq = None

    return {
        "trackId": track_id,
        "lat": round(ref_lat + y / 111320.0, 7),
        "lon": round(ref_lng + x / longitude_scale, 7),
        "alt": ref_alt + z,
        "vNorth": vy,
        "vEast": vx,
        "vUp": vz,
        "speed": math.hypot(vx, vy),
        "heading": math.degrees(math.atan2(vx, vy)) % 360.0,
        "targetType": target_type_for(track),
        "confidence": confidence,
        "sensorIds": sensor_ids,
        "status": TRACK_COASTING if track.get("is_coasting", age_ticks > 0) else TRACK_CONFIRMED,
        "updateTime": int(timestamp_ms),
        "rf_freq": rf_freq,
        "rcs": None,
        "sn_id": _optional_text(track.get("sn_id")),
        "model": _optional_text(track.get("model")),
    }


class FusionSnapshotState:
    """Keep the latest full snapshot and reliable one-shot deletion markers."""

    def __init__(self):
        self.current: dict[int, dict] = {}
        self.current_order: list[int] = []
        self.pending_deletions: set[int] = set()
        self.timestamp_ms = 0
        self.tick = 0
        self.revision = 0

    def ingest(self, message: dict) -> None:
        timestamp_ms = int(
            _finite_float(message.get("source_timestamp_ms"), time.time() * 1000.0)
        )
        if timestamp_ms <= 0:
            timestamp_ms = int(time.time() * 1000.0)
        coordinates = message.get("coordinates") or {}
        origin = coordinates.get("origin") or [120.0, 30.0, 0.0]
        tracks = message.get("external_tracks")
        if tracks is None:
            tracks = message.get("fused", [])

        next_current = {}
        next_order = []
        for track in tracks or []:
            try:
                row = format_track(
                    track, origin=origin, timestamp_ms=timestamp_ms
                )
            except (KeyError, TypeError, ValueError) as exc:
                LOG.warning("忽略无法转换的融合航迹: %s", exc)
                continue
            track_id = row["trackId"]
            next_current[track_id] = row
            if track_id not in next_order:
                next_order.append(track_id)

        previous_ids = set(self.current)
        current_ids = set(next_current)
        self.pending_deletions.update(previous_ids - current_ids)
        # If an ID is present again before its deletion was sent, current state wins.
        self.pending_deletions.difference_update(current_ids)
        self.current = next_current
        self.current_order = next_order
        self.timestamp_ms = timestamp_ms
        self.tick = int(_finite_float(message.get("tick"), self.tick))
        self.revision += 1

    def render(self):
        deleted_ids = set(self.pending_deletions)
        data = [self.current[track_id] for track_id in self.current_order]
        data.extend(
            {
                "trackId": track_id,
                "status": TRACK_DELETED,
                "updateTime": self.timestamp_ms,
            }
            for track_id in sorted(deleted_ids)
        )
        envelope = {
            "type": "Ele",
            "module": "DataFusion",
            "interface": "FusedTracks",
            "data": data,
        }
        return envelope, deleted_ids, self.revision

    def acknowledge_deletions(self, deleted_ids: set[int]) -> None:
        self.pending_deletions.difference_update(deleted_ids)


class ExternalWebSocketPublisher:
    def __init__(self, url: str, *, bus=None, poll_interval=0.01):
        self.url = url
        self.bus = bus or CompanyDdsBus(source="external_ws_sender")
        self.poll_interval = float(poll_interval)
        self.state = FusionSnapshotState()
        self.changed = asyncio.Event()

    async def poll_dds(self):
        while True:
            for message in self.bus.read(Topics.FUSED_TRACKS.name):
                self.state.ingest(message)
                self.changed.set()
            await asyncio.sleep(self.poll_interval)

    async def publish(self):
        try:
            import websockets
        except ImportError as exc:
            raise RuntimeError(
                "缺少 websockets，请安装 dds_swarm_fusion/requirements.txt"
            ) from exc

        backoffs = (1, 2, 5, 10, 30)
        backoff_index = 0
        sent_count = 0
        while True:
            try:
                LOG.info("正在连接外部后端: %s", self.url)
                async with websockets.connect(
                    self.url,
                    open_timeout=10,
                    close_timeout=5,
                    ping_interval=20,
                    ping_timeout=20,
                    max_queue=1,
                ) as websocket:
                    LOG.info("外部后端已连接: %s", self.url)
                    backoff_index = 0
                    self.changed.set()
                    while True:
                        await self.changed.wait()
                        self.changed.clear()
                        if self.state.revision == 0:
                            continue
                        envelope, deleted_ids, revision = self.state.render()
                        payload = json.dumps(
                            envelope,
                            ensure_ascii=False,
                            separators=(",", ":"),
                            allow_nan=False,
                        )
                        LOG.debug("发送JSON: %s", payload)
                        await websocket.send(payload)
                        self.state.acknowledge_deletions(deleted_ids)
                        sent_count += 1
                        if sent_count == 1 or sent_count % 20 == 0:
                            LOG.info(
                                "已发送融合帧: tick=%s tracks=%s deleted=%s",
                                self.state.tick,
                                len(self.state.current),
                                len(deleted_ids),
                            )
                        if self.state.revision != revision:
                            self.changed.set()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                delay = backoffs[min(backoff_index, len(backoffs) - 1)]
                backoff_index = min(backoff_index + 1, len(backoffs) - 1)
                LOG.warning("外部后端连接/发送失败: %s；%ss后重试", exc, delay)
                await asyncio.sleep(delay)

    async def run(self):
        await asyncio.gather(self.poll_dds(), self.publish())


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="订阅公司DDS融合结果并推送到外部WebSocket后端"
    )
    parser.add_argument("--url", required=True, type=validate_ws_url)
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    publisher = ExternalWebSocketPublisher(args.url)
    try:
        asyncio.run(publisher.run())
    except KeyboardInterrupt:
        LOG.info("外部WebSocket发送节点已停止")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
