"""Expose fused tracks as a read-only WebSocket service for external clients."""

from __future__ import annotations

import argparse
import asyncio
import json
import logging

import websockets

from fused_tracks_ws_sender import (
    CompanyDdsBus,
    FusionSnapshotState,
    Topics,
)


LOG = logging.getLogger("fused_tracks_ws_server")


def normalize_ws_path(value: str) -> str:
    path = str(value or "").strip()
    if not path:
        raise argparse.ArgumentTypeError("WebSocket路径不能为空")
    if not path.startswith("/"):
        path = f"/{path}"
    if "?" in path or "#" in path:
        raise argparse.ArgumentTypeError("WebSocket路径不能包含查询参数或片段")
    return path


class ExternalFusedTracksServer:
    """Subscribe to fused DDS snapshots and broadcast them to read-only clients."""

    def __init__(self, *, bus=None, poll_interval=0.01, path="/ws/main"):
        self.bus = bus or CompanyDdsBus(source="external_ws_server")
        self.poll_interval = float(poll_interval)
        self.path = normalize_ws_path(path)
        self.state = FusionSnapshotState()
        self.clients = set()
        self.latest_snapshot_payload: str | None = None
        self.broadcast_count = 0

    @staticmethod
    def _encode(envelope: dict) -> str:
        return json.dumps(
            envelope,
            ensure_ascii=False,
            separators=(",", ":"),
            allow_nan=False,
        )

    async def broadcast(self, payload: str) -> None:
        if not self.clients:
            return
        clients = list(self.clients)
        results = await asyncio.gather(
            *(client.send(payload) for client in clients),
            return_exceptions=True,
        )
        for client, result in zip(clients, results):
            if isinstance(result, BaseException):
                self.clients.discard(client)

    async def publish_message(self, message: dict) -> None:
        """Convert one DDS snapshot and send the same update to every client."""
        self.state.ingest(message)
        envelope, deleted_ids, _ = self.state.render()
        await self.broadcast(self._encode(envelope))
        self.state.acknowledge_deletions(deleted_ids)

        # A newly connected client needs the current full snapshot, not an old
        # one-shot deletion marker that was already broadcast to live clients.
        current_envelope, _, _ = self.state.render()
        self.latest_snapshot_payload = self._encode(current_envelope)
        self.broadcast_count += 1
        if self.broadcast_count == 1 or self.broadcast_count % 20 == 0:
            LOG.info(
                "已广播融合帧: tick=%s tracks=%s clients=%s deleted=%s",
                self.state.tick,
                len(self.state.current),
                len(self.clients),
                len(deleted_ids),
            )

    async def poll_dds(self) -> None:
        while True:
            for message in self.bus.read(Topics.FUSED_TRACKS.name):
                await self.publish_message(message)
            await asyncio.sleep(self.poll_interval)

    async def ws_handler(self, websocket, path=None) -> None:
        request = getattr(websocket, "request", None)
        request_path = (
            path
            or getattr(request, "path", None)
            or getattr(websocket, "path", None)
        )
        if request_path != self.path:
            await websocket.close(code=1008, reason="WebSocket path not found")
            return
        if self.latest_snapshot_payload is not None:
            await websocket.send(self.latest_snapshot_payload)
        self.clients.add(websocket)
        LOG.info(
            "外部客户端已连接: %s，当前客户端=%s",
            getattr(websocket, "remote_address", None),
            len(self.clients),
        )
        try:
            # This endpoint is intentionally read-only. Iterating incoming
            # messages only observes disconnects; all client payloads are ignored.
            async for _ in websocket:
                continue
        except Exception as exc:
            LOG.debug("外部客户端连接结束: %s", exc)
        finally:
            self.clients.discard(websocket)
            LOG.info("外部客户端已断开，当前客户端=%s", len(self.clients))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="订阅公司DDS融合结果并提供只读WebSocket服务"
    )
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=11409)
    parser.add_argument("--path", type=normalize_ws_path, default="/ws/main")
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=("DEBUG", "INFO", "WARNING", "ERROR"),
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port 必须在 1~65535 之间")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    async def run():
        server = ExternalFusedTracksServer(path=args.path)
        async with websockets.serve(
            server.ws_handler,
            args.host,
            args.port,
            ping_interval=20,
            ping_timeout=20,
            max_queue=1,
        ):
            LOG.info(
                "外部融合WebSocket服务启动: ws://%s:%s%s",
                args.host,
                args.port,
                args.path,
            )
            await server.poll_dds()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        LOG.info("外部融合WebSocket服务已停止")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
