"""Production FastAPI/WebSocket gateway backed by company DDS topics."""

import argparse
import asyncio
import json
import os
import time

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from company_dds import CompanyDdsBus, Topics
from dds_frame import DT, make_init_msg
from json_utils import clean
from official_fusion_monitor import OfficialFusionMonitor
from alignment_api import install_alignment_api
from output_filter_api import install_output_filter_api
from runtime_config import AREA


class Gateway:
    def __init__(self, bus=None):
        self.bus = bus or CompanyDdsBus(source="ws_gateway")
        self.clients = set()
        self.latest_init = make_init_msg({}, dt=DT)
        self.latest_frame = None
        self.fusion_params = {}
        self.nodes = {}
        self.node_received_at = {}
        self.poll_error = None
        self.last_poll_at = None
        self.started_at = time.time()
        self.official_fusion = OfficialFusionMonitor()
        self.alignment = None

    def diagnostics_snapshot(self):
        now = time.time()
        return clean({"server_time": now, "gateway": {
            "started_at": self.started_at, "last_poll_at": self.last_poll_at,
            "error": self.poll_error, "websocket_clients": len(self.clients),
        }, "nodes": {name: {**value,
            "heartbeat_age_s": max(0, now - self.node_received_at[name]),
            "alive": now - self.node_received_at[name] < 5,
        } for name, value in self.nodes.items()},
            "coordinates": self.latest_init.get("coordinates", {}),
            "last_frame_timestamp_ms": (self.latest_frame or {}).get("source_timestamp_ms"),
            "official_fusion": self.official_fusion.snapshot(now)})

    async def broadcast(self, msg):
        txt = json.dumps(clean(msg), ensure_ascii=False)
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_text(txt)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    async def poll_dds(self):
        while True:
            try:
                await self.poll_once()
                self.poll_error = None
                self.last_poll_at = time.time()
            except Exception as exc:
                self.poll_error = f"{type(exc).__name__}: {exc}"
                await asyncio.sleep(1)
            await asyncio.sleep(0.02)

    async def poll_once(self):
        for status in self.bus.read(Topics.DIAGNOSTICS.name):
            name = status.get("node")
            if name in {"input", "tracking", "fusion"}:
                self.nodes[name] = status
                self.node_received_at[name] = time.time()
        for info in self.bus.read(Topics.SENSOR_INFO.name):
            self.latest_init = make_init_msg(
                info.get("sensors", {}),
                fusion_params=self.fusion_params,
                dt=info.get("dt", DT),
                area=info.get("area", AREA),
                coordinates=info.get("coordinates", {}),
                data_source=info.get("data_source", "anti-drone-pkg"),
                dds_implementation=info.get("dds_implementation", "zy-anti-drone TopicAPI/FastDDS"),
                control_capabilities=info.get("control_capabilities", {}),
            )
            await self.broadcast(self.latest_init)
        for frame in self.bus.read(Topics.FUSED_FRAME.name):
            self.fusion_params = frame.get("fusion_params", self.fusion_params)
            if self.latest_init:
                self.latest_init["fusion_params"] = self.fusion_params
            self.latest_frame = frame
            await self.broadcast(frame)
        # Separate, read-only subscriber. Errors must not stop the existing gateway.
        try:
            for packet in self.bus.read_bytes(Topics.OFFICIAL_FUSION.name, max_samples=32):
                self.official_fusion.ingest(packet)
            self.official_fusion.reader_error = None
            self.official_fusion.last_poll_at = time.time()
        except Exception as exc:
            self.official_fusion.reader_error = f"{type(exc).__name__}: {exc}"
        if self.alignment is not None:
            self.alignment.poll(self.bus, self.latest_init, self.nodes.get("input", {}))

    async def ws_endpoint(self, ws: WebSocket):
        await ws.accept()
        self.clients.add(ws)
        try:
            await ws.send_text(json.dumps(clean(self.latest_init), ensure_ascii=False))
            if self.latest_frame:
                await ws.send_text(json.dumps(clean(self.latest_frame), ensure_ascii=False))
            while True:
                try:
                    cmd = json.loads(await ws.receive_text())
                except json.JSONDecodeError:
                    continue
                if not isinstance(cmd, dict) or cmd.get("cmd") == "output_filter_apply":
                    # Filter writes use the validated, token-protected HTTP API.
                    continue
                self.bus.write(
                    Topics.CONTROL_COMMAND.name,
                    cmd,
                    kind="control_command",
                    tick=int((self.latest_frame or {}).get("tick", 0)),
                )
        except WebSocketDisconnect:
            pass
        finally:
            self.clients.discard(ws)


def create_app(bus=None, *, alignment_profile_path=None):
    gateway = Gateway(bus=bus)

    app = FastAPI()
    app.state.gateway = gateway
    install_alignment_api(app, gateway, alignment_profile_path)
    install_output_filter_api(app, gateway)

    @app.on_event("startup")
    async def _startup():
        app.state.dds_poll_task = asyncio.ensure_future(gateway.poll_dds())

    @app.on_event("shutdown")
    async def _shutdown():
        task = getattr(app.state, "dds_poll_task", None)
        if task is not None:
            task.cancel()

    app.websocket("/ws")(gateway.ws_endpoint)

    here = os.path.dirname(os.path.abspath(__file__))

    @app.get("/health")
    def health():
        return {
            "ok": gateway.last_poll_at is not None and gateway.poll_error is None and time.time() - gateway.last_poll_at < 5,
            "mode": "company_dds_gateway",
            "area": gateway.latest_init.get("area", AREA),
            "data_source": gateway.latest_init.get("data_source"),
            "dds_implementation": gateway.latest_init.get(
                "dds_implementation"
            ),
            "has_frame": gateway.latest_frame is not None,
            "dds_poll_error": gateway.poll_error,
        }

    @app.get("/api/diagnostics")
    def diagnostics():
        return gateway.diagnostics_snapshot()

    @app.get("/api/official-fusion")
    def official_fusion():
        return gateway.official_fusion.snapshot()

    static_dir = os.path.join(here, "static")
    if os.path.isdir(static_dir) and os.path.exists(os.path.join(static_dir, "index.html")):
        app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")
    else:
        @app.get("/")
        def index():
            return {"error": "生产静态资源缺失: static/index.html"}

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()
    uvicorn.run(create_app(), host=args.host, port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
