"""Trusted-LAN UI bridge; live policy/persistence are owned by fusion, not HTTP."""
import asyncio
import json
import secrets
import time
from urllib.parse import urlsplit
import uuid

from fastapi import HTTPException, Request
from fastapi.responses import JSONResponse

from company_dds import Topics
from output_filter import validate


def install_output_filter_api(app, gateway):
    token = secrets.token_urlsafe(32)
    lock = asyncio.Lock()

    def state():
        node = gateway.nodes.get("fusion", {})
        alive = time.time()-gateway.node_received_at.get("fusion", 0) < 5
        return dict(alive=alive, available=bool(node.get("output_filter")),
                    state=node.get("output_filter"), token=token,
                    server_time=time.time())

    @app.get("/api/output-filter")
    async def get_state():
        return JSONResponse(state(), headers={"Cache-Control": "no-store"})

    @app.post("/api/output-filter/apply")
    async def apply(request: Request):
        origin = request.headers.get("origin")
        if origin and urlsplit(origin).netloc != request.headers.get("host"):
            raise HTTPException(403, "不接受跨站请求")
        if not secrets.compare_digest(request.headers.get("x-output-filter-token", ""), token):
            raise HTTPException(403, "页面令牌失效，请刷新")
        if request.headers.get("content-type", "").split(";")[0].strip() != "application/json":
            raise HTTPException(415, "请求必须为 application/json")
        body = bytearray()
        async for chunk in request.stream():
            body.extend(chunk)
            if len(body) > 8192:
                raise HTTPException(413, "请求过大")
        try:
            data = json.loads(body)
            if not isinstance(data, dict):
                raise ValueError("请求必须为 JSON 对象")
            settings = validate(data.get("settings"))
            if type(data.get("save")) is not bool or type(data.get("expected_revision")) is not int:
                raise ValueError("必须提供 save 和 expected_revision")
        except (TypeError, ValueError, UnicodeError) as exc:
            raise HTTPException(400, str(exc))
        async with lock:
            current = state()
            if not current["alive"] or not current["available"]:
                raise HTTPException(503, "融合节点未连接或尚未加载新版过滤功能；没有应用或保存")
            active = current["state"]
            if data.get("instance_id") != active["instance_id"] or data["expected_revision"] != active["revision"]:
                raise HTTPException(409, "运行参数已变化，请刷新后重新确认")
            if data["save"] and data.get("expected_saved_hash") != active["saved_hash"]:
                raise HTTPException(409, "已保存配置已变化，请刷新后重新确认")
            request_id = uuid.uuid4().hex
            gateway.bus.write(Topics.CONTROL_COMMAND.name, dict(cmd="output_filter_apply",
                request_id=request_id, expires_at=time.time()+5,
                settings=settings, save=data["save"], instance_id=active["instance_id"],
                expected_revision=active["revision"], expected_saved_hash=data.get("expected_saved_hash")),
                kind="control_command", tick=int((gateway.latest_frame or {}).get("tick", 0)))
            deadline = time.monotonic()+3
            while time.monotonic() < deadline:
                received = state()
                command = (received.get("state") or {}).get("last_command") or {}
                if command.get("request_id") == request_id:
                    return JSONResponse({**received, "result": command},
                        status_code=200 if command.get("ok") else 409,
                        headers={"Cache-Control": "no-store"})
                await asyncio.sleep(.05)
            # Delivery is not application. UI must wait for the matching ACK.
            return JSONResponse({**state(), "request_id": request_id,
                                 "pending": True, "message": "尚未收到融合节点确认，请等待状态更新"},
                                status_code=202, headers={"Cache-Control": "no-store"})
