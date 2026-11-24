"""FastAPI routes for a preview-only alignment workbench."""
import asyncio
import json
from pathlib import Path
from urllib.parse import urlsplit

from fastapi import HTTPException, Request

from alignment_debug import AlignmentWorkbench, FORMAT, MAX_RECORDS, compare, device_key, receiver_log_records, validate_record

MAX_UPLOAD = 8 * 1024 * 1024


def install_alignment_api(app, gateway, profile_path=None):
    workbench = AlignmentWorkbench(profile_path or Path(__file__).resolve().parents[1] / "configs/alignment_profiles.json")
    gateway.alignment = workbench

    async def payload(request):
        # Trusted-LAN debug UI, not an authentication system. Protect writes against
        # blind cross-site forms/fetch; there is no permissive CORS middleware.
        origin = request.headers.get("origin")
        if origin and urlsplit(origin).netloc != request.headers.get("host"):
            raise HTTPException(403, "不接受跨站请求")
        if request.headers.get("x-alignment-token") != workbench.token:
            raise HTTPException(403, "页面令牌失效，请刷新页面")
        raw = bytearray()
        async for chunk in request.stream():
            raw.extend(chunk)
            if len(raw) > MAX_UPLOAD:
                raise HTTPException(413, "单次上传最多 8 MiB")
        try:
            result = json.loads(raw)
            if not isinstance(result, dict):
                raise ValueError("请求必须是 JSON 对象")
            return result
        except (ValueError, UnicodeError) as exc:
            raise HTTPException(400, str(exc))

    def checked(func, *args, **kwargs):
        try:
            return func(*args, **kwargs)
        except FileExistsError as exc:
            raise HTTPException(409, str(exc))
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            raise HTTPException(400, str(exc))
        except OSError as exc:
            raise HTTPException(503, f"配置文件不可读写：{exc}")

    @app.get("/api/alignment")
    def state():
        return checked(workbench.state)

    @app.get("/api/alignment/capture")
    def capture(device: str = ""):
        return checked(workbench.export, device or None)

    @app.post("/api/alignment/preview")
    async def preview(request: Request):
        data = await payload(request)
        device = checked(device_key, data.get("device"))
        records = checked(workbench.export, device)["records"]
        return await asyncio.to_thread(checked, compare, records, data.get("params", {}))

    @app.post("/api/alignment/save")
    async def save(request: Request):
        data = await payload(request)
        if type(data.get("expected_revision")) is not int:
            raise HTTPException(400, "缺少整数 expected_revision")
        await asyncio.to_thread(checked, workbench.store.save, data["expected_revision"], device=data.get("device"),
                                params=data.get("params"), note=data.get("note", ""))
        return checked(workbench.state)

    @app.post("/api/alignment/rollback")
    async def rollback(request: Request):
        data = await payload(request)
        if type(data.get("expected_revision")) is not int or type(data.get("revision")) is not int:
            raise HTTPException(400, "版本号必须是整数")
        await asyncio.to_thread(checked, workbench.store.save, data["expected_revision"],
                                rollback=data["revision"], note=data.get("note", ""))
        return checked(workbench.state)

    @app.post("/api/alignment/replay")
    async def replay(request: Request):
        data = await payload(request)
        def calculate():
            content = data.get("content")
            if not isinstance(content, str):
                raise ValueError("请上传采集 JSON 或 1.log 文本")
            if data.get("format") == "receiver_log":
                context = data.get("context", workbench.default_context)
                records = receiver_log_records(content, context)
                note = "解码文本不含日期、原始字节或接收时间；只比较相对时间和坐标。站址/原点来自本次选择的上下文。"
            elif data.get("format") == "capture":
                document = json.loads(content)
                if not isinstance(document, dict) or document.get("format") != FORMAT:
                    raise ValueError("不是本工具导出的采集格式")
                if not isinstance(document.get("records"), list) or len(document["records"]) > MAX_RECORDS:
                    raise ValueError(f"最多回放 {MAX_RECORDS} 条记录")
                records = [validate_record(r) for r in document["records"]]
                note = "使用采集时的原点/站址与时间；原始文件未修改。"
            else:
                raise ValueError("未知回放格式")
            devices = sorted({f"radar_{r['raw']['device_id']}" for r in records})
            device = device_key(data.get("device"))
            records = [r for r in records if f"radar_{r['raw']['device_id']}" == device]
            if not records:
                raise ValueError("文件中没有所选设备，请选择：" + ", ".join(devices))
            result = compare(records, data.get("params", {}))
            result.update(note=note, devices=devices, dds_packets_sent=0)
            return result
        return await asyncio.to_thread(checked, calculate)

    return workbench
