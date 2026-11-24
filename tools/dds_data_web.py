#!/usr/bin/env python3
"""Local/LAN UI for dds_data_tool.py. No fusion imports and no DDS in HTTP process.

Only fixed CLI commands are spawned. A single owned child may run at a time;
inspection cannot publish, and publishing requires a matching preflight ticket.
"""
from __future__ import annotations

import argparse
from collections import deque
import fcntl
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import platform
import re
import secrets
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
from urllib.parse import parse_qs, quote, urlsplit

import dds_data_tool as data

ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).resolve().parent / "dds_data_ui"
ACTIVE = {"running", "stopping"}


class RequestError(Exception):
    def __init__(self, message, status=400):
        super().__init__(message)
        self.status = status


def number(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise RequestError(f"{name} 必须在 {low}–{high} 之间")
    return value


def flag(value, name):
    if type(value) is not bool:
        raise RequestError(name + " 必须为布尔值")
    return value


def filename(value):
    if not isinstance(value, str) or not re.fullmatch(r"[\w][\w .-]{0,110}\.jsonl", value, re.UNICODE) or ".." in value:
        raise RequestError("文件名须以字母、数字或中文开头，以 .jsonl 结尾；不能含路径或连续两点")
    return value


def playback_options(body):
    try:
        topics = list(data.select_topics(body.get("topics", list(data.TOPICS))))
    except (TypeError, ValueError) as exc:
        raise RequestError(str(exc)) from exc
    mode = body.get("time_mode", "original")
    if mode not in ("original", "rebase"):
        raise RequestError("未知时间策略")
    return {"topics": topics, "time_mode": mode,
            "speed": number(body.get("speed", 1), "回放速度", .1, 20),
            "allow_incomplete": flag(body.get("allow_incomplete", False), "恢复未完成文件")}


class Controller:
    def __init__(self, directory, popen=subprocess.Popen, storage_roots=None):
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.storage_roots = tuple(dict.fromkeys([self.directory, *(Path(p).resolve(strict=True) for p in (storage_roots or []))]))
        if any(not p.is_dir() or p == Path(p.anchor) for p in self.storage_roots):
            raise ValueError("存储根目录必须是已有文件夹，不能开放整个文件系统根目录")
        self.popen = popen
        self.lock = threading.RLock()
        self.upload_lock = threading.Lock()
        self.job = None
        self.process = None
        self.ticket = None
        self.last_recording = None
        self.closed = False
        self.native_available = platform.system() == "Linux" and (ROOT/"libcompany_dds_bridge.so").is_file()

    def resolve_directory(self, value=None):
        if value is None or value == "":
            value = str(self.directory)
        if not isinstance(value, str) or any(ord(c) < 32 for c in value):
            raise RequestError("目录路径无效")
        path = Path(value)
        if not path.is_absolute() or ".." in path.parts:
            raise RequestError("请选择设备上的绝对目录路径，不能包含 ..")
        for root in sorted(self.storage_roots, key=lambda p: len(p.parts), reverse=True):
            try:
                relative = path.relative_to(root)
            except ValueError:
                continue
            if any(part.startswith(".") for part in relative.parts):
                raise RequestError("不开放隐藏目录", 403)
            current = root
            for part in ("", *relative.parts):
                current = current / part
                if current.is_symlink():
                    raise RequestError("不允许通过符号链接访问目录", 403)
            if not path.is_dir():
                raise RequestError("目录不存在或不可访问", 404)
            return path
        raise RequestError("目录不在允许范围内；可在启动时用 --storage-root 添加数据目录", 403)

    def browse(self, directory=None):
        directory = self.resolve_directory(directory)
        folders = []
        with os.scandir(directory) as entries:
            for entry in entries:
                if not entry.name.startswith(".") and entry.is_dir(follow_symlinks=False):
                    folders.append({"name": entry.name, "path": str(directory/entry.name)})
        folders.sort(key=lambda item: item["name"].casefold())
        files, total = self.files(str(directory))
        try:
            parent = str(self.resolve_directory(str(directory.parent)))
        except RequestError:
            parent = None
        return {"directory": str(directory), "parent": parent,
                "roots": [str(p) for p in self.storage_roots],
                "folders": folders[:500], "folder_total": len(folders), "files": files, "file_total": total,
                "writable": os.access(directory, os.W_OK | os.X_OK),
                "free_bytes": shutil.disk_usage(directory).free}

    def mkdir(self, directory, name):
        directory = self.resolve_directory(directory)
        if not isinstance(name, str) or not re.fullmatch(r"[\w][\w .-]{0,79}", name) or ".." in name or name.endswith(" "):
            raise RequestError("新文件夹名称须以字母、数字或中文开头，不能含路径、连续两点或末尾空格")
        path = directory / name
        path.mkdir(mode=0o750, exist_ok=False)
        return {"directory": str(path)}

    def path(self, name, existing=True, directory=None):
        path = self.resolve_directory(directory) / filename(name)
        if path.is_symlink():
            raise RequestError("不允许访问符号链接")
        if existing:
            if not path.exists():
                raise RequestError("文件不存在", 404)
            if not stat.S_ISREG(path.stat().st_mode):
                raise RequestError("不是普通文件")
        return path

    def open_capture(self, name, directory=None):
        path = self.path(name, directory=directory)
        fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
        stream = os.fdopen(fd, "rb")
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            stream.close()
            raise RequestError("不是普通文件")
        return stream

    def files(self, directory=None):
        result = []
        directory = self.resolve_directory(directory)
        with os.scandir(directory) as entries:
            for entry in entries:
                if entry.is_file(follow_symlinks=False) and entry.name.endswith(".jsonl"):
                    try:
                        filename(entry.name)
                        info = entry.stat(follow_symlinks=False)
                        result.append({"name": entry.name, "bytes": info.st_size, "modified": info.st_mtime})
                    except (OSError, RequestError):
                        continue
        result.sort(key=lambda row: row["modified"], reverse=True)
        return result[:500], len(result)

    def file_status(self, item):
        if not item:
            return None
        item = dict(item)
        try:
            path = self.path(item["file"], directory=item["directory"])
            item["bytes"] = path.stat().st_size
            item["download_available"] = item["bytes"] <= data.MAX_FILE and not (
                self.job and self.job["state"] in ACTIVE and self.job["path"] == str(path))
        except (OSError, RequestError):
            item.update(bytes=None, download_available=False)
        return item

    def state(self, directory=None):
        with self.lock:
            job = None
            if self.job:
                job = {k: v for k, v in self.job.items() if not k.startswith("_")}
                job["events"] = list(job["events"])
                job["elapsed_s"] = round((self.job.get("ended_monotonic") or time.monotonic())-self.job["_started"], 1)
                job = self.file_status(job)
            file_error = None
            try:
                listing = self.resolve_directory(directory)
                files, total = self.files(str(listing))
            except (OSError, RequestError) as exc:
                listing, files, total, file_error = directory or str(self.directory), [], 0, str(exc)
            return {"job": job, "directory": str(self.directory), "files": files, "file_total": total,
                    "file_directory": str(listing), "file_error": file_error,
                    "storage_roots": [str(p) for p in self.storage_roots],
                    "last_recording": self.file_status(self.last_recording),
                    "native_available": self.native_available, "topics": data.TOPICS,
                    "preflight": self.ticket, "platform": platform.system() + " / " + platform.machine()}

    def _event(self, job, event):
        job["events"].append({"at": time.time(), **event})
        kind = event.get("event")
        if kind == "file_checked":
            job["summary"] = event
        if kind in ("record_progress", "record_finished", "replay_progress", "replay_stopped"):
            job["progress"].update(event)
        if kind in ("quiet_guard", "record_started", "replay_progress", "file_checked"):
            job["phase"] = kind
        if kind == "error":
            job["error"] = str(event.get("error", "未知错误"))[:2000]

    def start(self, body):
        with self.lock:
            if self.closed:
                raise RequestError("服务正在退出", 503)
            if self.job and self.job["state"] in ACTIVE:
                raise RequestError("已有任务运行，请先停止或等待完成", 409)
            action = body.get("action")
            if action not in ("record", "check", "replay"):
                raise RequestError("只支持录制、预检查、DDS 回放")
            if action != "check" and not self.native_available:
                raise RequestError("当前主机无可用 Linux DDS 运行环境；仍可上传、预检查和下载文件")
            name = body.get("file")
            path = self.path(name, existing=action != "record", directory=body.get("directory"))
            args = [sys.executable, "-u", str(ROOT / "tools/dds_data_tool.py")]
            options = None
            if action == "record":
                if path.exists():
                    raise RequestError("文件已存在，请换名字；不会覆盖", 409)
                options = playback_options(body)
                duration = number(body.get("duration", 0), "录制时长（秒）", 0, 604800)
                size = number(body.get("max_mib", 256), "大小上限（MiB）", 1, 512)
                # Fail before creating a DDS participant if the selected directory cannot be written.
                with tempfile.TemporaryFile(dir=path.parent, prefix=".dds-write-check-") as probe:
                    probe.write(b"\0")
                    probe.flush()
                if shutil.disk_usage(path.parent).free < 1024*1024:
                    raise RequestError("所选目录可用空间不足 1 MiB，请选择其他位置")
                args += ["record", str(path), "--topics", ",".join(options["topics"]),
                         "--duration", str(duration), "--max-mib", str(size)]
                self.ticket = None
            else:
                options = playback_options(body)
                args += ["replay", str(path), "--topics", ",".join(options["topics"]),
                         "--time-mode", options["time_mode"], "--speed", str(options["speed"])]
                if options["allow_incomplete"]:
                    args.append("--allow-incomplete")
                if action == "replay":
                    ticket = self.ticket
                    if body.get("inputs_stopped") is not True or body.get("confirm_send") is not True:
                        raise RequestError("需确认其他输入已停止/隔离，并明确确认发送 DDS")
                    if (not ticket or body.get("preflight_id") != ticket["id"] or ticket["path"] != str(path)
                            or ticket["options"] != options or time.time()-ticket["created"] > 1800):
                        raise RequestError("文件或参数改变 / 预检查过期，请重新预检查", 409)
                    args += ["--publish", "--inputs-stopped", "--expected-data-sha256", ticket["sha256"]]
                self.ticket = None  # one-use ticket, including on process failure
            env = dict(os.environ)
            env.pop("SB_DDS_DATA_TOOL_ENV", None)
            env.pop("DDS_WEB_TOKEN", None)
            child = self.popen(args, cwd=str(ROOT), env=env, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                               encoding="utf-8", errors="replace", start_new_session=True)
            job = {"id": secrets.token_hex(12), "action": action, "file": name,
                   "directory": str(path.parent), "path": str(path),
                   "state": "running", "phase": "starting", "started_at": time.time(),
                   "_started": time.monotonic(), "options": options, "events": deque(maxlen=160),
                   "progress": {}, "summary": None, "error": None, "forced_stop": False}
            self.job, self.process = job, child
            threading.Thread(target=self._watch, args=(job, child), daemon=True).start()
            return {"job_id": job["id"]}

    def _watch(self, job, child):
        try:
            while True:
                line = child.stdout.readline(65537)
                if not line:
                    break
                if len(line) > 65536:
                    while line and not line.endswith("\n"):
                        line = child.stdout.readline(65537)
                    event = {"event": "console", "message": "子进程超长日志已截断"}
                else:
                    try:
                        event = json.loads(line)
                        if not isinstance(event, dict) or "event" not in event:
                            raise ValueError()
                    except ValueError:
                        event = {"event": "console", "message": line.strip()[:2000]}
                with self.lock:
                    self._event(job, event)
            code = child.wait()
            with self.lock:
                was_stopping = job["state"] == "stopping"
                job.update(exit_code=code, ended_monotonic=time.monotonic())
                if job["forced_stop"]:
                    job["state"] = "failed"
                    job["error"] = "子进程未及时响应，已强制停止；录制可能不完整，请预检查后显式恢复"
                elif job["error"] or code not in (0, 130, -signal.SIGINT):
                    job["state"] = "failed"
                    job["error"] = job["error"] or f"子进程退出码 {code}，请查看日志"
                else:
                    job["state"] = "stopped" if was_stopping or code != 0 else "completed"
                if job["action"] == "record":
                    self.last_recording = {k: job[k] for k in ("id", "file", "directory", "path", "state")}
                summary = job["summary"]
                if job["action"] == "check" and job["state"] == "completed" and summary and summary["selected_records"] > 0:
                    self.ticket = {"id": job["id"], "file": job["file"], "options": job["options"],
                                   "directory": job["directory"], "path": job["path"],
                                   "sha256": summary["data_sha256"], "created": time.time(), "summary": summary}
        except Exception as exc:
            with self.lock:
                job.update(state="failed", error=f"任务监控异常：{exc}", ended_monotonic=time.monotonic())
                self.ticket = None
            if child.poll() is None:
                child.kill()
                child.wait()
        finally:
            child.stdout.close()

    def stop(self, job_id):
        with self.lock:
            if not self.job or job_id != self.job["id"]:
                raise RequestError("任务已变化，请刷新后再停止", 409)
            if self.job["state"] not in ACTIVE:
                return {"stopping": False}
            if self.job["state"] == "stopping":
                return {"stopping": True}
            self.job["state"] = "stopping"
            child, job = self.process, self.job
            # Popen checks its own process; never pgrep, pkill, killall or fusion control.
            child.send_signal(signal.SIGINT)
            threading.Thread(target=self._stop_timeout, args=(job, child), daemon=True).start()
            return {"stopping": True}

    def _stop_timeout(self, job, child):
        try:
            child.wait(timeout=8)
        except subprocess.TimeoutExpired:
            with self.lock:
                if child.poll() is None:
                    job["forced_stop"] = True
                    child.kill()

    def close(self):
        with self.lock:
            self.closed = True
            job, child = self.job, self.process
        if job and job["state"] in ACTIVE:
            self.stop(job["id"])
            try:
                child.wait(timeout=10)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait(timeout=2)


class Server(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, controller, access_key):
        self.controller, self.access_key = controller, access_key
        self.session_key = secrets.token_urlsafe(32)
        self.login_failures = deque(maxlen=1000)
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server_version = "DDS-Data-Workbench/1"

    def setup(self):
        super().setup()
        self.connection.settimeout(60)

    def log_message(self, *_):
        pass  # Do not leak cookie, access key, user filenames or query strings.

    def send(self, status, content=b"", mime="application/json; charset=utf-8", extra=()):
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(content)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'")
        for key, value in extra:
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(content)

    def json(self, value, status=200, extra=()):
        self.send(status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode(), extra=extra)

    def origin(self):
        origin = urlsplit(self.headers.get("Origin", ""))
        if origin.scheme not in ("http", "https") or origin.netloc != self.headers.get("Host"):
            raise RequestError("拒绝跨站操作，请在本服务页面操作", 403)

    def auth(self):
        cookie = SimpleCookie()
        try:
            cookie.load(self.headers.get("Cookie", ""))
            value = cookie["dds_ui_session"].value if "dds_ui_session" in cookie else ""
        except Exception:
            value = ""
        if not secrets.compare_digest(value, self.server.session_key):
            raise RequestError("请使用启动终端显示的访问口令登录", 401)

    def body_size(self, maximum):
        if self.headers.get("Transfer-Encoding"):
            raise RequestError("不支持分块上传", 400)
        try:
            size = int(self.headers.get("Content-Length", "-1"))
        except ValueError:
            raise RequestError("无效 Content-Length")
        if not 0 <= size <= maximum:
            raise RequestError(f"请求大小必须在 0–{maximum} 字节之间", 413)
        return size

    def read_json(self):
        if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            raise RequestError("需要 application/json", 415)
        raw = self.rfile.read(self.body_size(16384))
        body = json.loads(raw)
        if not isinstance(body, dict):
            raise RequestError("请求必须是 JSON 对象")
        return body

    def do_GET(self):
        self.dispatch("GET")

    def do_POST(self):
        self.dispatch("POST")

    def dispatch(self, method):
        try:
            self.route(method)
        except RequestError as exc:
            self.json({"error": str(exc)}, exc.status)
        except (ValueError, TypeError, OSError) as exc:
            self.json({"error": str(exc)[:1500]}, 400)

    def route(self, method):
        parsed = urlsplit(self.path)
        path = parsed.path
        ctl = self.server.controller
        if method == "GET" and path in ("/", "/app.js", "/style.css"):
            asset = {"/": ("index.html", "text/html; charset=utf-8"),
                     "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                     "/style.css": ("style.css", "text/css; charset=utf-8")}[path]
            return self.send(200, (STATIC/asset[0]).read_bytes(), asset[1])
        if method == "POST":
            self.origin()
        if method == "POST" and path == "/api/login":
            body = self.read_json()
            ip, now = self.client_address[0], time.monotonic()
            if sum(1 for peer, when in self.server.login_failures if peer == ip and now-when < 60) >= 5:
                raise RequestError("口令错误过多，请 60 秒后再试", 429)
            value = body.get("key", "")
            if not isinstance(value, str) or not secrets.compare_digest(value.encode(), self.server.access_key.encode()):
                self.server.login_failures.append((ip, now))
                raise RequestError("访问口令不正确，请查看启动终端", 401)
            return self.json({"ok": True}, extra=[("Set-Cookie", "dds_ui_session="+self.server.session_key+"; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800")])
        self.auth()
        if method == "GET" and path == "/api/state":
            return self.json(ctl.state(parse_qs(parsed.query).get("directory", [None])[0]))
        if method == "GET" and path == "/api/browse":
            return self.json(ctl.browse(parse_qs(parsed.query).get("directory", [None])[0]))
        if method == "POST" and path == "/api/mkdir":
            body = self.read_json()
            return self.json(ctl.mkdir(body.get("directory"), body.get("name")), 201)
        if method == "POST" and path == "/api/logout":
            self.read_json()
            return self.json({"ok": True}, extra=[("Set-Cookie", "dds_ui_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0")])
        if method == "POST" and path == "/api/jobs":
            return self.json(ctl.start(self.read_json()), 202)
        if method == "POST" and path == "/api/stop":
            return self.json(ctl.stop(self.read_json().get("job_id")))
        if method == "GET" and path == "/api/download":
            name = parse_qs(parsed.query).get("name", [""])[0]
            directory = parse_qs(parsed.query).get("directory", [None])[0]
            with ctl.lock:
                requested = ctl.path(name, directory=directory)
                if ctl.job and ctl.job["state"] in ACTIVE and ctl.job["path"] == str(requested):
                    raise RequestError("请等待当前文件任务结束后再下载", 409)
                stream = ctl.open_capture(name, directory)
            with stream:
                size = os.fstat(stream.fileno()).st_size
                if size > data.MAX_FILE:
                    raise RequestError("文件超过 512 MiB 上限", 413)
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(size))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Content-Type-Options", "nosniff")
                self.send_header("Content-Disposition", "attachment; filename*=UTF-8''" + quote(name))
                self.end_headers()
                shutil.copyfileobj(stream, self.wfile, 256*1024)
            return
        if method == "POST" and path == "/api/upload":
            name = parse_qs(parsed.query).get("name", [""])[0]
            directory = parse_qs(parsed.query).get("directory", [None])[0]
            destination = ctl.path(name, existing=False, directory=directory)
            size = self.body_size(data.MAX_FILE)
            if size == 0:
                raise RequestError("不能上传空文件")
            if destination.exists():
                raise RequestError("同名文件已存在，请重命名后上传", 409)
            if not ctl.upload_lock.acquire(blocking=False):
                raise RequestError("已有上传正在处理", 409)
            try:
                with tempfile.NamedTemporaryFile(dir=destination.parent, prefix=".upload-", suffix=".tmp") as uploaded:
                    remaining = size
                    while remaining:
                        chunk = self.rfile.read(min(256*1024, remaining))
                        if not chunk:
                            raise RequestError("上传连接提前中断")
                        uploaded.write(chunk)
                        remaining -= len(chunk)
                    uploaded.flush()
                    os.fsync(uploaded.fileno())
                    # Incomplete captures may be saved, but never replayed without explicit recovery.
                    with tempfile.TemporaryFile("w+b") as spool:
                        summary = data.prepare(uploaded.name, spool, allow_incomplete=True)
                    os.link(uploaded.name, destination)  # atomic no-overwrite, even under racing clients
                return self.json({"file": name, "directory": str(destination.parent), "summary": summary}, 201)
            finally:
                ctl.upload_lock.release()
        raise RequestError("接口不存在", 404)


def main(argv=None):
    parser = argparse.ArgumentParser(description="独立 DDS 录制/回放网页；不启动融合、不安装依赖")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8010)
    parser.add_argument("--recordings-dir", type=Path, default=ROOT/"recordings")
    parser.add_argument("--storage-root", type=Path, action="append", default=[],
                        help="额外允许浏览/保存的数据目录，可重复；默认开放录制目录与当前用户主目录（不含隐藏目录）")
    args = parser.parse_args(argv)
    key = os.environ.get("DDS_WEB_TOKEN") or secrets.token_urlsafe(24)
    if len(key) < 24:
        parser.error("DDS_WEB_TOKEN 至少需要 24 个字符")
    if not 1 <= args.port <= 65535:
        parser.error("端口范围为 1–65535")
    try:
        controller = Controller(args.recordings_dir, storage_roots=[Path.home(), *args.storage_root])
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    # Independent servers sharing the same folder must not start concurrent DDS jobs.
    lock_path = controller.directory/".dds-web.lock"
    with os.fdopen(os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600), "a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("已有录制界面使用该目录；请使用已启动的界面。", file=sys.stderr)
            return 1
        try:
            server = Server((args.host, args.port), controller, key)
        except OSError as exc:
            print(f"界面启动失败：{exc}。可用 --port 其他端口；不会停止占用端口的程序。", file=sys.stderr)
            return 1
        def stop_server(*_):
            raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM, stop_server)
        print(f"\n独立 DDS 录制 / 回放界面\n地址：http://<本机IP>:{args.port}/（绑定 {args.host}）\n"
              f"访问口令：{key}\n保存目录：{controller.directory}\n"
              f"可选目录范围：{', '.join(str(p) for p in controller.storage_roots)}\n"
              "只在可信调试局域网使用。HTTP 不加密，请勿暴露公网。\n"
              "网页关闭不会停止任务；Ctrl+C 退出服务会停止本工具自己的任务，不操作融合。\n", flush=True)
        try:
            server.serve_forever(poll_interval=.2)
        except KeyboardInterrupt:
            pass
        finally:
            controller.close()
            server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
