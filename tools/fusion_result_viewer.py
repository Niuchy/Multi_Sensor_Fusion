#!/usr/bin/env python3
"""订阅并解析融合结果 DDS 数据（FusionMultiTarget），打印或网页实时显示。

用法（在项目根目录或任意目录运行均可）:
  .rk3588-env/bin/python tools/fusion_result_viewer.py              # 控制台持续打印
  .rk3588-env/bin/python tools/fusion_result_viewer.py --once       # 抓一帧解析后退出
  .rk3588-env/bin/python tools/fusion_result_viewer.py --web 8088   # 同时开网页实时显示
  .rk3588-env/bin/python tools/fusion_result_viewer.py --topic RadarTarget   # 其他主题(仅十六进制)
  .rk3588-env/bin/python tools/fusion_result_viewer.py --internal   # 同时显示内部 JSON 主题

报文布局（公司 TopicAPI/ByteSequence，小端）:
  MsgHeader 26B: 7E | unitNums | src(2) | dest(2) | msgLength(4) | msgTag(2) |
                 msgID(2) | checkID(2) | bak(2) | timestamp(8)
  UnitHeader 14B: unitLength(4)=body大小 | unitTag(2)=0x0001 | timestamp(8)
  旧版桥接会在 UnitHeader 后再追加 8B 时间戳（生产版: 头共 48B，空目标包 49B）;
  新版(41B)不追加。本工具按 unitLength 一致性自动识别两种布局。
  body: target_count(1B) + N × FusionTarget(28B 固定 + 预测点×30B)
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

# ---------------------------------------------------------------------------
# 与 run_sb_fusion.py 相同的桥接库加载方式：使用根目录 libcompany_dds_bridge.so
# ---------------------------------------------------------------------------

ROOT = Path(__file__).resolve().parent.parent
DDS_DIR = ROOT / "dds_swarm_fusion"
sys.path.insert(0, str(DDS_DIR))

import company_dds  # noqa: E402
from company_dds import CompanyDdsBus, Topics  # noqa: E402

company_dds._bridge_path = lambda: ROOT / "libcompany_dds_bridge.so"

# ---------------------------------------------------------------------------
# fusion_proto.h 报文解析（按协议字段原样小端解析）
# ---------------------------------------------------------------------------

# 候选 body 偏移: 头长度 -> 布局说明。用 MsgHeader 里的 unitLength(4B@26)
# 与包总长互相校验来自动判定: unitLength == 总长 - 头长。
HEADER_CANDIDATES = [
    (48, "UnitHeader14+追加时间戳(生产旧版)"),
    (40, "UnitHeader14(修复版)"),
    (40, "UnitHeader6+追加时间戳"),
    (32, "UnitHeader6"),
]

TARGET_TYPES = {
    0: "无人机", 1: "鸟", 2: "大型目标", 3: "车辆", 4: "人员", 5: "其它", 0xFF: "无效",
}

SOURCE_NAMES = {
    0: "雷达", 1: "光电", 2: "电侦", 3: "声探", 4: "雷达+光电", 5: "雷达+电侦",
    6: "雷达+声探", 7: "光电+电侦", 8: "光电+声探", 9: "电侦+声探",
    10: "雷达+光电+电侦", 11: "雷达+光电+声探", 12: "光电+电侦+声探",
    13: "雷达+光电+电侦+声探", 0xFF: "无效",
}

IFF_NAMES = {0: "不明", 1: "我方", 2: "敌方"}

UNIT_TAGS = {
    0x0001: "FUSION_MULTI_TARGET(融合多目标)",
    0x0002: "SITUATION(态势推理)",
    0x0003: "WEAPON_DISTRIBUTION(武器分配)",
    0x0004: "STRIKE_RESULT(反制结果)",
    0x0005: "WEAPON_STATUS(武器状态)",
}


def _u8(b: bytes, o: int) -> int:
    return b[o]


def _i16(b: bytes, o: int) -> int:
    return int.from_bytes(b[o:o + 2], "little", signed=True)


def _u16(b: bytes, o: int) -> int:
    return int.from_bytes(b[o:o + 2], "little")


def _i32(b: bytes, o: int) -> int:
    return int.from_bytes(b[o:o + 4], "little", signed=True)


def _u32(b: bytes, o: int) -> int:
    return int.from_bytes(b[o:o + 4], "little")


def _u64(b: bytes, o: int) -> int:
    return int.from_bytes(b[o:o + 8], "little")


def _ms_to_str(ms: int) -> str:
    if not ms:
        return "0(未设置)"
    try:
        return datetime.fromtimestamp(ms / 1000.0).strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    except (OverflowError, OSError, ValueError):
        return str(ms)


def _deg(v: int) -> float:
    return v / 1e7


def _m(v: int) -> float:
    return v / 10.0


def _deg2(v: int) -> float:
    return v / 100.0


def _mps(v: int) -> float:
    return v / 100.0


def _parse_prediction_point(b: bytes, o: int):
    return {
        "lng_e7": _i32(b, o), "lat_e7": _i32(b, o + 4), "alt_dm": _u32(b, o + 8),
        "pitch_cdeg": _i16(b, o + 12), "roll_cdeg": _i16(b, o + 14), "yaw_cdeg": _u16(b, o + 16),
        "speed_cms": _u32(b, o + 18), "time_ms": _u64(b, o + 22),
    }


def _parse_fusion_target(b: bytes, o: int):
    """解析单个 FusionTarget（28B 固定 + 预测点）。"""
    t = {
        "global_id": _u8(b, o),
        "lng_e7": _i32(b, o + 1), "lat_e7": _i32(b, o + 5), "alt_dm": _u32(b, o + 9),
        "pitch_cdeg": _i16(b, o + 13), "roll_cdeg": _i16(b, o + 15), "yaw_cdeg": _u16(b, o + 17),
        "speed_cms": _u32(b, o + 19),
        "fusion_type": _u8(b, o + 23),
        "confidence": _u8(b, o + 24),
        "source_mask": _u8(b, o + 25),
        "iff": _u8(b, o + 26),
        "pred_point_count": _u8(b, o + 27),
    }
    o += 28
    points = []
    for _ in range(t["pred_point_count"]):
        if o + 30 > len(b):
            return None, "预测点数据不足"
        points.append(_parse_prediction_point(b, o))
        o += 30
    t["prediction_points"] = points
    return t, None


def parse_fusion_frame(data: bytes) -> dict:
    """解析 FusionMultiTarget 完整报文（MsgHeader + UnitHeader + body）。"""
    n = len(data)
    frame = {
        "received_at_ms": int(time.time() * 1000),
        "length": n,
        "hex": data.hex(" "),
    }
    if n < 32:
        frame["error"] = f"报文过短({n}B)，至少需要 32B 头部"
        return frame

    frame.update({
        "start_tag": _u8(data, 0),
        "unit_nums": _u8(data, 1),
        "src": _u16(data, 2),
        "dest": _u16(data, 4),
        "msg_length": _u32(data, 6),
        "msg_tag": _u16(data, 10),
        "msg_id": _u16(data, 12),
        "check_id": _u16(data, 14),
        "reserved": _u16(data, 16),
        "stamp_ms": _u64(data, 18),
        "unit_length": _u32(data, 26),
        "unit_tag": _u16(data, 30),
    })

    warnings = []
    if frame["start_tag"] != 0x7E:
        warnings.append(f"startTag={frame['start_tag']:#04x}，非 0x7E")
    if frame["unit_nums"] != 1:
        warnings.append(f"unitNums={frame['unit_nums']}，非 1")
    if frame["dest"] != 0xFFFF:
        warnings.append(f"dest=0x{frame['dest']:04X}，非 0xFFFF")
    frame["warnings"] = warnings

    body = None
    for header_len, layout in HEADER_CANDIDATES:
        if header_len + 1 <= n and frame["unit_length"] == n - header_len:
            body = data[header_len:]
            frame["header_len"] = header_len
            frame["layout"] = layout
            break
    if body is None:
        # 头部布局无法识别：尝试把整个报文当作裸 body 解析
        frame["error"] = (
            f"无法按已知头布局解析: unitLength={frame['unit_length']} 与总长 {n}B 不一致，"
            "按裸 body 尝试"
        )
        frame["header_len"] = 0
        frame["layout"] = "未知(按裸body)"
        body = data

    count = body[0]
    frame["target_count"] = count
    targets = []
    offset = 1
    for _ in range(count):
        if offset + 28 > len(body):
            frame.setdefault("errors", []).append(
                f"目标数据不足: 需要 28B 固定部分，剩余 {len(body) - offset}B")
            break
        target, err = _parse_fusion_target(body, offset)
        if err:
            frame.setdefault("errors", []).append(err)
            break
        # 单目标总长 = 28 + 预测点×30，用解析结果回推进位
        offset += 28 + target["pred_point_count"] * 30
        targets.append(target)
    if offset != len(body):
        frame.setdefault("errors", []).append(
            f"body 尾部多出 {len(body) - offset}B: {body[offset:].hex(' ')}")
    frame["targets"] = targets
    return frame


# ---------------------------------------------------------------------------
# 控制台打印
# ---------------------------------------------------------------------------

def _fmt_pos(t: dict) -> str:
    return (f"lng={_deg(t['lng_e7']):.7f}° lat={_deg(t['lat_e7']):.7f}° "
            f"alt={_m(t['alt_dm']):.1f}m")


def _fmt_att(t: dict) -> str:
    return (f"yaw={_deg2(t['yaw_cdeg']):.2f}° pitch={_deg2(t['pitch_cdeg']):.2f}° "
            f"roll={_deg2(t['roll_cdeg']):.2f}°")


def _enum_name(mapping: dict, value: int) -> str:
    return mapping.get(value, "0x%02X" % value)


def print_frame(frame: dict, topic: str, *, show_hex: bool) -> None:
    stamp = _ms_to_str(frame.get("stamp_ms", 0))
    err = frame.get("error")
    if err:
        print(f"[错误] {err}")
        print(f"  原始数据 {frame['length']}B: {frame['hex']}")
        return
    print(f"\n===== {topic} #{frame.get('msg_id', '?')} "
          f"接收于 {_ms_to_str(frame['received_at_ms'])} =====")
    print(f"总长 {frame['length']}B  目标数={frame['target_count']}  "
          f"src=0x{frame['src']:04X}({frame['src']})  "
          f"unitTag=0x{frame['unit_tag']:04X}({UNIT_TAGS.get(frame['unit_tag'], '未知')})")
    print(f"MsgHeader: start=0x{frame['start_tag']:02X} units={frame['unit_nums']} "
          f"dest=0x{frame['dest']:04X} msgLength={frame['msg_length']} "
          f"msgTag={frame['msg_tag']} checkID={frame['check_id']} "
          f"发送时刻={stamp}")
    print(f"UnitHeader: unitLength={frame['unit_length']}  布局={frame['layout']}")
    for w in frame.get("warnings", []):
        print(f"  [警告] {w}")
    for e in frame.get("errors", []):
        print(f"  [解析警告] {e}")
    for i, t in enumerate(frame["targets"], 1):
        print(f"  [{i}/{frame['target_count']}] id={t['global_id']} "
              f"类型={_enum_name(TARGET_TYPES, t['fusion_type'])} "
              f"置信度={t['confidence']}% 来源={_enum_name(SOURCE_NAMES, t['source_mask'])} "
              f"敌我={_enum_name(IFF_NAMES, t['iff'])}")
        print(f"       {_fmt_pos(t)}")
        print(f"       {_fmt_att(t)}  速度={_mps(t['speed_cms']):.2f}m/s  "
              f"预测点={t['pred_point_count']}")
        for j, p in enumerate(t["prediction_points"]):
            print(f"       pred[{j}] {_fmt_pos(p)} 速度={_mps(p['speed_cms']):.2f}m/s "
                  f"时刻={p['time_ms']}ms")
    if show_hex:
        print(f"  raw[{frame['length']}B]: {frame['hex']}")


# ---------------------------------------------------------------------------
# 网页显示
# ---------------------------------------------------------------------------

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>融合结果 DDS 监视器</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font: 14px/1.5 system-ui, sans-serif; background: #10151d; color: #dbe4ee; }
  header { position: sticky; top: 0; background: #182130; border-bottom: 1px solid #2a3a52;
           padding: 10px 18px; display: flex; gap: 22px; align-items: baseline; flex-wrap: wrap; }
  header h1 { font-size: 16px; margin: 0; }
  .stat { color: #8fd3a7; font-variant-numeric: tabular-nums; }
  .stat b { font-size: 18px; }
  main { padding: 14px 18px; max-width: 1100px; margin: 0 auto; }
  .frame { background: #182130; border: 1px solid #2a3a52; border-radius: 8px;
           margin-bottom: 14px; padding: 10px 14px; }
  .frame h2 { font-size: 14px; margin: 0 0 6px; color: #9fc3ea; }
  .kv { color: #93a5bb; font-size: 13px; }
  .warn { color: #e8c07a; } .err { color: #f08a8a; }
  table { border-collapse: collapse; width: 100%; margin-top: 8px; font-size: 13px; }
  th, td { border: 1px solid #2a3a52; padding: 4px 8px; text-align: left; white-space: nowrap; }
  th { background: #1e2b40; color: #a9c2e2; }
  details { margin-top: 8px; } summary { cursor: pointer; color: #93a5bb; }
  pre { background: #0c1118; padding: 8px; border-radius: 6px; overflow-x: auto;
        font-size: 12px; color: #a8c66c; }
  #status { color: #8fd3a7; } #status.off { color: #f08a8a; }
</style>
</head>
<body>
<header>
  <h1>融合结果 DDS 监视器 — FusionMultiTarget</h1>
  <span class="stat">接收 <b id="total">0</b></span>
  <span class="stat">最后一帧 <b id="last">-</b></span>
  <span id="status">等待数据…</span>
</header>
<main id="main"></main>
<script>
const TYPES = {0:"无人机",1:"鸟",2:"大型目标",3:"车辆",4:"人员",5:"其它",255:"无效"};
const SOURCES = {0:"雷达",1:"光电",2:"电侦",3:"声探",4:"雷达+光电",5:"雷达+电侦",
  6:"雷达+声探",7:"光电+电侦",8:"光电+声探",9:"电侦+声探",10:"雷达+光电+电侦",
  11:"雷达+光电+声探",12:"光电+电侦+声探",13:"雷达+光电+电侦+声探",255:"无效"};
const IFF = {0:"不明",1:"我方",2:"敌方"};
const esc = s => String(s).replace(/[&<>"]/g, c => ({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;"}[c]));
const fmtMs = ms => ms ? new Date(ms).toLocaleString("zh-CN", {hour12:false}) : "0(未设置)";

function targetRow(t) {
  const pos = `lng=${(t.lng_e7/1e7).toFixed(7)}° lat=${(t.lat_e7/1e7).toFixed(7)}° alt=${(t.alt_dm/10).toFixed(1)}m`;
  const att = `yaw=${(t.yaw_cdeg/100).toFixed(2)}° pitch=${(t.pitch_cdeg/100).toFixed(2)}° roll=${(t.roll_cdeg/100).toFixed(2)}°`;
  const preds = (t.prediction_points||[]).map((p,i) =>
    `<div class="kv">预测点${i}: lng=${(p.lng_e7/1e7).toFixed(7)}° lat=${(p.lat_e7/1e7).toFixed(7)}° alt=${(p.alt_dm/10).toFixed(1)}m 速度=${(p.speed_cms/100).toFixed(2)}m/s 时刻=${p.time_ms}ms</div>`).join("");
  return `<tr><td>${t.global_id}</td><td>${TYPES[t.fusion_type] ?? "0x"+t.fusion_type.toString(16)}</td>
    <td>${t.confidence}%</td><td>${SOURCES[t.source_mask] ?? "0x"+t.source_mask.toString(16)}</td>
    <td>${IFF[t.iff] ?? "0x"+t.iff.toString(16)}</td><td>${esc(pos)}</td><td>${esc(att)}</td>
    <td>${(t.speed_cms/100).toFixed(2)}m/s</td></tr>${preds ? `<tr><td colspan="8">${preds}</td></tr>` : ""}`;
}

function frameCard(f) {
  if (f.error) return `<div class="frame"><h2>错误</h2><div class="err">${esc(f.error)}</div><pre>${esc(f.hex)}</pre></div>`;
  const warns = (f.warnings||[]).map(w=>`<div class="warn">⚠ ${esc(w)}</div>`).join("");
  const errs = (f.errors||[]).map(e=>`<div class="err">✖ ${esc(e)}</div>`).join("");
  const targets = (f.targets||[]).map(targetRow).join("");
  const empty = f.target_count ? "" : `<div class="kv">本帧无确认目标（空目标包，目标数为 0）</div>`;
  return `<div class="frame">
    <h2>帧 #${f.msg_id} — 接收于 ${fmtMs(f.received_at_ms)}</h2>
    <div class="kv">总长 ${f.length}B | 目标数 ${f.target_count} | src=0x${f.src.toString(16).padStart(4,"0")} | unitTag=0x${f.unit_tag.toString(16).padStart(4,"0")} | 布局 ${esc(f.layout)}</div>
    <div class="kv">MsgHeader: start=0x${f.start_tag.toString(16).padStart(2,"0")} units=${f.unit_nums} dest=0x${f.dest.toString(16).padStart(4,"0")} msgLength=${f.msg_length} 发送时刻=${fmtMs(f.stamp_ms)}</div>
    ${warns}${errs}
    <table><tr><th>ID</th><th>类型</th><th>置信度</th><th>来源</th><th>敌我</th><th>位置</th><th>姿态</th><th>速度</th></tr>${targets}</table>
    ${empty}
    <details><summary>原始字节 (${f.length}B)</summary><pre>${esc(f.hex)}</pre></details>
  </div>`;
}

let since = 0;
async function poll() {
  try {
    const r = await fetch("/api/frames?since=" + since);
    const j = await r.json();
    since = j.now;
    document.getElementById("total").textContent = j.received;
    document.getElementById("last").textContent = j.last ? fmtMs(j.last) : "-";
    document.getElementById("status").textContent = j.received ? "数据接收中" : "等待数据…";
    document.getElementById("status").className = j.received ? "" : "off";
    if (j.frames.length) {
      const main = document.getElementById("main");
      main.innerHTML = j.frames.map(frameCard).join("") + main.innerHTML;
      if (main.children.length > 120) main.replaceChildren(...[...main.children].slice(0, 120));
    }
  } catch (e) { document.getElementById("status").textContent = "连接失败: " + e; }
}
setInterval(poll, 1000);
poll();
</script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# 接收与展示主流程
# ---------------------------------------------------------------------------

class Viewer:
    def __init__(self, topic: str, show_hex: bool, internal: bool, max_frames: int = 200):
        self.topic = topic
        self.show_hex = show_hex
        self.internal = internal
        self.bus = CompanyDdsBus(source="fusion_result_viewer")
        self.frames: deque = deque(maxlen=max_frames)
        self.lock = threading.Lock()
        self.received = 0
        self.started = time.time()

    def handle_sample(self, data: bytes) -> None:
        if self.topic == Topics.OFFICIAL_FUSION.name:
            frame = parse_fusion_frame(data)
            print_frame(frame, self.topic, show_hex=self.show_hex)
        elif self.internal:
            frame = {
                "received_at_ms": int(time.time() * 1000),
                "length": len(data),
                "hex": data.hex(" "),
                "json": None,
            }
            try:
                frame["json"] = json.loads(data.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                pass
            self.print_internal(frame)
        else:
            frame = {
                "received_at_ms": int(time.time() * 1000),
                "length": len(data),
                "hex": data.hex(" "),
                "note": f"{self.topic} 不是融合协议主题，仅显示十六进制",
            }
            print(f"\n[{_ms_to_str(frame['received_at_ms'])}] {self.topic} "
                  f"收到 {len(data)}B:\n  {data.hex(' ')}")
        with self.lock:
            self.frames.appendleft(frame)
            self.received += 1

    def print_internal(self, frame: dict) -> None:
        payload = frame["json"]
        if payload:
            summary = json.dumps(payload, ensure_ascii=False)
            if len(summary) > 900:
                summary = summary[:900] + "…"
            print(f"\n[{_ms_to_str(frame['received_at_ms'])}] {self.topic} "
                  f"{len(payload) if isinstance(payload, (list, dict)) else 1} 条 JSON:\n  {summary}")
        else:
            print(f"\n[{_ms_to_str(frame['received_at_ms'])}] {self.topic} "
                  f"{frame['length']}B 非 JSON:\n  {frame['hex']}")

    def api_frames(self) -> list:
        with self.lock:
            return list(self.frames)

    def run(self, once: bool, web_port: int | None) -> int:
        if web_port:
            server = ThreadingHTTPServer(("0.0.0.0", web_port), make_handler(self))
            threading.Thread(target=server.serve_forever, daemon=True).start()
            print(f"[网页] http://<本机IP>:{web_port}/  （本机查看 http://127.0.0.1:{web_port}/）",
                  flush=True)

        # 内部 JSON 主题（融合节点的 FusedTracks / FusedFrame）
        internal_topics = [Topics.FUSED_TRACKS.name, Topics.FUSED_FRAME.name] if self.internal else []
        topics = [self.topic] + internal_topics

        print(f"[监视] 订阅 {topics}（Ctrl-C 停止）", flush=True)
        got = 0
        last_status = time.time()
        try:
            while True:
                any_data = False
                for name in topics:
                    for sample in self.bus.read_bytes(name, timeout_ms=400):
                        self.handle_sample(sample)
                        any_data = True
                        got += 1
                if once and got:
                    return 0
                if not any_data and time.time() - last_status >= 10:
                    print(f"[等待] 尚未收到 DDS 数据（已等待 {int(time.time() - self.started)}s，"
                          f"共收到 {self.received} 帧）…", flush=True)
                    last_status = time.time()
        except KeyboardInterrupt:
            print("\n[退出] 停止接收", flush=True)
            return 0


def make_handler(viewer: Viewer):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):  # 安静模式
            return

        def do_GET(self):
            if self.path.startswith("/api/frames"):
                frames = viewer.api_frames()
                body = json.dumps({
                    "received": viewer.received,
                    "last": frames[0]["received_at_ms"] if frames else None,
                    "frames": frames[:50],
                    "now": int(time.time() * 1000),
                }, ensure_ascii=False).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                body = HTML_PAGE.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

    return Handler


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="接收并显示融合结果 DDS 数据")
    parser.add_argument("--topic", default=Topics.OFFICIAL_FUSION.name,
                        help="DDS 主题名（默认 FusionMultiTarget）")
    parser.add_argument("--once", action="store_true", help="收到一帧后退出")
    parser.add_argument("--web", type=int, metavar="PORT", default=None,
                        help="同时启动网页实时显示（端口）")
    parser.add_argument("--hex", action="store_true", dest="show_hex",
                        help="控制台同时打印原始十六进制")
    parser.add_argument("--internal", action="store_true",
                        help="同时订阅内部 SwarmFusion.FusedTracks/FusedFrame JSON 主题")
    args = parser.parse_args(argv)

    viewer = Viewer(args.topic, args.show_hex, args.internal)
    return viewer.run(args.once, args.web)


if __name__ == "__main__":
    raise SystemExit(main())
