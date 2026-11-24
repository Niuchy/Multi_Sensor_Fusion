#!/usr/bin/env python3
"""Standalone, stdlib-only DDS ByteSequence recorder/replayer.

No imports of fusion nodes, gateway, or orchestration code. Real DDS uses only
the existing native bridge. Inspection/dry-run never loads that library.
"""
from __future__ import annotations

import argparse
import base64
from collections import Counter, OrderedDict
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import signal
import struct
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
FORMAT = "sb-dds-capture-v1"
TOPICS = ("RadarTarget", "ESMTarget", "EOTarget", "AcousticTarget")
MAX_PACKET = 4 * 1024 * 1024
MAX_LINE = 6 * 1024 * 1024
MAX_FILE = 512 * 1024 * 1024
PXSB = "<BBHHIHHHHQ"
HEADER_SIZE = struct.calcsize(PXSB)
PROTOCOL = {"RadarTarget": (0x1A01, 0xA55AA55A, 66),
            "ESMTarget": (0x1B01, 0xB44BB44B, 139),
            "EOTarget": (0x1C01, 0xC33CC33C, 79),
            "AcousticTarget": (0x1D01, 0xD22DD22D, 67)}


def emit(event, **values):
    print(json.dumps({"event": event, **values}, ensure_ascii=False, allow_nan=False), flush=True)


def integer(value, name, minimum=0, maximum=(1 << 64)-1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(f"{name} 必须是 {minimum}–{maximum} 的整数")
    return value


def line_bytes(value):
    return (json.dumps(value, ensure_ascii=False, separators=(",", ":"), allow_nan=False)+"\n").encode()


def select_topics(value):
    topics = tuple(value.split(",")) if isinstance(value, str) else tuple(value)
    if not topics or any(t not in TOPICS for t in topics) or len(set(topics)) != len(topics):
        raise ValueError("只支持四类设备输入 Topic（英文逗号分隔且不可重复）："+", ".join(TOPICS))
    return topics


class CaptureWriter:
    """Exclusive-create, incremental flush, periodic fsync, recoverable JSONL."""
    def __init__(self, path, topics, max_bytes, source_info=None):
        self.path = Path(path)
        self.topics = select_topics(topics)
        self.max_bytes = max_bytes
        self.stream = self.path.open("xb")  # never overwrite an existing recording
        self.digest = hashlib.sha256()
        self.counts = Counter()
        self.sequence = self.size = self.last_offset = 0
        self.last_sync = time.monotonic()
        try:
            self._write({"type": "header", "format": FORMAT, "session_id": str(uuid.uuid4()),
                "created_at_ns": time.time_ns(), "topics": self.topics,
                "source": source_info or {"kind": "native_dds_reader"},
                "timing": "recorder monotonic dequeue intervals; not NIC arrival timestamps",
                "payload": "DDS ByteSequence application bytes; not RTPS/UDP wire capture"})
        except BaseException:
            self.stream.close()
            raise

    def _write(self, value, hashed=True):
        line = line_bytes(value)
        self.stream.write(line)
        self.stream.flush()
        self.size += len(line)
        if hashed:
            self.digest.update(line)

    def add(self, topic, packet, offset_ns, received_at_ns):
        if topic not in self.topics or not 0 < len(packet) <= MAX_PACKET:
            raise ValueError("收到非法 Topic 或报文长度，录制停止，不静默截断")
        integer(offset_ns, "offset_ns")
        integer(received_at_ns, "received_at_ns")
        if offset_ns < self.last_offset:
            raise ValueError("录制单调时钟倒退")
        row = {"type": "sample", "seq": self.sequence+1, "topic": topic,
               "offset_ns": offset_ns, "received_at_ns": received_at_ns,
               "length": len(packet), "sha256": hashlib.sha256(packet).hexdigest(),
               "payload_base64": base64.b64encode(packet).decode("ascii")}
        if self.size + len(line_bytes(row)) + 2048 > self.max_bytes:
            return False
        self._write(row)
        self.sequence += 1
        self.counts[topic] += 1
        self.last_offset = offset_ns
        if time.monotonic()-self.last_sync >= 1:
            os.fsync(self.stream.fileno())
            self.last_sync = time.monotonic()
        return True

    def finish(self, reason):
        if self.stream.closed:
            return
        try:
            self._write({"type": "footer", "records": self.sequence, "counts": dict(self.counts),
                         "reason": reason, "data_sha256": self.digest.hexdigest()}, hashed=False)
            os.fsync(self.stream.fileno())
        finally:
            self.stream.close()


def decode_sample(row, topics, sequence, last_offset):
    if not isinstance(row, dict) or row.get("type") != "sample":
        raise ValueError("录制文件包含未知记录类型")
    if integer(row.get("seq"), "seq", 1) != sequence:
        raise ValueError("录制序号不连续")
    if row.get("topic") not in topics:
        raise ValueError("报文 Topic 未在文件头声明")
    offset = integer(row.get("offset_ns"), "offset_ns")
    integer(row.get("received_at_ns"), "received_at_ns")
    if offset < last_offset:
        raise ValueError("录制文件的接收顺序倒退")
    length = integer(row.get("length"), "length", 1, MAX_PACKET)
    payload = row.get("payload_base64")
    if not isinstance(payload, str) or len(payload) > (MAX_PACKET+2)//3*4:
        raise ValueError("无效 Base64 报文")
    packet = base64.b64decode(payload, validate=True)
    if len(packet) != length or hashlib.sha256(packet).hexdigest() != row.get("sha256"):
        raise ValueError("报文长度 / SHA-256 校验失败")
    return packet, offset


def timestamp_fields(topic, packet):
    """Strict PXSB/dev_proto v1 walker; unknown/checksummed formats fail closed.

    Only transport/header time fields are relocated. Business body bytes (including
    ESM discovery time) stay unchanged. Zero/missing time fields stay unchanged.
    """
    if len(packet) < HEADER_SIZE:
        raise ValueError("重定位需要完整 PXSB 报文")
    header = struct.unpack_from(PXSB, packet)
    tag, magic, frame_size = PROTOCOL[topic]
    if header[0] != 0x7E or not header[1] or header[4] != len(packet)-HEADER_SIZE:
        raise ValueError("PXSB 标志 / 单元数 / 长度不符，不能安全重定位")
    fields = [(18, header[-1], "pxsb")]
    offset = HEADER_SIZE
    for _ in range(header[1]):
        if offset+6 > len(packet):
            raise ValueError("截断的 UnitHeader")
        length, unit_tag = struct.unpack_from("<IH", packet, offset)
        if unit_tag != tag or length != frame_size:
            raise ValueError("Topic / 单元类型 / v1 帧长度不符")
        base = offset+6
        candidates = [pos for pos in (base+8, base) if pos+length <= len(packet)
                      and struct.unpack_from("<I", packet, pos)[0] == magic]
        if len(candidates) != 1:
            raise ValueError("无法唯一识别 6 / 14 字节单元头")
        start = candidates[0]
        frame = struct.unpack_from("<IBHHHIQ", packet, start)
        if frame[1] != 1 or frame[2] != tag or frame[4] != length:
            raise ValueError("不支持的设备帧版本 / 类型 / 长度")
        if struct.unpack_from("<I", packet, start+length-4)[0] != 0:
            raise ValueError("设备帧 CRC 非零；未知校验算法，拒绝改时间，请用 original 模式")
        if start == base+8:
            fields.append((base, struct.unpack_from("<Q", packet, base)[0], "unit"))
        fields.append((start+15, frame[-1], "frame"))
        offset = start+length
    if offset != len(packet):
        raise ValueError("PXSB 存在未知尾部，拒绝改时间")
    return fields


def relocate(topic, packet, shift_ms):
    result = bytearray(packet)
    for offset, stamp, _ in timestamp_fields(topic, packet):
        if stamp:  # 0 is a sentinel, not a valid measurement epoch
            integer(stamp+shift_ms, "重定位时间", 1)
            struct.pack_into("<Q", result, offset, stamp+shift_ms)
    return bytes(result)


def prepare(source, spool, topics=None, time_mode="original", allow_incomplete=False):
    """Validate the entire file BEFORE opening DDS; freeze selected data privately.

    Spool is a TemporaryFile, so subsequent edits to the user's source cannot
    change the published packets and recording length does not set RAM usage.
    """
    selected = set(select_topics(topics)) if topics else set(TOPICS)
    counts, kept = Counter(), Counter()
    sequence = last_offset = 0
    first_kept = last_kept = None
    min_frame = min_stamp = max_stamp = None
    digest = hashlib.sha256()
    complete = False
    completion_reason = None
    ignored_tail = False
    total_size = 0
    with Path(source).open("rb") as stream:
        if os.fstat(stream.fileno()).st_size > MAX_FILE:
            raise ValueError("录制文件超过 512 MiB 上限，请分段录制")
        raw = stream.readline(MAX_LINE+1)
        total_size += len(raw)
        if len(raw) > MAX_LINE or not raw.endswith(b"\n"):
            raise ValueError("录制文件头截断 / 超长")
        header = json.loads(raw)
        if not isinstance(header, dict) or header.get("format") != FORMAT or header.get("type") != "header":
            raise ValueError("不是独立录制文件；网页采集 JSON / 1.log 不能冒充本格式")
        declared = select_topics(header.get("topics", []))
        digest.update(raw)
        while True:
            raw = stream.readline(MAX_LINE+1)
            if not raw:
                break
            total_size += len(raw)
            if total_size > MAX_FILE or len(raw) > MAX_LINE:
                raise ValueError("文件 / 单行超过安全大小上限")
            if not raw.endswith(b"\n"):
                if allow_incomplete:
                    ignored_tail = True
                    break
                raise ValueError("末行截断；可先 inspect --allow-incomplete 检查可恢复部分")
            row = json.loads(raw)
            if not isinstance(row, dict):
                raise ValueError("JSONL 每行必须是对象")
            if row.get("type") == "footer":
                if (row.get("data_sha256") != digest.hexdigest() or row.get("records") != sequence
                        or row.get("counts") != dict(counts)):
                    raise ValueError("文件尾计数 / 整体 SHA-256 不符")
                if stream.read(1):
                    raise ValueError("文件尾之后仍有内容")
                complete = True
                completion_reason = row.get("reason")
                break
            packet, last_offset = decode_sample(row, declared, sequence+1, last_offset)
            sequence += 1
            counts[row["topic"]] += 1
            digest.update(raw)
            if row["topic"] not in selected:
                continue
            kept[row["topic"]] += 1
            if first_kept is None:
                first_kept = last_offset
            last_kept = last_offset
            if time_mode == "rebase":
                fields = timestamp_fields(row["topic"], packet)
                if not any(stamp for _, stamp, _ in fields):
                    raise ValueError("存在所有时间字段均为零的报文，不能猜测其测量时间")
                for _, stamp, kind in fields:
                    if stamp:
                        min_stamp = stamp if min_stamp is None else min(min_stamp, stamp)
                        max_stamp = stamp if max_stamp is None else max(max_stamp, stamp)
                        if kind == "frame":
                            min_frame = stamp if min_frame is None else min(min_frame, stamp)
            spool.write(line_bytes(row))
    if not complete and not allow_incomplete:
        raise ValueError("文件没有完成标记；先用 --allow-incomplete 检查，不能默认当作完整录制")
    if time_mode == "rebase":
        if min_frame is None:
            raise ValueError("所选报文没有非零设备帧时间，不能猜测重定位基准")
        # Catch seconds vs milliseconds, relative vs epoch, or unrelated clocks.
        duration_ms = (last_kept-first_kept)//1_000_000
        if min_stamp < min_frame-86400000 or max_stamp > min_frame+duration_ms+86400000:
            raise ValueError("报文时间字段疑似不同时间基准（差超过一天），拒绝自动重定位")
    spool.seek(0)
    return {"format": FORMAT, "file_records": sequence, "file_counts": dict(counts),
            "selected_records": sum(kept.values()), "selected_counts": dict(kept),
            "duration_s": (last_kept-first_kept)/1e9 if first_kept is not None else 0,
            "first_offset_ns": first_kept, "time_anchor_ms": min_frame,
            "complete": complete, "ignored_truncated_tail": ignored_tail,
            "completion_reason": completion_reason, "source": header.get("source"),
            "data_sha256": digest.hexdigest(), "file_bytes": total_size}


def runtime_environment(environ=None, system=None, machine=None):
    env = dict(os.environ if environ is None else environ)
    system, machine = system or platform.system(), machine or platform.machine()
    if system != "Linux":
        raise RuntimeError("录制 / 真正发布 DDS 仅支持 Linux；Mac 可查看文件或 dry-run")
    name = {"aarch64": "linux-arm64", "arm64": "linux-arm64", "x86_64": "linux-x86", "amd64": "linux-x86"}.get(machine.lower())
    if name is None:
        raise RuntimeError("不支持的 CPU 架构："+machine)
    lib = ROOT / "zy-anti-drone/thirdparty" / name / "lib"
    dirs = [str(lib)]
    if (ROOT/".rk3588-env/lib").is_dir():
        dirs.append(str(ROOT/".rk3588-env/lib"))
    env["LD_LIBRARY_PATH"] = ":".join(dirs+[x for x in env.get("LD_LIBRARY_PATH", "").split(":") if x and x not in dirs])
    crypto, ssl = lib/"libcrypto.so.3", lib/"libssl.so.3"
    if name == "linux-arm64" and crypto.is_file() and ssl.is_file():
        pair = [str(crypto), str(ssl)]
        env["LD_PRELOAD"] = ":".join(pair+[x for x in env.get("LD_PRELOAD", "").split(":") if x and x not in pair])
        env["OPENSSL_MODULES"] = str(lib/"ossl-modules")
    env["SB_DDS_DATA_TOOL_ENV"] = "1"
    return env


def ensure_runtime(argv):
    if os.environ.get("SB_DDS_DATA_TOOL_ENV") != "1":
        env = runtime_environment()
        os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()), *argv], env)


class NativeBus:
    """Small independent ctypes adapter: no dependency on fusion Python modules."""
    def __init__(self, bridge):
        if platform.system() != "Linux":
            raise RuntimeError("真实 DDS 只支持 Linux")
        self.lib = ctypes.CDLL(str(Path(bridge).resolve()), mode=ctypes.RTLD_GLOBAL)
        u8p = ctypes.POINTER(ctypes.c_uint8)
        signatures = {
            "company_dds_init": ([], ctypes.c_int),
            "company_dds_create_reader": ([ctypes.c_char_p, ctypes.c_int], ctypes.c_int),
            "company_dds_create_writer": ([ctypes.c_char_p, ctypes.c_int], ctypes.c_int),
            "company_dds_poll": ([ctypes.c_char_p, u8p, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32), ctypes.c_int], ctypes.c_int),
            "company_dds_publish": ([ctypes.c_char_p, u8p, ctypes.c_uint32], ctypes.c_int),
            "company_dds_close": ([], None),
        }
        for name, (args, result) in signatures.items():
            fn = getattr(self.lib, name); fn.argtypes = args; fn.restype = result
        if self.lib.company_dds_init() != 0:
            self.lib.company_dds_close()
            raise RuntimeError("DDS 初始化失败")
        self.readers, self.writers = set(), set()
        self.capacity = 256*1024
        self.buffer = (ctypes.c_uint8*self.capacity)()

    def reader(self, topic):
        if topic not in self.readers:
            if self.lib.company_dds_create_reader(topic.encode(), 0) != 0:
                raise RuntimeError("DDS Reader 创建失败："+topic)
            self.readers.add(topic)

    def writer(self, topic):
        if topic not in self.writers:
            if self.lib.company_dds_create_writer(topic.encode(), 0) != 0:
                raise RuntimeError("DDS Writer 创建失败："+topic)
            self.writers.add(topic)

    def read_one(self, topic):
        self.reader(topic)
        while True:
            actual = ctypes.c_uint32()
            rc = self.lib.company_dds_poll(topic.encode(), self.buffer, self.capacity, ctypes.byref(actual), 0)
            if rc == -2 and self.capacity < actual.value <= MAX_PACKET:
                self.capacity = actual.value
                self.buffer = (ctypes.c_uint8*self.capacity)()
                continue
            if rc < 0 or actual.value > self.capacity:
                raise RuntimeError(f"DDS 读取失败 / 报文过大：{topic}, rc={rc}, bytes={actual.value}")
            if rc == 0:
                return None
            return bytes(self.buffer[:actual.value])

    def write_bytes(self, topic, packet):
        self.writer(topic)
        buffer = (ctypes.c_uint8*len(packet)).from_buffer_copy(packet)
        if self.lib.company_dds_publish(topic.encode(), buffer, len(packet)) != 0:
            raise RuntimeError("DDS 发送调用失败："+topic)

    def close(self):
        self.lib.company_dds_close()


def record(bus, output, topics, duration, max_bytes, monotonic=time.monotonic, wall_ns=time.time_ns, sleep=time.sleep):
    # Open file exclusively before subscribing; a wrong path must not silently overwrite evidence.
    writer = CaptureWriter(output, topics, max_bytes)
    reason, code = "duration", 0
    start = monotonic()
    last_status = start
    try:
        for topic in topics:
            bus.reader(topic)
        emit("record_started", file=str(output), topics=topics, duration_s=duration,
             note="独立只读订阅，不要求融合服务运行；Ctrl+C 安全结束")
        while duration == 0 or monotonic()-start < duration:
            found = False
            for topic in topics:
                packet = bus.read_one(topic)
                if packet is None:
                    continue
                found = True
                if not writer.add(topic, packet, round((monotonic()-start)*1e9), wall_ns()):
                    reason = "size_limit"
                    return 0
            if monotonic()-last_status >= 2:
                emit("record_progress", records=writer.sequence, counts=dict(writer.counts), bytes=writer.size)
                last_status = monotonic()
            if not found:
                sleep(.005)
    except KeyboardInterrupt:
        reason, code = "interrupted", 130
    except BaseException:
        reason = "error"
        raise
    finally:
        writer.finish(reason)
        emit("record_finished", file=str(output), reason=reason, records=writer.sequence, counts=dict(writer.counts))
    return code


class TrafficGuard:
    """Fail on other input traffic, while recognizing bounded recent self echoes.

    Identical bytes from another publisher are indistinguishable via this bridge.
    This check is not a replacement for stopping/isolating physical publishers.
    """
    def __init__(self, bus, monotonic=time.monotonic):
        self.bus, self.clock = bus, monotonic
        self.expected = OrderedDict()

    def sent(self, topic, packet):
        key = (topic, hashlib.sha256(packet).digest())
        self.expected[key] = self.clock()+30
        self.expected.move_to_end(key)
        while len(self.expected) > 4096:
            self.expected.popitem(last=False)

    def check(self):
        now = self.clock()
        while self.expected and next(iter(self.expected.values())) < now:
            self.expected.popitem(last=False)
        for topic in TOPICS:
            for _ in range(32):
                packet = self.bus.read_one(topic)
                if packet is None:
                    break
                if (topic, hashlib.sha256(packet).digest()) not in self.expected:
                    raise RuntimeError("检测到其他设备输入，回放已停止以避免混流："+topic)


def replay(bus, spool, summary, speed=1, time_mode="original", quiet_s=4,
           monotonic=time.monotonic, wall_ms=lambda: time.time_ns()//1_000_000, sleep=time.sleep):
    for topic in TOPICS:
        bus.reader(topic)
    for topic in summary["selected_counts"]:
        bus.writer(topic)
    guard = TrafficGuard(bus, monotonic)
    deadline = monotonic()+quiet_s
    emit("quiet_guard", seconds=quiet_s, note="DDS 发现及输入静默检查；尚未发送。此检查不代替物理隔离。")
    while monotonic() < deadline:
        guard.check(); sleep(min(.02, max(0, deadline-monotonic())))
    guard.check()
    shift = wall_ms()-summary["time_anchor_ms"] if time_mode == "rebase" else 0
    start = monotonic()
    sent = 0
    max_lag = 0
    last_progress = start
    spool.seek(0)
    try:
        for raw in spool:
            row = json.loads(raw)
            target = start+(row["offset_ns"]-summary["first_offset_ns"])/1e9/speed
            while monotonic() < target:
                guard.check(); sleep(min(.02, max(0, target-monotonic())))
            guard.check()
            packet = base64.b64decode(row["payload_base64"])
            if time_mode == "rebase":
                packet = relocate(row["topic"], packet, shift)
            guard.sent(row["topic"], packet)
            bus.write_bytes(row["topic"], packet)
            sent += 1
            lag = max(0, monotonic()-target)
            max_lag = max(max_lag, lag)
            if sent == 1 or sent % 100 == 0 or sent == summary["selected_records"] or monotonic()-last_progress >= 1:
                emit("replay_progress", published_calls=sent, total=summary["selected_records"], max_schedule_lag_ms=round(max_lag*1000, 3))
                last_progress = monotonic()
        # Allow a short drain for local reliable writer queues; not a receipt ACK.
        end = monotonic()+.5
        while monotonic() < end:
            guard.check(); sleep(min(.02, max(0, end-monotonic())))
    finally:
        emit("replay_stopped", published_calls=sent, time_mode=time_mode, time_shift_ms=shift,
             receiver_verified=False, note="发送调用成功不等于融合已接收，请查看接入诊断")
    return sent


def import_web(source, destination):
    """Convert prior alignment captures containing REAL packet_base64 fields.

    Older web captures emit one row per target, not per DDS sample. Deduplication
    is explicitly limited to identical bytes + identical receive time. Their
    missing packet IDs mean this conversion cannot prove original packet count.
    """
    with Path(source).open("rb") as stream:
        content = stream.read(8*1024*1024+1)
    if len(content) > 8*1024*1024:
        raise ValueError("网页采集 JSON 最大 8 MiB")
    data = json.loads(content)
    if not isinstance(data, dict) or data.get("format") != "sb-fusion-alignment-v1":
        raise ValueError("不是时空对齐工作台导出的原始采集 JSON")
    records = data.get("records")
    if not isinstance(records, list) or not 1 <= len(records) <= 2000:
        raise ValueError("网页采集须有 1–2000 条原始记录")
    if any(not isinstance(r, dict) for r in records):
        raise ValueError("网页采集记录格式无效")
    has_monotonic = all(r.get("captured_at_monotonic_s") is not None for r in records)
    unique, seen = [], set()
    for r in records:
        received = r.get("received_at_ms")
        stamp = r.get("captured_at_monotonic_s") if has_monotonic else received
        for value in (received, stamp):
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value < 1e15:
                raise ValueError("网页采集缺少有效接收时间，不能猜测原始发包间隔")
        encoded = r.get("packet_base64")
        if not isinstance(encoded, str) or len(encoded) > (MAX_PACKET+2)//3*4:
            raise ValueError("网页文件缺少原始报文字节；不能从对比结果 / 解码文本恢复原包")
        packet = base64.b64decode(encoded, validate=True)
        if not 0 < len(packet) <= MAX_PACKET:
            raise ValueError("无效原始报文长度")
        key = (received, packet)
        if key in seen:
            continue
        seen.add(key)
        captured_ns = round(stamp*(1e9 if has_monotonic else 1e6))
        unique.append((captured_ns, round(received*1e6), packet))
    if any(a[0] > b[0] for a,b in zip(unique, unique[1:])):
        raise ValueError("网页记录接收时间倒退，不能安全还原播放顺序")
    writer = CaptureWriter(destination, ("RadarTarget",), MAX_FILE, source_info={
        "kind": "alignment_web_import", "file_sha256": hashlib.sha256(content).hexdigest(),
        "source_rows": len(records), "merged_duplicate_rows": len(records)-len(unique),
        "timing_source": "monotonic" if has_monotonic else "wall_clock",
        "packet_count_exact": False,
        "note": "旧格式无独立报文ID；同接收时刻同字节合并。回放完整原包，可能含其他设备/目标。"})
    reason = "imported"
    try:
        for stamp, received, packet in unique:
            if not writer.add("RadarTarget", packet, stamp-unique[0][0], received):
                raise ValueError("转换结果超过大小上限")
    except BaseException:
        reason = "error"
        raise
    finally:
        writer.finish(reason)
    emit("web_imported", file=str(destination), records=writer.sequence,
         source_rows=len(records), merged_duplicate_rows=len(records)-len(unique),
         warning="旧网页文件按目标存行；已合并同接收时刻同原包。新录制请直接用 record，避免此歧义。",
         dds_packets_sent=0)


def parser():
    p = argparse.ArgumentParser(description="独立 DDS 原始数据录制与回放（不启动融合 / 网页，不自动安装依赖）")
    commands = p.add_subparsers(dest="command", required=True)
    rec = commands.add_parser("record", help="只读订阅设备输入并保存原始报文")
    rec.add_argument("file", type=Path)
    rec.add_argument("--topics", default=",".join(TOPICS))
    rec.add_argument("--duration", type=float, default=0, help="秒；0 表示直到 Ctrl+C 或大小上限")
    rec.add_argument("--max-mib", type=float, default=256, help="文件上限 MiB（1–512），默认 256")
    rec.add_argument("--bridge", type=Path, default=ROOT/"libcompany_dds_bridge.so")
    web = commands.add_parser("import-web", help="将已有网页原始采集 JSON 转为独立录制文件（不发 DDS）")
    web.add_argument("source", type=Path)
    web.add_argument("file", type=Path)
    for name in ("inspect", "replay"):
        sub = commands.add_parser(name, help="校验并查看录制文件" if name == "inspect" else "默认 dry-run；--publish 才发送 DDS")
        sub.add_argument("file", type=Path)
        sub.add_argument("--topics", help="筛选 Topic，英文逗号分隔")
        sub.add_argument("--allow-incomplete", action="store_true", help="显式恢复未完成文件；最多忽略截断的最后一行")
        if name == "replay":
            sub.add_argument("--publish", action="store_true")
            sub.add_argument("--inputs-stopped", action="store_true", help="确认已停止/隔离真实设备和其他仿真发布者")
            sub.add_argument("--speed", type=float, default=1, help="0.1–20 倍，仅调整发包节奏；联调建议 1")
            sub.add_argument("--time-mode", choices=("original", "rebase"), default="original")
            sub.add_argument("--quiet-seconds", type=float, default=4, help="发布前静默检查秒数，最少 4")
            sub.add_argument("--bridge", type=Path, default=ROOT/"libcompany_dds_bridge.so")
            sub.add_argument("--expected-data-sha256", help="要求内容与预检查摘要一致；不匹配则在接入 DDS 前拒绝")
    return p


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    args = parser().parse_args(argv)
    bus = None
    try:
        if args.command == "import-web":
            import_web(args.source, args.file)
            return 0
        topics = select_topics(args.topics) if args.topics else None
        if args.command == "record":
            if not math.isfinite(args.duration) or args.duration < 0:
                raise ValueError("录制时长必须是有限非负数")
            if not math.isfinite(args.max_mib) or not 1 <= args.max_mib <= 512:
                raise ValueError("文件大小上限须在 1–512 MiB")
            if args.file.exists():
                raise FileExistsError("录制文件已存在，不会覆盖："+str(args.file))
            if not args.file.parent.is_dir():
                raise ValueError("输出目录不存在，请先创建："+str(args.file.parent))
            ensure_runtime(argv)
            bus = NativeBus(args.bridge)
            return record(bus, args.file, topics, args.duration, int(args.max_mib*1024*1024))
        if args.command == "replay":
            if not math.isfinite(args.speed) or not .1 <= args.speed <= 20:
                raise ValueError("回放速度必须在 0.1–20 倍之间")
            if not math.isfinite(args.quiet_seconds) or not 4 <= args.quiet_seconds <= 300:
                raise ValueError("静默检查须为 4–300 秒")
            if args.publish and not args.inputs_stopped:
                raise ValueError("真正回放必须同时指定 --publish --inputs-stopped，并先停止/隔离其他设备发布者")
        time_mode = getattr(args, "time_mode", "original")
        with tempfile.TemporaryFile(mode="w+b") as spool:
            summary = prepare(args.file, spool, topics, time_mode, args.allow_incomplete)
            expected = getattr(args, "expected_data_sha256", None)
            if expected is not None and expected != summary["data_sha256"]:
                raise ValueError("文件内容已改变，请重新预检查；尚未接入 DDS")
            emit("file_checked", **summary, time_mode=time_mode, dds_packets_sent=0,
                 playback_duration_s=summary["duration_s"]/getattr(args, "speed", 1))
            if args.command == "inspect" or not args.publish:
                emit("dry_run", dds_packets_sent=0, native_library_loaded=False)
                return 0
            if not summary["selected_records"]:
                raise ValueError("所选 Topic 没有可回放报文")
            if args.speed != 1:
                emit("warning", note="倍速只改变发送间隔，设备时间差和业务字段不缩放；不等效实时实装验证")
            ensure_runtime(argv)
            bus = NativeBus(args.bridge)
            replay(bus, spool, summary, args.speed, time_mode, args.quiet_seconds)
            return 0
    except KeyboardInterrupt:
        emit("interrupted", note="用户终止")
        return 130
    except (OSError, ValueError, TypeError, KeyError, AttributeError, struct.error, RuntimeError) as exc:
        emit("error", error=f"{type(exc).__name__}: {exc}")
        return 1
    finally:
        if bus is not None:
            bus.close()


def stop_signal(*_):
    raise KeyboardInterrupt()


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, stop_signal)
    raise SystemExit(main())
