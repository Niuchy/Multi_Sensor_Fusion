"""Radar alignment workbench: passive DDS observation and offline comparison.

Saved profiles affect this workbench ONLY. No writer, clock update, production
configuration change, or live track reset is performed by this module.
"""
from collections import deque
from copy import deepcopy
import base64
import json
import math
import os
from pathlib import Path
import re
import secrets
import statistics
import struct
import tempfile
import threading
import time

from company_protocol import (Wgs84Enu, parse_company_packet, unpack_pxsb_units,
                              PXSB_HEADER_FORMAT, PXSB_HEADER_SIZE, FRAME_MAGIC)

FORMAT = "sb-fusion-alignment-v1"
MAX_RECORDS = 2000
MAX_PACKET = 65536
MAX_BUFFER_BYTES = 4 * 1024 * 1024
DEFAULTS = {
    "time_source": "frame", "clock_basis": "unknown", "time_offset_ms": 0.0,
    "coordinate_mode": "auto", "station_enu_m": None,
    "azimuth_bias_deg": 0.0, "elevation_bias_deg": 0.0,
    "range_bias_m": 0.0, "position_bias_enu_m": [0.0, 0.0, 0.0],
}
FIELDS = ("device_id", "track_id", "timestamp_ms", "range_m", "azimuth_deg",
          "elevation_deg", "lng_deg", "lat_deg", "alt_m", "speed_ms",
          "course_deg", "extrap_flag")


def number(value, bound=1e15):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("参数必须是数值")
    if not math.isfinite(value) or abs(value) > bound:
        raise ValueError("参数超出允许范围或不是有限数值")
    return float(value)


def vector(value, bound):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError("坐标必须是三个数值组成的数组")
    return [number(x, bound) for x in value]


def device_key(value):
    if not isinstance(value, str) or not re.fullmatch(r"radar_[0-9]{1,10}", value):
        raise ValueError("第一版仅支持 radar_设备号，例如 radar_3152")
    return value


def actual_unit_times(packet):
    """Unlike the production parser's fallback, absent Unit time stays absent."""
    offset, times = PXSB_HEADER_SIZE, []
    for tag, stamp, frame in unpack_pxsb_units(packet):
        base = offset + 6
        extended = packet[base+8:base+8+len(frame)] == frame
        if tag in FRAME_MAGIC:
            times.append(stamp if extended else None)
        offset = base + (8 if extended else 0) + len(frame)
    return times


def validate_params(values):
    if not isinstance(values, dict) or set(values) - set(DEFAULTS):
        raise ValueError("未知校准字段")
    p = {**deepcopy(DEFAULTS), **values}
    for key, choices in (("time_source", {"frame", "unit", "header"}),
                         ("clock_basis", {"unknown", "unix_ms", "relative_ms"}),
                         ("coordinate_mode", {"auto", "l1", "l2"})):
        if p[key] not in choices:
            raise ValueError(f"不支持的 {key}")
    for key, bound in (("time_offset_ms", 3600000), ("azimuth_bias_deg", 180),
                       ("elevation_bias_deg", 90), ("range_bias_m", 20000)):
        p[key] = number(p[key], bound)
    if p["station_enu_m"] is not None:
        p["station_enu_m"] = vector(p["station_enu_m"], 1e6)
    p["position_bias_enu_m"] = vector(p["position_bias_enu_m"], 100000)
    return p


class ProfileStore:
    """Optimistic revision check + atomic replace; no writes on GET/startup."""
    def __init__(self, path):
        self.path = Path(path)
        self.lock = threading.RLock()

    def read(self):
        with self.lock:
            if not self.path.exists():
                return {"schema": 1, "revision": 0, "devices": {}, "history": []}
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if (not isinstance(data, dict) or data.get("schema") != 1
                    or not isinstance(data.get("history"), list)
                    or not isinstance(data.get("devices"), dict)
                    or type(data.get("revision")) is not int):
                raise ValueError("校准配置文件损坏，拒绝覆盖；请从备份恢复")
            for key, value in data["devices"].items():
                device_key(key)
                validate_params(value)
            for entry in data["history"]:
                if not isinstance(entry, dict) or type(entry.get("revision")) is not int or not isinstance(entry.get("devices"), dict):
                    raise ValueError("校准配置历史损坏，拒绝覆盖")
                for key, value in entry["devices"].items():
                    device_key(key)
                    validate_params(value)
            return data

    def save(self, expected, device=None, params=None, rollback=None, note=""):
        with self.lock:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            # Linux target may have more than one gateway; serialize file writers too.
            import fcntl
            with self.path.with_suffix(".lock").open("a") as lock_file:
                fcntl.flock(lock_file, fcntl.LOCK_EX)
                data = self.read()
                if expected != data["revision"]:
                    raise FileExistsError("配置已被另一页面修改，请重新加载后再保存")
                if len(data["history"]) >= 500:
                    raise ValueError("已达到 500 个版本，请先归档配置文件；不会删除旧版本")
                if rollback is not None:
                    if rollback == 0:
                        devices = {}
                    else:
                        entry = next((h for h in data["history"] if h["revision"] == rollback), None)
                        if entry is None:
                            raise ValueError("回滚版本不存在")
                        devices = deepcopy(entry["devices"])
                    action = f"rollback:{rollback}"
                else:
                    devices = deepcopy(data["devices"])
                    devices[device_key(device)] = validate_params(params)
                    if len(devices) > 64:
                        raise ValueError("最多保存 64 个设备的参数")
                    action = f"save:{device}"
                revision = data["revision"] + 1
                entry = {"revision": revision, "created_at": time.time(), "action": action,
                         "note": str(note)[:200], "devices": devices}
                data = {"schema": 1, "revision": revision, "devices": devices,
                        "history": data["history"] + [entry]}
                fd, name = tempfile.mkstemp(prefix=".alignment-", dir=self.path.parent)
                try:
                    with os.fdopen(fd, "w", encoding="utf-8") as f:
                        json.dump(data, f, ensure_ascii=False, indent=2, allow_nan=False)
                        f.flush()
                        os.fsync(f.fileno())
                    os.replace(name, self.path)
                finally:
                    if os.path.exists(name):
                        os.unlink(name)
                return data


def positions(record, params):
    raw, context = record["raw"], record["context"]
    origin = context.get("origin")
    if origin is None:
        raise ValueError("尚无参考原点，等待融合配置；不能猜测绝对位置")
    origin = vector(origin, 1e6)
    if abs(origin[0]) > 180 or abs(origin[1]) >= 90:
        raise ValueError("参考原点经纬度无效")
    converter = Wgs84Enu(origin)
    l2_valid = (abs(raw["lng_deg"]) <= 180 and abs(raw["lat_deg"]) <= 90
                and converter.valid(raw["lng_deg"], raw["lat_deg"]))
    l2 = list(converter.to_enu(raw["lng_deg"], raw["lat_deg"], raw["alt_m"])) if l2_valid else None
    station = params["station_enu_m"] if params["station_enu_m"] is not None else context.get("station_enu_m")
    l1 = None
    if station is not None:
        station = vector(station, 1e6)
        r = raw["range_m"] + params["range_bias_m"]
        el = raw["elevation_deg"] + params["elevation_bias_deg"]
        az = raw["azimuth_deg"] + params["azimuth_bias_deg"]
        convention = context.get("azimuth_convention")
        if convention not in {"north_cw", "enu"}:
            raise ValueError("方位定义未知，不能计算 L1 位置")
        if r >= 0 and abs(el) <= 90:
            angle = math.radians(90 - az if convention == "north_cw" else az)
            ground = r * math.cos(math.radians(el))
            l1 = [station[0] + ground * math.cos(angle), station[1] + ground * math.sin(angle),
                  station[2] + r * math.sin(math.radians(el))]
    mode = params["coordinate_mode"]
    source = ("L2" if l2_valid else "L1") if mode == "auto" else mode.upper()
    pos = l2 if source == "L2" else l1
    if pos is None:
        raise ValueError(f"{source} 不可用：缺少有效经纬度、站址或距离角度")
    return {"source": source, "xyz": [p + bias for p, bias in zip(pos, params["position_bias_enu_m"])],
            "l1_xyz": l1, "l2_xyz": l2, "l2_valid": l2_valid,
            "l1_l2_distance_m": math.dist(l1, l2) if l1 is not None and l2 is not None else None}


def compare(records, params):
    p = validate_params(params)
    rows, last, intervals, diffs, deltas = [], {}, [], [], []
    duplicates = late = 0
    for record in records:
        raw = record["raw"]
        frame_time = raw["timestamp_ms"]
        selected = frame_time if p["time_source"] == "frame" else record.get(p["time_source"] + "_timestamp_ms")
        corrected_time = selected + p["time_offset_ms"] if selected is not None else None
        key = (raw["device_id"], raw["track_id"])
        previous = last.get(key)
        interval = (frame_time - previous) / 1000 if previous is not None else None
        if interval is not None:
            duplicates += int(interval == 0)
            late += int(interval < 0)
            if interval > 0:
                intervals.append(interval)
        last[key] = frame_time
        receive_ms = record.get("received_at_ms")
        delta = (receive_ms - corrected_time if receive_ms is not None and corrected_time is not None
                 and p["clock_basis"] == "unix_ms" else None)
        warnings = []
        if p["clock_basis"] == "unix_ms" and selected is not None and not 946684800000 <= selected <= 4102444800000:
            warnings.append("时间戳不像 2000–2100 年 Unix 毫秒；接收时间差不具参考意义")
            delta = None
        if selected is None:
            warnings.append("所选时间戳在这份记录中不存在")
        elif selected == 0:
            warnings.append("所选时间戳为 0；不能视为有效绝对时刻")
        if delta is not None:
            deltas.append(delta)
            if delta < 0:
                warnings.append("校正时刻晚于接收时刻，请检查钟差与补偿符号")
        row = {"sequence": record.get("sequence"), "device_id": raw["device_id"],
               "track_id": raw["track_id"], "frame_timestamp_ms": frame_time,
               "unit_timestamp_ms": record.get("unit_timestamp_ms"),
               "header_timestamp_ms": record.get("header_timestamp_ms"),
               "received_at_ms": receive_ms, "corrected_timestamp_ms": corrected_time,
               "receive_minus_corrected_ms": delta, "interval_s": interval,
               "tag": raw["extrap_flag"], "raw_position": None, "corrected_position": None,
               "raw": deepcopy(raw), "context": deepcopy(record["context"]),
               "position_change_m": None, "errors": [], "warnings": warnings}
        for label, values in (("raw_position", DEFAULTS), ("corrected_position", p)):
            try:
                row[label] = positions(record, values)
            except ValueError as exc:
                row["errors"].append(f"{label}: {exc}")
        if row["raw_position"] and row["corrected_position"]:
            row["position_change_m"] = math.dist(row["raw_position"]["xyz"], row["corrected_position"]["xyz"])
            diffs.append(row["position_change_m"])
        if row["corrected_position"] and row["corrected_position"]["source"] == "L2":
            warnings.append("当前 L2 路径不使用站址、距离、方位或俯仰补偿；ENU 位置偏置仍有效")
        rows.append(row)
    ordered = sorted(deltas)
    return {"rows": rows, "params": p, "summary": {"records": len(rows),
            "duplicate_observations": duplicates, "out_of_order_observations": late,
            "interval_mean_s": statistics.mean(intervals) if intervals else None,
            "interval_jitter_s": statistics.pstdev(intervals) if intervals else None,
            "receive_delta_p50_ms": statistics.median(deltas) if deltas else None,
            "receive_delta_p95_ms": ordered[max(0, math.ceil(len(ordered)*.95)-1)] if ordered else None,
            "position_change_rms_m": math.sqrt(statistics.mean(x*x for x in diffs)) if diffs else None,
            "position_change_max_m": max(diffs) if diffs else None,
            "invalid_positions": sum(bool(r["errors"]) for r in rows)},
            "scope": "preview_only", "accuracy_assessed": False}


def validate_record(record):
    if not isinstance(record, dict) or not isinstance(record.get("raw"), dict):
        raise ValueError("回放记录缺少 raw")
    raw = {k: number(record["raw"][k]) for k in FIELDS}
    for key in ("device_id", "track_id", "extrap_flag"):
        if raw[key] != int(raw[key]):
            raise ValueError("设备号、航迹号、标记必须是整数")
        raw[key] = int(raw[key])
    device_key(f"radar_{raw['device_id']}")
    if raw["extrap_flag"] not in (0, 1) or raw["timestamp_ms"] < 0:
        raise ValueError("无效时间戳或 tag")
    context = record.get("context", {})
    if not isinstance(context, dict):
        raise ValueError("坐标上下文必须是对象")
    station_source = context.get("station_source", "capture_or_user")
    if not isinstance(station_source, str) or len(station_source) > 80:
        raise ValueError("无效站址来源")
    context = {"origin": vector(context["origin"], 1e6) if context.get("origin") is not None else None,
               "station_enu_m": vector(context["station_enu_m"], 1e6) if context.get("station_enu_m") is not None else None,
               "station_source": station_source,
               "azimuth_convention": context.get("azimuth_convention", "north_cw")}
    if context["azimuth_convention"] not in ("north_cw", "enu"):
        raise ValueError("无效方位定义")
    out = {"raw": raw, "context": context}
    for key in ("received_at_ms", "unit_timestamp_ms", "header_timestamp_ms", "sequence"):
        out[key] = number(record[key]) if record.get(key) is not None else None
    return out


def receiver_log_records(text, context):
    """Decode the supplied text log without files, native DDS, or fake receive times."""
    records, previous, day = [], None, 0
    for block in re.split(r"╔══ Radar #\d+", text)[1:]:
        def val(key):
            match = re.search(r"\b" + key + r"=(-?[\d.]+)", block)
            if not match:
                raise ValueError(f"日志缺少 {key}")
            return float(match[1])
        stamp = re.search(r"ts=(\d+):(\d+):(\d+)\.(\d{3})", block)
        if not stamp:
            raise ValueError("日志缺少时分秒毫秒")
        h,m,s,ms = map(int, stamp.groups())
        if h > 23 or m > 59 or s > 59:
            raise ValueError("日志时间非法")
        current = ((h*60+m)*60+s)*1000+ms
        if previous is not None and current < previous-43200000:
            day += 86400000
        previous = current
        tag = re.search(r"\btag=(外推点|真实点)", block)
        if not tag:
            raise ValueError("日志缺少可识别的 tag")
        raw = {name: val(field) for name, field in
               (("device_id","dev"),("track_id","track_id"),("range_m","dis"),
                ("azimuth_deg","azi"),("elevation_deg","ele"),("lng_deg","lng"),
                ("lat_deg","lat"),("alt_m","alt"),("speed_ms","speed"),("course_deg","course"))}
        raw.update(timestamp_ms=day+current, extrap_flag=int(tag[1] == "真实点"))
        records.append(validate_record({"raw":raw, "context":context, "sequence":len(records)+1}))
        if len(records) > MAX_RECORDS:
            raise ValueError(f"单次回放最多 {MAX_RECORDS} 条记录")
    if not records:
        raise ValueError("未找到 Radar 日志记录")
    return records


class AlignmentWorkbench:
    def __init__(self, profile_path):
        self.store = ProfileStore(profile_path)
        self.token = secrets.token_urlsafe(32)
        self.lock = threading.RLock()
        self.records = deque()
        self.buffer_bytes = self.evicted_records = 0
        self.devices = {}
        self.packets = self.errors = self.sequence = 0
        self.last_error = None
        self.reader_error = None
        self.last_poll_at = None
        self.default_context = {"origin": None, "station_enu_m": None, "azimuth_convention": "north_cw"}

    def poll(self, bus, init, input_status):
        try:
            origin = init.get("coordinates", {}).get("origin")
            convention = input_status.get("azimuth_convention", "north_cw")
            self.default_context = {"origin": origin, "station_enu_m": None, "azimuth_convention": convention}
            for packet in bus.read_bytes("RadarTarget", max_samples=32):
                self.observe(packet, init, convention)
            self.reader_error = None
            self.last_poll_at = time.time()
        except Exception as exc:
            self.reader_error = f"{type(exc).__name__}: {exc}"

    def observe(self, packet, init, convention, received_at_ms=None):
        with self.lock:
            self.packets += 1
            received = time.time()*1000 if received_at_ms is None else received_at_ms
            try:
                if len(packet) > MAX_PACKET:
                    raise ValueError("雷达报文超过调试采集大小上限")
                header_time = struct.unpack_from(PXSB_HEADER_FORMAT, packet)[-1]
                for (tag, _, obj), unit_time in zip(parse_company_packet(packet), actual_unit_times(packet)):
                    if tag != 0x1A01:
                        continue
                    sid = f"radar_{obj.device_id}"
                    sensor = init.get("sensors", {}).get(sid)
                    station_source = sid
                    if sensor is None:
                        sensor = init.get("sensors", {}).get("radar_101", {})
                        station_source = "radar_101_template" if sensor else "unknown"
                    context = {"origin": init.get("coordinates", {}).get("origin"),
                               "station_enu_m": sensor.get("pos"), "azimuth_convention": convention,
                               "station_source": station_source}
                    record = validate_record({"raw": {k: getattr(obj,k) for k in FIELDS}, "context":context,
                                              "received_at_ms":received, "unit_timestamp_ms":unit_time,
                                              "header_timestamp_ms":header_time})
                    self.sequence += 1
                    record.update(sequence=self.sequence, packet_base64=base64.b64encode(packet).decode(),
                                  captured_at_monotonic_s=time.monotonic())
                    size = len(json.dumps(record, ensure_ascii=False).encode("utf-8"))
                    record["_size"] = size
                    self.records.append(record)
                    self.buffer_bytes += size
                    while len(self.records) > MAX_RECORDS or self.buffer_bytes > MAX_BUFFER_BYTES:
                        self.buffer_bytes -= self.records.popleft()["_size"]
                        self.evicted_records += 1
                    self.devices[sid] = {"last_received_at_ms":received, "context":context}
                    if len(self.devices) > 256:
                        del self.devices[min(self.devices, key=lambda key: self.devices[key]["last_received_at_ms"])]
            except (ValueError, TypeError, KeyError, AttributeError, struct.error, OverflowError) as exc:
                self.errors += 1
                self.last_error = str(exc)

    def export(self, device=None):
        if device:
            device_key(device)
        with self.lock:
            records = [deepcopy(r) for r in self.records if not device or f"radar_{r['raw']['device_id']}" == device]
            for record in records:
                record.pop("_size", None)
        profiles = self.store.read()
        return {"format": FORMAT, "exported_at": time.time(), "records": records,
                "limit": MAX_RECORDS, "scope": "passive_debug_receiver",
                "profiles": {"revision": profiles["revision"], "devices": profiles["devices"]}}

    def state(self):
        data = self.store.read()
        with self.lock:
            return self._state(data)

    def _state(self, data):
        return {"scope":"preview_only", "token":self.token, "defaults":deepcopy(DEFAULTS),
                "revision":data["revision"], "profiles":data["devices"],
                "history":[{k:v for k,v in h.items() if k != "devices"} for h in reversed(data["history"])],
                "devices":deepcopy(self.devices), "packets":self.packets, "errors":self.errors,
                "last_error":self.last_error, "reader_error":self.reader_error,
                "last_poll_at":self.last_poll_at, "buffered_records":len(self.records),
                "evicted_records": self.evicted_records, "buffer_bytes": self.buffer_bytes,
                "default_context":self.default_context, "profile_file":str(self.store.path)}
