"""Read-only, bounded inspection of received FusionMultiTarget DDS bytes.

This module never creates a writer or changes the production serializer.
The legacy extra-timestamp layout is diagnostic-only, never reported as valid.
"""

from collections import deque
import struct
import time


HEADER = struct.Struct("<BBHHIHHHHQ")
UNIT = struct.Struct("<IHQ")
TARGET = struct.Struct("<BiiIhhHIBBBBB")
PREDICTION = struct.Struct("<iiIhhHIQ")
MAX_PACKET_BYTES = 65536
SOURCE_NAMES = ["雷达", "光电", "电侦", "声探", "雷达+光电", "雷达+电侦",
                "雷达+声探", "光电+电侦", "光电+声探", "电侦+声探",
                "雷达+光电+电侦", "雷达+光电+声探", "光电+电侦+声探", "全部"]
TYPE_NAMES = ["无人机", "鸟", "大型目标", "车辆", "人员", "其他"]


def _motion(values):
    lng, lat, alt, pitch, roll, yaw, speed = values[:7]
    return {"lng_deg": lng / 1e7, "lat_deg": lat / 1e7, "alt_m": alt / 10,
            "pitch_deg": pitch / 100, "roll_deg": roll / 100,
            "yaw_deg": yaw / 100, "speed_ms": speed / 100}


def _decode_body(body):
    if not body:
        raise ValueError("消息体缺少 target_count")
    count, offset, targets = body[0], 1, []
    for index in range(count):
        if offset + TARGET.size > len(body):
            raise ValueError(f"目标 {index + 1}/{count} 的固定字段不足 28 字节")
        values = TARGET.unpack_from(body, offset)
        offset += TARGET.size
        identity, *motion, kind, confidence, source, iff, prediction_count = values
        row = {"global_id": identity, **_motion(motion), "fusion_type": kind,
               "type_name": TYPE_NAMES[kind] if kind < len(TYPE_NAMES) else f"未知枚举({kind})",
               "confidence": confidence, "source_mask": source,
               "source_name": SOURCE_NAMES[source] if source < len(SOURCE_NAMES) else f"未知枚举({source})",
               "iff": iff, "pred_point_count": prediction_count, "prediction_points": []}
        if offset + prediction_count * PREDICTION.size > len(body):
            raise ValueError(f"目标 {index + 1} 的预测点数据被截断")
        for _ in range(prediction_count):
            point = PREDICTION.unpack_from(body, offset)
            offset += PREDICTION.size
            row["prediction_points"].append({**_motion(point), "timestamp_ms": str(point[-1])})
        targets.append(row)
    if offset != len(body):
        raise ValueError(f"消息体解析后还剩 {len(body) - offset} 字节，目标数量/长度不匹配")
    return count, targets


def decode_official_packet(packet):
    """Keep raw bytes even on failure; only decode a fully bounded known layout."""
    result = {"valid": False, "body_decoded": False, "decode_mode": "unknown",
              "actual_bytes": len(packet), "raw_hex": packet[:MAX_PACKET_BYTES].hex(" "),
              "raw_truncated": len(packet) > MAX_PACKET_BYTES,
              "header": None, "unit_header": None, "target_count": None,
              "targets": [], "errors": [], "warnings": []}
    errors = result["errors"]
    if len(packet) > MAX_PACKET_BYTES:
        errors.append(f"报文超过只读面板 {MAX_PACKET_BYTES} 字节上限；仅保存前缀，不解析目标")
        return result
    if len(packet) < HEADER.size:
        errors.append("报文不足 26 字节 MsgHeader")
        return result
    values = HEADER.unpack_from(packet)
    header = dict(zip(("startTag", "unitNums", "srcAddress", "destAddress", "msgLength",
                       "msgTag", "msgID", "checkID", "bak", "timestamp_ms"), values))
    # uint64 can exceed JavaScript's exact integer range, especially legacy uninitialized stamps.
    header["timestamp_ms"] = str(header["timestamp_ms"])
    result["header"] = header
    result["actual_payload_bytes"] = len(packet) - HEADER.size
    if header["startTag"] != 0x7E:
        errors.append("startTag 不是 0x7E")
    if header["unitNums"] != 1:
        errors.append("此面板只解析单个 FusionMultiTarget 信息单元")
    if len(packet) < HEADER.size + UNIT.size:
        errors.append("报文不足 14 字节 UnitHeader")
        return result
    length, tag, stamp = UNIT.unpack_from(packet, HEADER.size)
    result["unit_header"] = {"unitLength": length, "unitTag": tag, "timestamp_ms": str(stamp)}
    if tag != 1:
        errors.append(f"unitTag={tag}，不是 FusionMultiTarget (0x01)")
    if errors:
        return result
    if header["msgLength"] != len(packet) - HEADER.size:
        errors.append(f"MsgHeader 声明载荷 {header['msgLength']} 字节，实际 {len(packet) - HEADER.size} 字节")
    if header["msgLength"] != UNIT.size + length:
        errors.append("msgLength 与 14 字节 UnitHeader + unitLength 不一致")
    offset = HEADER.size + UNIT.size
    if len(packet) == offset + length:
        result["decode_mode"] = "pxsb14"
    elif (len(packet) == offset + 8 + length and header["msgLength"] == UNIT.size + length
          and struct.unpack_from("<Q", packet, offset)[0] == values[-1]):
        result["decode_mode"] = "legacy_extra_timestamp"
        result["extra_timestamp_ms"] = str(struct.unpack_from("<Q", packet, offset)[0])
        offset += 8
        errors.append("疑似原版重复追加 8 字节时间戳；按旧布局展示消息体，仅供诊断，不算有效报文")
    else:
        errors.append("实际消息体长度与 unitLength 不匹配，拒绝猜测目标偏移")
        return result
    result["body_offset"] = offset
    if str(stamp) != header["timestamp_ms"]:
        result["warnings"].append("源时间戳与发送时间戳不同；原版重复时间戳布局也可能含未初始化值")
    try:
        result["target_count"], result["targets"] = _decode_body(packet[offset:])
        result["body_decoded"] = True
    except ValueError as exc:
        errors.append(str(exc))
    result["valid"] = not errors
    return result


class OfficialFusionMonitor:
    def __init__(self):
        self.started_at = time.time()
        self.reader_error = None
        self.last_poll_at = None
        self.last_received_at = None
        self.packets = self.total_bytes = self.valid_packets = self.invalid_packets = 0
        self.empty_packets = self.legacy_packets = 0
        self.latest = None
        self.recent = deque(maxlen=20)

    def ingest(self, packet, received_at=None):
        now = time.time() if received_at is None else received_at
        decoded = decode_official_packet(packet)
        self.packets += 1
        self.total_bytes += len(packet)
        self.last_received_at = now
        self.valid_packets += int(decoded["valid"])
        self.invalid_packets += int(not decoded["valid"])
        self.empty_packets += int(decoded["valid"] and decoded["target_count"] == 0)
        self.legacy_packets += int(decoded["decode_mode"] == "legacy_extra_timestamp")
        self.latest = {**decoded, "sequence": self.packets, "received_at": now}
        self.recent.append({key: self.latest[key] for key in
                            ("sequence", "received_at", "actual_bytes", "valid", "decode_mode", "target_count")})

    def snapshot(self, now=None):
        now = time.time() if now is None else now
        return {"topic": "FusionMultiTarget", "read_only": True, "started_at": self.started_at,
                "reader_error": self.reader_error, "last_poll_at": self.last_poll_at,
                "last_received_at": self.last_received_at, "packets": self.packets,
                "total_bytes": self.total_bytes, "valid_packets": self.valid_packets,
                "invalid_packets": self.invalid_packets, "empty_packets": self.empty_packets,
                "legacy_packets": self.legacy_packets,
                "average_hz": self.packets / max(1, now - self.started_at),
                "latest": self.latest, "recent": list(reversed(self.recent)),
                "max_packet_bytes": MAX_PACKET_BYTES}
