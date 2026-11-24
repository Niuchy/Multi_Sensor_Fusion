"""Python adapter for zy-anti-drone's TopicAPI/FastDDS implementation.

All inter-process topics in the integrated stack use the company's
``ByteSequence`` DDS type.  JSON is only the application payload used by the
Python nodes; DDS discovery, transport, readers and writers all come from
``zy-anti-drone``.
"""

from __future__ import annotations

import ctypes
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

from json_utils import clean


@dataclass(frozen=True)
class TopicSpec:
    name: str
    description: str


class Topics:
    # Company simulation device topics (dev_proto.h inside a PXSB envelope).
    COMPANY_RADAR = TopicSpec("RadarTarget", "Company radar target frames.")
    COMPANY_ESM = TopicSpec("ESMTarget", "Company ESM target frames.")
    COMPANY_EO = TopicSpec("EOTarget", "Company EO target frames.")
    COMPANY_ACOUSTIC = TopicSpec(
        "AcousticTarget", "Company acoustic target frames."
    )

    # Fusion pipeline topics.  They still use the company's TopicAPI and
    # ByteSequence DDS type; names are separated from the equipment protocol.
    SENSOR_INFO = TopicSpec(
        "SwarmFusion.SensorInfo", "Company sensor layout snapshots."
    )
    RAW_MEASUREMENTS = TopicSpec(
        "SwarmFusion.RawMeasurements", "Normalized company equipment frames."
    )
    SENSOR_TRACKS = TopicSpec(
        "SwarmFusion.SensorTracks", "Per-company-sensor tracks."
    )
    FUSED_TRACKS = TopicSpec(
        "SwarmFusion.FusedTracks", "Fusion algorithm tracks and diagnostics."
    )
    FUSED_FRAME = TopicSpec(
        "SwarmFusion.FusedFrame", "Frontend-compatible live frame."
    )
    CONTROL_COMMAND = TopicSpec(
        "SwarmFusion.ControlCommand", "Local pipeline control commands."
    )
    DIAGNOSTICS = TopicSpec(
        "SwarmFusion.Diagnostics", "Node heartbeats and pipeline diagnostics."
    )

    # Official zy-anti-drone fusion_proto.h topics.
    OFFICIAL_FUSION = TopicSpec(
        "FusionMultiTarget", "fusion_proto.h FUSION_MULTI_TARGET."
    )


COMPANY_INPUT_TOPICS = (
    Topics.COMPANY_RADAR.name,
    Topics.COMPANY_ESM.name,
    Topics.COMPANY_EO.name,
    Topics.COMPANY_ACOUSTIC.name,
)


class CompanyFusionTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("global_id", ctypes.c_uint8),
        ("lng_e7", ctypes.c_int32),
        ("lat_e7", ctypes.c_int32),
        ("alt_dm", ctypes.c_uint32),
        ("pitch_cdeg", ctypes.c_int16),
        ("roll_cdeg", ctypes.c_int16),
        ("yaw_cdeg", ctypes.c_uint16),
        ("speed_cms", ctypes.c_uint32),
        ("fusion_type", ctypes.c_uint8),
        ("confidence", ctypes.c_uint8),
        ("source_mask", ctypes.c_uint8),
        ("iff", ctypes.c_uint8),
    ]


_NATIVE = None
_NATIVE_ERROR = None


def _bridge_path() -> Path:
    return Path(__file__).resolve().parent / "company_bridge/libcompany_dds_bridge.so"


def _configure_native(lib):
    u8p = ctypes.POINTER(ctypes.c_uint8)

    lib.company_dds_init.restype = ctypes.c_int
    lib.company_dds_init.argtypes = []
    lib.company_dds_ready.restype = ctypes.c_int
    lib.company_dds_ready.argtypes = []
    lib.company_dds_create_writer.restype = ctypes.c_int
    lib.company_dds_create_writer.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.company_dds_create_reader.restype = ctypes.c_int
    lib.company_dds_create_reader.argtypes = [ctypes.c_char_p, ctypes.c_int]
    lib.company_dds_publish.restype = ctypes.c_int
    lib.company_dds_publish.argtypes = [
        ctypes.c_char_p,
        u8p,
        ctypes.c_uint32,
    ]
    lib.company_dds_poll.restype = ctypes.c_int
    lib.company_dds_poll.argtypes = [
        ctypes.c_char_p,
        u8p,
        ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint32),
        ctypes.c_int,
    ]
    lib.company_dds_available.restype = ctypes.c_int
    lib.company_dds_available.argtypes = [ctypes.c_char_p]
    lib.company_dds_publish_fusion.restype = ctypes.c_int
    lib.company_dds_publish_fusion.argtypes = [
        ctypes.c_char_p,
        ctypes.c_uint16,
        ctypes.c_uint64,
        ctypes.POINTER(CompanyFusionTargetData),
        ctypes.c_uint32,
    ]
    lib.company_dds_close.restype = None
    lib.company_dds_close.argtypes = []
    lib.company_dds_version.restype = ctypes.c_char_p
    lib.company_dds_version.argtypes = []
    return lib


def require_company_dds():
    global _NATIVE, _NATIVE_ERROR
    if _NATIVE is not None:
        return _NATIVE

    path = _bridge_path()
    try:
        if not path.exists():
            raise FileNotFoundError(path)
        _NATIVE = _configure_native(ctypes.CDLL(str(path), mode=ctypes.RTLD_GLOBAL))
        if _NATIVE.company_dds_init() != 0:
            raise RuntimeError("zy-anti-drone InitTopic() failed")
        _NATIVE_ERROR = None
        return _NATIVE
    except Exception as exc:
        _NATIVE = None
        _NATIVE_ERROR = exc
        raise RuntimeError(
            "Company DDS bridge is unavailable. On the Linux target run "
            "`./build_dgx.sh` from the sb_fusion workspace root first. "
            f"Bridge path: {path}. Original error: {exc}"
        ) from exc


class CompanyDdsBus:
    """Publish and receive Python payloads through company TopicAPI/FastDDS."""

    def __init__(self, source: str = "node", native=None):
        self.source = source
        self._native = native or require_company_dds()
        self._writers = set()
        self._readers = set()
        self._initial_buffer_size = 256 * 1024

    @staticmethod
    def _topic_bytes(topic_name: str):
        return topic_name.encode("utf-8")

    def writer(self, topic_name: str):
        if topic_name not in self._writers:
            rc = self._native.company_dds_create_writer(
                self._topic_bytes(topic_name), 0
            )
            if rc != 0:
                raise RuntimeError(
                    f"Company DDS createWriter({topic_name!r}) failed: {rc}"
                )
            self._writers.add(topic_name)
        return topic_name

    def reader(self, topic_name: str):
        if topic_name not in self._readers:
            rc = self._native.company_dds_create_reader(
                self._topic_bytes(topic_name), 0
            )
            if rc != 0:
                raise RuntimeError(
                    f"Company DDS createReader({topic_name!r}) failed: {rc}"
                )
            self._readers.add(topic_name)
        return topic_name

    def write(
        self,
        topic_name: str,
        payload: Dict[str, Any],
        *,
        kind: str = "",
        tick: Optional[int] = None,
        source: Optional[str] = None,
    ):
        wire = {
            "source": source or self.source,
            "kind": kind or str(payload.get("type", "")),
            "tick": int(payload.get("tick", 0) if tick is None else tick),
            "stamp": time.time(),
            "payload": clean(payload),
        }
        data = json.dumps(
            wire, ensure_ascii=False, separators=(",", ":")
        ).encode("utf-8")
        self.write_bytes(topic_name, data)

    def write_bytes(self, topic_name: str, data: bytes):
        self.writer(topic_name)
        if not data:
            raise ValueError("Company DDS cannot publish an empty payload")
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        rc = self._native.company_dds_publish(
            self._topic_bytes(topic_name), buf, len(data)
        )
        if rc != 0:
            raise RuntimeError(
                f"Company DDS publish({topic_name!r}) failed: {rc}"
            )

    def _poll_bytes(self, topic_name: str, timeout_ms: int = 0):
        self.reader(topic_name)
        capacity = self._initial_buffer_size
        while True:
            buf = (ctypes.c_uint8 * capacity)()
            actual = ctypes.c_uint32()
            rc = self._native.company_dds_poll(
                self._topic_bytes(topic_name),
                buf,
                capacity,
                ctypes.byref(actual),
                timeout_ms,
            )
            if rc == -2 and actual.value > capacity:
                capacity = actual.value
                timeout_ms = 0
                continue
            if rc < 0:
                raise RuntimeError(
                    f"Company DDS poll({topic_name!r}) failed: {rc}"
                )
            if rc == 0:
                return None
            return bytes(buf[: actual.value])

    def read_bytes(
        self,
        topic_name: str,
        *,
        timeout_ms: int = 0,
        max_samples: int = 1024,
    ) -> List[bytes]:
        out = []
        for index in range(max_samples):
            sample = self._poll_bytes(
                topic_name, timeout_ms=timeout_ms if index == 0 else 0
            )
            if sample is None:
                break
            out.append(sample)
        return out

    def read(self, topic_name: str) -> List[Dict[str, Any]]:
        out = []
        for sample in self.read_bytes(topic_name):
            try:
                wire = json.loads(sample.decode("utf-8"))
                payload = wire["payload"]
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError):
                continue
            if not isinstance(payload, dict):
                continue
            payload.setdefault(
                "_company_dds",
                {
                    "source": wire.get("source"),
                    "kind": wire.get("kind"),
                    "tick": wire.get("tick"),
                    "stamp": wire.get("stamp"),
                    "topic": topic_name,
                },
            )
            out.append(payload)
        return out

    def publish_official_fusion(
        self,
        fused_tracks,
        *,
        origin,
        sensors_info: Mapping[str, Mapping],
        timestamp_ms: int = 0,
        source_address: int = 201,
    ):
        rows = list(fused_tracks)[:255]
        array = (CompanyFusionTargetData * len(rows))()
        ref_lng, ref_lat, ref_alt = [float(x) for x in origin]
        m_per_deg_lng = 111320.0 * math.cos(math.radians(ref_lat))

        for i, track in enumerate(rows):
            x = float(track.get("x", 0.0))
            y = float(track.get("y", 0.0))
            z = float(track.get("z", 0.0))
            vx = float(track.get("vx", 0.0))
            vy = float(track.get("vy", 0.0))
            speed = math.hypot(vx, vy)
            # fusion_proto heading: north=0, clockwise positive.
            heading = math.degrees(math.atan2(vx, vy)) % 360.0

            array[i].global_id = _protocol_id(track.get("id"))
            array[i].lng_e7 = round((ref_lng + x / m_per_deg_lng) * 1e7)
            array[i].lat_e7 = round((ref_lat + y / 111320.0) * 1e7)
            array[i].alt_dm = max(0, round((ref_alt + z) * 10.0))
            array[i].pitch_cdeg = 0
            array[i].roll_cdeg = 0
            array[i].yaw_cdeg = min(35999, max(0, round(heading * 100.0)))
            array[i].speed_cms = max(0, round(speed * 100.0))
            array[i].fusion_type = _fusion_target_type(track.get("cls"))
            array[i].confidence = min(
                100, max(0, round(float(track.get("conf", 0.0)) * 100.0))
            )
            array[i].source_mask = _source_mask(track, sensors_info)
            array[i].iff = 2  # Company scenarios model incoming hostile UAVs.

        ptr = array if rows else None
        rc = self._native.company_dds_publish_fusion(
            Topics.OFFICIAL_FUSION.name.encode("utf-8"),
            source_address,
            int(timestamp_ms),
            ptr,
            len(rows),
        )
        if rc != 0:
            raise RuntimeError(f"Official FusionMultiTarget publish failed: {rc}")

def _protocol_id(value) -> int:
    try:
        value = int(value)
    except (TypeError, ValueError):
        return 1
    return ((value - 1) % 255) + 1


def _fusion_target_type(value) -> int:
    """Map normalized classes to fusion_proto.h TargetType values."""
    return {
        "drone": 0,
        "quad": 0,
        "fixed_wing": 0,
        "heli": 0,
        "bird": 1,
        "large_target": 2,
        "vehicle": 3,
        "person": 4,
        "other": 5,
        "unknown": 0xFF,
    }.get(str(value or "unknown"), 0xFF)


_SOURCE_MASKS = {
    frozenset({"radar"}): 0,
    frozenset({"eo"}): 1,
    frozenset({"es"}): 2,
    frozenset({"acoustic"}): 3,
    frozenset({"radar", "eo"}): 4,
    frozenset({"radar", "es"}): 5,
    frozenset({"radar", "acoustic"}): 6,
    frozenset({"eo", "es"}): 7,
    frozenset({"eo", "acoustic"}): 8,
    frozenset({"es", "acoustic"}): 9,
    frozenset({"radar", "eo", "es"}): 10,
    frozenset({"radar", "eo", "acoustic"}): 11,
    frozenset({"eo", "es", "acoustic"}): 12,
    frozenset({"radar", "eo", "es", "acoustic"}): 13,
}


def _source_mask(track, sensors_info: Mapping[str, Mapping]) -> int:
    kinds = set()
    for source in track.get("sources", []):
        if source == "radar":
            kinds.add("radar")
            continue
        kind = sensors_info.get(source, {}).get("kind")
        if kind == "rf_df":
            kinds.add("es")
        elif kind in ("radar", "eo", "acoustic"):
            kinds.add(kind)
    return _SOURCE_MASKS.get(frozenset(kinds), 0xFF)
