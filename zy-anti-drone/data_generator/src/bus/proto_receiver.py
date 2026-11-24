"""DDS 协议接收端 — poll 模式从 DDS 获取 dev_proto 帧。

通过 ctypes 调用 libad_proto_receiver.so，内部用线程安全队列缓冲。
支持阻塞 poll 和非阻塞检查。

用法:
    rx = ProtoReceiver()
    rx.init()

    # 阻塞等待一帧 (100ms 超时)
    frame = rx.poll(timeout_ms=100)
    if frame:
        print(f"收到 {frame.frame_type:#x}")

    # 解析为结构化数据
    radar = rx.parse_radar(frame.data, frame.data_len)

    rx.close()
"""

from __future__ import annotations

import ctypes
import logging
import os
import struct
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# ============================================================
# ctypes 结构体
# ============================================================

AD_RX_MAX_FRAME = 2048


class AdRxFrame(ctypes.Structure):
    _fields_ = [
        ("frame_type",   ctypes.c_uint16),
        ("device_id",    ctypes.c_uint32),
        ("timestamp_ms", ctypes.c_uint64),
        ("data_len",     ctypes.c_uint32),
        ("data",         ctypes.c_uint8 * AD_RX_MAX_FRAME),
    ]


# ============================================================
# 帧解析结果 (纯 Python，不依赖 C++)
# ============================================================

class RadarTarget:
    """解析后的雷达目标帧。"""
    __slots__ = (
        "track_id", "device_id", "timestamp_ms",
        "range_m", "azimuth_deg", "elevation_deg",
        "height_m", "radial_vel_ms",
        "lng_deg", "lat_deg", "alt_m",
        "speed_ms", "course_deg", "extrap_flag",
        "target_type", "confidence",
    )

    def __repr__(self):
        return (f"<Radar tid={self.track_id} r={self.range_m:.0f}m "
                f"az={self.azimuth_deg:.1f}° lng={self.lng_deg:.6f}°>")


class ESMTarget:
    """解析后的频侦目标帧。"""
    __slots__ = (
        "target_id", "device_id", "timestamp_ms",
        "sn", "brand_model",
        "lng_deg", "lat_deg", "alt_m", "height_m",
        "speed_ms", "distance_m",
        "rctrl_lng_deg", "rctrl_lat_deg",
        "freq_hz", "bandw_hz", "power_dbm",
        "azimuth_deg", "discovery_time_ms", "confidence",
    )

    def __repr__(self):
        return (f"<ESM tid={self.target_id} freq={self.freq_hz/1e9:.2f}GHz "
                f"az={self.azimuth_deg:.1f}°>")


class EOTarget:
    """解析后的光电目标帧。"""
    __slots__ = (
        "target_id", "device_id", "timestamp_ms",
        "fault_status", "track_status", "track_count", "camfield",
        "ptz_lng_deg", "ptz_lat_deg", "ptz_alt_m",
        "ptz_roll_deg", "ptz_pitch_deg", "ptz_yaw_deg", "ptz_zeroazi_deg",
        "target_lng_deg", "target_lat_deg", "target_alt_m",
        "target_dis_m", "target_azimuth_deg", "target_elevation_deg",
        "target_type", "confidence",
    )

    def __repr__(self):
        return (f"<EO tid={self.target_id} az={self.ptz_yaw_deg:.1f}° "
                f"track={'Y' if self.track_status else 'N'}>")


class AcousticTarget:
    """解析后的声探测目标帧。"""
    __slots__ = (
        "target_id", "device_id", "timestamp_ms",
        "point_count",
        "device_lng_deg", "device_lat_deg", "device_alt_m",
        "device_roll_deg", "device_pitch_deg", "device_yaw_deg",
        "target_roll_deg", "target_pitch_deg", "target_yaw_deg",
        "target_type", "confidence",
    )

    def __repr__(self):
        return (f"<Acoustic tid={self.target_id} "
                f"az={self.target_yaw_deg:.1f}°>")


# ============================================================
# 帧解析函数 — 按 dev_proto.h 线格式手动解析
# ============================================================
# 帧格式: DevFrameHeader(23B) + Body(NB) + DevFrameTail(4B)
#   Header: magic(4) ver(1) ftype(2) dtype(2) len(2) dev(4) ts(8) = 23B
#   Tail:   crc32(4) = 4B

_HEADER_FMT = "<I B H H H I Q"   # 23 bytes
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)


def _parse_header(data: bytes, offset: int = 0):
    """解析 DevFrameHeader。返回 (header_dict, body_offset)。"""
    magic, ver, ftype, dtype, length, dev_id, ts = struct.unpack_from(
        _HEADER_FMT, data, offset)
    return {
        "magic": magic, "proto_ver": ver, "frame_type": ftype,
        "data_type": dtype, "length": length,
        "device_id": dev_id, "timestamp_ms": ts,
    }, offset + _HEADER_SIZE


# ── 雷达 66B body: 39B ──
_RADAR_BODY_FMT = "<I I h h I h i i I I H B B B"
_RADAR_BODY_SIZE = struct.calcsize(_RADAR_BODY_FMT)


def parse_radar_frame(data: bytes, data_len: int,
                      device_id: int = 0) -> Optional[RadarTarget]:
    """将 dev_proto 雷达帧解析为 RadarTarget。"""
    if data_len < _HEADER_SIZE + _RADAR_BODY_SIZE + 4:
        return None
    hdr, off = _parse_header(data)
    body = struct.unpack_from(_RADAR_BODY_FMT, data, off)

    r = RadarTarget()
    r.device_id     = hdr["device_id"]
    r.timestamp_ms  = hdr["timestamp_ms"]
    r.track_id      = body[0]
    r.range_m       = body[1] * 0.1
    r.azimuth_deg   = body[2] * 0.01
    r.elevation_deg = body[3] * 0.01
    r.height_m      = body[4] * 0.1
    r.radial_vel_ms = body[5] * 0.1
    r.lng_deg       = body[6] * 1e-7
    r.lat_deg       = body[7] * 1e-7
    r.alt_m         = body[8] * 0.1
    r.speed_ms      = body[9] * 0.1
    r.course_deg    = body[10] * 0.01
    r.extrap_flag   = body[11]
    r.target_type   = body[12]
    r.confidence    = body[13]
    return r


# ── 频侦 139B body: 112B ──
_ESM_BODY_FMT = "<I 16s 32s i i I I I I i i I I h H Q B 7s"
_ESM_BODY_SIZE = struct.calcsize(_ESM_BODY_FMT)


def parse_esm_frame(data: bytes, data_len: int,
                    device_id: int = 0) -> Optional[ESMTarget]:
    """将 dev_proto 频侦帧解析为 ESMTarget。"""
    if data_len < _HEADER_SIZE + _ESM_BODY_SIZE + 4:
        return None
    hdr, off = _parse_header(data)
    body = struct.unpack_from(_ESM_BODY_FMT, data, off)

    e = ESMTarget()
    e.device_id     = hdr["device_id"]
    e.timestamp_ms  = hdr["timestamp_ms"]
    e.target_id     = body[0]
    e.sn            = body[1].rstrip(b'\0').decode('utf-8', errors='replace')
    e.brand_model   = body[2].rstrip(b'\0').decode('utf-8', errors='replace')
    e.lng_deg       = body[3] * 1e-7
    e.lat_deg       = body[4] * 1e-7
    e.alt_m         = body[5] * 0.1
    e.height_m      = body[6] * 0.1
    e.speed_ms      = body[7] * 0.1
    e.distance_m    = body[8] * 0.1
    e.rctrl_lng_deg = body[9] * 1e-7
    e.rctrl_lat_deg = body[10] * 1e-7
    e.freq_hz       = body[11]
    e.bandw_hz      = body[12]
    e.power_dbm     = body[13] * 0.1
    e.azimuth_deg   = body[14] * 0.01
    e.discovery_time_ms = body[15]
    e.confidence    = body[16]
    return e


# ── 光电 79B body: 52B ──
_EO_BODY_FMT = "<I B B B B H H h H i i I i i I I H h B B 2s"
_EO_BODY_SIZE = struct.calcsize(_EO_BODY_FMT)


def parse_eo_frame(data: bytes, data_len: int,
                   device_id: int = 0) -> Optional[EOTarget]:
    """将 dev_proto 光电帧解析为 EOTarget。"""
    if data_len < _HEADER_SIZE + _EO_BODY_SIZE + 4:
        return None
    hdr, off = _parse_header(data)
    body = struct.unpack_from(_EO_BODY_FMT, data, off)

    eo = EOTarget()
    eo.device_id       = hdr["device_id"]
    eo.timestamp_ms    = hdr["timestamp_ms"]
    eo.target_id       = body[0]
    eo.fault_status    = body[1]
    eo.track_status    = body[2]
    eo.track_count     = body[3]
    eo.camfield        = body[4]
    eo.ptz_zeroazi_deg = body[5] * 0.01
    eo.ptz_roll_deg    = body[6] * 0.01
    eo.ptz_pitch_deg   = body[7] * 0.01
    eo.ptz_yaw_deg     = body[8] * 0.01
    eo.ptz_lng_deg     = body[9] * 1e-7
    eo.ptz_lat_deg     = body[10] * 1e-7
    eo.ptz_alt_m       = body[11] * 0.1
    eo.target_lng_deg  = body[12] * 1e-7
    eo.target_lat_deg  = body[13] * 1e-7
    eo.target_alt_m    = body[14] * 0.1
    eo.target_dis_m    = body[15] * 0.1
    eo.target_azimuth_deg  = body[16] * 0.01
    eo.target_elevation_deg = body[17] * 0.01
    eo.target_type     = body[18]
    eo.confidence      = body[19]
    return eo


# ── 声探测 67B body: 40B ──
_AC_BODY_FMT = "<I B 3s h h h 2s H H H B B 4s i i I"
_AC_BODY_SIZE = struct.calcsize(_AC_BODY_FMT)


def parse_acoustic_frame(data: bytes, data_len: int,
                         device_id: int = 0) -> Optional[AcousticTarget]:
    """将 dev_proto 声探测帧解析为 AcousticTarget。"""
    if data_len < _HEADER_SIZE + _AC_BODY_SIZE + 4:
        return None
    hdr, off = _parse_header(data)
    body = struct.unpack_from(_AC_BODY_FMT, data, off)

    a = AcousticTarget()
    a.device_id      = hdr["device_id"]
    a.timestamp_ms   = hdr["timestamp_ms"]
    a.target_id      = body[0]
    a.point_count    = body[1]
    a.device_roll_deg  = body[3] * 0.01
    a.device_pitch_deg = body[4] * 0.01
    a.device_yaw_deg   = body[5] * 0.01
    a.target_roll_deg  = body[7] * 0.01
    a.target_pitch_deg = body[8] * 0.01
    a.target_yaw_deg   = body[9] * 0.01
    a.target_type   = body[10]
    a.confidence    = body[11]
    a.device_lng_deg = body[13] * 1e-7
    a.device_lat_deg = body[14] * 1e-7
    a.device_alt_m   = body[15] * 0.1
    return a


# ── 帧类型 → 解析函数映射 ──
_PARSE_MAP = {
    0x1A01: parse_radar_frame,
    0x1B01: parse_esm_frame,
    0x1C01: parse_eo_frame,
    0x1D01: parse_acoustic_frame,
}


def parse_frame(frame_type: int, data: bytes, data_len: int,
                device_id: int = 0):
    """通用帧解析。返回 RadarTarget / ESMTarget / EOTarget / AcousticTarget 或 None。"""
    parser = _PARSE_MAP.get(frame_type)
    if parser:
        return parser(data, data_len, device_id)
    return None


# ============================================================
# ProtoReceiver — ctypes 封装
# ============================================================

class ProtoReceiver:
    """DDS 协议接收器 (poll 模式)。"""

    def __init__(self, lib_path: str | None = None):
        if lib_path is None:
            pkg_root = Path(__file__).parent.parent.parent     # data_generator/
            prj_root = Path(__file__).parent.parent.parent     # 项目根
            candidates = [
                prj_root / "bridge/libad_proto_receiver.so",
                pkg_root / "thirdparty/bridge/libad_proto_receiver.so",
            ]
            lib_path = None
            for c in candidates:
                if c.exists():
                    lib_path = str(c.resolve())
                    break

        if lib_path is None:
            self._available = False
            logger.warning("接收库 libad_proto_receiver.so 未找到")
            return

        try:
            # 设置 LD_LIBRARY_PATH 以便找到依赖的 .so
            pkg_root = Path(__file__).parent.parent.parent
            prj_root = Path(__file__).parent.parent.parent.parent
            if (pkg_root / "thirdparty").exists():
                extra_dir = pkg_root / "thirdparty/linux-x86/lib"
            else:
                extra_dir = prj_root / "thirdparty/linux-x86/lib"
            existing = os.environ.get('LD_LIBRARY_PATH', '')
            os.environ['LD_LIBRARY_PATH'] = f'{extra_dir.resolve()}:{existing}'

            self._lib = ctypes.CDLL(lib_path)
            self._setup_sigs()
            self._available = True
            logger.info(f"接收库已加载: {lib_path}")
        except Exception as e:
            self._available = False
            logger.warning(f"接收库加载失败: {e}")

        self._initialized = False

    def _setup_sigs(self):
        L = self._lib

        L.ad_proto_receive_init.restype = ctypes.c_int
        L.ad_proto_receive_init.argtypes = []

        L.ad_proto_receive_poll.restype = ctypes.c_int
        L.ad_proto_receive_poll.argtypes = [
            ctypes.POINTER(AdRxFrame), ctypes.c_int,
        ]

        L.ad_proto_receive_available.restype = ctypes.c_int
        L.ad_proto_receive_available.argtypes = []

        L.ad_proto_receive_close.restype = None
        L.ad_proto_receive_close.argtypes = []

    @property
    def available(self) -> bool:
        return self._available

    def init(self) -> bool:
        """初始化 DDS 接收端。"""
        if not self.available:
            return False
        rc = self._lib.ad_proto_receive_init()
        self._initialized = (rc == 0)
        if self._initialized:
            logger.info("DDS 接收端已就绪")
        return self._initialized

    def poll(self, timeout_ms: int = 100) -> Optional[AdRxFrame]:
        """阻塞 poll 一帧。

        Args:
            timeout_ms: 超时 (ms)，-1=永久阻塞，0=立即返回

        Returns:
            AdRxFrame 或 None (超时/无数据)
        """
        if not self._initialized:
            return None
        frame = AdRxFrame()
        rc = self._lib.ad_proto_receive_poll(ctypes.byref(frame), timeout_ms)
        if rc <= 0:
            return None
        return frame

    def poll_parsed(self, timeout_ms: int = 100):
        """poll 并自动解析为结构化对象。

        Returns:
            RadarTarget / ESMTarget / EOTarget / AcousticTarget 或 None
        """
        frame = self.poll(timeout_ms)
        if frame is None:
            return None
        raw = bytes(frame.data[:frame.data_len])
        return parse_frame(frame.frame_type, raw, frame.data_len, frame.device_id)

    def available_count(self) -> int:
        """队列中待取帧数量。"""
        if not self._initialized:
            return 0
        return self._lib.ad_proto_receive_available()

    def close(self):
        if self.available and self._initialized:
            self._lib.ad_proto_receive_close()
            self._initialized = False

    def __enter__(self):
        self.init()
        return self

    def __exit__(self, *args):
        self.close()

    # ── 便捷方法 ──

    def iter_frames(self, timeout_ms: int = 100):
        """生成器：持续 poll 并解析，直到被中断。"""
        while True:
            obj = self.poll_parsed(timeout_ms)
            if obj is None:
                continue
            yield obj

    def drain(self) -> list:
        """一次性取出队列中所有帧（非阻塞）。"""
        frames = []
        while True:
            obj = self.poll_parsed(timeout_ms=0)
            if obj is None:
                break
            frames.append(obj)
        return frames
