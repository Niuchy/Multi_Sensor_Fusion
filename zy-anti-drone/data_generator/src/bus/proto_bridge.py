"""标准化协议桥接器 — dev_proto.h 二进制帧输出。

将仿真数据按 dev_proto.h 定义的固定长度二进制帧序列化，通过 DDS 发送。

协议帧格式：
- 雷达: 66 字节, FrameType=0x1A01, Magic=A55AA55A
- 频侦: 139 字节, FrameType=0x1B01, Magic=B44BB44B
- 光电: 79 字节, FrameType=0x1C01, Magic=C33CC33C
- 声探测: 67 字节, FrameType=0x1D01, Magic=D22DD22D

多目标批量模式：
  所有匹配的航迹/检测打包为一条 PXSB 多单元报文，
  一次 FFI 调用 → 一次 DDS publish，避免逐帧发送。
"""

import ctypes
import math
import logging
import time
from pathlib import Path
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


# ============================================================
# ctypes 结构体 —— 与 ad_proto_bridge.cpp 中 POD 结构体一一对应
# ============================================================

class RadarTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("track_id",       ctypes.c_uint32),
        ("extrap_flag",    ctypes.c_uint8),
        ("range_dm",       ctypes.c_uint32),
        ("azi_cdeg",       ctypes.c_int16),
        ("ele_cdeg",       ctypes.c_int16),
        ("height_dm",      ctypes.c_uint32),
        ("radial_vel_dms", ctypes.c_int16),
        ("lng_e7",         ctypes.c_int32),
        ("lat_e7",         ctypes.c_int32),
        ("alt_dm",         ctypes.c_uint32),
        ("speed_dms",      ctypes.c_uint32),
        ("course_cdeg",    ctypes.c_uint16),
        ("target_type",    ctypes.c_uint8),
        ("confidence",     ctypes.c_uint8),
    ]


class FreqDetectTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id",         ctypes.c_uint32),
        ("sn",                ctypes.c_char * 16),
        ("brand_model",       ctypes.c_char * 32),
        ("lng_e7",            ctypes.c_int32),
        ("lat_e7",            ctypes.c_int32),
        ("alt_dm",            ctypes.c_uint32),
        ("height_dm",         ctypes.c_uint32),
        ("speed_dms",         ctypes.c_uint32),
        ("distance_dm",       ctypes.c_uint32),
        ("rctrl_lng_e7",      ctypes.c_int32),
        ("rctrl_lat_e7",      ctypes.c_int32),
        ("freq_hz",           ctypes.c_uint32),
        ("bandw_hz",          ctypes.c_uint32),
        ("power_db",          ctypes.c_int16),
        ("azimuth_cdeg",      ctypes.c_uint16),
        ("discovery_time_ms", ctypes.c_uint64),
        ("confidence",        ctypes.c_uint8),
    ]


class CamTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id",           ctypes.c_uint32),
        ("fault_status",        ctypes.c_uint8),
        ("track_status",        ctypes.c_uint8),
        ("track_count",         ctypes.c_uint8),
        ("camfield",            ctypes.c_uint8),
        ("ptz_zeroazi",         ctypes.c_uint16),
        ("ptz_roll",            ctypes.c_uint16),
        ("ptz_pitch",           ctypes.c_int16),
        ("ptz_yaw",             ctypes.c_uint16),
        ("ptz_lng_e7",          ctypes.c_int32),
        ("ptz_lat_e7",          ctypes.c_int32),
        ("ptz_alt_dm",          ctypes.c_uint32),
        ("target_lng_e7",       ctypes.c_int32),
        ("target_lat_e7",       ctypes.c_int32),
        ("target_alt_dm",       ctypes.c_uint32),
        ("target_dis_dm",       ctypes.c_uint32),
        ("target_azimuth_cdeg", ctypes.c_uint16),
        ("target_elevation_cdeg", ctypes.c_int16),
        ("target_type",         ctypes.c_uint8),
        ("confidence",          ctypes.c_uint8),
    ]


class AcousticTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id",     ctypes.c_uint32),
        ("point_count",   ctypes.c_uint8),
        ("device_roll",   ctypes.c_int16),
        ("device_pitch",  ctypes.c_int16),
        ("device_yaw",    ctypes.c_int16),
        ("target_roll",   ctypes.c_uint16),
        ("target_pitch",  ctypes.c_uint16),
        ("target_yaw",    ctypes.c_uint16),
        ("target_type",   ctypes.c_uint8),
        ("confidence",    ctypes.c_uint8),
        ("device_lng_e7", ctypes.c_int32),
        ("device_lat_e7", ctypes.c_int32),
        ("device_alt_dm", ctypes.c_uint32),
    ]

# ============================================================
# ENU → WGS84 坐标转换
# ============================================================

# WGS84 椭球参数
WGS84_A = 6378137.0          # 长半轴 (m)
WGS84_F = 1.0 / 298.257223563  # 扁率
METERS_PER_DEG_LAT = 111320.0  # 每度纬度 ≈ 111.32 km


def meters_per_deg_lng(lat_deg: float) -> float:
    """给定纬度下每度经度的米数。"""
    return METERS_PER_DEG_LAT * math.cos(math.radians(lat_deg))


class CoordConverter:
    """ENU (局部直角坐标) → WGS84 (经纬高) 转换器。

    使用球面近似，精度足以覆盖 <100km 范围的仿真场景。
    """

    def __init__(self, ref_lng: float = 120.08363526778648, ref_lat: float = 30.13911824931559, ref_alt: float = 0.0):
        """
        Args:
            ref_lng: ENU 原点经度 (度)
            ref_lat: ENU 原点纬度 (度)
            ref_alt: ENU 原点海拔 (m)
        """
        self.ref_lng = ref_lng
        self.ref_lat = ref_lat
        self.ref_alt = ref_alt
        self._m_per_deg_lng = meters_per_deg_lng(ref_lat)

    def enu_to_wgs84(self, x: float, y: float, z: float):
        """ENU (东-北-天, 米) → WGS84 (1e-7°, 1e-7°, 0.1m)。

        Returns:
            (lng_e7, lat_e7, alt_dm)
        """
        lng_deg = self.ref_lng + x / self._m_per_deg_lng
        lat_deg = self.ref_lat + y / METERS_PER_DEG_LAT
        alt_m = self.ref_alt + z
        return (
            int(round(lng_deg * 1e7)),
            int(round(lat_deg * 1e7)),
            int(round(alt_m * 10)),
        )

    def sensor_to_wgs84(self, sensor_enu, target_rel):
        """传感器ENU位置 + 目标相对位置 → 目标WGS84。

        Args:
            sensor_enu: [sx, sy, sz] 传感器 ENU 坐标
            target_rel: [dx, dy, dz] 目标相对传感器位置

        Returns:
            (lng_e7, lat_e7, alt_dm)
        """
        tx = sensor_enu[0] + target_rel[0]
        ty = sensor_enu[1] + target_rel[1]
        tz = sensor_enu[2] + target_rel[2]
        return self.enu_to_wgs84(tx, ty, tz)


# ============================================================
# ctypes 桥接
# ============================================================

class ProtoBridge:
    """dev_proto.h 协议 DDS 桥接器。"""

    def __init__(self, lib_path: str | None = None):
        if lib_path is None:
            # src/bus/proto_bridge.py → data_generator/ → 项目根 = 上溯 3~4 级
            pkg_root = Path(__file__).parent.parent.parent       # data_generator/
            prj_root = Path(__file__).parent.parent.parent.parent # 项目根
            candidates = [
                prj_root / "bridge/libad_proto.so",               # 源码树 (bridge/ 在项目根)
                pkg_root / "bridge/libad_proto.so",               # 迁移包 (bridge/ 在包根)
                pkg_root / "thirdparty/bridge/libad_proto.so",    # 旧版迁移包 (兼容)
            ]
            lib_path = None
            for c in candidates:
                if c.exists():
                    lib_path = str(c.resolve())
                    break

        if lib_path is None:
            self._available = False
            logger.warning("协议桥接库 libad_proto.so 未找到")
            return

        try:
            import os
            pkg_root = Path(__file__).parent.parent.parent
            prj_root = Path(__file__).parent.parent.parent.parent
            # 检测 thirdparty 位置
            if (pkg_root / "thirdparty").exists():
                extra_dir = pkg_root / "thirdparty/linux-x86/lib"
            else:
                extra_dir = prj_root / "thirdparty/linux-x86/lib"
            existing = os.environ.get('LD_LIBRARY_PATH', '')
            os.environ['LD_LIBRARY_PATH'] = f'{extra_dir.resolve()}:{existing}'

            self._lib = ctypes.CDLL(lib_path)
            self._setup_sigs()
            self._available = True
            logger.info(f"协议桥接库已加载: {lib_path}")
        except Exception as e:
            self._available = False
            logger.warning(f"协议桥接库加载失败: {e}")

        self._initialized = False
        self._coord = CoordConverter()

    def _setup_sigs(self):
        L = self._lib
        L.ad_proto_version.restype = ctypes.c_char_p

        L.ad_proto_init.restype = ctypes.c_int
        L.ad_proto_init.argtypes = []

        L.ad_proto_send_radar.restype = ctypes.c_int
        L.ad_proto_send_radar.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64,  # device_id, timestamp_ms
            ctypes.c_uint32, ctypes.c_uint8,    # track_id, extrap_flag
            ctypes.c_uint32, ctypes.c_int16, ctypes.c_int16,  # range, azi, ele
            ctypes.c_uint32, ctypes.c_int16,    # height, radial_vel
            ctypes.c_int32, ctypes.c_int32, ctypes.c_uint32,  # lng, lat, alt
            ctypes.c_uint32, ctypes.c_uint16,   # speed, course
            ctypes.c_uint8, ctypes.c_uint8,     # type, confidence
        ]

        L.ad_proto_send_freqdetect.restype = ctypes.c_int
        L.ad_proto_send_freqdetect.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint32,
            ctypes.c_char_p, ctypes.c_char_p,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint32,
            ctypes.c_int32, ctypes.c_int32,
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int16,
            ctypes.c_uint16, ctypes.c_uint64, ctypes.c_uint8,
        ]

        L.ad_proto_send_camera.restype = ctypes.c_int
        L.ad_proto_send_camera.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint32,
            ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint8,
            ctypes.c_uint16, ctypes.c_uint16, ctypes.c_int16, ctypes.c_uint16,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_uint32,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_uint32,
            ctypes.c_uint32, ctypes.c_uint16, ctypes.c_int16,
            ctypes.c_uint8, ctypes.c_uint8,
        ]

        L.ad_proto_send_acoustic.restype = ctypes.c_int
        L.ad_proto_send_acoustic.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64, ctypes.c_uint32, ctypes.c_uint8,
            ctypes.c_int16, ctypes.c_int16, ctypes.c_int16,
            ctypes.c_uint16, ctypes.c_uint16, ctypes.c_uint16,
            ctypes.c_uint8, ctypes.c_uint8,
            ctypes.c_int32, ctypes.c_int32, ctypes.c_uint32,  # device lng/lat/alt
        ]

        L.ad_proto_ready.restype = ctypes.c_int
        L.ad_proto_close.restype = None

        # ── 批量接口 ──

        L.ad_proto_send_radar_batch.restype = ctypes.c_int
        L.ad_proto_send_radar_batch.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64,
            ctypes.POINTER(RadarTargetData), ctypes.c_uint32,
        ]

        L.ad_proto_send_freqdetect_batch.restype = ctypes.c_int
        L.ad_proto_send_freqdetect_batch.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64,
            ctypes.POINTER(FreqDetectTargetData), ctypes.c_uint32,
        ]

        L.ad_proto_send_camera_batch.restype = ctypes.c_int
        L.ad_proto_send_camera_batch.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64,
            ctypes.POINTER(CamTargetData), ctypes.c_uint32,
        ]

        L.ad_proto_send_acoustic_batch.restype = ctypes.c_int
        L.ad_proto_send_acoustic_batch.argtypes = [
            ctypes.c_uint32, ctypes.c_uint64,
            ctypes.POINTER(AcousticTargetData), ctypes.c_uint32,
        ]

    @property
    def available(self) -> bool:
        return self._available

    def init(self, ref_lng=120.08363526778648, ref_lat=30.13911824931559, ref_alt=0.0) -> bool:
        """初始化协议桥接（DDS 参数全部使用 third_party 库默认值）。

        Args:
            ref_lng/lat/alt: ENU 原点对应的 WGS84 坐标（度, 度, 米）
        """
        if not self.available:
            return False
        self._coord = CoordConverter(ref_lng, ref_lat, ref_alt)
        rc = self._lib.ad_proto_init()
        self._initialized = (rc == 0)
        if self._initialized:
            logger.info(f"协议桥接已就绪，ENU原点: ({ref_lng}°E, {ref_lat}°N, {ref_alt}m)")
        return self._initialized

    def close(self):
        if self.available and self._initialized:
            self._lib.ad_proto_close()
            self._initialized = False

    @property
    def ready(self) -> bool:
        return self._initialized and self._lib.ad_proto_ready() == 1

    # ── 转换工具 ──

    def _ts_ms(self, sim_time: float) -> int:
        """仿真时间 → Unix 毫秒时间戳（简化：以仿真起始为基准）。"""
        return int((time.time() - sim_time + sim_time) * 1000)

    def _pack_enu(self, x, y, z):
        """ENU 坐标 → (lng_e7, lat_e7, alt_dm)。"""
        return self._coord.enu_to_wgs84(x, y, z)

    # ── 发送接口 ──

    def send_radar(self, device_id: int, sim_time: float,
                   track: dict, sensor_enu, target_enu) -> bool:
        """发送雷达目标帧。

        Args:
            device_id: 设备编号
            sim_time: 仿真时间 (s)
            track: 航迹字典 (含 track_id, 测量值, 状态等)
            sensor_enu: [sx, sy, sz] 传感器ENU坐标
            target_enu: [tx, ty, tz] 目标ENU坐标
        """
        ts = int(sim_time * 1000)
        tid = track.get('track_id', 0)
        tag = 0 if track.get('state') == 'coasting' else 1

        # L1 量测 (雷达局部坐标)
        sx, sy, sz = sensor_enu
        tx, ty, tz = target_enu
        dx, dy, dz = tx - sx, ty - sy, tz - sz
        r = math.sqrt(dx*dx + dy*dy + dz*dz)
        az = math.degrees(math.atan2(dy, dx)) % 360
        el = math.degrees(math.atan2(dz, math.sqrt(dx*dx + dy*dy)))
        vr = track.get('doppler', 0)

        range_dm = int(r * 10)
        azi_cdeg = int(az * 100)
        ele_cdeg = int(el * 100)
        height_dm = int(dz * 10)
        vr_dms = int(vr * 10)

        # L2 航迹 (WGS84)
        lng_e7, lat_e7, alt_dm = self._pack_enu(tx, ty, tz)

        # 速度和航向
        speed_dms = int(track.get('speed', 0) * 10)
        course_cdeg = int(track.get('heading', 0) * 100)

        # L3 识别
        target_type = 2  # DRONE
        confidence = int(track.get('confidence', 0.8) * 100)

        logger.info(f"[DDS] → RadarTarget tid={tid} r={r:.0f}m az={az:.1f}° lng={lng_e7/1e7:.5f}E")
        return self._lib.ad_proto_send_radar(
            device_id, ts, tid, tag,
            range_dm, azi_cdeg, ele_cdeg, height_dm, vr_dms,
            lng_e7, lat_e7, alt_dm, speed_dms, course_cdeg,
            target_type, confidence,
        ) == 0

    def send_esm(self, device_id: int, sim_time: float,
                 detection: dict, sensor_enu, target_enu, target) -> bool:
        """发送频侦(电侦)目标帧。"""
        ts = int(sim_time * 1000)
        tid = detection.get('track_id', 0)

        tx, ty, tz = target_enu
        sx, sy, sz = sensor_enu
        dx, dy, dz = tx - sx, ty - sy, tz - sz
        r = math.sqrt(dx*dx + dy*dy + dz*dz)

        lng_e7, lat_e7, alt_dm = self._pack_enu(tx, ty, tz)

        emitter = target.emitter if hasattr(target, 'emitter') else None
        sn = (target.label or b"").encode('utf-8')[:15] if hasattr(target, 'label') else b""
        brand = (f"{target.target_type}" if hasattr(target, 'target_type') else "UAV").encode('utf-8')[:31]
        freq_hz = int(emitter.freq) if emitter and emitter.freq else 2400000000
        az_cdeg = int((detection.get('azimuth', 0) or 0) * 100)
        conf = int((detection.get('confidence', 0.5) or 0.5) * 100)

        logger.info(f"[DDS] → ESMTarget tid={tid} az={az_cdeg/100:.1f}° freq={freq_hz/1e9:.2f}GHz")
        return self._lib.ad_proto_send_freqdetect(
            device_id, ts, tid,
            sn, brand,
            lng_e7, lat_e7, alt_dm, int(tz * 10),     # height_dm
            int(detection.get('speed', 0) * 10), int(r * 10),  # speed_dms, distance_dm
            0, 0,                                       # rctrl pos (unknown)
            freq_hz, int(freq_hz * 0.01), int(detection.get('power_db', -50) * 10),
            az_cdeg, ts, conf,
        ) == 0

    def send_eo(self, device_id: int, sim_time: float,
                eo_state: dict, sensor_enu, target_enu=None) -> bool:
        """发送光电目标帧。"""
        ts = int(sim_time * 1000)
        tid = 0  # track_id from state

        sx, sy, sz = sensor_enu
        lng_e7, lat_e7, alt_dm = self._pack_enu(sx, sy, sz)

        track_status = {'idle': 0, 'slewing': 0, 'acquiring': 0,
                        'locked': 3, 'tracking': 1}.get(eo_state.get('state', 'idle'), 0)
        n_locked = eo_state.get('n_locked', 0)

        cur_az = eo_state.get('current_az', 0) or 0
        cur_el = eo_state.get('current_el', 0) or 0
        cue_az = eo_state.get('cue_az') or cur_az

        if target_enu is not None:
            tx, ty, tz = target_enu
            tlng_e7, tlat_e7, talt_dm = self._pack_enu(tx, ty, tz)
            dx, dy, dz = tx - sx, ty - sy, tz - sz
            tdis = math.sqrt(dx*dx + dy*dy + dz*dz)
            taz_cdeg = int(cur_az * 100)
            tel_cdeg = int(cur_el * 100)
            tdis_dm = int(tdis * 10)
        else:
            tlng_e7 = tlat_e7 = 0
            talt_dm = tdis_dm = taz_cdeg = tel_cdeg = 0

        logger.info(f'[DDS] → EOTarget state={eo_state.get("state","?")} az={cur_az:.1f}°')
        return self._lib.ad_proto_send_camera(
            device_id, ts, tid,
            0, track_status, n_locked, 1,             # fault, track, count, camfield(visible)
            int(cue_az * 100), 0, int(cur_el * 100), int(cur_az * 100),  # ptz angles
            lng_e7, lat_e7, alt_dm,
            tlng_e7, tlat_e7, talt_dm,
            tdis_dm, taz_cdeg, tel_cdeg,
            2, 80,  # target_type=DRONE, confidence
        ) == 0

    def send_acoustic(self, device_id: int, sim_time: float,
                      detection: dict, sensor_enu) -> bool:
        """发送声探测目标帧 (含设备自身经纬高)。"""
        ts = int(sim_time * 1000)
        tid = detection.get('track_id', 0)

        az = detection.get('azimuth', 0)
        el = detection.get('elevation', 0)

        # 目标相对角度
        target_yaw = int((az or 0) * 100)
        target_pitch = int((el or 0) * 100)
        target_roll = 0

        # 设备自身经纬高 (同雷达L2格式)
        dlng, dlat, dalt = self._pack_enu(sensor_enu[0], sensor_enu[1], sensor_enu[2])

        target_type = 2  # DRONE
        confidence = int((detection.get('confidence', 0.6) or 0.6) * 100)

        logger.info(f"[DDS] → AcousticTarget tid={tid} az={target_yaw/100:.1f}° dlng={dlng/1e7:.5f}E")
        return self._lib.ad_proto_send_acoustic(
            device_id, ts, tid, 1,
            0, 0, 0,                  # device attitude (水平安装)
            target_roll, target_pitch, target_yaw,
            target_type, confidence,
            dlng, dlat, dalt,         # 设备经纬高
        ) == 0

    # ── 批量发送接口 ──

    def _fill_radar_data(self, arr, i, tid, info, sensor_enu, t_enu, sim_time):
        """填充单个 RadarTargetData 结构体。"""
        sx, sy, sz = sensor_enu
        tx, ty, tz = t_enu
        dx, dy, dz = tx - sx, ty - sy, tz - sz
        r = math.sqrt(dx*dx + dy*dy + dz*dz)
        az = math.degrees(math.atan2(dy, dx)) % 360
        el = math.degrees(math.atan2(dz, math.sqrt(dx*dx + dy*dy)))

        lng, lat, alt = self._pack_enu(tx, ty, tz)

        arr[i].track_id       = tid
        arr[i].extrap_flag    = 0 if info.get('state') == 'coasting' else 1
        arr[i].range_dm       = int(r * 10)
        arr[i].azi_cdeg       = int(az * 100)
        arr[i].ele_cdeg       = int(el * 100)
        arr[i].height_dm      = int(dz * 10)
        arr[i].radial_vel_dms = int(info.get('doppler', 0) * 10)
        arr[i].lng_e7         = lng
        arr[i].lat_e7         = lat
        arr[i].alt_dm         = alt
        arr[i].speed_dms      = int(info.get('speed', 0) * 10)
        arr[i].course_cdeg    = int(info.get('heading', 0) * 100)
        arr[i].target_type    = 2   # DRONE
        arr[i].confidence     = int(info.get('confidence', 0.8) * 100)

    def send_radar_batch(self, device_id: int, sim_time: float,
                          active_tracks: list, sensor_enu,
                          targets) -> bool:
        """批量发送雷达航迹 — 一次 FFI → 一条 PXSB 多单元报文。"""
        n = len(active_tracks)
        if n == 0:
            return True
        ts = int(sim_time * 1000)
        arr = (RadarTargetData * n)()

        # 建立 target_id → target 快速查找
        tmap = {t.id: t for t in targets}

        for i, (tid, info) in enumerate(active_tracks):
            t_enu = info.get('predicted')
            t = tmap.get(tid)
            if t is not None:
                state = t.get_state(sim_time)
                if state is not None:
                    t_enu = state.position
            if t_enu is None:
                t_enu = (0, 0, 0)
            self._fill_radar_data(arr, i, tid, info, sensor_enu, t_enu, sim_time)

        logger.info(f"[DDS] → RadarTarget batch x{n}")
        return self._lib.ad_proto_send_radar_batch(device_id, ts, arr, n) == 0

    def send_freqdetect_batch(self, device_id: int, sim_time: float,
                               detections: list, sensor_enu,
                               targets) -> bool:
        """批量发送频侦检测。"""
        n = len(detections)
        if n == 0:
            return True
        ts = int(sim_time * 1000)
        arr = (FreqDetectTargetData * n)()
        tmap = {t.id: t for t in targets}

        for i, det in enumerate(detections):
            tid = det.get('track_id', -1)
            t = tmap.get(tid)

            # 目标位置
            if t is not None:
                state = t.get_state(sim_time)
                t_enu = state.position if state else (0, 0, 0)
            else:
                t_enu = (0, 0, 0)
            tx, ty, tz = t_enu
            lng, lat, alt = self._pack_enu(tx, ty, tz)

            # 传感器相对信息
            sx, sy, sz = sensor_enu
            dx, dy, dz = tx - sx, ty - sy, tz - sz
            r = math.sqrt(dx*dx + dy*dy + dz*dz)

            arr[i].target_id         = tid
            arr[i].sn                = (t.label or b"").encode('utf-8')[:15].ljust(16, b'\0') if t and hasattr(t, 'label') else b'\0' * 16
            arr[i].brand_model       = (f"{t.target_type}" if t and hasattr(t, 'target_type') else "UAV").encode('utf-8')[:31].ljust(32, b'\0')
            arr[i].lng_e7            = lng
            arr[i].lat_e7            = lat
            arr[i].alt_dm            = alt
            arr[i].height_dm         = int(tz * 10)
            arr[i].speed_dms         = int(det.get('speed', 0) * 10)
            arr[i].distance_dm       = int(r * 10)
            arr[i].rctrl_lng_e7      = 0
            arr[i].rctrl_lat_e7      = 0
            arr[i].freq_hz           = int(t.emitter.freq) if t and hasattr(t, 'emitter') and t.emitter and t.emitter.freq else 2400000000
            arr[i].bandw_hz          = int(arr[i].freq_hz * 0.01)
            arr[i].power_db          = int((det.get('power_db', -50) or -50) * 10)
            arr[i].azimuth_cdeg      = int((det.get('azimuth', 0) or 0) * 100)
            arr[i].discovery_time_ms = ts
            arr[i].confidence        = int((det.get('confidence', 0.5) or 0.5) * 100)

        logger.info(f"[DDS] → ESMTarget batch x{n}")
        return self._lib.ad_proto_send_freqdetect_batch(device_id, ts, arr, n) == 0

    def send_camera_batch(self, device_id: int, sim_time: float,
                           eo_state: dict, sensor_enu,
                           targets) -> bool:
        """批量发送光电目标（目前仅发送单个锁定目标）。"""
        n_locked = eo_state.get('n_locked', 0)
        if n_locked == 0:
            return False
        ts = int(sim_time * 1000)

        # 收集有有效状态的目标
        locked = [t.get_state(sim_time) for t in targets if t.get_state(sim_time) is not None]

        # 防止 n_locked > 实际可用目标数
        actual = min(n_locked, len(locked))
        if actual == 0:
            return False

        arr = (CamTargetData * actual)()
        sx, sy, sz = sensor_enu
        lng_e7, lat_e7, alt_dm = self._pack_enu(sx, sy, sz)

        track_status = {'idle': 0, 'slewing': 0, 'acquiring': 0,
                        'locked': 3, 'tracking': 1}.get(eo_state.get('state', 'idle'), 0)
        cur_az = eo_state.get('current_az', 0) or 0
        cur_el = eo_state.get('current_el', 0) or 0
        cue_az = eo_state.get('cue_az') or cur_az

        for i, st in enumerate(locked[:actual]):
            tx, ty, tz = st.position
            tlng, tlat, talt = self._pack_enu(tx, ty, tz)
            dx, dy, dz = tx - sx, ty - sy, tz - sz
            tdis = math.sqrt(dx*dx + dy*dy + dz*dz)

            arr[i].target_id            = 0
            arr[i].fault_status         = 0
            arr[i].track_status         = track_status
            arr[i].track_count          = 1
            arr[i].camfield             = 1  # visible
            arr[i].ptz_zeroazi          = int(cue_az * 100)
            arr[i].ptz_roll            = 0
            arr[i].ptz_pitch           = int(cur_el * 100)
            arr[i].ptz_yaw             = int(cur_az * 100)
            arr[i].ptz_lng_e7          = lng_e7
            arr[i].ptz_lat_e7          = lat_e7
            arr[i].ptz_alt_dm          = alt_dm
            arr[i].target_lng_e7       = tlng
            arr[i].target_lat_e7       = tlat
            arr[i].target_alt_dm       = talt
            arr[i].target_dis_dm       = int(tdis * 10)
            arr[i].target_azimuth_cdeg = int(cur_az * 100)
            arr[i].target_elevation_cdeg = int(cur_el * 100)
            arr[i].target_type         = 2   # DRONE
            arr[i].confidence           = 80

        logger.info(f"[DDS] → EOTarget batch x{actual}")
        return self._lib.ad_proto_send_camera_batch(device_id, ts, arr, actual) == 0

    def send_acoustic_batch(self, device_id: int, sim_time: float,
                             detections: list, sensor_enu) -> bool:
        """批量发送声探测。"""
        n = len(detections)
        if n == 0:
            return True
        ts = int(sim_time * 1000)
        arr = (AcousticTargetData * n)()

        dlng, dlat, dalt = self._pack_enu(sensor_enu[0], sensor_enu[1], sensor_enu[2])

        for i, det in enumerate(detections):
            az = det.get('azimuth', 0) or 0
            el = det.get('elevation', 0) or 0

            arr[i].target_id     = det.get('track_id', 0)
            arr[i].point_count   = 1
            arr[i].device_roll   = 0
            arr[i].device_pitch  = 0
            arr[i].device_yaw    = 0
            arr[i].target_roll   = 0
            arr[i].target_pitch  = int(el * 100)
            arr[i].target_yaw    = int(az * 100)
            arr[i].target_type   = 2   # DRONE
            arr[i].confidence    = int((det.get('confidence', 0.6) or 0.6) * 100)
            arr[i].device_lng_e7 = dlng
            arr[i].device_lat_e7 = dlat
            arr[i].device_alt_dm = dalt

        logger.info(f"[DDS] → AcousticTarget batch x{n}")
        return self._lib.ad_proto_send_acoustic_batch(device_id, ts, arr, n) == 0


class ProtoPublisher:
    """仿真航迹 → 标准化协议发布器。"""

    def __init__(self, bridge: ProtoBridge, device_id_base: int = 100):
        self._b = bridge
        # 设备ID: 雷达=101, 光电=102, 电侦=103, 声探测=104
        self._did = {
            'radar': device_id_base + 1,
            'eo': device_id_base + 2,
            'esm': device_id_base + 3,
            'acoustic': device_id_base + 4,
        }

    def publish_radar_tracks(self, sim_time, radar_sensor, targets):
        """批量发布所有雷达航迹 — 一次 FFI → 一条 PXSB 多单元报文。"""
        tracks = radar_sensor.get_track_states()
        s_enu = radar_sensor.position
        # 收集活跃航迹
        active = [
            (tid, info) for tid, info in tracks.items()
            if info['state'] in ('confirmed', 'coasting') and info.get('predicted') is not None
        ]
        if not active:
            return
        self._b.send_radar_batch(self._did['radar'], sim_time, active, s_enu, targets)

    def publish_esm_detections(self, sim_time, detections, esm_sensor, targets):
        """批量发布电侦检测。"""
        if not detections:
            return
        s_enu = esm_sensor.position
        self._b.send_freqdetect_batch(self._did['esm'], sim_time, detections, s_enu, targets)

    def publish_eo_state(self, sim_time, eo_sensor, targets) -> bool:
        """批量发布光电状态。

        Returns:
            bool: 是否实际发送了数据
        """
        state = eo_sensor.get_state()
        if state.get('n_locked', 0) == 0:
            return False
        s_enu = eo_sensor.position
        return self._b.send_camera_batch(self._did['eo'], sim_time, state, s_enu, targets)

    def publish_acoustic_detections(self, sim_time, detections, acoustic_sensor):
        """批量发布声探测。"""
        if not detections:
            return
        s_enu = acoustic_sensor.position
        self._b.send_acoustic_batch(self._did['acoustic'], sim_time, detections, s_enu)
