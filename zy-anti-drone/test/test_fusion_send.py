#!/usr/bin/env python3
"""融合多目标数据发送测试 — Python → ctypes → libad_proto_fusion.so → DDS。

可从任意目录运行，只需设 LD_LIBRARY_PATH:

    LD_LIBRARY_PATH=<root>/thirdparty/linux-x86/lib:<root>/bridge \
        python test/test_fusion_send.py
"""

import ctypes
import math
import time
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BRIDGE_DIR = ROOT / "bridge"
LIB_DIR = ROOT / "thirdparty/linux-x86/lib"
os.environ['LD_LIBRARY_PATH'] = f"{LIB_DIR}:{BRIDGE_DIR}:{os.environ.get('LD_LIBRARY_PATH', '')}"

# ═══════════════════════════════════════════════════════════════
# ctypes POD structs (与 C++ #pragma pack(1) 一致)
# ═══════════════════════════════════════════════════════════════

FUSION_MAX_PRED_POINTS = 8


class FusionPredPointData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("lng_e7", ctypes.c_int32),
        ("lat_e7", ctypes.c_int32),
        ("alt",     ctypes.c_uint32),
        ("pitch",   ctypes.c_int16),
        ("roll",    ctypes.c_int16),
        ("yaw",     ctypes.c_uint16),
        ("speed",   ctypes.c_uint32),
        ("time",    ctypes.c_uint64),
    ]


class FusionTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("global_id",        ctypes.c_uint8),
        ("lng_e7",           ctypes.c_int32),
        ("lat_e7",           ctypes.c_int32),
        ("alt",              ctypes.c_uint32),
        ("pitch",            ctypes.c_int16),
        ("roll",             ctypes.c_int16),
        ("yaw",              ctypes.c_uint16),
        ("speed",            ctypes.c_uint32),
        ("fusion_type",      ctypes.c_uint8),
        ("confidence",       ctypes.c_uint8),
        ("source_mask",      ctypes.c_uint8),
        ("iff",              ctypes.c_uint8),
        ("pred_point_count", ctypes.c_uint8),
        ("pred_points",      FusionPredPointData * FUSION_MAX_PRED_POINTS),
    ]


# ═══════════════════════════════════════════════════════════════
# .so 加载
# ═══════════════════════════════════════════════════════════════

class FusionBridge:
    """融合层 DDS 发送桥接器。"""

    def __init__(self):
        lib_path = BRIDGE_DIR / "libad_proto_fusion.so"
        self._lib = ctypes.CDLL(str(lib_path))
        self._setup()
        self._inited = False

    def _setup(self):
        L = self._lib
        L.ad_proto_fusion_version.restype = ctypes.c_char_p

        L.ad_proto_fusion_init.restype = ctypes.c_int
        L.ad_proto_fusion_init.argtypes = []

        L.ad_proto_send_fusion_multi_target.restype = ctypes.c_int
        L.ad_proto_send_fusion_multi_target.argtypes = [
            ctypes.c_uint32,                         # device_id
            ctypes.c_uint64,                         # timestamp_ms
            ctypes.POINTER(FusionTargetData),         # targets array
            ctypes.c_uint32,                         # count
        ]

        L.ad_proto_fusion_ready.restype = ctypes.c_int
        L.ad_proto_fusion_close.restype = None

    def init(self) -> bool:
        if self._inited:
            return True
        rc = self._lib.ad_proto_fusion_init()
        self._inited = (rc == 0)
        if self._inited:
            ver = self._lib.ad_proto_fusion_version().decode()
            print(f"[Fusion] 已就绪 ({ver})")
        return self._inited

    def send(self, device_id: int, timestamp_ms: int,
             targets: list[FusionTargetData]) -> bool:
        n = len(targets)
        if n == 0:
            return True
        arr = (FusionTargetData * n)(*targets)
        rc = self._lib.ad_proto_send_fusion_multi_target(
            device_id, timestamp_ms, arr, n)
        return rc == 0

    def close(self):
        if self._inited:
            self._lib.ad_proto_fusion_close()
        self._inited = False


# ═══════════════════════════════════════════════════════════════
# 辅助函数
# ═══════════════════════════════════════════════════════════════

REF_LNG, REF_LAT = 120.0, 30.0
_MPL = 111320.0 * math.cos(math.radians(REF_LAT))


def enu_to_wgs84(x, y, z):
    return (int(round(REF_LNG + x / _MPL * 1e7)),
            int(round(REF_LAT + y / 111320.0 * 1e7)),
            int(round(z * 10)))


def make_target(global_id: int, x_m: float, y_m: float, z_m: float,
                speed_ms: float = 80.0, heading_deg: float = 0.0,
                source_mask: int = 13,   # ALL: 雷达+光电+电侦+声探
                iff: int = 2,             # 敌方
                **kwargs) -> FusionTargetData:
    """创建单个融合目标。"""
    t = FusionTargetData()
    t.global_id = global_id
    lng, lat, alt = enu_to_wgs84(x_m, y_m, z_m)
    t.lng_e7 = lng
    t.lat_e7 = lat
    t.alt = alt
    t.pitch = int(kwargs.get('pitch', 0) * 100)
    t.roll  = int(kwargs.get('roll', 0) * 100)
    t.yaw   = int(heading_deg * 100)
    t.speed = int(speed_ms * 100)   # 0.01 m/s
    t.fusion_type = kwargs.get('fusion_type', 2)  # DRONE
    t.confidence  = int(kwargs.get('confidence', 0.8) * 100)
    t.source_mask = source_mask
    t.iff = iff
    t.pred_point_count = 0  # no predictions
    return t


def add_prediction(target: FusionTargetData, x_m: float, y_m: float,
                   z_m: float, speed_ms: float, time_ms: int,
                   heading_deg: float = 0.0):
    """给融合目标添加一个预测轨迹点。"""
    n = target.pred_point_count
    if n >= FUSION_MAX_PRED_POINTS:
        return
    pp = target.pred_points[n]
    lng, lat, alt = enu_to_wgs84(x_m, y_m, z_m)
    pp.lng_e7 = lng
    pp.lat_e7 = lat
    pp.alt = alt
    pp.pitch = 0
    pp.roll = 0
    pp.yaw = int(heading_deg * 100)
    pp.speed = int(speed_ms * 100)
    pp.time = time_ms
    target.pred_point_count = n + 1


# ═══════════════════════════════════════════════════════════════
# 测试
# ═══════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  融合多目标发送测试")
    print("=" * 60)

    bridge = FusionBridge()
    assert bridge.init(), "融合桥接初始化失败"

    # 模拟 5 个融合目标 (类似 swarm_saturation 方向)
    directions = [(0.7, -3.9), (-3.0, -2.6), (-4.0, 0.0),
                  (-3.0, 2.6), (0.7, 3.9)]
    heights = [50, 150, 300, 200, 100]

    targets = []
    for i, ((dx_km, dy_km), h) in enumerate(zip(directions, heights)):
        x, y = dx_km * 1000, dy_km * 1000
        speed = 80 + i * 10
        heading = (math.degrees(math.atan2(-x, -y)) + 180) % 360
        t = make_target(i + 1, x, y, h, speed_ms=speed,
                        heading_deg=heading, source_mask=13)
        # 添加 2 个预测点 (1s, 2s 后)
        for dt_s in [1, 2]:
            dist = speed * dt_s
            future_x = x - dist * math.sin(math.radians(heading))
            future_y = y - dist * math.cos(math.radians(heading))
            add_prediction(t, future_x, future_y, h, speed,
                           int(time.time() * 1000) + dt_s * 1000)
        targets.append(t)

    print(f"  发送 {len(targets)} 个融合目标")
    for t in targets:
        npp = t.pred_point_count
        print(f"    ID={t.global_id} lng={t.lng_e7/1e7:.6f}° "
              f"lat={t.lat_e7/1e7:.6f}° alt={t.alt/10:.0f}m "
              f"speed={t.speed/100:.0f}m/s pred={npp}")

    ts = int(time.time() * 1000)
    ok = bridge.send(200, ts, targets)
    print(f"  发送结果: {'✓ OK' if ok else '✗ FAIL'}")

    bridge.close()
    print("\n测试结束")


if __name__ == "__main__":
    main()
