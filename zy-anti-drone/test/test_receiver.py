#!/usr/bin/env python3
"""DDS 端到端测试 — 模拟 swarm_saturation.yaml (15目标) 发送 → 接收。

零 Python 依赖（不含 numpy/scipy），直接 ctypes 调 .so。

用法:
    cd /home/liuyg/codes/zy-anti-drone/data_generator
    LD_LIBRARY_PATH=../thirdparty/linux-x86/lib:../bridge \
        python ../test/test_receiver.py
"""

import os
import math
import time
import ctypes
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BRIDGE_DIR = ROOT / "bridge"
LIB_DIR = ROOT / "thirdparty/linux-x86/lib"
os.environ['LD_LIBRARY_PATH'] = f"{LIB_DIR}:{BRIDGE_DIR}:{os.environ.get('LD_LIBRARY_PATH', '')}"

# ═══════════════════════════════════════════════════════════════
# ctypes POD structs (与 C++ #pragma pack(1) 一致)
# ═══════════════════════════════════════════════════════════════

class RadarTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("track_id", ctypes.c_uint32), ("extrap_flag", ctypes.c_uint8),
        ("range_dm", ctypes.c_uint32), ("azi_cdeg", ctypes.c_int16),
        ("ele_cdeg", ctypes.c_int16), ("height_dm", ctypes.c_uint32),
        ("radial_vel_dms", ctypes.c_int16), ("lng_e7", ctypes.c_int32),
        ("lat_e7", ctypes.c_int32), ("alt_dm", ctypes.c_uint32),
        ("speed_dms", ctypes.c_uint32), ("course_cdeg", ctypes.c_uint16),
        ("target_type", ctypes.c_uint8), ("confidence", ctypes.c_uint8),
    ]

class FreqDetectTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id", ctypes.c_uint32), ("sn", ctypes.c_char * 16),
        ("brand_model", ctypes.c_char * 32), ("lng_e7", ctypes.c_int32),
        ("lat_e7", ctypes.c_int32), ("alt_dm", ctypes.c_uint32),
        ("height_dm", ctypes.c_uint32), ("speed_dms", ctypes.c_uint32),
        ("distance_dm", ctypes.c_uint32), ("rctrl_lng_e7", ctypes.c_int32),
        ("rctrl_lat_e7", ctypes.c_int32), ("freq_hz", ctypes.c_uint32),
        ("bandw_hz", ctypes.c_uint32), ("power_db", ctypes.c_int16),
        ("azimuth_cdeg", ctypes.c_uint16), ("discovery_time_ms", ctypes.c_uint64),
        ("confidence", ctypes.c_uint8),
    ]

class CamTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id", ctypes.c_uint32), ("fault_status", ctypes.c_uint8),
        ("track_status", ctypes.c_uint8), ("track_count", ctypes.c_uint8),
        ("camfield", ctypes.c_uint8), ("ptz_zeroazi", ctypes.c_uint16),
        ("ptz_roll", ctypes.c_uint16), ("ptz_pitch", ctypes.c_int16),
        ("ptz_yaw", ctypes.c_uint16), ("ptz_lng_e7", ctypes.c_int32),
        ("ptz_lat_e7", ctypes.c_int32), ("ptz_alt_dm", ctypes.c_uint32),
        ("target_lng_e7", ctypes.c_int32), ("target_lat_e7", ctypes.c_int32),
        ("target_alt_dm", ctypes.c_uint32), ("target_dis_dm", ctypes.c_uint32),
        ("target_azimuth_cdeg", ctypes.c_uint16),
        ("target_elevation_cdeg", ctypes.c_int16),
        ("target_type", ctypes.c_uint8), ("confidence", ctypes.c_uint8),
    ]

class AcousticTargetData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("target_id", ctypes.c_uint32), ("point_count", ctypes.c_uint8),
        ("device_roll", ctypes.c_int16), ("device_pitch", ctypes.c_int16),
        ("device_yaw", ctypes.c_int16), ("target_roll", ctypes.c_uint16),
        ("target_pitch", ctypes.c_uint16), ("target_yaw", ctypes.c_uint16),
        ("target_type", ctypes.c_uint8), ("confidence", ctypes.c_uint8),
        ("device_lng_e7", ctypes.c_int32), ("device_lat_e7", ctypes.c_int32),
        ("device_alt_dm", ctypes.c_uint32),
    ]

class AdRxFrame(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_type", ctypes.c_uint16), ("device_id", ctypes.c_uint32),
        ("timestamp_ms", ctypes.c_uint64), ("data_len", ctypes.c_uint32),
        ("data", ctypes.c_uint8 * 2048),
    ]

# ═══════════════════════════════════════════════════════════════
# 帧解析 (纯 Python struct)
# ═══════════════════════════════════════════════════════════════

_HDR = struct.Struct("<I B H H H I Q")   # DevFrameHeader 23B

def _parse_radar(raw: bytes):
    hdr = _HDR.unpack_from(raw, 0)
    body = struct.unpack_from("<I I h h I h i i I I H B B B", raw, 23)
    return {
        "type": "Radar", "dev": hdr[5], "ts_ms": hdr[6],
        "tid": body[0], "range_m": body[1]*0.1, "az_deg": body[2]*0.01,
        "el_deg": body[3]*0.01, "lng": body[6]*1e-7, "lat": body[7]*1e-7,
        "alt_m": body[8]*0.1, "speed_ms": body[9]*0.1,
        "course_deg": body[10]*0.01, "tag": body[11], "conf": body[13],
    }

def _parse_esm(raw: bytes):
    hdr = _HDR.unpack_from(raw, 0)
    body = struct.unpack_from("<I 16s 32s i i I I I I i i I I h H Q B 7s", raw, 23)
    return {
        "type": "ESM", "dev": hdr[5], "ts_ms": hdr[6],
        "tid": body[0], "sn": body[1].rstrip(b'\0').decode(errors='replace'),
        "freq_hz": body[11], "az_deg": body[14]*0.01, "conf": body[16],
    }

def _parse_eo(raw: bytes):
    hdr = _HDR.unpack_from(raw, 0)
    body = struct.unpack_from("<I B B B B H H h H i i I i i I I H h B B 2s", raw, 23)
    return {
        "type": "EO", "dev": hdr[5], "ts_ms": hdr[6],
        "tid": body[0], "track_status": body[2],
        "ptz_yaw_deg": body[8]*0.01, "target_lng": body[12]*1e-7,
        "target_dis_m": body[15]*0.1, "conf": body[19],
    }

def _parse_acoustic(raw: bytes):
    hdr = _HDR.unpack_from(raw, 0)
    body = struct.unpack_from("<I B 3s h h h 2s H H H B B 4s i i I", raw, 23)
    return {
        "type": "Acoustic", "dev": hdr[5], "ts_ms": hdr[6],
        "tid": body[0], "az_deg": body[9]*0.01, "el_deg": body[8]*0.01,
        "conf": body[11],
    }

_PARSE_MAP = {0x1A01: _parse_radar, 0x1B01: _parse_esm,
              0x1C01: _parse_eo, 0x1D01: _parse_acoustic}

# ═══════════════════════════════════════════════════════════════
# .so 加载
# ═══════════════════════════════════════════════════════════════

def load_tx():
    lib = ctypes.CDLL(str(BRIDGE_DIR / "libad_proto.so"))
    lib.ad_proto_version.restype = ctypes.c_char_p
    lib.ad_proto_init.restype = ctypes.c_int
    for name, ptr_t in [("radar", RadarTargetData), ("freqdetect", FreqDetectTargetData),
                         ("camera", CamTargetData), ("acoustic", AcousticTargetData)]:
        fn = getattr(lib, f"ad_proto_send_{name}_batch")
        fn.restype = ctypes.c_int
        fn.argtypes = [ctypes.c_uint32, ctypes.c_uint64,
                       ctypes.POINTER(ptr_t), ctypes.c_uint32]
    lib.ad_proto_close.restype = None
    return lib

def load_rx():
    lib = ctypes.CDLL(str(BRIDGE_DIR / "libad_proto_receiver.so"))
    lib.ad_proto_receive_init.restype = ctypes.c_int
    lib.ad_proto_receive_poll.restype = ctypes.c_int
    lib.ad_proto_receive_poll.argtypes = [ctypes.POINTER(AdRxFrame), ctypes.c_int]
    lib.ad_proto_receive_available.restype = ctypes.c_int
    lib.ad_proto_receive_close.restype = None
    return lib

# ═══════════════════════════════════════════════════════════════
# 模拟数据 (仿 swarm_saturation: 15 目标, 5 方向 × 3 高度层)
# ═══════════════════════════════════════════════════════════════

DIRECTIONS = [(0.7,-3.9),(0.7,-3.9),(0.7,-3.9),(-3.0,-2.6),(-3.0,-2.6),
              (-3.0,-2.6),(-4.0,0),(-4.0,0),(-4.0,0),(-3.0,2.6),(-3.0,2.6),
              (-3.0,2.6),(0.7,3.9),(0.7,3.9),(0.7,3.9)]
HEIGHTS = [50, 150, 300]
SPEEDS  = [80 + i * 3 for i in range(15)]
REF_LNG, REF_LAT = 120.0, 30.0
_MPL = 111320.0 * math.cos(math.radians(REF_LAT))

def enu_to_wgs84(x, y, z):
    return (int(round(REF_LNG + x/_MPL * 1e7)),
            int(round(REF_LAT + y/111320.0 * 1e7)),
            int(round(z * 10)))

def main():
    print("=" * 60)
    print("  DDS 端到端测试 (swarm_saturation: 15目标 + 4传感器)")
    print("=" * 60)

    # ── 1. 加载 ──
    tx = load_tx()
    rx = load_rx()
    assert tx.ad_proto_init() == 0, "TX init fail"
    print(f"[TX] {tx.ad_proto_version().decode()} ready")
    assert rx.ad_proto_receive_init() == 0, "RX init fail"
    print("[RX] ready")
    print("  等待 DDS 发现完成...")
    time.sleep(1.5)

    ts = int(time.time() * 1000)

    # ── 2. 发送 15 个雷达 ──
    print("\n─ 发送 RadarTarget × 15")
    arr = (RadarTargetData * 15)()
    for i, ((dx_km, dy_km), spd) in enumerate(zip(DIRECTIONS, SPEEDS)):
        h = HEIGHTS[i % 3]
        x, y = dx_km * 1000, dy_km * 1000
        lng, lat, alt = enu_to_wgs84(x, y, h)
        r = math.sqrt(x*x + y*y + h*h)
        arr[i].track_id = i + 1
        arr[i].extrap_flag = 1
        arr[i].range_dm = int(r * 10)
        arr[i].azi_cdeg = int(math.degrees(math.atan2(x, y)) * 100) % 36000
        arr[i].ele_cdeg = int(math.degrees(math.atan2(h, math.sqrt(x*x+y*y))) * 100)
        arr[i].height_dm = h * 10
        arr[i].radial_vel_dms = int(spd * -10)
        arr[i].lng_e7 = lng; arr[i].lat_e7 = lat; arr[i].alt_dm = alt
        arr[i].speed_dms = int(spd * 10)
        arr[i].course_cdeg = int(math.degrees(math.atan2(-x, -y)) * 100) % 36000
        arr[i].target_type = 2; arr[i].confidence = 85
    assert tx.ad_proto_send_radar_batch(101, ts, arr, 15) == 0
    print("  OK"); time.sleep(0.2)

    # ── 3. 发送 8 个 ESM ──
    print("─ 发送 ESMTarget × 8")
    ea = (FreqDetectTargetData * 8)()
    for i in range(8):
        dx_km, dy_km = DIRECTIONS[i]; h = HEIGHTS[i%3]
        x, y = dx_km*1000, dy_km*1000
        lng, lat, alt = enu_to_wgs84(x, y, h)
        ea[i].target_id = i+1
        ea[i].sn = f"UAV-{i+1:03d}".encode().ljust(16, b'\0')
        ea[i].brand_model = b"fixed_wing".ljust(32, b'\0')
        ea[i].lng_e7 = lng; ea[i].lat_e7 = lat; ea[i].alt_dm = alt
        ea[i].height_dm = h*10; ea[i].speed_dms = SPEEDS[i]*10
        ea[i].distance_dm = int(math.sqrt(x*x+y*y+h*h)*10)
        ea[i].freq_hz = 2400000000 + i*100000000
        ea[i].bandw_hz = 10000000; ea[i].power_db = -500
        ea[i].azimuth_cdeg = int(math.degrees(math.atan2(x, y))*100) % 36000
        ea[i].discovery_time_ms = ts; ea[i].confidence = 70+i
    assert tx.ad_proto_send_freqdetect_batch(103, ts, ea, 8) == 0
    print("  OK"); time.sleep(0.2)

    # ── 4. 发送 1 个 EO ──
    print("─ 发送 EOTarget × 1")
    ca = (CamTargetData * 1)()
    dx_km, dy_km = DIRECTIONS[0]; h = HEIGHTS[0]
    x, y = dx_km*1000, dy_km*1000
    lng, lat, alt = enu_to_wgs84(x, y, h)
    slng, slat, salt = enu_to_wgs84(0, 0, 5)
    ca[0].target_id = 1; ca[0].track_status = 1; ca[0].track_count = 1
    ca[0].camfield = 1
    ca[0].ptz_pitch = int(math.degrees(math.atan2(h, math.sqrt(x*x+y*y)))*100)
    ca[0].ptz_yaw = int(math.degrees(math.atan2(x, y))*100) % 36000
    ca[0].ptz_lng_e7 = slng; ca[0].ptz_lat_e7 = slat; ca[0].ptz_alt_dm = salt
    ca[0].target_lng_e7 = lng; ca[0].target_lat_e7 = lat; ca[0].target_alt_dm = alt
    ca[0].target_dis_dm = int(math.sqrt(x*x+y*y+h*h)*10)
    ca[0].target_azimuth_cdeg = ca[0].ptz_yaw
    ca[0].target_elevation_cdeg = ca[0].ptz_pitch
    ca[0].target_type = 2; ca[0].confidence = 90
    assert tx.ad_proto_send_camera_batch(102, ts, ca, 1) == 0
    print("  OK"); time.sleep(0.2)

    # ── 5. 发送 3 个 Acoustic ──
    print("─ 发送 AcousticTarget × 3")
    aa = (AcousticTargetData * 3)()
    dlng, dlat, dalt = enu_to_wgs84(0, 0, 10)
    for i in range(3):
        dx_km, dy_km = DIRECTIONS[i]; h = HEIGHTS[i%3]
        x, y = dx_km*1000, dy_km*1000
        aa[i].target_id = i+1; aa[i].point_count = 1
        aa[i].target_pitch = int(math.degrees(math.atan2(h, math.sqrt(x*x+y*y)))*100)
        aa[i].target_yaw = int(math.degrees(math.atan2(x, y))*100) % 36000
        aa[i].target_type = 2; aa[i].confidence = 60+i*10
        aa[i].device_lng_e7 = dlng; aa[i].device_lat_e7 = dlat
        aa[i].device_alt_dm = dalt
    assert tx.ad_proto_send_acoustic_batch(104, ts, aa, 3) == 0
    print("  OK")

    # ── 6. 等待 DDS 传递 ──
    print("\n  等待 DDS 传递...")
    time.sleep(1.0)
    avail = rx.ad_proto_receive_available()
    print(f"  队列帧数: {avail}")

    # ── 7. 接收 ──
    frame = AdRxFrame()
    stats = {"Radar": 0, "ESM": 0, "EO": 0, "Acoustic": 0}
    samples, unknown = [], set()

    while rx.ad_proto_receive_poll(ctypes.byref(frame), 0) > 0:
        raw = bytes(frame.data[:frame.data_len])
        ftype = frame.frame_type
        tag = {0x1A01:"Radar",0x1B01:"ESM",0x1C01:"EO",0x1D01:"Acoustic"}.get(ftype)
        if tag is None:
            unknown.add(f"{ftype:#06x}")
            continue
        stats[tag] += 1
        parser = _PARSE_MAP.get(ftype)
        if parser and len(samples) < 10:
            samples.append(parser(raw))

    total = sum(stats.values())
    print()
    print("=" * 60)
    print("  接收结果")
    print("=" * 60)
    if unknown:
        print(f"  未知类型: {unknown}")
    print(f"  收到帧数: {total}")
    for k, v in stats.items():
        print(f"    {k:<12s} {v:3d}  {'✓' if v>0 else '✗'}")

    print()
    print("─ 样例帧 ─")
    for s in samples:
        print(f"  {s['type']:8s} tid={s['tid']:2d} dev={s['dev']} "
              f"ts={s['ts_ms']} conf={s.get('conf','?')}")

    # ── 8. 验证 ──
    print()
    errors = []
    for k, exp in [("Radar",15),("ESM",8),("EO",1),("Acoustic",3)]:
        if stats[k] == 0:
            errors.append(f"{k}: 收到 0 帧 (期望 {exp})")
        elif stats[k] != exp:
            print(f"  ~ {k:<12s} 收到 {stats[k]} 帧 (期望 {exp}) — 部分丢失")
        else:
            print(f"  ✓ {k:<12s} 收到 {stats[k]} 帧 ✓")
    if errors:
        for e in errors:
            print(f"  ✗ {e}")
    elif all(stats[k] == exp for k, exp in
             [("Radar",15),("ESM",8),("EO",1),("Acoustic",3)]):
        print("\n  ✓ 全部通过! 27/27 帧")
    else:
        print("\n  ~ 部分通过")

    # ── 9. 清理 ──
    rx.ad_proto_receive_close()
    tx.ad_proto_close()
    print("\n测试结束")

if __name__ == "__main__":
    main()
