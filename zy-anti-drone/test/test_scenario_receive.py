#!/usr/bin/env python3
"""逐场景端到端测试: 启动模拟器 → DDS → test_receiver 接收。

用法:
    LD_LIBRARY_PATH=<root>/thirdparty/linux-x86/lib:<root>/bridge \
        python test/test_scenario_receive.py
"""

import subprocess
import sys
import time
import os
import ctypes
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SCENARIO_DIR = ROOT / "data_generator/config/scenarios"
MAIN_PY = ROOT / "data_generator/src/main.py"
os.environ['LD_LIBRARY_PATH'] = f"{ROOT}/thirdparty/linux-x86/lib:{ROOT}/bridge"

# ── ctypes receiver (复用 libad_proto_receiver.so) ──

class AdRxFrame(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("frame_type", ctypes.c_uint16), ("device_id", ctypes.c_uint32),
        ("timestamp_ms", ctypes.c_uint64), ("data_len", ctypes.c_uint32),
        ("data", ctypes.c_uint8 * 2048),
    ]

rx_lib = ctypes.CDLL(str(ROOT / "bridge/libad_proto_receiver.so"))
rx_lib.ad_proto_receive_init.restype = ctypes.c_int
rx_lib.ad_proto_receive_poll.restype = ctypes.c_int
rx_lib.ad_proto_receive_poll.argtypes = [ctypes.POINTER(AdRxFrame), ctypes.c_int]
rx_lib.ad_proto_receive_available.restype = ctypes.c_int
rx_lib.ad_proto_receive_close.restype = None

TAG_MAP = {0x1A01: "Radar", 0x1B01: "ESM", 0x1C01: "EO", 0x1D01: "Acoustic"}


def drain_all():
    """非阻塞取空接收队列，返回统计。"""
    frame = AdRxFrame()
    stats = {"Radar": 0, "ESM": 0, "EO": 0, "Acoustic": 0}
    while rx_lib.ad_proto_receive_poll(ctypes.byref(frame), 0) > 0:
        tag = TAG_MAP.get(frame.frame_type)
        if tag:
            stats[tag] += 1
    return stats


# ── 主逻辑 ──

scenarios = sorted(SCENARIO_DIR.glob("*.yaml"))
print(f"{'='*60}")
print(f"  全场景端到端测试 ({len(scenarios)} 个场景)")
print(f"{'='*60}")

# 全局初始化接收端一次
assert rx_lib.ad_proto_receive_init() == 0, "RX init fail"
time.sleep(0.5)
print("RX ready\n")

results = []
for sp in scenarios:
    name = sp.stem
    print(f"  [{name}] ", end="", flush=True)

    # 清空上轮残余
    drain_all()

    # 启动模拟器子进程 (--no-gui -o)
    proc = subprocess.Popen(
        [sys.executable, str(MAIN_PY), str(sp), "--no-gui", "-o"],
        cwd=str(ROOT / "data_generator"),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )

    # 等待模拟器运行 + DDS 发现 + 数据发送 (模拟器 2Hz 发布)
    time.sleep(10)
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()

    # 再等一下确保最后的数据到达
    time.sleep(1)

    # 接收
    stats = drain_all()
    total = sum(stats.values())
    details = " ".join(f"{k[0]}:{v}" for k, v in stats.items() if v > 0)
    ok = total > 0
    mark = "✓" if ok else "✗ 无数据!"
    print(f"{total:3d}帧 [{details}] {mark}")
    results.append((name, ok, total, stats))

# ── 汇总 ──
print(f"\n{'='*60}")
print("  汇总")
print(f"{'='*60}")
passed = 0
for name, ok, total, stats in results:
    mark = "✓" if ok else "✗"
    print(f"  {mark} {name:<25s} {total:3d}帧")
    if ok:
        passed += 1
print(f"\n  通过: {passed}/{len(results)}")

rx_lib.ad_proto_receive_close()
