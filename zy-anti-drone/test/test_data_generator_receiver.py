#!/usr/bin/env python3
"""data_generator DDS 接收测试 — 兼容所有场景配置文件。

与 data_generator/src/main.py -o data_generator/config/scenarios/*.yaml 配合使用，
接收四种传感器（雷达/频侦/光电/声探测）的 dev_proto 二进制帧并统计。

用法:
    # 终端1: 启动数据生成器
    cd /home/liuyg/codes/zy-anti-drone/data_generator
    LD_LIBRARY_PATH=../thirdparty/linux-x86/lib:../bridge \
        python src/main.py config/scenarios/multi_target.yaml -o --no-gui

    # 终端2: 启动接收测试 (自动等待 DDS 数据)
    cd /home/liuyg/codes/zy-anti-drone
    LD_LIBRARY_PATH=thirdparty/linux-x86/lib:bridge \
        python test/test_data_generator_receiver.py

    # 指定最长等待时间和详细程度
    python test/test_data_generator_receiver.py --max-wait 30 --verbose
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from collections import defaultdict
from pathlib import Path

# ── 将 data_generator/src 加入 sys.path，以便导入 ProtoReceiver ──
_HERE = Path(__file__).resolve().parent
_PROJ_ROOT = _HERE.parent
_DG_SRC = _PROJ_ROOT / "data_generator" / "src"
if str(_DG_SRC) not in sys.path:
    sys.path.insert(0, str(_DG_SRC))

# 确保 LD_LIBRARY_PATH 包含 bridge 和 thirdparty 的 .so
_EXTRA_LD = os.pathsep.join([
    str(_PROJ_ROOT / "thirdparty" / "linux-x86" / "lib"),
    str(_PROJ_ROOT / "bridge"),
])
_prev = os.environ.get("LD_LIBRARY_PATH", "")
os.environ["LD_LIBRARY_PATH"] = (
    f"{_EXTRA_LD}:{_prev}" if _prev else _EXTRA_LD
)

from bus.proto_receiver import (
    ProtoReceiver,
    RadarTarget,
    ESMTarget,
    EOTarget,
    AcousticTarget,
    parse_frame,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("test_receiver")


# ═══════════════════════════════════════════════════════════════
# 帧处理器 — 统计 + 可选的详细打印
# ═══════════════════════════════════════════════════════════════

_FRAME_LABEL = {
    0x1A01: "Radar",
    0x1B01: "ESM",
    0x1C01: "EO",
    0x1D01: "Acoustic",
}


class FrameStats:
    """传感器帧统计器。"""

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.reset()

    def reset(self):
        self.counts: dict[str, int] = defaultdict(int)
        self.total = 0
        self.start_time: float | None = None
        self.last_time: float | None = None
        # 每个类型保留最近一帧的简要信息
        self.last: dict[str, str] = {}
        # 见过的设备 ID
        self.device_ids: set[int] = set()
        # 见过的 track_id
        self.track_ids: dict[str, set] = defaultdict(set)

    def on_frame(self, frame_type: int, data: bytes, data_len: int,
                 device_id: int, timestamp_ms: int):
        """处理一帧。"""
        tag = _FRAME_LABEL.get(frame_type, f"0x{frame_type:04X}")
        self.counts[tag] += 1
        self.total += 1
        self.device_ids.add(device_id)

        if self.start_time is None:
            self.start_time = time.time()
        self.last_time = time.time()

        # 详细打印
        obj = parse_frame(frame_type, data, data_len, device_id)
        if obj is not None:
            tid = getattr(obj, "track_id",
                          getattr(obj, "target_id",
                                  getattr(obj, "global_id", "?")))
            self.track_ids[tag].add(tid)
            self.last[tag] = repr(obj)
            if self.verbose:
                print(f"  [{tag:<8s}] dev={device_id} ts={timestamp_ms} {obj}")

    def summary(self) -> str:
        """生成统计摘要。"""
        elapsed = ""
        if self.start_time and self.last_time:
            dt = self.last_time - self.start_time
            elapsed = f"  运行时长: {dt:.1f}s\n"

        lines = [
            f"  设备数: {len(self.device_ids)}  (IDs: {sorted(self.device_ids)})",
            f"  总帧数: {self.total}",
        ]
        for tag in ["Radar", "ESM", "EO", "Acoustic"]:
            c = self.counts[tag]
            tids = self.track_ids.get(tag, set())
            tinfo = f" (track_ids: {sorted(tids)})" if tids else ""
            lines.append(f"    {tag:<8s}: {c:5d} 帧{tinfo}")

        return elapsed + "\n".join(lines)


# ═══════════════════════════════════════════════════════════════
# 主逻辑
# ═══════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="data_generator DDS 接收测试 — 兼容所有场景 YAML"
    )
    parser.add_argument(
        "--max-wait", type=float, default=0,
        help="最长等待时间 (秒)。0=持续等待直到 Ctrl+C (默认)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="打印每一帧的详细信息"
    )
    parser.add_argument(
        "--idle-timeout", type=float, default=10.0,
        help="无数据超时 (秒)。超过此时间无新帧则自动退出 (默认: 10)"
    )
    parser.add_argument(
        "--dump", type=int, default=0,
        help="打印前 N 帧的原始 hex (默认: 0, 不打印)"
    )
    args = parser.parse_args()

    # 检查 .so 是否存在
    rx_so = _PROJ_ROOT / "bridge" / "libad_proto_receiver.so"
    if not rx_so.exists():
        print(f"错误: 接收库不存在: {rx_so}")
        print("请先编译 bridge: cd bridge && bash build.sh")
        sys.exit(1)

    print("=" * 64)
    print("  data_generator DDS 接收测试")
    print(f"  场景兼容: single_target / multi_target / complex_engagement")
    print(f"            swarm_layered / swarm_loitering / swarm_saturation")
    print(f"            swarm_wolfpack")
    print("=" * 64)
    print()

    stats = FrameStats(verbose=args.verbose)

    # 初始化接收器
    rx = ProtoReceiver()
    if not rx.available:
        print("错误: ProtoReceiver 不可用，无法加载 libad_proto_receiver.so")
        sys.exit(1)

    if not rx.init():
        print("错误: DDS 接收端初始化失败")
        sys.exit(1)

    print("[*] DDS 接收端已就绪，等待数据...")
    print(f"    话题: RadarTarget / ESMTarget / EOTarget / AcousticTarget")
    print(f"    空闲超时: {args.idle_timeout}s")
    if args.max_wait > 0:
        print(f"    最长等待: {args.max_wait}s")
    print(f"    按 Ctrl+C 停止")
    print()

    # 信号处理 — 优雅退出
    should_stop = False

    def on_signal(sig, frame):
        nonlocal should_stop
        should_stop = True
        print("\n[*] 收到停止信号，正在退出...")

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    start_ts = time.time()
    last_data_ts = start_ts
    dump_count = 0
    spinner_chars = "|/-\\"
    spinner_idx = 0
    last_status_line = False

    try:
        while not should_stop:
            elapsed = time.time() - start_ts

            # 检查最大等待时间
            if args.max_wait > 0 and elapsed >= args.max_wait:
                print("\n[*] 达到最长等待时间")
                break

            # 检查空闲超时 (仅在已收到过数据后生效)
            idle = time.time() - last_data_ts
            if stats.total > 0 and idle >= args.idle_timeout:
                print(f"\n[*] 空闲 {idle:.0f}s 无新帧，自动退出")
                break

            # poll 一帧 (100ms 超时)
            frame = rx.poll(timeout_ms=100)
            if frame is not None:
                last_data_ts = time.time()
                raw = bytes(frame.data[:frame.data_len])

                # 可选 hex dump
                if args.dump > 0 and dump_count < args.dump:
                    hex_str = raw[:64].hex(" ")
                    print(f"\n  [HEX {dump_count+1}/{args.dump}] "
                          f"type=0x{frame.frame_type:04X} len={frame.data_len}")
                    print(f"  {hex_str}")
                    dump_count += 1

                stats.on_frame(
                    frame.frame_type, raw, frame.data_len,
                    frame.device_id, frame.timestamp_ms,
                )

            # 状态行 (覆盖式更新)
            if stats.total > 0 and not args.verbose:
                spinner = spinner_chars[spinner_idx % len(spinner_chars)]
                spinner_idx += 1
                status = (
                    f"\r  {spinner} 收到 {stats.total} 帧 | "
                    f"Radar:{stats.counts['Radar']} "
                    f"ESM:{stats.counts['ESM']} "
                    f"EO:{stats.counts['EO']} "
                    f"AC:{stats.counts['Acoustic']} | "
                    f"空闲 {idle:.0f}s"
                )
                sys.stdout.write(status)
                sys.stdout.flush()
                last_status_line = True

    finally:
        if last_status_line:
            print()  # 换行
        rx.close()

    # ── 输出结果 ──
    print()
    if stats.total == 0:
        print("=" * 64)
        print("  未收到任何帧!")
        print()
        print("  请确认:")
        print("    1. 数据生成器已启动并带 -o 参数")
        print("       cd data_generator")
        print("       LD_LIBRARY_PATH=../thirdparty/linux-x86/lib:../bridge \\")
        print("         python src/main.py config/scenarios/multi_target.yaml -o --no-gui")
        print("    2. LD_LIBRARY_PATH 包含 bridge/ 和 thirdparty/linux-x86/lib/")
        print("    3. DDS 发现已完成 (通常需要 1-3 秒)")
        print("=" * 64)
        sys.exit(1)

    print("=" * 64)
    print("  接收统计")
    print("=" * 64)
    print(stats.summary())

    # 最近一帧
    if stats.last:
        print()
        print("  最近帧:")
        for tag in ["Radar", "ESM", "EO", "Acoustic"]:
            if tag in stats.last:
                print(f"    [{tag}] {stats.last[tag]}")

    print()
    print("  测试完成 ✓")
    print("=" * 64)


if __name__ == "__main__":
    main()
