"""反无人机数据生成系统 — 主入口。

启动仿真控制器和可视化界面，在同一个进程中交替推进仿真和刷新显示。
"""

from __future__ import annotations

import sys
import argparse
import logging
import time
from pathlib import Path

import numpy as np

from scenario.controller import SimulationController

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s'
)
logger = logging.getLogger(__name__)


class SimulationRunner:
    """仿真运行器 — 连接控制器和可视化。

    使用 QTimer 驱动仿真步进，每个 tick:
    1. 推进仿真一个步长
    2. 将最新数据推送到各视图
    3. 更新时间显示
    """

    def __init__(self, scenario_path: str):
        # Keep GUI dependencies out of the headless DDS path.  Target machines
        # that only publish company protocol data do not need PyQt6/pyqtgraph.
        from PyQt6.QtCore import QTimer
        from PyQt6.QtWidgets import QApplication
        from viz.app import MainWindow

        self.controller = SimulationController(scenario_path)
        self.app = QApplication.instance() or QApplication(sys.argv)
        self.app.setStyle('Fusion')

        duration = self.controller.get_duration()
        self.window = MainWindow(duration)

        # 仿真状态
        self._running = False
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)

        # 连接控制信号
        self.window.time_controls.play_signal.connect(self.play)
        self.window.time_controls.pause_signal.connect(self.pause)
        self.window.time_controls.stop_signal.connect(self.stop)
        self.window.time_controls.speed_changed.connect(self.set_speed)

        # 设置传感器位置标记
        self._init_sensor_markers()

    def _init_sensor_markers(self):
        """在综合视图上标记传感器位置。"""
        configs = [
            (self.controller.config.radar, 'radar'),
            (self.controller.config.eo, 'eo'),
            (self.controller.config.esm, 'esm'),
        ]
        for cfg, name in configs:
            if cfg is not None:
                self.window.truth_view.renderer.set_sensor_position(
                    name, cfg.position[0], cfg.position[1], cfg.id
                )

        # 声探测传感器位置（使用默认配置）
        from scenario.config import AcousticConfig
        ac_cfg = self.controller.config.acoustic or AcousticConfig()
        self.window.truth_view.renderer.set_sensor_position(
            'acoustic', ac_cfg.position[0], ac_cfg.position[1], ac_cfg.id
        )

    def _tick(self):
        """仿真步进。"""
        if not self._running:
            return

        # 推进一个或多个步长（根据速度可能需要多步）
        dt = self.controller.get_dt()
        sim_time = self.controller.sim_time
        speed = self.controller.speed

        # 执行仿真步
        result = self.controller.step()

        # 日志文件
        if hasattr(self, '_logger') and self._logger:
            self._logger.tick(result['time'], result)

        # 更新各视图
        self._update_views(result)

        # 更新时间显示
        self.window.set_sim_time(result['time'])

        # 状态栏：雷达显示航迹数（有意义），EO/ESM 显示检测数
        radar_mode = "搜索"
        n_tracks = n_confirmed = 0
        if hasattr(self.controller, '_radar_sensor'):
            radar_mode = self.controller._radar_sensor.get_mode()
            tracks = self.controller._radar_sensor.get_track_states()
            n_tracks = len(tracks)
            n_confirmed = sum(1 for v in tracks.values() if v['state'] == 'confirmed')

        self.window.set_status(
            f"T={result['time']:.1f}s | "
            f"雷达: {radar_mode} 已确认{n_confirmed}/{n_tracks}批 | "
            f"光电: {len(result['eo_detections'])} | "
            f"电侦: {len(result['esm_detections'])}"
        )

        # 检查是否结束
        if self.controller.is_stopped():
            self._running = False
            self.window.set_status("仿真结束")

    def _update_views(self, result: dict):
        """将仿真结果推送到各视图。"""
        views = self.window.get_views()

        # 真值
        for t in result['truth']:
            views['truth'].add_truth(
                t['target_id'], t['label'], t['type'],
                t['x'], t['y'], t['z'], t['speed']
            )
            # 同时更新雷达视图的真值轨迹
            views['radar'].add_truth(t['target_id'], t['x'], t['y'])

        # 雷达检测
        for det in result['radar_detections']:
            views['radar'].add_detection(det)

        # 光电检测 + 状态同步
        for det in result['eo_detections']:
            views['eo'].add_detection(det)
        if hasattr(self.controller, '_eo_sensor'):
            views['eo'].set_state(self.controller._eo_sensor.get_state())

        # 电侦检测
        for det in result['esm_detections']:
            views['esm'].add_detection(det)

        # 声探测
        acoustic_dets = result.get('acoustic_detections', [])
        for det in acoustic_dets:
            views['acoustic'].add_detection(det)

        # 综合态势图上显示声探测位置（批量更新，避免频繁刷新）
        if acoustic_dets and hasattr(self.controller, '_acoustic_sensor'):
            ax, ay, _ = self.controller._acoustic_sensor.position
            positions = []
            for det in acoustic_dets:
                az = np.radians(det.get('azimuth', 0) or 0)
                el = np.radians(det.get('elevation', 0) or 0)
                r_est = det.get('range_estimate', 300) or 300
                gr = r_est * np.cos(el)
                wx = ax + gr * np.cos(az)
                wy = ay + gr * np.sin(az)
                positions.append([wx, wy])
                tid = det.get('track_id', -1)
                if tid >= 0:
                    views['truth'].add_sensor_track_pos('acoustic', tid, wx, wy)
            if positions:
                views['truth'].renderer.plot_detections(
                    'acoustic', np.array(positions),
                    color=(100, 255, 180, 180), symbol='d', size=10
                )

    def play(self):
        """开始/继续仿真。"""
        if self.controller.is_stopped():
            self.controller.reset()
        self.controller.resume()
        self._running = True
        # 定时器间隔取仿真步长和可视化刷新率的折中
        self._timer.start(int(self.controller.get_dt() * 1000))
        self.window.set_status("仿真运行中...")

    def pause(self):
        """暂停仿真。"""
        self._running = False
        self.controller.pause()
        self.window.set_status("已暂停")

    def stop(self):
        """停止仿真。"""
        self._running = False
        self.controller.stop()
        self.controller.reset()
        self.window.set_sim_time(0.0)
        self.window.set_status("已停止")

    def set_speed(self, speed: float):
        """设置仿真速度倍率。"""
        self.controller.speed = speed
        self.window.set_status(f"速度: {speed}x")

    def run(self):
        """启动应用。"""
        self.window.show()
        sys.exit(self.app.exec())

    def shutdown(self):
        """清理资源。"""
        if hasattr(self, '_logger') and self._logger:
            self._logger.close()
        self.controller.shutdown()


class OutputLogger:
    """将四传感器数据按 dev_proto.h 协议格式写入文本文件。"""
    def __init__(self, filepath: str, controller):
        import os
        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        self._f = open(filepath, 'w', encoding='utf-8')
        self._ctrl = controller
        self._last_write = -1.0
        self._interval = 2.0
        self._conv = None

        cfg = controller.config
        self._f.write(f'================================================================================\n')
        self._f.write(f'  dev_proto.h 协议输出 — {cfg.name} ({cfg.simulation.duration}s)\n')
        self._f.write(f'  雷达: {cfg.radar.id} | 光电: {cfg.eo.id} | 电侦: {cfg.esm.id} | 声探测: {cfg.acoustic.id}\n')
        self._f.write(f'  ENU原点: {cfg.coordinates.origin} (WGS84)\n')
        self._f.write(f'================================================================================\n\n')

    def _conv_coord(self):
        if self._conv is None:
            from bus.proto_bridge import CoordConverter
            self._conv = CoordConverter(*self._ctrl.config.coordinates.origin)
        return self._conv

    def tick(self, sim_time, result):
        if sim_time - self._last_write < self._interval:
            return
        self._last_write = sim_time

        truths = result['truth']
        if not truths:
            return
        t = truths[0]
        rx, ry, rz = t['x'], t['y'], t['z']
        conv = self._conv_coord()
        lng_e7, lat_e7, alt_dm = conv.enu_to_wgs84(rx, ry, rz)

        import math
        f = self._f
        f.write(f'═══ T={sim_time:.0f}s ENU({rx:.0f},{ry:.0f},{rz:.0f}) ═══\n\n')

        # ── 雷达帧 (PXSB: 20+4+66=90B) ──
        f.write(f'  [雷达] PXSB报文 MsgHeader(20B)+UnitHeader(4B)+Frame(66B)=90B\n')
        f.write(f'         MsgHdr: startTag=0x7E unitNums=1 msgID={int(sim_time*2)} srcAddr=101\n')
        f.write(f'         UnitHdr: unitTag=0x1A01 unitLen=66\n')
        f.write(f'         Frame: Magic=0xA55AA55A\n')
        if hasattr(self._ctrl, '_radar_sensor'):
            tracks = self._ctrl._radar_sensor.get_track_states()
            for tid, info in tracks.items():
                if info['state'] in ('confirmed','coasting') and info.get('predicted') is not None:
                    px, py, pz = info['predicted']
                    plng, plat, palt = conv.enu_to_wgs84(px, py, pz)
                    sx, sy, sz = self._ctrl._radar_sensor.position
                    dx, dy, dz = px-sx, py-sy, pz-sz
                    r = math.sqrt(dx*dx+dy*dy+dz*dz)
                    az = math.degrees(math.atan2(dy, dx)) % 360
                    el = math.degrees(math.atan2(dz, math.sqrt(dx*dx+dy*dy)))
                    f.write(f'    T{tid}: state={info["state"]} hits={info["hits"]}\n')
                    f.write(f'    L1量测: range={r*10:.0f}dm az={az*100:.0f}cdeg el={el*100:.0f}cdeg height={dz*10:.0f}dm\n')
                    f.write(f'    L2航迹: lng_e7={plng} lat_e7={plat} alt_dm={palt}\n')
                    f.write(f'            → WGS84({plng/1e7:.6f}E, {plat/1e7:.6f}N, {palt/10:.1f}m)\n')
                    f.write(f'    L3识别: type=2(无人机) confidence=80%\n')
        else:
            f.write(f'    (无确认航迹)\n')

        # ── 光电帧 (PXSB: 20+4+79=103B) ──
        f.write(f'  [光电] PXSB报文 MsgHeader(20B)+UnitHeader(4B)+Frame(79B)=103B\n')
        f.write(f'         UnitHdr: unitTag=0x1C01 Magic=0xC33CC33C\n')
        if hasattr(self._ctrl, '_eo_sensor'):
            eo = self._ctrl._eo_sensor.get_state()
            f.write(f'    state={eo["state"]} n_locked={eo["n_locked"]} pointing=Az{eo["current_az"]:.1f}° El{eo["current_el"]:.1f}°\n')
            sx, sy, sz = self._ctrl._eo_sensor.position
            slng, slat, salt = conv.enu_to_wgs84(sx, sy, sz)
            f.write(f'    转台WGS84: {slng/1e7:.6f}E, {slat/1e7:.6f}N, {salt/10:.1f}m\n')
            f.write(f'    目标WGS84: {lng_e7/1e7:.6f}E, {lat_e7/1e7:.6f}N, {alt_dm/10:.1f}m\n')
        else:
            f.write(f'    (未就绪)\n')

        # ── 频侦帧 (PXSB: 20+4+139=163B) ──
        f.write(f'  [频侦] PXSB报文 MsgHeader(20B)+UnitHeader(4B)+Frame(139B)=163B\n')
        f.write(f'         UnitHdr: unitTag=0x1B01 Magic=0xB44BB44B\n')
        esm_dets = getattr(self._ctrl, '_last_esm_dets', [])
        if esm_dets:
            d = esm_dets[0]
            f.write(f'    track_id={d["track_id"]} azimuth={d["azimuth"]:.1f}° freq={d.get("frequency",0)/1e9:.2f}GHz\n')
            f.write(f'    confidence={d["confidence"]:.0%} 目标WGS84: {lng_e7/1e7:.6f}E, {lat_e7/1e7:.6f}N\n')
        else:
            f.write(f'    (当前帧无检测)\n')

        # ── 声探测帧 (PXSB: 20+4+67=91B) ──
        f.write(f'  [声探测] PXSB报文 MsgHeader(20B)+UnitHeader(4B)+Frame(67B)=91B\n')
        f.write(f'         UnitHdr: unitTag=0x1D01 Magic=0xD22DD22D (含设备经纬高)\n')
        ac_dets = getattr(self._ctrl, '_last_acoustic_dets', [])
        if ac_dets:
            d = ac_dets[0]
            f.write(f'    track_id={d["track_id"]} azimuth={d["azimuth"]:.1f}° elevation={d["elevation"]:.1f}°\n')
            f.write(f'    class={d.get("classification","?")} confidence={d["confidence"]:.0%}\n')
        else:
            f.write(f'    (距离远，无检测)\n')

        f.write('\n')

    def close(self):
        self._f.write('================================================================================\n')
        self._f.write('  输出完成\n')
        self._f.write('================================================================================\n')
        self._f.close()


def run_headless(scenario_path: str, output: bool = False,
                 log_file: str | None = None, realtime: bool = False):
    """无 GUI 模式：运行仿真并将数据输出到终端。"""
    controller = SimulationController(scenario_path)
    dt = controller.get_dt()
    duration = controller.get_duration()
    steps = int(duration / dt)

    # 启用二进制协议输出（DDS 参数全部用 third_party 默认值）
    if output:
        if controller.enable_output():
            print("二进制协议输出已启用 (DDS third_party 默认)")
        else:
            print("警告: 协议输出不可用 (libad_proto.so 未找到)")

    # 启用日志文件输出
    logger = None
    if log_file:
        logger = OutputLogger(log_file, controller)
        print(f"传感器数据将保存到: {log_file}")

    print(f"场景: {controller.config.name}")
    print(f"时长: {duration}s, 步长: {dt}s, 总步数: {steps}")
    print(f"目标数: {len(controller.targets)}")
    print(f"{'='*60}")

    try:
        acc_radar = acc_eo = acc_esm = 0  # 累积计数器
        for i in range(steps):
            wall_started = time.monotonic() if realtime else None
            result = controller.step()
            t = result['time']

            # 累积检测数
            acc_radar += len(result['radar_detections'])
            acc_eo += len(result['eo_detections'])
            acc_esm += len(result['esm_detections'])
            acc_acoustic = len(result.get('acoustic_detections', []))

            # 日志文件
            if logger:
                logger.tick(t, result)

            if i % 40 == 39:  # 每 2 秒输出一次（累积值）
                truths = result['truth']
                pos_str = ", ".join(
                    f"T{t['target_id']}({t['x']:.0f},{t['y']:.0f})"
                    for t in truths[:3]
                )
                # 传感器状态
                radar_mode = "?"
                eo_mode = "?"
                if hasattr(controller, '_radar_sensor'):
                    radar_mode = controller._radar_sensor.get_mode()
                    tracks = controller._radar_sensor.get_track_states()
                    n_confirmed = sum(1 for v in tracks.values() if v['state'] == 'confirmed')
                    radar_mode = f"{radar_mode[:4]}({n_confirmed}c)"

                # 光电状态 + 引导事件
                cue_info = ""
                if hasattr(controller, '_eo_sensor'):
                    eo_state = controller._eo_sensor.get_state()
                    eo_mode = f"{eo_state['state'][:4]}"
                    if eo_state['state'] == 'slewing':
                        cue_info = f" →Az{eo_state['cue_az']:.0f}°"
                    elif eo_state['n_locked'] > 0:
                        eo_mode += f"({eo_state['n_locked']}锁)"
                    else:
                        eo_mode += f"({eo_state['n_locked']})"
                    if eo_state['cue_az'] is not None and eo_state['state'] != 'tracking':
                        cue_info = f" cue:{eo_state['cue_az']:.0f}°"

                print(f"T={t:6.1f}s | {pos_str}... | 雷达:{radar_mode} 光电:{eo_mode}{cue_info} R:{acc_radar} EO:{acc_eo} ESM:{acc_esm} AC:{acc_acoustic}")
                acc_radar = acc_eo = acc_esm = 0  # 清零

            if realtime:
                elapsed = time.monotonic() - wall_started
                time.sleep(max(0.0, dt / max(controller.speed, 0.01) - elapsed))

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        if logger:
            logger.close()
        controller.shutdown()
        print(f"\n仿真结束。最终时间: {controller.sim_time:.1f}s")


def main():
    parser = argparse.ArgumentParser(
        description="反无人机数据生成系统 — 多传感器目标轨迹仿真"
    )
    parser.add_argument(
        'scenario',
        nargs='?',
        default='config/scenarios/multi_target.yaml',
        help='场景配置文件路径 (默认: config/scenarios/multi_target.yaml)'
    )
    parser.add_argument(
        '--list-scenarios',
        action='store_true',
        help='列出所有可用的场景文件'
    )
    parser.add_argument(
        '--no-gui',
        action='store_true',
        help='无 GUI 模式，仅输出仿真数据到终端'
    )
    parser.add_argument(
        '--output', '-o',
        action='store_true',
        default=False,
        help='启用 DDS 轨迹输出 (使用 third_party 默认配置)'
    )
    parser.add_argument(
        '--realtime',
        action='store_true',
        default=False,
        help='无 GUI 模式按仿真步长实时运行，供在线 DDS 消费者使用'
    )
    parser.add_argument(
        '--log',
        action='store_true',
        default=False,
        help='保存四传感器输出到 output/<场景名>_output.txt'
    )
    parser.add_argument(
        '--log-file',
        default=None,
        metavar='FILE',
        help='指定输出文件路径 (覆盖默认路径)'
    )

    args = parser.parse_args()

    if args.list_scenarios:
        scenarios_dir = Path('config/scenarios')
        if scenarios_dir.exists():
            for f in sorted(scenarios_dir.glob('*.yaml')):
                print(f"  {f}")
        return

    scenario_path = Path(args.scenario)
    if not scenario_path.exists():
        print(f"错误: 场景文件不存在: {scenario_path}")
        print("使用 --list-scenarios 查看可用场景")
        sys.exit(1)

    # 日志文件路径
    log_file = None
    if args.log or args.log_file:
        if args.log_file:
            log_file = args.log_file
        else:
            import os
            name = os.path.splitext(os.path.basename(scenario_path))[0]
            log_file = f'output/{name}_output.txt'

    if args.no_gui:
        run_headless(
            str(scenario_path),
            output=args.output,
            log_file=log_file,
            realtime=args.realtime,
        )
    else:
        # GUI 模式：也支持 --output / --log
        runner = SimulationRunner(str(scenario_path))
        if args.output:
            runner.controller.enable_output()
        if log_file:
            runner._logger = OutputLogger(log_file, runner.controller)
        try:
            runner.run()
        except KeyboardInterrupt:
            logger.info("用户中断")
        finally:
            runner.shutdown()


if __name__ == '__main__':
    main()
