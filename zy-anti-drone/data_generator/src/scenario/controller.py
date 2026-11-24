"""场景控制器。

管理仿真生命周期：加载配置、初始化目标与传感器、推进仿真时钟、发布数据。
"""

from __future__ import annotations

import sys
import time
import signal
import logging
import numpy as np
from multiprocessing import Process, Event
from pathlib import Path
from typing import Optional

from scenario.config import ScenarioConfig, load_scenario
from truth.target import Target, EmitterParams
from truth.trajectory import WaypointTrajectory
from truth.motion_models import MotionState
from bus.message import Detection, Track, SimulationClock, DetectionFlag
from bus.publisher import Publisher
from bus.shm_pool import ShmPool

logger = logging.getLogger(__name__)


class SimulationController:
    """仿真总控制器。

    协调真值模型、传感器进程和可视化之间的数据流通。
    """

    def __init__(self, scenario_path: str | Path):
        self.config = load_scenario(scenario_path)
        self.scenario_path = Path(scenario_path)
        self.targets: list[Target] = []
        self.sensor_publishers: dict[str, Publisher] = {}
        self.shm_pool = ShmPool()
        self._stop_event = Event()
        self._paused = Event()
        self._paused.set()  # 初始为暂停状态

        # 仿真状态
        self.sim_time: float = 0.0
        self.speed: float = 1.0

        # 外部轨迹输出（可选，UDP 发送）
        self._output_bridge = None
        self._output_publisher = None
        self._output_enabled = False

        self._init_targets()
        self._init_publishers()

    def _init_targets(self):
        """从配置初始化目标。"""
        for tc in self.config.targets:
            trajectory = WaypointTrajectory.from_waypoint_lists(tc.waypoints)
            emitter = EmitterParams(
                active=tc.emitter.active,
                freq=tc.emitter.freq,
                pri=tc.emitter.pri,
                pw=tc.emitter.pw,
                modulation=tc.emitter.modulation,
                power=tc.emitter.power,
                active_after=tc.emitter.active_after,
                active_until=tc.emitter.active_until,
            )
            target = Target(
                id=tc.id,
                label=tc.label,
                target_type=tc.type,
                rcs=tc.rcs,
                trajectory=trajectory,
                emitter=emitter,
            )
            self.targets.append(target)

        logger.info(f"已初始化 {len(self.targets)} 个目标")

    def _init_publishers(self):
        """为每个传感器创建发布者。"""
        sensor_configs = {
            'radar': self.config.radar,
            'eo': self.config.eo,
            'esm': self.config.esm,
            'acoustic': self.config.acoustic,
        }
        for name, cfg in sensor_configs.items():
            if cfg is not None:
                publisher = Publisher(cfg.id, self.shm_pool)
                self.sensor_publishers[name] = publisher
                logger.info(f"传感器 {cfg.id} ({name}) 发布者已就绪")

    def get_duration(self) -> float:
        """获取仿真总时长。"""
        return self.config.simulation.duration

    def get_dt(self) -> float:
        """获取仿真步长。"""
        return self.config.simulation.dt

    def step(self) -> dict:
        """推进一个仿真步长。

        Returns:
            dict: {
                'time': float,
                'truth': list[dict],            # 真值状态
                'radar_detections': list[dict],  # 雷达检测
                'eo_detections': list[dict],     # 光电检测
                'esm_detections': list[dict],    # 电侦检测
            }
        """
        dt = self.config.simulation.dt
        self.sim_time += dt * self.speed

        # 检查是否结束
        if self.sim_time >= self.config.simulation.duration:
            self._stop_event.set()

        result = {
            'time': self.sim_time,
            'truth': self._compute_truth(),
            'radar_detections': self._simulate_radar(),
            'eo_detections': self._simulate_eo(),
            'esm_detections': self._simulate_esm(),
            'acoustic_detections': self._simulate_acoustic(),
        }

        # UDP 轨迹输出
        self._publish_output()

        return result

    def _compute_truth(self) -> list[dict]:
        """计算所有目标的真值状态。"""
        truths = []
        for target in self.targets:
            state = target.get_state(self.sim_time)
            if state is None:
                continue
            truths.append({
                'target_id': target.id,
                'label': target.label,
                'type': target.target_type,
                'x': float(state.position[0]),
                'y': float(state.position[1]),
                'z': float(state.position[2]),
                'vx': float(state.velocity[0]),
                'vy': float(state.velocity[1]),
                'vz': float(state.velocity[2]),
                'speed': float(state.speed),
                'heading': float(state.heading),
                'emitting': target.is_emitting(self.sim_time),
            })
        return truths

    def _simulate_radar(self) -> list[dict]:
        """模拟雷达检测（使用 RadarSensor 模型）。"""
        cfg = self.config.radar
        if cfg is None:
            return []

        # 延迟创建 RadarSensor 实例（使用真实模型参数）
        if not hasattr(self, '_radar_sensor'):
            scan_cfg = cfg.scan if hasattr(cfg, 'scan') else None
            self._radar_sensor = self._build_radar_sensor(cfg, scan_cfg)
            self._radar_sensor.set_targets(self.targets)

        # 执行检测
        raw_detections = self._radar_sensor.detect(self.sim_time)

        # 转换为字典格式（供可视化使用）
        sensor_pos = np.array(cfg.position)
        detections = []
        for det in raw_detections:
            # 世界坐标
            if det.truth_position is not None:
                wx, wy = det.truth_position[0], det.truth_position[1]
            else:
                az_rad = np.radians(det.measurements.get('azimuth', 0))
                el_rad = np.radians(det.measurements.get('elevation', 0))
                gr = det.measurements.get('range', 0) * np.cos(el_rad)
                wx = sensor_pos[0] + gr * np.cos(az_rad)
                wy = sensor_pos[1] + gr * np.sin(az_rad)

            is_clutter = DetectionFlag.CLUTTER in det.flags
            detections.append({
                'timestamp': det.timestamp,
                'sensor_id': det.sensor_id,
                'track_id': det.track_id,
                'range': det.measurements.get('range', 0),
                'azimuth': det.measurements.get('azimuth', 0),
                'elevation': det.measurements.get('elevation', 0),
                'doppler': det.measurements.get('doppler', 0),
                'x': wx, 'y': wy,
                'confidence': det.confidence,
                'is_clutter': is_clutter,
            })

        # 发布到共享内存
        if 'radar' in self.sensor_publishers:
            pub = self.sensor_publishers['radar']
            import json
            for det in detections:
                pub.get_buffer().write(json.dumps(det).encode('utf-8'))

        return detections

    def _build_radar_sensor(self, cfg, scan_cfg):
        """从配置构建 RadarSensor 实例。"""
        from sensors.radar import RadarSensor

        scan_type = scan_cfg.type if scan_cfg else "staring"
        kwargs = dict(
            sensor_id=cfg.id,
            position=cfg.position,
            orientation=cfg.orientation,
            update_rate=cfg.update_rate,
            max_range=cfg.max_range,
            min_range=cfg.min_range,
            fov={'azimuth': cfg.fov.azimuth, 'elevation': cfg.fov.elevation},
            error_sigma={
                'range': cfg.error.range_sigma,
                'azimuth': cfg.error.azimuth_sigma,
                'elevation': cfg.error.elevation_sigma,
                'doppler': cfg.error.doppler_sigma,
            },
            error_bias={
                'range': cfg.error.range_bias,
                'azimuth': cfg.error.azimuth_bias,
                'elevation': cfg.error.elevation_bias,
            },
            pd=cfg.detection.pd,
            pfa=cfg.detection.pfa,
            snr_threshold=cfg.detection.snr_threshold,
            clutter_density=cfg.clutter.density,
            clutter_enabled=cfg.clutter.enabled,
            scan_type=scan_type,
            azimuth_rate=scan_cfg.azimuth_rate if scan_cfg else 60.0,
            azimuth_beamwidth=scan_cfg.azimuth_beamwidth if scan_cfg else 3.6,
            scan_sector=scan_cfg.scan_sector if scan_cfg else [0.0, 360.0],
            reference_rcs=cfg.reference_rcs if hasattr(cfg, 'reference_rcs') else 0.01,
            reference_range=cfg.reference_range if hasattr(cfg, 'reference_range') else 5000.0,
            max_velocity=cfg.max_velocity if hasattr(cfg, 'max_velocity') else 100.0,
        )
        return RadarSensor(**kwargs)

    def _simulate_eo(self) -> list[dict]:
        """模拟光电检测 — 雷达引导模式。"""
        cfg = self.config.eo
        if cfg is None:
            return []

        # 延迟创建 EO 传感器
        if not hasattr(self, '_eo_sensor'):
            self._eo_sensor = self._build_eo_sensor(cfg)
            self._eo_sensor.set_targets(self.targets)

        # ── 雷达引导光电 ──
        # 如果 EO 空闲且雷达有确认航迹，发送引导指令
        if hasattr(self, '_radar_sensor'):
            eo_state = self._eo_sensor.get_state()
            if eo_state['state'] == 'idle':
                radar_tracks = self._radar_sensor.get_track_states()
                # 优先引导确认的航迹
                confirmed = [
                    (tid, info) for tid, info in radar_tracks.items()
                    if info['state'] == 'confirmed' and info['predicted'] is not None
                ]
                if confirmed:
                    # 选择最近的确认航迹
                    tid, info = confirmed[0]
                    pred = info['predicted']
                    az = np.degrees(np.arctan2(
                        pred[1] - cfg.position[1],
                        pred[0] - cfg.position[0]
                    )) % 360
                    el = np.degrees(np.arctan2(
                        pred[2] - cfg.position[2],
                        np.linalg.norm(pred[:2] - np.array(cfg.position)[:2])
                    ))
                    self._eo_sensor.receive_cue(az, el, tid, confidence=0.7)

        # 执行检测
        raw_detections = self._eo_sensor.detect(self.sim_time)

        # 转换为字典
        sensor_pos = np.array(cfg.position)
        eo_state = self._eo_sensor.get_state()
        detections = []
        for det in raw_detections:
            detections.append({
                'timestamp': det.timestamp,
                'sensor_id': det.sensor_id,
                'track_id': det.track_id,
                'azimuth': det.measurements.get('azimuth', 0),
                'elevation': det.measurements.get('elevation', 0),
                'confidence': det.confidence,
                'locked': True,
                'error_az': cfg.error.azimuth_sigma,
                'error_el': cfg.error.elevation_sigma,
                'eo_state': eo_state['state'],
            })

        # 发布
        if 'eo' in self.sensor_publishers:
            pub = self.sensor_publishers['eo']
            import json
            for det in detections:
                pub.get_buffer().write(json.dumps(det).encode('utf-8'))

        return detections

    def _build_eo_sensor(self, cfg):
        """从配置构建 EO 传感器。"""
        from sensors.electro_optical import ElectroOpticalSensor
        return ElectroOpticalSensor(
            sensor_id=cfg.id,
            position=cfg.position,
            orientation=cfg.orientation,
            update_rate=cfg.update_rate,
            fov={'azimuth': cfg.fov.azimuth, 'elevation': cfg.fov.elevation},
            max_track_range=cfg.max_track_range,
            error_sigma={
                'azimuth': cfg.error.azimuth_sigma,
                'elevation': cfg.error.elevation_sigma,
            },
            pd=cfg.detection.pd,
            lock_on_time=cfg.tracking.lock_on_time,
            max_track_targets=cfg.tracking.max_track_targets,
            handover_delay=cfg.tracking.handover_delay,
            visibility_km=cfg.weather.visibility,
        )

    def _simulate_esm(self) -> list[dict]:
        """模拟电侦检测（简化版）。"""
        cfg = self.config.esm
        if cfg is None:
            return []

        # 延迟构建 ESMSensor 对象（用于 DDS 协议发布）
        if not hasattr(self, '_esm_sensor'):
            self._esm_sensor = self._build_esm_sensor(cfg)
            self._esm_sensor.set_targets(self.targets)
            logger.info(f"电侦传感器已就绪: {cfg.id}")

        detections = []
        sensor_pos = np.array(cfg.position)

        for target in self.targets:
            if not target.is_emitting(self.sim_time):
                continue

            state = target.get_state(self.sim_time)
            if state is None:
                continue

            # 截获概率
            poi = cfg.detection.poi
            if np.random.random() > poi:
                continue

            rel_pos = state.position - sensor_pos
            az = float(np.arctan2(rel_pos[1], rel_pos[0]))

            # 全向 ESM，但仍检查 FOV
            if not (np.radians(cfg.fov.azimuth[0]) <= az <= np.radians(cfg.fov.azimuth[1])):
                continue

            # 测量值
            az_meas = az + np.random.normal(0, np.radians(cfg.error.azimuth_sigma))
            freq_meas = target.emitter.freq + np.random.normal(0, cfg.error.frequency_sigma)
            pri_meas = target.emitter.pri + np.random.normal(0, cfg.error.pri_quantization)
            pw_meas = target.emitter.pw + np.random.normal(0, cfg.error.pw_quantization)

            detections.append({
                'timestamp': self.sim_time,
                'sensor_id': cfg.id,
                'track_id': target.id,
                'azimuth': np.degrees(az_meas),
                'frequency': freq_meas,
                'pri': pri_meas,
                'pw': pw_meas,
                'modulation': target.emitter.modulation,
                'confidence': poi,
            })

        # 发布
        if 'esm' in self.sensor_publishers:
            pub = self.sensor_publishers['esm']
            import json
            for det in detections:
                pub.get_buffer().write(json.dumps(det).encode('utf-8'))

        self._last_esm_dets = detections  # 用于协议输出
        return detections

    def _simulate_acoustic(self) -> list[dict]:
        """模拟声探测。场景未配置时自动使用默认参数。"""
        cfg = self.config.acoustic

        if not hasattr(self, '_acoustic_sensor'):
            self._acoustic_sensor = self._build_acoustic_sensor(cfg)
            self._acoustic_sensor.set_targets(self.targets)
            logger.info(f"声探测传感器已就绪: {cfg.id}")

        raw = self._acoustic_sensor.detect(self.sim_time)

        detections = []
        for det in raw:
            d = {
                'timestamp': det.timestamp,
                'sensor_id': det.sensor_id,
                'track_id': det.track_id,
                'azimuth': det.measurements.get('azimuth', 0),
                'elevation': det.measurements.get('elevation', 0),
                'frequency': det.measurements.get('frequency', 0),
                'sound_level': det.measurements.get('sound_level', 0),
                'range_estimate': det.measurements.get('range_estimate', 0),
                'classification': det.measurements.get('classification', ''),
                'confidence': det.confidence,
            }
            detections.append(d)
        self._last_acoustic_dets = detections  # 用于协议输出

        if 'acoustic' in self.sensor_publishers:
            pub = self.sensor_publishers['acoustic']
            import json
            for det in detections:
                pub.get_buffer().write(json.dumps(det).encode('utf-8'))

        return detections

    def _build_esm_sensor(self, cfg):
        from sensors.esm import ESMSensor
        return ESMSensor(
            sensor_id=cfg.id, position=cfg.position, orientation=cfg.orientation,
            update_rate=cfg.update_rate,
            fov={'azimuth': cfg.fov.azimuth, 'elevation': cfg.fov.elevation},
            frequency_range=(cfg.frequency_range[0], cfg.frequency_range[1]),
            error_sigma={
                'azimuth': cfg.error.azimuth_sigma,
                'frequency': cfg.error.frequency_sigma,
            },
            poi=cfg.detection.poi,
            sensitivity_dbm=cfg.detection.sensitivity,
        )

    def _build_acoustic_sensor(self, cfg):
        from sensors.acoustic import AcousticSensor
        return AcousticSensor(
            sensor_id=cfg.id, position=cfg.position, orientation=cfg.orientation,
            update_rate=cfg.update_rate,
            fov={'azimuth': cfg.fov.azimuth, 'elevation': cfg.fov.elevation},
            max_range=cfg.max_range, min_range=cfg.min_range,
            error_sigma={
                'azimuth': cfg.error.azimuth_sigma,
                'elevation': cfg.error.elevation_sigma,
                'frequency': cfg.error.frequency_sigma,
            },
            pd=cfg.detection.pd,
            reference_range=getattr(cfg, 'reference_range', 500.0),
            sound_level_ref=getattr(cfg, 'sound_level_ref', 80.0),
            ambient_noise_db=cfg.environment.ambient_noise_db,
            wind_speed=cfg.environment.wind_speed,
        )

    def pause(self):
        self._paused.clear()

    def resume(self):
        self._paused.set()

    def stop(self):
        self._stop_event.set()

    def is_stopped(self) -> bool:
        return self._stop_event.is_set()

    def reset(self):
        self.sim_time = 0.0
        self._stop_event.clear()

    def enable_output(self) -> bool:
        """启用二进制协议输出（DDS 参数全部使用 third_party 库默认值）。

        Returns:
            bool: 是否成功
        """
        try:
            from bus.proto_bridge import ProtoBridge, ProtoPublisher

            # 从配置读取 ENU 原点对应的 WGS84 坐标
            coord = self.config.coordinates
            ref_lng, ref_lat, ref_alt = coord.origin[0], coord.origin[1], coord.origin[2]

            self._output_bridge = ProtoBridge()
            if not self._output_bridge.available:
                logger.warning("协议桥接库 libad_proto.so 不可用")
                return False

            if not self._output_bridge.init(ref_lng, ref_lat, ref_alt):
                logger.warning("协议桥接初始化失败")
                return False

            self._output_publisher = ProtoPublisher(self._output_bridge)
            self._output_enabled = True
            logger.info("二进制协议输出已启用 (dev_proto.h) → DDS (third_party 默认配置)")
            return True
        except Exception as e:
            logger.error(f"启用协议输出失败: {e}")
            return False

    def _publish_output(self):
        """按 dev_proto.h 二进制协议发布当前帧的传感器数据。

        发布频率：约 2Hz（每 0.5s），避免高频调用 DDS C 层。
        """
        if not self._output_enabled or self._output_publisher is None:
            return

        # 限频：约 2Hz
        last = getattr(self, '_last_output_time', 0)
        if self.sim_time - last < 0.5:
            return
        self._last_output_time = self.sim_time

        # 发布计数
        counts = getattr(self, '_pub_counts', None)
        if counts is None:
            self._pub_counts = {'radar': 0, 'eo': 0, 'esm': 0, 'acoustic': 0}
            self._pub_last_report = 0
            counts = self._pub_counts

        try:
            pub = self._output_publisher

            # 雷达航迹 → ad/radar/target (0x1A01)
            if hasattr(self, '_radar_sensor'):
                pub.publish_radar_tracks(self.sim_time, self._radar_sensor, self.targets)
                tracks = self._radar_sensor.get_track_states()
                counts['radar'] += sum(1 for t in tracks.values() if t['state'] in ('confirmed','coasting'))

            # 光电状态 → EOTarget (0x1C01)  仅在有目标跟踪时发送
            if hasattr(self, '_eo_sensor'):
                if pub.publish_eo_state(self.sim_time, self._eo_sensor, self.targets):
                    counts['eo'] += 1

            # 电侦检测 → ad/freqdetect/target (0x1B01)
            if hasattr(self, '_esm_sensor'):
                esm_dets = getattr(self, '_last_esm_dets', [])
                if esm_dets:
                    pub.publish_esm_detections(self.sim_time, esm_dets, self._esm_sensor, self.targets)
                    counts['esm'] += len(esm_dets)

            # 声探测 → ad/acoustic/target (0x1D01)
            if hasattr(self, '_acoustic_sensor'):
                ac_dets = getattr(self, '_last_acoustic_dets', [])
                if ac_dets:
                    pub.publish_acoustic_detections(self.sim_time, ac_dets, self._acoustic_sensor)
                    counts['acoustic'] += len(ac_dets)
        except Exception as e:
            if not getattr(self, '_output_error_logged', False):
                import traceback
                logger.warning(f"协议输出异常: {e}\n{traceback.format_exc()}")
                self._output_error_logged = True

        # 每5秒打印发布统计
        if self.sim_time - self._pub_last_report >= 5.0:
            c = self._pub_counts
            logger.info(f"[DDS] T={self.sim_time:.0f}s 累计发布: "
                        f"RadarTarget={c['radar']} EOTarget={c['eo']} "
                        f"ESMTarget={c['esm']} AcousticTarget={c['acoustic']}")
            self._pub_last_report = self.sim_time

    def shutdown(self):
        """关闭所有资源。"""
        if self._output_bridge:
            try:
                self._output_bridge.close()
            except Exception:
                pass  # DDS 析构可能 crash，忽略
        self.shm_pool.close_all()
        self.shm_pool.unlink_all()
