"""雷达传感器模型。

支持两种扫描体制：
- staring (凝视/电扫): 瞬时覆盖整个FOV，高数据率
- mechanical (机械扫描): 波束旋转扫描，目标被周期性照射

参照《雷达探测设备技术协议》Ku波段三坐标雷达建模。
"""

from __future__ import annotations

import numpy as np
from sensors.base import SensorBase
from sensors.error_models import (
    GaussianNoise, Bias, OutlierModel, TimeDelay,
    DetectionProbability, ErrorPipeline,
)
from truth.target import Target
from bus.message import Detection, DetectionFlag


class RadarSensor(SensorBase):
    """雷达传感器模拟器。

    测量量:
    - range: 距离 (m)
    - azimuth: 方位角 (°)
    - elevation: 俯仰角 (°)
    - doppler: 径向速度 (m/s)

    扫描体制:
    - staring: 电扫/凝视，瞬时覆盖FOV
    - mechanical: 方位机械扫描，波束旋转照射
    """

    def __init__(
        self,
        sensor_id: str = "radar-01",
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        update_rate: float = 10.0,
        max_range: float = 5000.0,
        min_range: float = 200.0,
        fov: dict | None = None,
        error_sigma: dict | None = None,
        error_bias: dict | None = None,
        pd: float = 0.85,
        pfa: float = 1e-6,
        snr_threshold: float = 13.0,
        clutter_density: float = 1e-4,
        clutter_enabled: bool = True,
        outlier_prob: float = 0.01,
        time_delay: float = 0.01,
        # ── 扫描参数 ──
        scan_type: str = "staring",
        azimuth_rate: float = 60.0,
        azimuth_beamwidth: float = 3.6,
        scan_sector: list[float] | None = None,
        # ── 参考参数 ──
        reference_rcs: float = 0.01,
        reference_range: float = 5000.0,
        max_velocity: float = 100.0,
    ):
        """
        Args:
            sensor_id: 传感器标识
            position: [x, y, z] 部署位置 (m)
            orientation: [az, el] 指向 (°)
            update_rate: 数据更新频率 (Hz)
            max_range: 最大探测距离 (m)
            min_range: 最小探测距离/盲区 (m)
            fov: 视场范围 (°)
            error_sigma: 测量噪声标准差
            error_bias: 系统偏差
            pd: 标称检测概率
            pfa: 虚警概率
            snr_threshold: 检测门限 SNR (dB)
            clutter_density: 杂波密度
            clutter_enabled: 是否启用地杂波
            outlier_prob: 野值概率
            time_delay: 处理延迟 (s)
            scan_type: 扫描体制 "staring" / "mechanical"
            azimuth_rate: 方位扫描速率 (°/s)，仅 mechanical
            azimuth_beamwidth: 方位波束宽度 (°)，仅 mechanical
            scan_sector: 扫描扇区 [min, max] (°)，仅 mechanical
            reference_rcs: 参考 RCS (m²)，用于 Pd 计算
            reference_range: 参考距离 (m)，Pd=85% 时的距离
            max_velocity: 最大可测量速度 (m/s)
        """
        position = position or [0.0, 0.0, 5.0]
        orientation = orientation or [0.0, 0.0]
        fov = fov or {'azimuth': [0.0, 360.0], 'elevation': [0.0, 50.0]}

        super().__init__(sensor_id, 'radar', position, orientation, update_rate, fov)

        self.max_range = max_range
        self.min_range = min_range
        self.error_sigma = error_sigma or {
            'range': 10.0, 'azimuth': 0.3, 'elevation': 0.85, 'doppler': 0.5,
        }
        self.error_bias = error_bias or {
            'range': 0.0, 'azimuth': 0.0, 'elevation': 0.0,
        }
        self.pd_nominal = pd
        self.pfa = pfa
        self.snr_threshold = snr_threshold
        self.clutter_density = clutter_density
        self.clutter_enabled = clutter_enabled
        self.outlier_prob = outlier_prob
        self.time_delay = time_delay

        # 扫描参数
        self.scan_type = scan_type
        self.azimuth_rate = azimuth_rate      # °/s
        self.azimuth_beamwidth = azimuth_beamwidth  # °
        self.scan_sector = scan_sector or [0.0, 360.0]  # °

        # 参考参数
        self.reference_rcs = reference_rcs
        self.reference_range = reference_range
        self.max_velocity = max_velocity

        # 检测概率模型
        self.pd_model = DetectionProbability(
            pd_nominal=pd, snr_threshold=snr_threshold
        )

        # 目标跟踪
        self._targets: list[Target] = []
        self._last_known_positions: dict[int, np.ndarray] = {}
        self._last_detect_time: dict[int, float] = {}  # 上次检测到的时间

        # ── 航迹管理 ──
        # 每个目标的航迹状态: "searching" | "tentative" | "confirmed" | "coasting"
        self._track_state: dict[int, str] = {}
        self._track_hits: dict[int, int] = {}       # 累计检测次数
        self._track_misses: dict[int, int] = {}     # 连续漏检次数
        self._track_predicted: dict[int, np.ndarray] = {}  # 预测位置
        self._track_velocity: dict[int, np.ndarray] = {}   # 估计速度

        # M/N 确认逻辑: 需要 M 次检测中至少有 N 次才确认
        self._confirm_m = 3   # 检测窗口大小 (扫描圈数)
        self._confirm_n = 2   # 最少检测次数 (确认门限)
        self._coast_max = 6   # 最大外推圈数 (超出则删除航迹)
        self._track_sector_width = 30.0  # 跟踪扇区宽度 (°)，确认后聚焦此扇区

        # 当前工作模式: "search" (全周) / "track" (扇区跟) / "tws" (边扫边跟)
        self._mode = "search"
        self._mode_track_centers: list[float] = []  # 跟踪扇区中心方位角
        self._tws_enabled = True  # 是否启用 TWS 模式（确认后保持搜索+预测）

    def set_targets(self, targets: list[Target]):
        self._targets = targets
        for t in targets:
            self._track_state.setdefault(t.id, "searching")

    def _setup_error_pipeline(self):
        self._error_pipeline.add_stage(GaussianNoise(self.error_sigma))
        self._error_pipeline.add_stage(Bias(self.error_bias))
        self._error_pipeline.add_stage(OutlierModel(
            probability=self.outlier_prob, scale_factor=10.0,
        ))

    # ── 机械扫描 ──

    def _get_beam_center(self, sim_time: float) -> float:
        """计算机扫波束当前指向的方位角 (°)。

        搜索模式：全周 360° 或配置的扇区扫描。
        跟踪模式：在确认目标的扇区内快速往返扫描。
        """
        if self._mode == "track" and self._mode_track_centers:
            # 纯跟踪模式：窄扇区聚焦扫描，快速更新
            centers = self._mode_track_centers
            centers = self._mode_track_centers
            if len(centers) == 1:
                # 单目标：窄扇区围绕目标
                tc = centers[0]
                half_width = self._track_sector_width / 2
                sec_start = tc - half_width
                sec_end = tc + half_width
            else:
                # 多目标：覆盖所有目标的扇区
                tc_min, tc_max = min(centers), max(centers)
                padding = self._track_sector_width / 2
                sec_start = tc_min - padding
                sec_end = tc_max + padding

            sec_start = sec_start % 360
            sec_end = sec_end % 360
            if sec_end < sec_start:
                sec_end += 360

            sector_width = sec_end - sec_start
            # 往返扫描，跟踪模式加速（周期缩短）
            period = 2 * sector_width / self.azimuth_rate
            t_in_period = sim_time % period
            if t_in_period < period / 2:
                return sec_start + t_in_period * self.azimuth_rate
            else:
                return sec_end - (t_in_period - period / 2) * self.azimuth_rate

        # 搜索模式
        sector_start, sector_end = self.scan_sector
        sector_width = sector_end - sector_start

        if sector_width >= 360.0:
            return (sim_time * self.azimuth_rate) % 360.0
        else:
            period = 2 * sector_width / self.azimuth_rate
            t_in_period = sim_time % period
            half_period = period / 2
            if t_in_period < half_period:
                return sector_start + t_in_period * self.azimuth_rate
            else:
                return sector_end - (t_in_period - half_period) * self.azimuth_rate

    def _is_in_beam(self, target_az_deg: float, sim_time: float) -> bool:
        """检查目标是否在当前波束内。

        Args:
            target_az_deg: 目标方位角 (°)
            sim_time: 仿真时间

        Returns:
            bool: 是否被波束照射
        """
        if self.scan_type == "staring":
            return True  # 凝视模式，瞬时覆盖

        beam_center = self._get_beam_center(sim_time)
        half_bw = self.azimuth_beamwidth / 2

        # 计算目标与波束中心的角距离（考虑360°环绕）
        delta = abs((target_az_deg - beam_center + 180) % 360 - 180)
        return delta <= half_bw

    def _time_since_last_illumination(self, target_az_deg: float, sim_time: float) -> float:
        """估算目标上次被波束照射的时间 (s)。

        Args:
            target_az_deg: 目标方位角 (°)
            sim_time: 当前时间

        Returns:
            float: 距上次照射的时间 (s)，第一次则为 inf
        """
        tid = hash(target_az_deg)  # 简化，实际按target_id跟踪
        return sim_time - self._last_detect_time.get(tid, -float('inf'))

    # ── SNR 计算 ──

    def _compute_snr(self, target_rcs: float, range_m: float) -> float:
        """计算 SNR (dB)。

        雷达方程：SNR ∝ RCS / R^4
        标校点：reference_rcs @ reference_range → SNR=snr_threshold+3dB (Pd≈85%)

        Args:
            target_rcs: 目标 RCS (m²)
            range_m: 距离 (m)

        Returns:
            SNR (dB)
        """
        # 标校：参考目标@参考距离 → SNR = threshold + 3dB (对应 Pd=85%)
        ref_snr = self.snr_threshold + 3.0
        snr = ref_snr + 10 * np.log10(target_rcs / self.reference_rcs) - \
              40 * np.log10(range_m / self.reference_range)
        return float(snr)

    # ── 检测 ──

    def detect(self, sim_time: float) -> list[Detection]:
        """执行雷达检测。

        流程：
        1. 对每个目标，检查距离/俯仰是否在覆盖范围内
        2. 机械扫描：检查波束是否照射到目标
        3. 跟踪模式：已确认目标额外检查是否在跟踪扇区内
        4. 计算 SNR → Pd，判定是否检测到
        5. 施加测量噪声，生成检测报告
        6. 更新航迹状态 + 管理扫描模式
        7. 生成杂波点
        """
        detections = []
        detected_ids = set()

        for target in self._targets:
            state = target.get_state(sim_time)
            if state is None:
                continue

            # 坐标转换
            rel_pos = state.position - self.position
            r_true = float(np.linalg.norm(rel_pos))

            # 距离检查
            if r_true < self.min_range or r_true > self.max_range:
                continue

            az_true = float(np.arctan2(rel_pos[1], rel_pos[0]))
            el_true = float(np.arctan2(rel_pos[2], np.linalg.norm(rel_pos[:2])))

            az_deg = np.degrees(az_true) % 360
            el_deg = np.degrees(el_true)

            # 俯仰覆盖检查
            if not (self.fov['elevation'][0] <= el_deg <= self.fov['elevation'][1]):
                continue

            # 方位检查
            if self.scan_type == "staring":
                if not self.is_in_fov(az_true, el_true):
                    continue
            else:
                # 搜索模式：全周扫描；跟踪模式：扇区扫描
                if self._mode in ("search", "tws"):
                    # 搜索/TWS 模式：全周扫描，波束按方位旋转
                    if not self._is_in_beam(az_deg, sim_time):
                        continue
                else:
                    # 纯跟踪模式：波束在跟踪扇区内
                    if not self._is_in_beam(az_deg, sim_time):
                        continue

            # 计算 SNR 和检测概率
            rcs = target.get_rcs()
            snr = self._compute_snr(rcs, r_true)
            pd = self.pd_model.compute_pd(snr)

            if np.random.random() > pd:
                continue  # 漏检

            # 径向速度（多普勒）
            vr_true = float(np.dot(state.velocity, rel_pos / r_true))
            vr_true = np.clip(vr_true, -self.max_velocity, self.max_velocity)

            # 施加误差
            raw_meas = {
                'range': r_true,
                'azimuth': az_deg,
                'elevation': el_deg,
                'doppler': vr_true,
            }
            meas = self._error_pipeline.process(raw_meas)

            # 检测报告
            flags = {DetectionFlag.NORMAL}
            if snr < self.snr_threshold + 3:
                flags.add(DetectionFlag.LOW_SNR)

            az_rad = np.radians(meas['azimuth'])
            el_rad = np.radians(meas['elevation'])
            ground_range = meas['range'] * np.cos(el_rad)
            wx = self.position[0] + ground_range * np.cos(az_rad)
            wy = self.position[1] + ground_range * np.sin(az_rad)

            det = self.make_detection(
                timestamp=sim_time - self.time_delay,
                measurements=meas,
                covariance={
                    'range': self.error_sigma['range'] ** 2,
                    'azimuth': self.error_sigma['azimuth'] ** 2,
                    'elevation': self.error_sigma['elevation'] ** 2,
                    'doppler': self.error_sigma['doppler'] ** 2,
                },
                confidence=pd,
                flags=flags,
            )
            det.track_id = target.id
            det.truth_position = state.position.copy()
            detections.append(det)

            self._last_known_positions[target.id] = state.position.copy()
            self._last_detect_time[target.id] = sim_time
            detected_ids.add(target.id)

            # 更新航迹：命中
            self._track_hits[target.id] = self._track_hits.get(target.id, 0) + 1
            self._track_misses[target.id] = 0
            self._track_predicted[target.id] = state.position.copy()
            self._track_velocity[target.id] = state.velocity.copy()

            # 首次检测：searching → tentative
            if self._track_state.get(target.id, "searching") == "searching":
                self._track_state[target.id] = "tentative"

        # 更新未检测到的目标的航迹状态
        self._update_tracks(sim_time, detected_ids)

        # 管理扫描模式
        self._manage_scan_mode()

        # 生成杂波
        if self.clutter_enabled and self.clutter_density > 0:
            clutter_dets = self._generate_clutter(sim_time)
            detections.extend(clutter_dets)

        return detections

    # ── 航迹管理 ──

    def _update_tracks(self, sim_time: float, detected_ids: set):
        """更新所有目标的航迹状态。

        M/N 确认逻辑（按扫描周期而非检测步进计数）：
        - searching → tentative: 首次检测
        - tentative → confirmed: M 次扫描中至少 N 次检测
        - confirmed → coasting: 连续漏检（按扫描周期）
        - coasting → deleted: 连续漏检超过 coast_max
        - coasting → confirmed: 重新检测到

        扫描周期 = 360°/azimuth_rate（全周模式）或扇区宽度/azimuth_rate（扇扫模式）
        """
        # 估算扫描周期
        if self.scan_sector[1] - self.scan_sector[0] >= 360:
            scan_period = 360.0 / self.azimuth_rate
        else:
            scan_period = (self.scan_sector[1] - self.scan_sector[0]) / self.azimuth_rate

        for target in self._targets:
            tid = target.id

            if tid in detected_ids:
                continue  # 已在 detect() 中处理

            # 只在上次更新后经过一个扫描周期才计一次 miss
            last_update = self._last_detect_time.get(tid, 0)
            state = self._track_state.get(tid, "searching")

            if state in ("confirmed", "tentative"):
                if sim_time - last_update > scan_period * 1.2:
                    # 一个完整扫描周期内未检测到 → 计一次漏检
                    self._track_misses[tid] = self._track_misses.get(tid, 0) + 1
                    self._last_detect_time[tid] = sim_time  # 避免反复计数

                    # 外推预测
                    if tid in self._track_predicted and tid in self._track_velocity:
                        pred = self._track_predicted[tid] + \
                               self._track_velocity[tid] * scan_period
                        self._track_predicted[tid] = pred

                    if self._track_misses[tid] >= self._coast_max:
                        self._track_state[tid] = "deleted"
                    elif self._track_misses[tid] >= 2:
                        self._track_state[tid] = "coasting"

    def _manage_scan_mode(self):
        """管理扫描模式切换。

        - 搜索模式 → 跟踪模式：有确认航迹时切换
        - 跟踪模式 → 搜索模式：所有航迹丢失时恢复
        """
        confirmed_ids = [
            tid for tid, state in self._track_state.items()
            if state in ("confirmed", "coasting")
        ]
        tentative_ids = [
            tid for tid, state in self._track_state.items()
            if state == "tentative"
        ]

        # 对 tentative 目标做 M/N 确认
        for tid in tentative_ids:
            hits = self._track_hits.get(tid, 0)
            if hits >= self._confirm_n:
                self._track_state[tid] = "confirmed"
            elif self._track_misses.get(tid, 0) >= self._confirm_m:
                self._track_state[tid] = "deleted"

        # 更新已确认目标
        confirmed_ids.extend([
            tid for tid, state in self._track_state.items()
            if state == "confirmed"
        ])

        if confirmed_ids:
            # 计算跟踪扇区中心
            centers = []
            for tid in confirmed_ids:
                if tid in self._track_predicted:
                    pos = self._track_predicted[tid]
                    az = np.degrees(np.arctan2(
                        pos[1] - self.position[1],
                        pos[0] - self.position[0]
                    )) % 360
                    centers.append(az)
                elif tid in self._last_known_positions:
                    pos = self._last_known_positions[tid]
                    az = np.degrees(np.arctan2(
                        pos[1] - self.position[1],
                        pos[0] - self.position[0]
                    )) % 360
                    centers.append(az)

            self._mode_track_centers = centers

            if self._tws_enabled:
                # TWS 模式：继续 360° 搜索，用预测维持航迹
                self._mode = "tws"
            else:
                # 纯跟踪模式：窄扇区聚焦
                self._mode = "track"
        else:
            self._mode = "search"
            self._mode_track_centers = []

        # 清理已删除航迹
        deleted = [
            tid for tid, state in self._track_state.items()
            if state == "deleted"
        ]
        for tid in deleted:
            for d in (self._track_state, self._track_hits, self._track_misses,
                       self._track_predicted, self._track_velocity,
                       self._last_known_positions, self._last_detect_time):
                d.pop(tid, None)

            # 将 tentative → searching 以便重新检测
            for tid in list(self._track_state.keys()):
                if self._track_state[tid] == "tentative" and \
                   self._track_misses.get(tid, 0) >= self._confirm_m:
                    self._track_state[tid] = "searching"

    def get_track_states(self) -> dict:
        """获取当前所有航迹状态（用于诊断/可视化）。"""
        result = {}
        for target in self._targets:
            tid = target.id
            result[tid] = {
                'state': self._track_state.get(tid, 'searching'),
                'hits': self._track_hits.get(tid, 0),
                'misses': self._track_misses.get(tid, 0),
                'predicted': self._track_predicted.get(tid, None),
            }
        return result

    def get_mode(self) -> str:
        """获取当前工作模式。"""
        return self._mode

    def _generate_clutter(self, sim_time: float) -> list[Detection]:
        """生成杂波检测（考虑当前波束位置）。"""
        fov_az = self.fov['azimuth']
        fov_el = self.fov['elevation']

        # 机械扫描时杂波只出现在当前波束内
        if self.scan_type == "mechanical":
            beam_c = self._get_beam_center(sim_time)
            hbw = self.azimuth_beamwidth / 2
            clutter_az_min = max(fov_az[0], beam_c - hbw)
            clutter_az_max = min(fov_az[1], beam_c + hbw)
        else:
            clutter_az_min, clutter_az_max = fov_az

        az_span = np.radians(clutter_az_max - clutter_az_min)
        el_span = np.radians(fov_el[1] - fov_el[0])
        if az_span <= 0 or el_span <= 0:
            return []

        volume = (self.max_range - self.min_range) * az_span * el_span
        n_clutter = np.random.poisson(self.clutter_density * volume)
        clutter_dets = []

        for _ in range(n_clutter):
            rc = np.random.uniform(self.min_range, self.max_range)
            azc = np.random.uniform(np.radians(clutter_az_min), np.radians(clutter_az_max))
            elc = np.random.uniform(np.radians(fov_el[0]), np.radians(fov_el[1]))
            ground_range = rc * np.cos(elc)
            wx = self.position[0] + ground_range * np.cos(azc)
            wy = self.position[1] + ground_range * np.sin(azc)

            det = Detection(
                timestamp=sim_time,
                sensor_id=self.sensor_id,
                track_id=-1,
                measurements={
                    'range': rc,
                    'azimuth': np.degrees(azc),
                    'elevation': np.degrees(elc),
                },
                covariance={},
                confidence=0.0,
                flags={DetectionFlag.CLUTTER},
                truth_position=np.array([wx, wy, 0.0]),
            )
            clutter_dets.append(det)

        return clutter_dets

    # ── 诊断信息 ──

    def get_scan_info(self, sim_time: float) -> dict:
        """获取当前扫描状态（用于可视化）。"""
        if self.scan_type == "staring":
            return {'type': 'staring', 'fov': self.fov}
        return {
            'type': 'mechanical',
            'beam_center': self._get_beam_center(sim_time),
            'beamwidth': self.azimuth_beamwidth,
            'sector': self.scan_sector,
            'rate': self.azimuth_rate,
        }
