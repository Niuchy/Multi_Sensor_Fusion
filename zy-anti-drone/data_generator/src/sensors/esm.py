"""电侦传感器模型。

特点：
- 被动侦测，依赖目标辐射源开机
- 截获概率 (POI)
- 方位测量（中等精度）
- 信号参数测量（频率、PRI、脉宽、调制类型）
- 全向或宽视场
"""

from __future__ import annotations

import numpy as np
from sensors.base import SensorBase
from sensors.error_models import GaussianNoise, ErrorPipeline
from truth.target import Target
from bus.message import Detection, DetectionFlag


class ESMSensor(SensorBase):
    """电侦传感器模拟器。

    测量量:
    - azimuth: 方位角 (deg)
    - frequency: 载频 (Hz)
    - pri: 脉冲重复间隔 (s)
    - pw: 脉宽 (s)
    - modulation: 调制类型 (字符串)
    """

    def __init__(
        self,
        sensor_id: str = "esm-01",
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        update_rate: float = 1.0,
        fov: dict | None = None,
        frequency_range: tuple[float, float] = (0.1e9, 18.0e9),
        error_sigma: dict | None = None,
        poi: float = 0.8,
        sensitivity_dbm: float = -80.0,
    ):
        """
        Args:
            sensor_id: 传感器标识
            position: [x, y, z]
            orientation: [az, el] (度)
            update_rate: 侦测周期 (Hz)
            fov: 视场 (全向默认)
            frequency_range: 频率覆盖 (min, max) Hz
            error_sigma: 各参数测量误差标准差
            poi: 截获概率
            sensitivity_dbm: 灵敏度 (dBm)
        """
        position = position or [-50.0, 0.0, 8.0]
        orientation = orientation or [0.0, 0.0]
        fov = fov or {'azimuth': [-180.0, 180.0], 'elevation': [-10.0, 40.0]}

        super().__init__(sensor_id, 'esm', position, orientation, update_rate, fov)

        self.freq_range = frequency_range
        self.error_sigma = error_sigma or {
            'azimuth': 1.0,
            'frequency': 1.0e6,
            'pri': 1.0e-9,
            'pw': 50.0e-9,
        }
        self.poi = poi
        self.sensitivity_dbm = sensitivity_dbm

        self._targets: list[Target] = []
        self._detection_history: dict[int, list[float]] = {}  # target_id -> [timestamps]

    def set_targets(self, targets: list[Target]):
        self._targets = targets

    def _setup_error_pipeline(self):
        self._error_pipeline.add_stage(GaussianNoise(self.error_sigma))

    def _compute_received_power(self, target: Target, range_m: float) -> float:
        """计算接收功率 (dBm)。

        自由空间传播: Pr = Pt + Gt + Gr - 20*log10(4πR/λ)

        简化处理。
        """
        wavelength = 3e8 / target.emitter.freq
        # 简化 Friis 方程
        pr = 10 * np.log10(target.emitter.power * 1000)  # W → dBm
        pr -= 20 * np.log10(4 * np.pi * range_m / wavelength)
        pr += 0  # 天线增益简化
        return float(pr)

    def detect(self, sim_time: float) -> list[Detection]:
        detections = []

        for target in self._targets:
            # 目标辐射源必须开机
            if not target.is_emitting(sim_time):
                continue

            state = target.get_state(sim_time)
            if state is None:
                continue

            # 坐标转换
            rel_pos = state.position - self.position
            r = float(np.linalg.norm(rel_pos))
            az_true = float(np.arctan2(rel_pos[1], rel_pos[0]))

            # 频率覆盖检查
            if not (self.freq_range[0] <= target.emitter.freq <= self.freq_range[1]):
                continue

            # 灵敏度检查
            pr = self._compute_received_power(target, r)
            if pr < self.sensitivity_dbm:
                continue

            # 截获概率
            poi = self.poi
            # 扫描式接收机，POI 是周期性的
            if np.random.random() > poi:
                continue

            # 视场检查
            if not self.is_in_fov(az_true, 0.0):
                continue

            # 原始测量
            raw_meas = {
                'azimuth': np.degrees(az_true),
                'frequency': target.emitter.freq,
                'pri': target.emitter.pri,
                'pw': target.emitter.pw,
            }

            # 施加误差
            meas = self._error_pipeline.process(raw_meas)

            # 调制类型识别（可能有误识别）
            mod_conf = 0.9 if target.emitter.modulation in ('CW', 'BPSK', 'QPSK') else 0.7
            mod = target.emitter.modulation
            if np.random.random() > mod_conf:
                mod = 'UNKNOWN'

            det = self.make_detection(
                timestamp=sim_time,
                measurements={
                    'azimuth': meas['azimuth'],
                    'frequency': meas['frequency'],
                    'pri': meas['pri'],
                    'pw': meas['pw'],
                    'modulation': mod,
                },
                covariance={
                    'azimuth': self.error_sigma['azimuth'] ** 2,
                    'frequency': self.error_sigma['frequency'] ** 2,
                },
                confidence=poi * (1.0 if pr >= self.sensitivity_dbm + 10 else 0.5),
                flags={DetectionFlag.NORMAL},
            )
            det.track_id = target.id
            det.truth_position = state.position.copy()

            # 记录检测历史
            if target.id not in self._detection_history:
                self._detection_history[target.id] = []
            self._detection_history[target.id].append(sim_time)

            detections.append(det)

        return detections
