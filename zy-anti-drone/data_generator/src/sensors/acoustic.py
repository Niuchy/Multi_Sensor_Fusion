"""声探测传感器模型。

被动探测：通过麦克风阵列检测无人机旋翼/电机噪声。
特点：
- 仅方位+俯仰（和雷达/光电一样是被动测角），无距离
- 可粗略估计距离（通过声压级衰减）
- 全向覆盖（圆形阵列）
- 受环境噪声和风速影响大
- 可识别无人机类型（声纹特征）
"""

import numpy as np
from sensors.base import SensorBase
from sensors.error_models import (
    GaussianNoise, ErrorPipeline, DetectionProbability,
)
from truth.target import Target
from bus.message import Detection, DetectionFlag


class AcousticSensor(SensorBase):
    """声探测传感器模拟器。

    测量量:
    - azimuth: 方位角 (deg)
    - elevation: 俯仰角 (deg)
    - frequency: 主频 (Hz) — 旋翼转速特征
    - sound_level: 声压级 (dB) — 可推算大致距离
    - classification: 目标类型猜测
    """

    def __init__(
        self,
        sensor_id: str = "acoustic-01",
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        update_rate: float = 5.0,
        fov: dict | None = None,
        max_range: float = 1500.0,
        min_range: float = 10.0,
        error_sigma: dict | None = None,
        pd: float = 0.80,
        reference_range: float = 500.0,
        sound_level_ref: float = 80.0,
        ambient_noise_db: float = 40.0,
        wind_speed: float = 3.0,
    ):
        position = position or [-30.0, 0.0, 2.0]
        orientation = orientation or [0.0, 0.0]
        fov = fov or {'azimuth': [-180.0, 180.0], 'elevation': [-20.0, 60.0]}

        super().__init__(sensor_id, 'acoustic', position, orientation, update_rate, fov)

        self.max_range = max_range
        self.min_range = min_range
        self.error_sigma = error_sigma or {
            'azimuth': 3.0, 'elevation': 8.0, 'frequency': 50.0,
        }
        self.pd_nominal = pd
        self.reference_range = reference_range
        self.sound_level_ref = sound_level_ref

        # 环境参数
        self.ambient_noise_db = ambient_noise_db
        self.wind_speed = wind_speed

        # 检测概率模型 (声学 SNR 阈值为 0dB，比雷达低)
        self.pd_model = DetectionProbability(pd_nominal=pd, snr_threshold=0.0)

        self._targets: list[Target] = []

    def set_targets(self, targets: list[Target]):
        self._targets = targets

    def _setup_error_pipeline(self):
        self._error_pipeline.add_stage(GaussianNoise(self.error_sigma))

    def _compute_acoustic_snr(self, target_range: float, drone_db1m: float = 80.0) -> float:
        """计算声学信噪比 (dB)。

        声压级衰减：6dB/倍距离（球面波）
        SNR = drone_SPL_at_sensor - ambient_noise
        drone_SPL_at_sensor = drone_db1m - 20*log10(R)

        Args:
            target_range: 目标距离 (m)
            drone_db1m: 无人机 1m 处声压级 (dB)

        Returns:
            SNR (dB)
        """
        if target_range < 1.0:
            target_range = 1.0

        # 球面波衰减 + 大气吸收 + 风噪
        spreading_loss = 20 * np.log10(target_range)
        atmospheric_loss = 0.005 * target_range  # ~5dB/km
        wind_loss = 0.05 * self.wind_speed * np.log10(target_range + 1)

        spl_at_sensor = drone_db1m - spreading_loss - atmospheric_loss - wind_loss
        snr = spl_at_sensor - self.ambient_noise_db

        return float(snr)

    def _estimate_range_from_sound(self, measured_spl_db: float, drone_db1m: float = 80.0) -> float:
        """从声压级粗略推算距离。

        逆传播模型：R = 10^((dB1m - measured_dB) / 20)
        精度很差（±30%），仅供融合算法参考。
        """
        return float(10 ** ((drone_db1m - measured_spl_db) / 20.0))

    def detect(self, sim_time: float) -> list[Detection]:
        detections = []

        for target in self._targets:
            state = target.get_state(sim_time)
            if state is None:
                continue

            rel_pos = state.position - self.position
            r_true = float(np.linalg.norm(rel_pos))

            if r_true < self.min_range or r_true > self.max_range:
                continue

            az_true = float(np.arctan2(rel_pos[1], rel_pos[0]))
            el_true = float(np.arctan2(rel_pos[2], np.linalg.norm(rel_pos[:2])))

            az_deg = np.degrees(az_true) % 360
            el_deg = np.degrees(el_true)

            # 俯仰检查
            if not (self.fov['elevation'][0] <= el_deg <= self.fov['elevation'][1]):
                continue

            # 声学 SNR → 检测概率
            drone_db = self.sound_level_ref + 10 * np.log10(target.rcs / 0.01 + 1e-6)
            # 小 RCS → 小型机 → 噪声小，大 RCS → 大型机 → 噪声大
            snr = self._compute_acoustic_snr(r_true, drone_db)
            pd = self.pd_model.compute_pd(snr)

            if np.random.random() > pd:
                continue

            # 声压级测量（含噪声）
            spreading_loss = 20 * np.log10(r_true + 1e-9)
            spl_true = drone_db - spreading_loss
            spl_meas = spl_true + np.random.normal(0, 3.0)  # ±3dB 测量误差

            # 粗略距离估计
            range_est = self._estimate_range_from_sound(spl_meas, drone_db)

            # 频率测量
            # 四旋翼典型旋翼频率: 50-200Hz (与转速、桨数相关)
            base_freq = 80.0 + 40.0 * (target.rcs / 0.5)  # 简化的频率-RCS关联
            freq_true = base_freq * (1 + 0.1 * np.sin(sim_time))  # 频率微变
            freq_meas = freq_true + np.random.normal(0, self.error_sigma['frequency'])

            # 测量值
            raw_meas = {
                'azimuth': az_deg,
                'elevation': el_deg,
                'frequency': freq_true,
                'sound_level': spl_true,
            }
            meas = self._error_pipeline.process(raw_meas)
            meas['sound_level'] = spl_meas
            meas['frequency'] = freq_meas
            meas['range_estimate'] = range_est  # 不精确！

            # 分类（简化：基于频率猜测类型）
            if freq_true < 60:
                classification = "大型固定翼"
            elif freq_true < 120:
                classification = "四旋翼/多旋翼"
            else:
                classification = "小型高速无人机"
            meas['classification'] = classification

            det = self.make_detection(
                timestamp=sim_time,
                measurements=meas,
                covariance={
                    'azimuth': self.error_sigma['azimuth'] ** 2,
                    'elevation': self.error_sigma['elevation'] ** 2,
                    'frequency': self.error_sigma['frequency'] ** 2,
                },
                confidence=pd,
                flags={DetectionFlag.NORMAL},
            )
            det.track_id = target.id
            det.truth_position = state.position.copy()
            detections.append(det)

        return detections
