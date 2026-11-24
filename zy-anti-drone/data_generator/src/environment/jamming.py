"""干扰模型模块。

模拟电子干扰对传感器的影响：
- 压制干扰 (Noise Jamming): 提升噪底，降低 SNR → 降低 Pd，增大测角误差
- 欺骗干扰 (Deception Jamming): 产生虚假目标
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from enum import Enum


class JammerType(Enum):
    NOISE = "noise"           # 压制噪声干扰
    DECEPTION = "deception"   # 欺骗干扰
    COMBINED = "combined"     # 复合干扰


@dataclass
class Jammer:
    """干扰源模型。

    模拟单个干扰源对雷达传感器的影响。
    """

    position: tuple[float, float, float] = (2000.0, 1000.0, 50.0)
    power_dbm: float = -30.0          # 干扰功率 (dBm)
    jammer_type: JammerType = JammerType.NOISE
    bandwidth: float = 100e6           # 干扰带宽 (Hz)
    active: bool = True
    activation_time: float = 0.0       # 开始工作的时间
    deactivation_time: float = float('inf')  # 停止工作的时间

    def is_active(self, sim_time: float) -> bool:
        """检查干扰是否在给定时间激活。"""
        return self.active and self.activation_time <= sim_time <= self.deactivation_time


class JammingEffects:
    """干扰效应计算器。

    计算压制干扰和欺骗干扰对传感器性能的影响。
    """

    def __init__(self, jammer: Jammer | None = None):
        self.jammer = jammer

    def set_jammer(self, jammer: Jammer):
        self.jammer = jammer

    def get_snr_degradation(self, sensor_pos: np.ndarray, target_pos: np.ndarray,
                            sim_time: float) -> float:
        """计算 SNR 衰减 (dB)。

        压制干扰提升噪底，有效 SNR 下降：
        SNR_eff = SNR / (1 + JNR)
        其中 JNR = 干噪比

        Args:
            sensor_pos: 传感器位置
            target_pos: 目标位置
            sim_time: 仿真时间

        Returns:
            SNR 衰减 (dB)，正值表示 SNR 下降
        """
        if self.jammer is None or not self.jammer.is_active(sim_time):
            return 0.0

        if self.jammer.jammer_type not in (JammerType.NOISE, JammerType.COMBINED):
            return 0.0

        # 干扰源到传感器的距离
        d_js = float(np.linalg.norm(np.array(self.jammer.position) - sensor_pos))

        # 简化的 JNR 计算
        # JNR ∝ P_jammer / d_js²
        jnr_linear = 10 ** (self.jammer.power_dbm / 10) / (d_js ** 2 + 1e-9)
        jnr_db = 10 * np.log10(jnr_linear + 1e-9)

        # SNR 下降等于 JNR（简化模型）
        degradation = max(0, jnr_db - 10)  # 减去 10dB 的处理增益
        return degradation

    def get_angle_error_increase(self, sensor_pos: np.ndarray,
                                 sim_time: float) -> float:
        """计算角度测量误差的增加倍数。

        压制干扰导致测角精度恶化。

        Args:
            sensor_pos: 传感器位置
            sim_time: 仿真时间

        Returns:
            误差放大倍数 (≥1.0)
        """
        if self.jammer is None or not self.jammer.is_active(sim_time):
            return 1.0

        if self.jammer.jammer_type not in (JammerType.NOISE, JammerType.COMBINED):
            return 1.0

        d_js = float(np.linalg.norm(np.array(self.jammer.position) - sensor_pos))
        jnr_db = self.jammer.power_dbm - 20 * np.log10(d_js + 1e-9) + 130
        factor = 1.0 + 0.1 * max(0, jnr_db)
        return factor

    def generate_false_targets(self, sensor_pos: np.ndarray,
                               n_true_targets: int,
                               sim_time: float) -> list[dict]:
        """生成欺骗干扰产生的虚假目标。

        Args:
            sensor_pos: 传感器位置
            n_true_targets: 真实目标数（用于决定虚假目标数量）
            sim_time: 仿真时间

        Returns:
            list[dict]: 虚假目标信息
        """
        if self.jammer is None or not self.jammer.is_active(sim_time):
            return []

        if self.jammer.jammer_type not in (JammerType.DECEPTION, JammerType.COMBINED):
            return []

        # 欺骗干扰产生若干虚假目标，分布在真实目标附近
        n_false = np.random.poisson(n_true_targets * 2)
        false_targets = []

        # 干扰源作为虚假目标的参考点
        jammer_pos = np.array(self.jammer.position)

        for _ in range(n_false):
            # 虚假目标位置在干扰源和传感器之间随机散布
            angle = np.random.uniform(0, 2 * np.pi)
            range_offset = np.random.uniform(100, 2000)
            r = float(np.linalg.norm(jammer_pos - sensor_pos))
            r_false = max(50, r - range_offset * np.random.random())

            false_targets.append({
                'range': r_false,
                'azimuth': np.degrees(angle),
                'elevation': np.degrees(np.random.uniform(-5, 15)),
                'doppler': np.random.normal(0, 5),
                'is_false': True,
                'snr': np.random.uniform(5, 20),
            })

        return false_targets
