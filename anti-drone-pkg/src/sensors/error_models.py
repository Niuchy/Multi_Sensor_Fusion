"""误差模型模块。

提供传感器测量误差的各种模型：
- 高斯噪声（随机误差）
- 系统偏差（固定 + 漂移）
- 野值
- 时间延迟
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field


@dataclass
class GaussianNoise:
    """高斯噪声模型。

    每个测量维度独立的高斯分布。
    """
    sigma: dict[str, float] = field(default_factory=dict)
    # 示例: {"range": 5.0, "azimuth": 0.5, "elevation": 0.5}

    def apply(self, measurements: dict, seed: int | None = None) -> dict:
        """对测量值施加高斯噪声。

        Args:
            measurements: {"range": 1000.0, "azimuth": 0.5, ...}
            seed: 随机种子（可选）

        Returns:
            加噪后的测量值
        """
        if seed is not None:
            np.random.seed(seed)
        result = {}
        for key, value in measurements.items():
            if key in self.sigma:
                noise = np.random.normal(0, self.sigma[key])
                result[key] = value + noise
            else:
                result[key] = value
        return result


@dataclass
class Bias:
    """系统偏差模型。

    包含固定偏差和慢漂移两项。
    """
    fixed: dict[str, float] = field(default_factory=dict)
    # 固定偏差: {"range": 2.0, "azimuth": 0.1}

    drift_rate: dict[str, float] = field(default_factory=dict)
    # 漂移速率 (单位/秒): {"range": 0.01, "azimuth": 0.001}

    def __init__(self, fixed: dict | None = None, drift_rate: dict | None = None):
        self.fixed = fixed or {}
        self.drift_rate = drift_rate or {}
        self._accumulated: dict[str, float] = {k: 0.0 for k in self.drift_rate}

    def apply(self, measurements: dict, dt: float = 0.0) -> dict:
        """施加系统偏差。

        Args:
            measurements: 测量值
            dt: 距离上次测量的时间步长（用于漂移累积）

        Returns:
            加偏差后的测量值
        """
        result = {}
        for key, value in measurements.items():
            # 固定偏差
            bias = self.fixed.get(key, 0.0)
            # 漂移
            if key in self.drift_rate:
                self._accumulated[key] += self.drift_rate[key] * dt
                bias += self._accumulated[key]
            result[key] = value + bias
        return result

    def reset_drift(self):
        """重置漂移累积。"""
        for key in self._accumulated:
            self._accumulated[key] = 0.0


@dataclass
class OutlierModel:
    """野值模型。

    以一定概率将测量值替换为大幅偏离的野值。
    """
    probability: float = 0.01  # 野值概率
    scale_factor: float = 10.0  # 野值幅度（乘以 sigma）

    def apply(self, measurements: dict, sigma: dict[str, float]) -> dict:
        """施加野值。

        Args:
            measurements: 测量值
            sigma: 各维度的标准差

        Returns:
            可能含野值的测量值
        """
        result = {}
        for key, value in measurements.items():
            if np.random.random() < self.probability and key in sigma:
                outlier = np.random.normal(0, sigma[key] * self.scale_factor)
                result[key] = value + outlier
            else:
                result[key] = value
        return result


@dataclass
class TimeDelay:
    """时间延迟模型。

    模拟传感器处理延迟和通信延迟。
    """
    fixed_delay: float = 0.01     # 固定延迟 (秒)
    random_jitter: float = 0.005  # 随机抖动 σ (秒)

    def apply(self, timestamp: float) -> float:
        """返回延迟后的时间戳。

        Args:
            timestamp: 原始时间戳

        Returns:
            延迟后的时间戳
        """
        jitter = max(0, np.random.normal(0, self.random_jitter))
        return timestamp - (self.fixed_delay + jitter)


@dataclass
class DetectionProbability:
    """检测概率模型。

    基于 SNR 计算检测概率（Swerling 模型）。
    """
    pd_nominal: float = 0.9       # 标称检测概率
    snr_threshold: float = 13.0   # 检测门限 SNR (dB)
    swerling_case: int = 1        # Swerling 起伏模型 (0=None, 1, 2, 3, 4)

    def compute_pd(self, snr_db: float) -> float:
        """根据 SNR 计算检测概率。

        Swerling I 近似: Pd = Pfa^(1/(1+SNR))

        Args:
            snr_db: 信噪比 (dB)

        Returns:
            检测概率 [0, 1]
        """
        if snr_db < self.snr_threshold - 10:
            return 0.0
        if snr_db > self.snr_threshold + 20:
            return self.pd_nominal

        # 简化的 sigmoid 过渡
        k = 0.5
        x = k * (snr_db - self.snr_threshold)
        pd = 1.0 / (1.0 + np.exp(-x))
        return float(np.clip(pd, 0.0, self.pd_nominal))


class ErrorPipeline:
    """误差流水线 — 按顺序施加多种误差。

    用法:
        pipeline = ErrorPipeline()
        pipeline.add_stage(GaussianNoise(...))
        pipeline.add_stage(Bias(...))
        pipeline.add_stage(OutlierModel(...))
        result = pipeline.process(measurements, dt=0.1)
    """

    def __init__(self):
        self._stages: list = []

    def add_stage(self, stage):
        """添加误差处理阶段。"""
        self._stages.append(stage)

    def process(self, measurements: dict, dt: float = 0.0,
                timestamp: float = 0.0) -> dict:
        """依次执行所有误差阶段。

        Args:
            measurements: 原始测量值
            dt: 时间步长
            timestamp: 当前时间戳

        Returns:
            处理后的测量值（可能含噪声、偏差、野值）
        """
        result = measurements.copy()
        for stage in self._stages:
            if isinstance(stage, GaussianNoise):
                result = stage.apply(result)
            elif isinstance(stage, Bias):
                result = stage.apply(result, dt)
            elif isinstance(stage, OutlierModel):
                # 需要 sigma 信息，从 GaussianNoise 阶段获取
                sigma = {}
                for s in self._stages:
                    if isinstance(s, GaussianNoise):
                        sigma = s.sigma
                        break
                result = stage.apply(result, sigma)
            elif isinstance(stage, TimeDelay):
                # 时间延迟在这里不合适处理，跳过
                pass
        return result
