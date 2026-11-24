"""杂波生成模块。

基于 Poisson RFS (Random Finite Set) 模型生成空间分布的杂波点。
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class ClutterPoint:
    """单个杂波点。"""
    range: float
    azimuth: float     # rad
    elevation: float   # rad
    intensity: float   # 杂波强度 (SNR dB)


class PoissonClutterGenerator:
    """Poisson 杂波生成器。

    在探测空间内按 Poisson 过程产生杂波点。
    密度 λ 定义为单位探测体积内的期望杂波数。

    探测体积 V = range_span × azimuth_span × elevation_span
    """

    def __init__(
        self,
        density: float = 1.0e-4,        # 单位体积期望杂波数
        distribution: str = "uniform",  # uniform / weibull / rayleigh
        min_range: float = 50.0,
        max_range: float = 5000.0,
        az_min: float = -60.0,
        az_max: float = 60.0,
        el_min: float = -10.0,
        el_max: float = 30.0,
        seed: int | None = None,
    ):
        """
        Args:
            density: 杂波密度 (每 (m·deg²) 的期望点数)
            distribution: 杂波空间分布类型
            min_range, max_range: 距离范围 (m)
            az_min, az_max: 方位范围 (度)
            el_min, el_max: 俯仰范围 (度)
            seed: 随机种子
        """
        self.density = density
        self.distribution = distribution
        self.min_range = min_range
        self.max_range = max_range
        self.az_min = np.radians(az_min)
        self.az_max = np.radians(az_max)
        self.el_min = np.radians(el_min)
        self.el_max = np.radians(el_max)

        self._rng = np.random.RandomState(seed)

    @property
    def volume(self) -> float:
        """探测空间体积（近似）。"""
        range_span = self.max_range - self.min_range
        az_span = self.az_max - self.az_min
        el_span = self.el_max - self.el_min
        return range_span * az_span * el_span

    def generate(self, n_points: int | None = None) -> list[ClutterPoint]:
        """生成一批杂波点。

        Args:
            n_points: 指定数量的杂波点，None 则按 Poisson 分布随机生成

        Returns:
            list[ClutterPoint]: 杂波点列表
        """
        if n_points is None:
            n_points = self._rng.poisson(self.density * self.volume)

        points = []
        for _ in range(n_points):
            r = self._sample_range()
            az = self._sample_azimuth()
            el = self._sample_elevation()
            intensity = self._rng.uniform(-10, 20)  # SNR dB
            points.append(ClutterPoint(r, az, el, intensity))

        return points

    def generate_positions(self, n_points: int | None = None) -> np.ndarray:
        """生成杂波点的世界坐标。

        Args:
            n_points: 杂波点数

        Returns:
            (N, 2) 或 (N, 3) 位置数组
        """
        points = self.generate(n_points)
        positions = []
        for p in points:
            ground_range = p.range * np.cos(p.elevation)
            x = ground_range * np.cos(p.azimuth)
            y = ground_range * np.sin(p.azimuth)
            z = p.range * np.sin(p.elevation)
            positions.append([x, y, z])
        return np.array(positions) if positions else np.empty((0, 3))

    def _sample_range(self) -> float:
        if self.distribution == "uniform":
            return self._rng.uniform(self.min_range, self.max_range)
        elif self.distribution == "weibull":
            # Weibull 分布更接近实测杂波
            shape = 2.0
            scale = (self.max_range - self.min_range) / 3.0
            return self.min_range + self._rng.weibull(shape) * scale
        elif self.distribution == "rayleigh":
            scale = (self.max_range - self.min_range) / 3.0
            return self.min_range + self._rng.rayleigh(scale)
        return self._rng.uniform(self.min_range, self.max_range)

    def _sample_azimuth(self) -> float:
        return self._rng.uniform(self.az_min, self.az_max)

    def _sample_elevation(self) -> float:
        return self._rng.uniform(self.el_min, self.el_max)


class WeibullClutterGenerator(PoissonClutterGenerator):
    """Weibull 分布杂波生成器（更真实的杂波幅度分布）。"""

    def __init__(self, shape: float = 1.5, **kwargs):
        super().__init__(distribution="weibull", **kwargs)
        self.shape = shape

    def _sample_range(self) -> float:
        scale = (self.max_range - self.min_range) / 3.0
        return self.min_range + self._rng.weibull(self.shape) * scale
