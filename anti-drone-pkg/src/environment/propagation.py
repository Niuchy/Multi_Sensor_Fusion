"""传播效应模块。

模拟大气衰减和多径效应对传感器性能的影响。
"""

from __future__ import annotations

import numpy as np


def atmospheric_attenuation(frequency_hz: float, range_m: float,
                           humidity: float = 50.0, temperature_c: float = 20.0) -> float:
    """计算大气衰减。

    ITU-R P.676 简化模型。

    Args:
        frequency_hz: 频率 (Hz)
        range_m: 传播距离 (m)
        humidity: 相对湿度 (%)
        temperature_c: 温度 (°C)

    Returns:
        衰减 (dB)
    """
    freq_ghz = frequency_hz / 1e9

    # 氧气衰减 (dB/km)
    if freq_ghz < 10:
        o2_atten = 0.007  # dB/km
    elif freq_ghz < 60:
        o2_atten = 0.007 + 0.01 * (freq_ghz / 10)**2
    else:
        o2_atten = 15.0  # 氧气吸收峰

    # 水汽衰减 (dB/km) — 简化
    h2o_atten = 0.002 * humidity / 50.0 * (freq_ghz / 10)**2

    total_atten_db_per_km = o2_atten + h2o_atten
    return total_atten_db_per_km * range_m / 1000.0


def multipath_factor(range_m: float, target_height_m: float,
                     sensor_height_m: float, frequency_hz: float,
                     surface: str = "land") -> float:
    """计算多径效应导致的信号强度变化因子。

    简化双射线地面反射模型。

    Args:
        range_m: 斜距 (m)
        target_height_m: 目标高度 (m)
        sensor_height_m: 传感器高度 (m)
        frequency_hz: 频率 (Hz)
        surface: 地面类型 ('land', 'sea')

    Returns:
        信号强度变化因子 (相对于自由空间，dB)
    """
    wavelength = 3e8 / frequency_hz

    # 地面反射系数
    reflectivity = 0.3 if surface == "land" else 0.8

    # 直接路径和反射路径的路径差
    # 简化：ΔR ≈ 2 * ht * hr / R
    if range_m < 1:
        return 0.0

    delta_r = 2 * target_height_m * sensor_height_m / range_m

    # 相位差
    phase_diff = 2 * np.pi * delta_r / wavelength

    # 信号强度 = |1 + ρ * exp(j * φ)|²
    # 用对数表示
    factor = abs(1 + reflectivity * np.exp(1j * phase_diff))
    if factor < 1e-9:
        return -30.0  # 深度衰落

    return float(20 * np.log10(factor))


def refractive_bending(range_m: float, elevation_rad: float) -> float:
    """大气折射导致的高度测量误差。

    简化模型：4/3 地球半径近似。

    Args:
        range_m: 距离 (m)
        elevation_rad: 仰角 (rad)

    Returns:
        高度误差 (m)
    """
    # 标准大气折射的曲率修正
    earth_radius_eff = 4 / 3 * 6371e3  # 4/3 地球半径 (m)
    return float(range_m**2 / (2 * earth_radius_eff) * np.cos(elevation_rad))
