"""运动模型模块。

提供标准目标运动模型，用于生成复杂轨迹。
- CV: 等速运动
- CA: 等加速运动
- CT: 协调转弯
- Hover: 悬停（带微扰）
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class MotionState:
    """运动状态向量。"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z, self.vx, self.vy, self.vz])

    @classmethod
    def from_array(cls, arr: np.ndarray) -> 'MotionState':
        return cls(arr[0], arr[1], arr[2], arr[3], arr[4], arr[5])

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])

    @property
    def velocity(self) -> np.ndarray:
        return np.array([self.vx, self.vy, self.vz])

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.velocity[:2]))

    @property
    def heading(self) -> float:
        return float(np.arctan2(self.vy, self.vx))


def cv_model(state: MotionState, dt: float) -> MotionState:
    """等速运动模型 (Constant Velocity).

    x_{k+1} = x_k + vx_k * dt
    速度不变。

    Args:
        state: 当前状态
        dt: 时间步长 (秒)

    Returns:
        下一时刻状态
    """
    F = np.array([
        [1, 0, 0, dt, 0, 0],
        [0, 1, 0, 0, dt, 0],
        [0, 0, 1, 0, 0, dt],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1],
    ])
    return MotionState.from_array(F @ state.to_array())


def ca_model(state: MotionState, ax: float, ay: float, az: float, dt: float) -> MotionState:
    """等加速运动模型 (Constant Acceleration).

    Args:
        state: 当前状态
        ax, ay, az: 加速度分量 (m/s²)
        dt: 时间步长 (秒)

    Returns:
        下一时刻状态
    """
    x_new = state.x + state.vx * dt + 0.5 * ax * dt**2
    y_new = state.y + state.vy * dt + 0.5 * ay * dt**2
    z_new = state.z + state.vz * dt + 0.5 * az * dt**2
    vx_new = state.vx + ax * dt
    vy_new = state.vy + ay * dt
    vz_new = state.vz + az * dt
    return MotionState(x_new, y_new, z_new, vx_new, vy_new, vz_new)


def ct_model(state: MotionState, turn_rate: float, dt: float) -> MotionState:
    """协调转弯模型 (Coordinated Turn).

    水平面内恒定转弯率。

    Args:
        state: 当前状态
        turn_rate: 转弯率 (rad/s)，正值为左转
        dt: 时间步长 (秒)

    Returns:
        下一时刻状态
    """
    omega = turn_rate
    if abs(omega) < 1e-9:
        return cv_model(state, dt)

    sin_wt = np.sin(omega * dt)
    cos_wt = np.cos(omega * dt)

    x_new = state.x + (state.vx * sin_wt - state.vy * (1 - cos_wt)) / omega
    y_new = state.y + (state.vx * (1 - cos_wt) + state.vy * sin_wt) / omega
    z_new = state.z + state.vz * dt

    vx_new = state.vx * cos_wt - state.vy * sin_wt
    vy_new = state.vx * sin_wt + state.vy * cos_wt
    vz_new = state.vz

    return MotionState(x_new, y_new, z_new, vx_new, vy_new, vz_new)


def hover_model(state: MotionState, dt: float, position_noise: float = 0.5,
                altitude_noise: float = 0.3) -> MotionState:
    """悬停模型 — 位置带小幅随机漂移。

    Args:
        state: 当前状态（初始悬停位置）
        dt: 时间步长 (秒)
        position_noise: 水平位置噪声 σ (m)
        altitude_noise: 高度噪声 σ (m)

    Returns:
        带漂移的悬停状态
    """
    x_new = state.x + np.random.normal(0, position_noise * dt)
    y_new = state.y + np.random.normal(0, position_noise * dt)
    z_new = state.z + np.random.normal(0, altitude_noise * dt)
    return MotionState(x_new, y_new, z_new, 0.0, 0.0, 0.0)


def generate_formation(leader_state: MotionState, offsets: list[tuple[float, float, float]],
                       dt: float) -> list[MotionState]:
    """生成编队状态。

    每个跟随者相对于领头的位置偏移是固定的。

    Args:
        leader_state: 领头状态
        offsets: 各跟随者的偏移 [(dx, dy, dz), ...]
        dt: 时间步长 (未使用，为接口统一保留)

    Returns:
        各跟随者的状态列表
    """
    followers = []
    for dx, dy, dz in offsets:
        followers.append(MotionState(
            leader_state.x + dx,
            leader_state.y + dy,
            leader_state.z + dz,
            leader_state.vx,
            leader_state.vy,
            leader_state.vz,
        ))
    return followers
