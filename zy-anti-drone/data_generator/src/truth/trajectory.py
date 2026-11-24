"""轨迹生成模块。

支持通过关键航点定义路径，并提供多种插值方式。
"""

from __future__ import annotations

import numpy as np
from scipy.interpolate import CubicSpline, interp1d
from dataclasses import dataclass, field


@dataclass
class Waypoint:
    """单个航点：位置 + 到达时间。"""
    x: float
    y: float
    z: float
    t: float  # 从轨迹起点开始的时间偏移 (秒)


@dataclass
class TrajectoryState:
    """轨迹在某一时刻的状态。"""
    t: float
    position: np.ndarray   # [x, y, z] (m)
    velocity: np.ndarray   # [vx, vy, vz] (m/s)
    acceleration: np.ndarray  # [ax, ay, az] (m/s²)
    heading: float         # 水平航向角 (rad)
    speed: float           # 水平速率 (m/s)


class Trajectory:
    """轨迹基类。"""

    def get_state(self, t: float) -> TrajectoryState:
        """获取 t 时刻的轨迹状态。"""
        raise NotImplementedError

    def get_duration(self) -> float:
        """轨迹总时长。"""
        raise NotImplementedError


class WaypointTrajectory(Trajectory):
    """通过航点定义的轨迹。

    在航点间使用三次样条插值，支持平滑的速度和加速度计算。
    第一个航点的时间必须为 0。
    """

    def __init__(self, waypoints: list[Waypoint], loop: bool = False):
        """
        Args:
            waypoints: 航点列表，按时间排序
            loop: 是否循环播放
        """
        if len(waypoints) < 2:
            raise ValueError("至少需要 2 个航点")

        self.waypoints = sorted(waypoints, key=lambda w: w.t)
        self.loop = loop

        # 提取时间和位置数组
        self._t = np.array([w.t for w in self.waypoints])
        self._x = np.array([w.x for w in self.waypoints])
        self._y = np.array([w.y for w in self.waypoints])
        self._z = np.array([w.z for w in self.waypoints])

        # 三次样条插值（位置）
        self._spline_x = CubicSpline(self._t, self._x, bc_type='natural')
        self._spline_y = CubicSpline(self._t, self._y, bc_type='natural')
        self._spline_z = CubicSpline(self._t, self._z, bc_type='natural')

        # 速度 = 位置的一阶导数
        self._spline_vx = self._spline_x.derivative(1)
        self._spline_vy = self._spline_y.derivative(1)
        self._spline_vz = self._spline_z.derivative(1)

        # 加速度 = 位置的二阶导数
        self._spline_ax = self._spline_x.derivative(2)
        self._spline_ay = self._spline_y.derivative(2)
        self._spline_az = self._spline_z.derivative(2)

        self._duration = self._t[-1]

    def get_duration(self) -> float:
        return self._duration

    def _clamp_time(self, t: float) -> float:
        """处理时间越界。"""
        if self.loop:
            return t % self._duration
        return np.clip(t, self._t[0], self._t[-1])

    def get_state(self, t: float) -> TrajectoryState:
        t = self._clamp_time(t)

        pos = np.array([
            float(self._spline_x(t)),
            float(self._spline_y(t)),
            float(self._spline_z(t)),
        ])

        vel = np.array([
            float(self._spline_vx(t)),
            float(self._spline_vy(t)),
            float(self._spline_vz(t)),
        ])

        acc = np.array([
            float(self._spline_ax(t)),
            float(self._spline_ay(t)),
            float(self._spline_az(t)),
        ])

        speed = float(np.linalg.norm(vel[:2]))
        heading = float(np.arctan2(vel[1], vel[0]))

        return TrajectoryState(
            t=t,
            position=pos,
            velocity=vel,
            acceleration=acc,
            heading=heading,
            speed=speed,
        )

    def sample(self, dt: float) -> list[TrajectoryState]:
        """按固定时间步长采样整个轨迹。

        Args:
            dt: 采样时间步长 (秒)

        Returns:
            list[TrajectoryState]: 采样状态列表
        """
        times = np.arange(0, self._duration, dt)
        return [self.get_state(t) for t in times]

    @classmethod
    def from_waypoint_lists(cls, waypoint_data: list[list[float]]) -> WaypointTrajectory:
        """从航点列表创建轨迹。

        Args:
            waypoint_data: [[x, y, z, t], ...]

        Returns:
            WaypointTrajectory
        """
        waypoints = [Waypoint(x=w[0], y=w[1], z=w[2], t=w[3]) for w in waypoint_data]
        return cls(waypoints)


class LinearTrajectory(WaypointTrajectory):
    """使用线性插值的轨迹（速度在航点间恒定）。"""

    def __init__(self, waypoints: list[Waypoint], loop: bool = False):
        super().__init__(waypoints, loop)
        # 用线性插值替换三次样条
        self._spline_x = interp1d(self._t, self._x, kind='linear', fill_value='extrapolate')
        self._spline_y = interp1d(self._t, self._y, kind='linear', fill_value='extrapolate')
        self._spline_z = interp1d(self._t, self._z, kind='linear', fill_value='extrapolate')

        # 线性插值下，速度是分段常数
        self._vx = np.diff(self._x) / np.diff(self._t)
        self._vy = np.diff(self._y) / np.diff(self._t)
        self._vz = np.diff(self._z) / np.diff(self._t)

    def get_state(self, t: float) -> TrajectoryState:
        t = self._clamp_time(t)

        pos = np.array([
            float(self._spline_x(t)),
            float(self._spline_y(t)),
            float(self._spline_z(t)),
        ])

        # 找到当前段
        idx = np.searchsorted(self._t, t) - 1
        idx = np.clip(idx, 0, len(self._t) - 2)

        vel = np.array([self._vx[idx], self._vy[idx], self._vz[idx]])
        speed = float(np.linalg.norm(vel[:2]))
        heading = float(np.arctan2(vel[1], vel[0]))

        return TrajectoryState(
            t=t,
            position=pos,
            velocity=vel,
            acceleration=np.zeros(3),
            heading=heading,
            speed=speed,
        )
