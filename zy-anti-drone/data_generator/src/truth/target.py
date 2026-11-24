"""目标定义模块。"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field

from .trajectory import Trajectory, WaypointTrajectory, TrajectoryState


@dataclass
class EmitterParams:
    """辐射源参数（电侦传感器用）。

    支持时变辐射状态：
    - active: 基础开关状态
    - active_after: 在此时间之后辐射源开启（秒），None 表示不限制
    - active_until: 在此时间之后辐射源关闭（秒），None 表示不限制
    """
    active: bool = True
    freq: float = 2.4e9           # 载频 (Hz)
    pri: float = 1000e-6          # 脉冲重复间隔 (s)
    pw: float = 10e-6             # 脉宽 (s)
    modulation: str = "OFDM"      # 调制类型
    power: float = 0.1            # 发射功率 (W)
    active_after: float | None = None   # 秒，此时间后开启
    active_until: float | None = None   # 秒，此时间后关闭


@dataclass
class Target:
    """仿真目标。

    包含目标标识、物理属性、运动轨迹和辐射源参数。
    """
    id: int
    label: str = ""
    target_type: str = "quadcopter"  # quadcopter / fixed_wing / other
    rcs: float = 0.01                # 雷达散射截面 (m²)，平均值
    trajectory: Trajectory | None = None
    emitter: EmitterParams = field(default_factory=EmitterParams)

    def get_state(self, t: float) -> TrajectoryState | None:
        """获取目标在 t 时刻的真值状态。"""
        if self.trajectory is None:
            return None
        return self.trajectory.get_state(t)

    def get_rcs(self, aspect_angle: float = 0.0, frequency: float = 3.0e9) -> float:
        """获取目标的 RCS 值。

        简化模型：使用 Swerling I 型起伏，RCS 服从指数分布。

        Args:
            aspect_angle: 雷达视线方位角 (rad)，此处简化未用
            frequency: 雷达频率 (Hz)，此处简化未用

        Returns:
            float: 瞬时 RCS 值 (m²)
        """
        # Swerling I: 指数分布，均值为 self.rcs
        return float(np.random.exponential(self.rcs))

    def is_emitting(self, t: float) -> bool:
        """检查目标在 t 时刻是否正在辐射。

        支持时变辐射状态：
        - 如果 active=False，始终不辐射
        - 如果 active=True 且 active_after 有值，在该时间之后才开始辐射
        - 如果 active=True 且 active_until 有值，在该时间之后停止辐射
        """
        if not self.emitter.active:
            return False
        if self.emitter.active_after is not None and t < self.emitter.active_after:
            return False
        if self.emitter.active_until is not None and t >= self.emitter.active_until:
            return False
        return True
