"""消息定义模块。

定义仿真系统中所有消息类型。
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass, field
from enum import Enum, auto


class DetectionFlag(Enum):
    """检测标志。"""
    NORMAL = auto()
    CLUTTER = auto()
    JAMMED = auto()
    LOW_SNR = auto()
    OUTLIER = auto()
    FALSE_TRACK = auto()


@dataclass
class Detection:
    """单次传感器检测结果。

    包含传感器局部坐标系下的测量值和误差信息。
    """
    timestamp: float
    sensor_id: str
    track_id: int = -1            # 传感器内部航迹号，-1 表示未关联
    measurements: dict = field(default_factory=dict)
    # 典型测量键值：
    #   雷达: range, azimuth, elevation, doppler
    #   光电: azimuth, elevation
    #   电侦: azimuth, frequency, pri, pw, modulation

    covariance: dict = field(default_factory=dict)
    # 测量协方差（对角元素）

    confidence: float = 1.0        # 检测置信度 [0, 1]
    flags: set = field(default_factory=lambda: {DetectionFlag.NORMAL})

    # 真值信息（仅用于调试/评估，实际系统中不存在）
    truth_position: np.ndarray | None = None

    def to_dict(self) -> dict:
        """转换为可序列化的字典。"""
        return {
            'timestamp': self.timestamp,
            'sensor_id': self.sensor_id,
            'track_id': self.track_id,
            'measurements': self.measurements,
            'covariance': self.covariance,
            'confidence': self.confidence,
            'flags': [f.name for f in self.flags],
        }

    def to_bytes(self) -> bytes:
        """序列化为字节（简化版用 JSON）。"""
        import json
        d = self.to_dict()
        if self.truth_position is not None:
            d['truth_position'] = self.truth_position.tolist()
        return json.dumps(d).encode('utf-8')

    @classmethod
    def from_bytes(cls, data: bytes) -> 'Detection':
        """从字节反序列化。"""
        import json
        d = json.loads(data.decode('utf-8'))
        flags = {DetectionFlag[f] for f in d.pop('flags', [])}
        truth = d.pop('truth_position', None)
        det = cls(**d)
        det.flags = flags
        if truth is not None:
            det.truth_position = np.array(truth)
        return det


@dataclass
class Track:
    """传感器内部航迹。

    包含世界坐标系下的状态估计及协方差。
    """
    track_id: int
    sensor_id: str
    target_id: int = -1           # 关联的真值目标 ID（-1 表示未知）
    state: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # [x, y, z, vx, vy, vz] (世界坐标系)

    covariance: np.ndarray = field(default_factory=lambda: np.eye(6) * 100)
    # 6x6 状态协方差矩阵

    update_time: float = 0.0
    updates: list[Detection] = field(default_factory=list)
    confidence: float = 1.0

    def to_dict(self) -> dict:
        return {
            'track_id': self.track_id,
            'sensor_id': self.sensor_id,
            'target_id': self.target_id,
            'state': self.state.tolist(),
            'covariance': self.covariance.tolist(),
            'update_time': self.update_time,
            'num_updates': len(self.updates),
            'confidence': self.confidence,
        }


@dataclass
class SimulationClock:
    """仿真时钟消息。"""
    sim_time: float
    dt: float
    status: str = "running"      # running / paused / stopped
