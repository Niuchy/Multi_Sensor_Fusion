"""传感器基类。

定义所有传感器的公共接口和进程框架。
"""

from __future__ import annotations

import time
import logging
import numpy as np
from multiprocessing import Process, Event
from typing import Optional

from truth.target import Target
from bus.message import Detection, DetectionFlag
from bus.publisher import Publisher
from bus.shm_pool import ShmPool
from sensors.error_models import ErrorPipeline

logger = logging.getLogger(__name__)


class SensorBase(Process):
    """传感器进程基类。

    每个传感器作为独立进程运行，有自己的时钟和更新频率。
    通过共享内存发布检测/航迹数据。
    """

    def __init__(
        self,
        sensor_id: str,
        sensor_type: str,
        position: list[float],
        orientation: list[float],
        update_rate: float,
        fov: dict | None = None,
        shm_pool: ShmPool | None = None,
    ):
        """
        Args:
            sensor_id: 传感器标识
            sensor_type: 'radar' / 'eo' / 'esm'
            position: [x, y, z] 部署位置 (m)
            orientation: [azimuth, elevation] 初始指向 (度)
            update_rate: 更新频率 (Hz)
            fov: 视场 {'azimuth': [min, max], 'elevation': [min, max]} (度)
            shm_pool: 共享内存池
        """
        super().__init__()
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type
        self.position = np.array(position, dtype=np.float64)
        self.orientation = np.array(orientation, dtype=np.float64)
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate
        self.fov = fov or {'azimuth': [-180, 180], 'elevation': [-90, 90]}

        self._shm_pool = shm_pool or ShmPool()
        self._publisher: Publisher | None = None
        self._stop_event = Event()
        self._error_pipeline = ErrorPipeline()

        # 状态
        self._last_update_time = 0.0
        self._track_counter = 0

    def run(self):
        """进程主循环。"""
        logger.info(f"[{self.sensor_id}] 传感器进程启动, update_rate={self.update_rate}Hz")

        self._publisher = Publisher(self.sensor_id, self._shm_pool)
        self._setup_error_pipeline()

        while not self._stop_event.is_set():
            cycle_start = time.perf_counter()

            # 获取当前仿真时间（从共享内存读取，此处简化）
            # 实际系统中从场景控制器获取
            sim_time = time.time()

            # 执行检测
            detections = self.detect(sim_time)

            # 发布检测结果
            for det in detections:
                self._publisher.publish_detection(det)

            self._last_update_time = sim_time

            # 等待到下一个更新周期
            elapsed = time.perf_counter() - cycle_start
            sleep_time = max(0, self.update_interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info(f"[{self.sensor_id}] 传感器进程停止")

    def _setup_error_pipeline(self):
        """配置误差流水线（子类重写）。"""
        pass

    def detect(self, sim_time: float) -> list[Detection]:
        """执行一次检测。

        Args:
            sim_time: 当前仿真时间

        Returns:
            list[Detection]: 检测结果列表
        """
        raise NotImplementedError

    def stop(self):
        """请求停止进程。"""
        self._stop_event.set()

    def make_detection(
        self,
        timestamp: float,
        measurements: dict,
        covariance: dict | None = None,
        confidence: float = 1.0,
        flags: set | None = None,
    ) -> Detection:
        """创建检测对象。

        Args:
            timestamp: 时间戳
            measurements: 测量值字典
            covariance: 测量协方差
            confidence: 检测置信度
            flags: 标志位

        Returns:
            Detection 对象
        """
        return Detection(
            timestamp=timestamp,
            sensor_id=self.sensor_id,
            track_id=self._track_counter,
            measurements=measurements,
            covariance=covariance or self._default_covariance(measurements),
            confidence=confidence,
            flags=flags or {DetectionFlag.NORMAL},
        )

    def _default_covariance(self, measurements: dict) -> dict:
        """生成默认的测量协方差（对角矩阵的 dict 表示）。"""
        return {k: 1.0 for k in measurements}

    def world_to_sensor(self, world_pos: np.ndarray) -> tuple[float, float, float]:
        """世界坐标 → 传感器球坐标。

        Args:
            world_pos: [x, y, z] 世界坐标

        Returns:
            (range, azimuth_rad, elevation_rad)
        """
        rel = world_pos - self.position
        r = float(np.linalg.norm(rel))
        az = float(np.arctan2(rel[1], rel[0]))
        el = float(np.arctan2(rel[2], np.linalg.norm(rel[:2])))
        return r, az, el

    def is_in_fov(self, az_rad: float, el_rad: float) -> bool:
        """检查角度是否在视场内。"""
        az_deg = np.degrees(az_rad)
        el_deg = np.degrees(el_rad)
        fov_az = self.fov['azimuth']
        fov_el = self.fov['elevation']
        return (
            fov_az[0] <= az_deg <= fov_az[1] and
            fov_el[0] <= el_deg <= fov_el[1]
        )
