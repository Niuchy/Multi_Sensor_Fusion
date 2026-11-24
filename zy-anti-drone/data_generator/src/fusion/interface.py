"""融合算法接口模块。

提供标准化的数据访问接口，供外部融合算法消费传感器数据。

支持两种消费模式：
1. 回调模式：注册回调函数，数据到达时自动调用
2. 轮询模式：主动拉取最新数据
"""

from __future__ import annotations

import time
import json
import logging
import threading
from collections import deque
from typing import Callable, Optional

import numpy as np

from bus.message import Detection, Track
from bus.subscriber import Subscriber
from bus.shm_pool import ShmPool

logger = logging.getLogger(__name__)


class FusionInterface:
    """融合算法数据接口。

    用法:
        # 回调模式
        fi = FusionInterface()
        fi.on_detection("radar-01", my_callback)
        fi.start()

        # 轮询模式
        detections = fi.get_latest_detections("radar-01")
    """

    def __init__(self, shm_pool: ShmPool | None = None):
        """
        Args:
            shm_pool: 共享内存池，None 则自动创建
        """
        self._pool = shm_pool or ShmPool()
        self._subscribers: dict[str, Subscriber] = {}
        self._callbacks: dict[str, list[Callable]] = {}

        self._polling_thread: threading.Thread | None = None
        self._running = False
        self._poll_interval = 0.01  # 10ms

        # 本地数据缓存
        self._detection_buffers: dict[str, deque] = {}
        self._max_buffer_size = 5000

    def register_sensor(self, sensor_id: str):
        """注册传感器。

        Args:
            sensor_id: 传感器标识（如 "radar-01"）
        """
        if sensor_id not in self._subscribers:
            self._subscribers[sensor_id] = Subscriber(sensor_id, self._pool)
            self._detection_buffers[sensor_id] = deque(maxlen=self._max_buffer_size)
            self._callbacks[sensor_id] = []
        logger.info(f"FusionInterface: 注册传感器 {sensor_id}")

    def on_detection(self, sensor_id: str, callback: Callable[[Detection], None]):
        """注册检测回调。

        Args:
            sensor_id: 传感器标识
            callback: 回调函数，接收 Detection 对象
        """
        if sensor_id not in self._subscribers:
            self.register_sensor(sensor_id)
        self._callbacks[sensor_id].append(callback)

    def on_track(self, sensor_id: str, callback: Callable[[Track], None]):
        """注册航迹回调。

        Args:
            sensor_id: 传感器标识
            callback: 回调函数，接收 Track 对象
        """
        # Track 回调的处理方式类似，此处简化
        if sensor_id not in self._subscribers:
            self.register_sensor(sensor_id)

    def get_latest_detections(self, sensor_id: str, n: int = 100) -> list[Detection]:
        """获取最近 N 条检测（轮询模式）。

        Args:
            sensor_id: 传感器标识
            n: 获取数量

        Returns:
            list[Detection]: 检测列表
        """
        if sensor_id not in self._subscribers:
            self.register_sensor(sensor_id)

        # 尝试从共享内存读取新数据
        try:
            new_dets = self._subscribers[sensor_id].get_new_detections()
            buf = self._detection_buffers.setdefault(
                sensor_id, deque(maxlen=self._max_buffer_size)
            )
            for det in new_dets:
                buf.append(det)
                # 触发回调
                for cb in self._callbacks.get(sensor_id, []):
                    try:
                        cb(det)
                    except Exception as e:
                        logger.error(f"回调异常: {e}")
        except Exception as e:
            logger.debug(f"读取共享内存失败 ({sensor_id}): {e}")

        # 返回缓存中的最近 N 条
        buf = self._detection_buffers.get(sensor_id, deque())
        return list(buf)[-n:]

    def get_all_detections(self, sensor_id: str,
                           since_time: float = 0.0) -> list[Detection]:
        """获取某个时间之后的所有检测。

        Args:
            sensor_id: 传感器标识
            since_time: 起始时间

        Returns:
            list[Detection]: 检测列表
        """
        dets = self.get_latest_detections(sensor_id, self._max_buffer_size)
        return [d for d in dets if d.timestamp >= since_time]

    def get_detections_as_array(self, sensor_id: str, n: int = 100) -> np.ndarray:
        """将检测转换为 NumPy 数组（方便融合算法处理）。

        Args:
            sensor_id: 传感器标识
            n: 检测数量

        Returns:
            np.ndarray: shape (N, 5) = [timestamp, range, az, el, doppler]
                        或根据传感器类型调整
        """
        dets = self.get_latest_detections(sensor_id, n)
        if not dets:
            return np.empty((0,))

        data = []
        for d in dets:
            m = d.measurements
            row = [
                d.timestamp,
                m.get('range', np.nan),
                m.get('azimuth', np.nan),
                m.get('elevation', np.nan),
                m.get('doppler', np.nan),
                d.confidence,
            ]
            data.append(row)

        return np.array(data)

    def start(self):
        """启动后台轮询线程。"""
        self._running = True
        self._polling_thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._polling_thread.start()
        logger.info("FusionInterface: 后台轮询启动")

    def stop(self):
        """停止后台轮询。"""
        self._running = False
        if self._polling_thread:
            self._polling_thread.join(timeout=1.0)
        logger.info("FusionInterface: 已停止")

    def _poll_loop(self):
        """后台轮询循环。"""
        while self._running:
            for sensor_id in list(self._subscribers.keys()):
                try:
                    self.get_latest_detections(sensor_id, n=10)
                except Exception:
                    pass
            time.sleep(self._poll_interval)

    def close(self):
        """关闭所有订阅者。"""
        self.stop()
        for sub in self._subscribers.values():
            sub.close()
        self._subscribers.clear()
