"""数据订阅模块。

可视化、融合算法等消费者通过 Subscriber 从共享内存读取数据。
"""

from __future__ import annotations

import json
from typing import Optional

from .shm_pool import RingBuffer, ShmPool
from .message import Detection, Track


class Subscriber:
    """传感器数据订阅者。"""

    def __init__(self, sensor_id: str, pool: ShmPool | None = None):
        """
        Args:
            sensor_id: 传感器标识
            pool: 共享内存池
        """
        self.sensor_id = sensor_id
        self._pool = pool or ShmPool()
        self._buffer: RingBuffer | None = None
        self._last_read_index: int = -1

    def _get_buffer(self) -> RingBuffer:
        if self._buffer is None:
            self._buffer = self._pool.get_buffer(self.sensor_id)
        return self._buffer

    def get_new_detections(self) -> list[Detection]:
        """获取上次读取以来的新检测。

        Returns:
            list[Detection]: 新检测列表
        """
        buf = self._get_buffer()
        latest = buf.get_latest_index()

        detections = []
        start = max(self._last_read_index + 1, max(0, latest - 500))
        for idx in range(start, latest + 1):
            data = buf.read(idx)
            if data:
                try:
                    det = Detection.from_bytes(data)
                    detections.append(det)
                except Exception:
                    pass

        self._last_read_index = latest
        return detections

    def get_latest_detections(self, n: int = 50) -> list[Detection]:
        """获取最近 N 条检测。

        Args:
            n: 数量

        Returns:
            list[Detection]
        """
        buf = self._get_buffer()
        latest = buf.get_latest_index()
        if latest < 0:
            return []

        detections = []
        start = max(0, latest - n + 1)
        for idx in range(start, latest + 1):
            data = buf.read(idx)
            if data:
                try:
                    det = Detection.from_bytes(data)
                    detections.append(det)
                except Exception:
                    pass

        return detections

    def get_latest_tracks(self, n: int = 50) -> list[Track]:
        """获取最近 N 条航迹。

        Args:
            n: 数量

        Returns:
            list[Track]
        """
        buf = self._get_buffer()
        latest = buf.get_latest_index()
        if latest < 0:
            return []

        tracks = []
        start = max(0, latest - n + 1)
        for idx in range(start, latest + 1):
            data = buf.read(idx)
            if data:
                try:
                    d = json.loads(data.decode('utf-8'))
                    d['state'] = __import__('numpy').array(d['state'])
                    d['covariance'] = __import__('numpy').array(d['covariance'])
                    tracks.append(Track(**d))
                except Exception:
                    pass

        return tracks

    def close(self):
        if self._buffer:
            self._buffer.close()
