"""数据发布模块。

传感器进程通过 Publisher 将检测/航迹数据写入共享内存。
"""

from __future__ import annotations

from .shm_pool import RingBuffer, ShmPool
from .message import Detection, Track


class Publisher:
    """传感器数据发布者。"""

    def __init__(self, sensor_id: str, pool: ShmPool | None = None):
        """
        Args:
            sensor_id: 传感器标识（如 "radar-01"）
            pool: 共享内存池（可选，默认创建新的）
        """
        self.sensor_id = sensor_id
        self._pool = pool or ShmPool()
        self._buffer: RingBuffer | None = None

    def _get_buffer(self) -> RingBuffer:
        if self._buffer is None:
            self._buffer = self._pool.get_buffer(self.sensor_id)
        return self._buffer

    def publish_detection(self, detection: Detection) -> int:
        """发布一条检测。

        Args:
            detection: 检测对象

        Returns:
            int: 写入索引
        """
        return self._get_buffer().write(detection.to_bytes())

    def publish_track(self, track: Track) -> int:
        """发布一条航迹。

        Args:
            track: 航迹对象

        Returns:
            int: 写入索引
        """
        import json
        return self._get_buffer().write(
            json.dumps(track.to_dict()).encode('utf-8')
        )

    def get_buffer(self) -> RingBuffer:
        """获取底层缓冲区（供订阅者使用）。"""
        return self._get_buffer()

    def close(self):
        if self._buffer:
            self._buffer.close()
