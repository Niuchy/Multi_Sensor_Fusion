"""共享内存池管理模块。

使用 multiprocessing.shared_memory 实现进程间的高效数据共享。
每个传感器拥有一个环形缓冲区，存储最近 N 条检测/航迹数据。
"""

from __future__ import annotations

import json
import time
import struct
import numpy as np
from multiprocessing import shared_memory
from dataclasses import dataclass
from typing import Optional


# ──────────────────────────────────────────────
# 环形缓冲区布局
# ──────────────────────────────────────────────
#
# 每个缓冲区的前 16 字节为头部：
#   - bytes 0-3:   magic (0xAD01)
#   - bytes 4-7:   write_index (uint32)
#   - bytes 8-11:  capacity (uint32, max items)
#   - bytes 12-15: item_size (uint32, bytes per item)
#
# 之后是 capacity * item_size 的数据区。
# ──────────────────────────────────────────────

HEADER_SIZE = 16
MAGIC = 0xAD01
DEFAULT_CAPACITY = 2000
DEFAULT_ITEM_SIZE = 512  # 每条消息最大字节数


class RingBuffer:
    """共享内存环形缓冲区。

    单个生产者、多个消费者的场景。
    """

    def __init__(self, name: str, capacity: int = DEFAULT_CAPACITY,
                 item_size: int = DEFAULT_ITEM_SIZE, create: bool = False):
        """
        Args:
            name: 共享内存名称
            capacity: 缓冲区容量（条目数）
            item_size: 每条消息的最大字节数
            create: True=创建新的共享内存，False=连接已有
        """
        self.name = name
        self.capacity = capacity
        self.item_size = item_size
        total_size = HEADER_SIZE + capacity * item_size

        # 移除名称中的路径分隔符（Linux 共享内存名称不能含 /）
        safe_name = name.replace('/', '_').replace('\\', '_')

        if create:
            # 先尝试清理旧的内存段
            try:
                old = shared_memory.SharedMemory(name=safe_name)
                old.close()
                old.unlink()
            except FileNotFoundError:
                pass
            self._shm = shared_memory.SharedMemory(
                name=safe_name, create=True, size=total_size
            )
            self._buf = memoryview(self._shm.buf)
            self._init_header()
        else:
            self._shm = shared_memory.SharedMemory(name=safe_name)
            self._buf = memoryview(self._shm.buf)

        self._buf = memoryview(self._shm.buf)

    def _init_header(self):
        """初始化头部字段。"""
        struct.pack_into('<IIII', self._buf, 0, MAGIC, 0, self.capacity, self.item_size)

    def _read_header(self):
        """读取头部信息。"""
        magic, write_idx, cap, item_sz = struct.unpack_from('<IIII', self._buf, 0)
        if magic != MAGIC:
            raise RuntimeError(f"共享内存 magic 不匹配: 0x{magic:04X}")
        return write_idx, cap, item_sz

    def write(self, data: bytes) -> int:
        """写入一条消息。

        Args:
            data: 消息字节（不能超过 item_size）

        Returns:
            int: 写入位置索引
        """
        if len(data) > self.item_size:
            raise ValueError(f"消息大小 {len(data)} 超过 item_size {self.item_size}")

        write_idx, cap, item_sz = self._read_header()

        # 计算写入偏移
        offset = HEADER_SIZE + (write_idx % cap) * item_sz

        # 写入消息长度 + 数据
        msg = struct.pack('<I', len(data)) + data.ljust(self.item_size - 4, b'\x00')
        self._buf[offset:offset + self.item_size] = msg

        # 更新写指针
        new_idx = write_idx + 1
        struct.pack_into('<I', self._buf, 4, new_idx)

        return write_idx

    def read(self, index: int) -> Optional[bytes]:
        """读取指定索引的消息。

        Args:
            index: 消息索引

        Returns:
            bytes | None: 消息数据，如果已被覆盖则返回 None
        """
        write_idx, cap, item_sz = self._read_header()

        # 检查是否已被覆盖
        if index < write_idx - cap:
            return None

        offset = HEADER_SIZE + (index % cap) * item_sz
        msg_len = struct.unpack_from('<I', self._buf, offset)[0]
        if msg_len == 0 or msg_len > item_sz:
            return None

        return bytes(self._buf[offset + 4:offset + 4 + msg_len])

    def get_latest_index(self) -> int:
        """获取最新写入的索引（-1 表示无数据）。"""
        write_idx, _, _ = self._read_header()
        if write_idx == 0:
            return -1
        return write_idx - 1

    def get_write_index(self) -> int:
        """获取当前写索引。"""
        write_idx, _, _ = self._read_header()
        return write_idx

    def close(self):
        """关闭共享内存（不删除）。"""
        try:
            self._buf = None  # 释放 memoryview 引用
            self._shm.close()
        except BufferError:
            pass  # 共享内存被其他进程引用时忽略

    def unlink(self):
        """删除共享内存。"""
        try:
            self._shm.unlink()
        except Exception:
            pass


class ShmPool:
    """共享内存池 — 管理多个环形缓冲区。

    用法:
        pool = ShmPool()
        pool.create_buffer("radar")
        buf = pool.get_buffer("radar")
        buf.write(data)
    """

    def __init__(self):
        self._buffers: dict[str, RingBuffer] = {}

    def create_buffer(self, name: str, capacity: int = DEFAULT_CAPACITY,
                      item_size: int = DEFAULT_ITEM_SIZE) -> RingBuffer:
        """创建新的环形缓冲区。"""
        buf = RingBuffer(f"ad_{name}", capacity, item_size, create=True)
        self._buffers[name] = buf
        return buf

    def get_buffer(self, name: str) -> RingBuffer:
        """获取已有缓冲区（如果不存在则创建）。"""
        if name not in self._buffers:
            try:
                buf = RingBuffer(f"ad_{name}", create=False)
                self._buffers[name] = buf
            except FileNotFoundError:
                buf = self.create_buffer(name)
        return self._buffers[name]

    def close_all(self):
        """关闭所有缓冲区。"""
        for buf in self._buffers.values():
            buf.close()
        self._buffers.clear()

    def unlink_all(self):
        """删除所有缓冲区。"""
        for buf in self._buffers.values():
            buf.unlink()
        self._buffers.clear()
