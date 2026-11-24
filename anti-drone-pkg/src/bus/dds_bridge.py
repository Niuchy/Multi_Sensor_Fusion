"""轨迹输出桥接模块。

通过 UDP 将雷达/光电/电侦的航迹数据实时发送到外部消费者（融合算法等）。
底层使用纯 C 库 libad_output.so，无额外依赖。

用法:
    bridge = OutputBridge()
    bridge.init("127.0.0.1", 9999)
    bridge.send_track("radar", track_data)
"""

from __future__ import annotations

import ctypes
import json
import logging
import time
from pathlib import Path

logger = logging.getLogger(__name__)


class OutputBridge:
    """UDP 轨迹输出桥接器。

    将仿真产生的航迹数据通过 UDP 发送到指定 IP:端口。
    JSON 格式，便于任何语言/框架消费。
    """

    def __init__(self, lib_path: str | None = None, use_dds: bool = True):
        """初始化输出桥接。

        Args:
            lib_path: .so 文件路径
            use_dds: True=使用 DDS (libad_dds_bridge.so)，False=使用 UDP (libad_output.so)
        """
        if lib_path is None:
            project_root = Path(__file__).parent.parent.parent
            if use_dds:
                candidates = [
                    project_root / "thirdparty/bridge/libad_dds_bridge.so",
                    Path("thirdparty/bridge/libad_dds_bridge.so"),
                ]
            else:
                candidates = [
                    project_root / "thirdparty/bridge/libad_output.so",
                    Path("thirdparty/bridge/libad_output.so"),
                ]
            lib_path = None
            for c in candidates:
                if c.exists():
                    lib_path = str(c.resolve())
                    break

        if lib_path is None:
            self._available = False
            mode = "DDS" if use_dds else "UDP"
            logger.warning(f"输出桥接库未找到 ({mode})，轨迹输出不可用")
            return

        try:
            # 设置 LD_LIBRARY_PATH（DDS 需要）
            if use_dds:
                import os
                extra = [str((project_root / p).resolve()) for p in [
                    "thirdparty/PubSubTools", "thirdparty/common", "thirdparty/extracted/lib"
                ]]
                existing = os.environ.get('LD_LIBRARY_PATH', '')
                os.environ['LD_LIBRARY_PATH'] = ':'.join(extra + ([existing] if existing else []))

            self._use_dds = use_dds
            self._lib = ctypes.CDLL(lib_path)
            self._setup_signatures()
            self._available = True
            logger.info(f"输出桥接库已加载 ({'DDS' if use_dds else 'UDP'}): {lib_path}")
        except Exception as e:
            self._available = False
            logger.warning(f"输出桥接库加载失败: {e}")

        self._initialized = False

    def _setup_signatures(self):
        lib = self._lib
        lib.ad_output_version.restype = ctypes.c_char_p

        lib.ad_output_init.restype = ctypes.c_int
        lib.ad_output_init.argtypes = [ctypes.c_char_p, ctypes.c_uint16]

        lib.ad_output_send_json.restype = ctypes.c_int
        lib.ad_output_send_json.argtypes = [ctypes.c_char_p]

        lib.ad_output_ready.restype = ctypes.c_int
        lib.ad_output_close.restype = None

        # DDS 特有函数
        if self._use_dds:
            lib.ad_output_create_writer.restype = ctypes.c_int
            lib.ad_output_create_writer.argtypes = [ctypes.c_char_p, ctypes.c_int]
            lib.ad_output_publish.restype = ctypes.c_int
            lib.ad_output_publish.argtypes = [ctypes.c_char_p,
                ctypes.POINTER(ctypes.c_uint8), ctypes.c_uint32, ctypes.c_int]

    @property
    def available(self) -> bool:
        return self._available

    # ── 生命周期 ──

    def init(self, dest_ip: str = "127.0.0.1", dest_port: int = 9999) -> bool:
        """初始化输出。

        Args:
            dest_ip: 目标 IP（DDS 模式用于多播，UDP 模式用于单播）
            dest_port: 目标端口（UDP 模式使用）
        """
        if not self.available:
            return False
        rc = self._lib.ad_output_init(dest_ip.encode('utf-8'), dest_port)
        self._initialized = (rc == 0)
        if self._initialized:
            mode = "DDS" if self._use_dds else "UDP"
            logger.info(f"轨迹输出已就绪 ({mode}) → {dest_ip}:{dest_port}")
        return self._initialized

    def create_writer(self, topic: str, reliable: bool = False) -> bool:
        """DDS 模式：创建话题发布者。UDP 模式：无操作。"""
        if not self.ready or not self._use_dds:
            return False
        rc = self._lib.ad_output_create_writer(topic.encode('utf-8'), 1 if reliable else 0)
        return rc == 0

    def publish_raw(self, topic: str, data: bytes, level: int = 2) -> bool:
        """DDS 模式：发布二进制数据到指定话题。"""
        if not self.ready or not self._use_dds:
            return False
        buf = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        rc = self._lib.ad_output_publish(topic.encode('utf-8'), buf, len(data), level)
        return rc == 0

    def close(self):
        """关闭输出（DDS 模式不调用析构）。"""
        if self.available and self._initialized and not self._use_dds:
            self._lib.ad_output_close()
        self._initialized = False

    @property
    def ready(self) -> bool:
        return self._initialized and self._lib.ad_output_ready() == 1

    # ── 便捷发送 ──

    def send_json(self, data: str) -> bool:
        """发送 JSON 字符串。DDS 模式返回是否成功，UDP 模式返回字节数。"""
        if not self.ready:
            return False
        if self._use_dds:
            rc = self._lib.ad_output_send_json(data.encode('utf-8'))
            return rc == 0  # DDS: 0=成功
        else:
            return self._lib.ad_output_send_json(data.encode('utf-8')) > 0

    def send_dict(self, data: dict) -> int:
        return self.send_json(json.dumps(data, ensure_ascii=False))

    def send_detection(self, sensor_type: str, detection: dict) -> int:
        """发送单条检测数据。

        Args:
            sensor_type: 'radar' / 'eo' / 'esm'
            detection: 检测字典
        """
        msg = {
            'type': sensor_type,
            'msg': 'detection',
            'ts': detection.get('timestamp', time.time()),
            'tid': detection.get('track_id', -1),
            'meas': {k: v for k, v in detection.items()
                     if k in ('range', 'azimuth', 'elevation', 'doppler',
                              'frequency', 'pri', 'pw', 'modulation')},
            'conf': detection.get('confidence', 0),
        }
        return self.send_dict(msg)

    def send_track(self, sensor_type: str, track: dict) -> int:
        """发送航迹数据。

        Args:
            sensor_type: 'radar' / 'eo' / 'esm' / 'truth'
            track: 航迹字典，包含 track_id, state, position 等
        """
        msg = {
            'type': sensor_type,
            'msg': 'track',
            'ts': track.get('update_time', time.time()),
            'tid': track.get('track_id', -1),
            'state': track.get('state', 'unknown'),
            'pos': track.get('position'),
            'az': track.get('azimuth'),
            'el': track.get('elevation'),
        }
        return self.send_dict(msg)


class TrackOutputPublisher:
    """仿真航迹发布器。

    DDS 模式独话题：
    - radar/track — 雷达航迹
    - eo/track — 光电跟踪
    - esm/track — 电侦检测
    - truth/track — 真值（可选）

    UDP 模式所有数据发到同一个端口。
    """

    def __init__(self, bridge: OutputBridge):
        self._bridge = bridge
        self._seq = 0
        self._writers_created: set[str] = set()

    def _ensure_writer(self, topic: str):
        if topic not in self._writers_created:
            self._bridge.create_writer(topic)
            self._writers_created.add(topic)

    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def publish_detections(self, sensor: str, detections: list[dict], sim_time: float):
        for det in detections:
            det['timestamp'] = det.get('timestamp', sim_time)
            self._bridge.send_detection(sensor, det)

    def publish_track(self, sensor: str, track: dict):
        topic = f"ad/{sensor}/track"
        self._ensure_writer(topic)
        track['seq'] = self._next_seq()
        data = json.dumps(track, ensure_ascii=False).encode('utf-8')
        self._bridge.publish_raw(topic, data)
