"""雷达 PPI 视图。

以 Plan Position Indicator 方式显示雷达检测：
- 极坐标俯视图（可切换为直角坐标）
- 检测点、航迹、杂波点
- 探测范围圈
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox
from PyQt6.QtCore import QTimer, Qt

from viz.renderers.top_down import TopDownRenderer, COLORS


class RadarView(QWidget):
    """雷达传感器独立视图窗口。"""

    def __init__(self, sensor_id: str = "radar-01", parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self._data_buffer: list[dict] = []
        self._max_buffer = 1000

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 顶部信息栏
        info_layout = QHBoxLayout()
        self.title_label = QLabel(f"<b>雷达 PPI — {self.sensor_id}</b>")
        self.info_label = QLabel("目标: 0 | 检测: 0 | 杂波: 0")
        self.show_clutter_cb = QCheckBox("显示杂波")
        self.show_clutter_cb.setChecked(True)
        self.polar_cb = QCheckBox("极坐标")
        self.polar_cb.setChecked(False)
        info_layout.addWidget(self.title_label)
        info_layout.addStretch()
        info_layout.addWidget(self.show_clutter_cb)
        info_layout.addWidget(self.polar_cb)
        info_layout.addWidget(self.info_label)
        layout.addLayout(info_layout)

        # pyqtgraph 绘图区
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_item = self.plot_widget.addPlot()
        layout.addWidget(self.plot_widget, 1)

        # 渲染器
        self.renderer = TopDownRenderer(self.plot_item)

        # 探测范围圈
        self._range_ring = None
        self._draw_range_ring(5000.0)

        # 刷新定时器 (30fps)
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_display)
        self._timer.start(33)

    def _draw_range_ring(self, max_range: float):
        """绘制最大探测范围圆。"""
        theta = np.linspace(0, 2 * np.pi, 200)
        x = max_range * np.cos(theta)
        y = max_range * np.sin(theta)
        pen = pg.mkPen(color=(0, 255, 0, 60), width=1, style=Qt.PenStyle.DashLine)
        if self._range_ring:
            self.plot_item.removeItem(self._range_ring)
        self._range_ring = self.plot_item.plot(x, y, pen=pen)

    def add_detection(self, detection: dict):
        """添加一条检测数据。

        Args:
            detection: {
                'timestamp': float,
                'track_id': int,
                'x': float, 'y': float,  # 世界坐标
                'is_clutter': bool,
                'confidence': float,
                ...
            }
        """
        self._data_buffer.append(detection)
        if len(self._data_buffer) > self._max_buffer:
            self._data_buffer = self._data_buffer[-self._max_buffer:]

    def add_truth(self, target_id: int, x: float, y: float):
        """添加真值数据。"""
        self.renderer.update_truth(target_id, x, y)

    def _refresh_display(self):
        """刷新显示。"""
        if not self._data_buffer:
            return

        # 取最近一秒的数据
        recent = self._data_buffer[-200:]

        # 分离检测点和杂波点
        det_positions = []
        clutter_positions = []
        track_points: dict[int, list] = {}  # track_id -> [(x,y), ...]

        for d in recent:
            x, y = d.get('x', 0), d.get('y', 0)
            if d.get('is_clutter', False):
                clutter_positions.append([x, y])
            else:
                det_positions.append([x, y])
                tid = d.get('track_id', -1)
                if tid >= 0:
                    if tid not in track_points:
                        track_points[tid] = []
                    track_points[tid].append([x, y])

        # 绘制
        if det_positions:
            self.renderer.plot_detections(
                self.sensor_id, np.array(det_positions),
                color=COLORS['radar'], symbol='o', size=8
            )

        if clutter_positions and self.show_clutter_cb.isChecked():
            self.renderer.plot_clutter(np.array(clutter_positions))

        # 更新航迹线
        for tid, points in track_points.items():
            if len(points) >= 2:
                pts = np.array(points)
                # 重新绘制整个航迹（简化处理）
                key = f"radar_track_{tid}"
                # 如果已存在，更新；否则通过 renderer 添加
                self.renderer.update_sensor_track('radar', tid, pts[-1, 0], pts[-1, 1])

        # 更新信息标签
        self.info_label.setText(
            f"点迹: {len(det_positions)} | 杂波: {len(clutter_positions)} | "
            f"航迹: {len(track_points)} | 累计: {len(self._data_buffer)}"
        )

    def set_max_range(self, max_range: float):
        self._draw_range_ring(max_range)

    def clear(self):
        self._data_buffer.clear()
        self.renderer.clear_all()
        self._draw_range_ring(5000.0)
