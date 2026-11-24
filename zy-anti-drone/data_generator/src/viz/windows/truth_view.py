"""综合态势视图。

叠加显示真值轨迹和所有传感器的检测/航迹。
包含目标列表。
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QSplitter
)
from PyQt6.QtCore import QTimer, Qt

from viz.renderers.top_down import TopDownRenderer, COLORS


class TruthView(QWidget):
    """综合态势视图。

    显示：
    - 2D 俯视图：真值轨迹 + 各传感器航迹 + 检测点
    - 目标列表
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._truth_data: dict[int, dict] = {}  # target_id -> {x:[], y:[], ...}
        self._sensor_data: dict[str, list[dict]] = {}
        self._max_history = 1000

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 标题
        self.title_label = QLabel("<b>综合态势</b>")
        layout.addWidget(self.title_label)

        # 分割器
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # 左侧：2D 俯视图
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_item = self.plot_widget.addPlot()
        self.renderer = TopDownRenderer(self.plot_item)
        splitter.addWidget(self.plot_widget)

        # 右侧：目标列表
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(2, 2, 2, 2)

        target_label = QLabel("<b>目标列表</b>")
        right_layout.addWidget(target_label)

        self.target_table = QTableWidget()
        self.target_table.setColumnCount(5)
        self.target_table.setHorizontalHeaderLabels([
            'ID', '标签', '类型', '位置 (x,y,z)', '速度 (m/s)'
        ])
        self.target_table.setColumnWidth(0, 30)
        self.target_table.setColumnWidth(1, 100)
        self.target_table.setColumnWidth(2, 80)
        self.target_table.setColumnWidth(3, 150)
        self.target_table.setColumnWidth(4, 80)
        right_layout.addWidget(self.target_table)

        splitter.addWidget(right_widget)
        splitter.setSizes([650, 300])
        layout.addWidget(splitter, 1)

        # 刷新定时器
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_display)
        self._timer.start(50)  # 20fps

    def add_truth(self, target_id: int, label: str, target_type: str,
                  x: float, y: float, z: float, speed: float):
        """添加真值状态。

        Args:
            target_id: 目标 ID
            label: 标签
            target_type: 类型
            x, y, z: 位置 (m)
            speed: 速度 (m/s)
        """
        if target_id not in self._truth_data:
            self._truth_data[target_id] = {
                'label': label,
                'type': target_type,
                'x': [], 'y': [], 'z': [], 'speed': [],
            }
        entry = self._truth_data[target_id]
        entry['x'].append(x)
        entry['y'].append(y)
        entry['z'].append(z)
        entry['speed'].append(speed)

        # 限制历史长度
        for key in ('x', 'y', 'z', 'speed'):
            if len(entry[key]) > self._max_history:
                entry[key] = entry[key][-self._max_history:]

        # 更新渲染
        self.renderer.update_truth(target_id, x, y)

    def add_sensor_detection(self, sensor_id: str, detection: dict):
        """添加传感器检测数据。

        Args:
            sensor_id: 传感器标识
            detection: 检测字典（需包含 x, y, track_id 等）
        """
        if sensor_id not in self._sensor_data:
            self._sensor_data[sensor_id] = []
        self._sensor_data[sensor_id].append(detection)
        if len(self._sensor_data[sensor_id]) > 200:
            self._sensor_data[sensor_id] = self._sensor_data[sensor_id][-200:]

    def add_sensor_track_pos(self, sensor_id: str, track_id: int, x: float, y: float):
        """更新传感器航迹位置。"""
        color = COLORS.get(sensor_id, (255, 255, 255, 200))
        self.renderer.update_sensor_track(sensor_id, track_id, x, y)

    def _refresh_display(self):
        """刷新目标列表。"""
        self.target_table.setRowCount(0)
        for tid, data in self._truth_data.items():
            row = self.target_table.rowCount()
            self.target_table.insertRow(row)
            self.target_table.setItem(row, 0, QTableWidgetItem(str(tid)))
            self.target_table.setItem(row, 1, QTableWidgetItem(data['label']))
            self.target_table.setItem(row, 2, QTableWidgetItem(data['type']))

            if data['x']:
                pos_str = f"({data['x'][-1]:.0f}, {data['y'][-1]:.0f}, {data['z'][-1]:.0f})"
                spd_str = f"{data['speed'][-1]:.1f}"
            else:
                pos_str = "(-, -, -)"
                spd_str = "-"
            self.target_table.setItem(row, 3, QTableWidgetItem(pos_str))
            self.target_table.setItem(row, 4, QTableWidgetItem(spd_str))

    def clear(self):
        self._truth_data.clear()
        self._sensor_data.clear()
        self.renderer.clear_all()
