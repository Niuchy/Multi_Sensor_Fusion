"""电侦传感器视图。

显示方位-时间瀑布图、频率-方位散点图、辐射源参数。
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTableWidget,
    QTableWidgetItem, QSplitter
)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QColor


class ESMView(QWidget):
    """电侦传感器独立视图窗口。

    显示内容：
    - 方位-时间图（上半）
    - 频率-方位散点图（下半）
    - 辐射源参数表（右侧）
    """

    def __init__(self, sensor_id: str = "esm-01", parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self._data_buffer: list[dict] = []

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 信息栏
        info_layout = QHBoxLayout()
        self.title_label = QLabel(f"<b>电侦侦测 — {self.sensor_id}</b>")
        self.info_label = QLabel("截获: 0 | 辐射源: 0")
        info_layout.addWidget(self.title_label)
        info_layout.addStretch()
        info_layout.addWidget(self.info_label)
        layout.addLayout(info_layout)

        # 分割器
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # 左侧：图形区
        graphics_widget = pg.GraphicsLayoutWidget()

        # 方位-时间图
        self.az_time_plot = graphics_widget.addPlot(row=0, col=0)
        self.az_time_plot.setLabel('bottom', '时间 (s)')
        self.az_time_plot.setLabel('left', '方位角 (°)')
        self.az_time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.az_time_curve = self.az_time_plot.plot(
            [], [], pen=pg.mkPen(color=(255, 200, 50), width=2),
            symbol='o', symbolSize=6, symbolBrush=(255, 200, 50)
        )
        self._az_times = []
        self._az_values = []

        # 频率-方位散点图
        self.freq_az_plot = graphics_widget.addPlot(row=1, col=0)
        self.freq_az_plot.setLabel('bottom', '方位角 (°)')
        self.freq_az_plot.setLabel('left', '频率 (MHz)')
        self.freq_az_plot.showGrid(x=True, y=True, alpha=0.3)

        splitter.addWidget(graphics_widget)

        # 右侧：辐射源参数表
        self.param_table = QTableWidget()
        self.param_table.setColumnCount(3)
        self.param_table.setHorizontalHeaderLabels(['辐射源', '参数', '值'])
        self.param_table.setColumnWidth(0, 80)
        self.param_table.setColumnWidth(1, 80)
        self.param_table.setColumnWidth(2, 120)
        self.param_table.setMaximumWidth(320)
        splitter.addWidget(self.param_table)

        splitter.setSizes([600, 280])
        layout.addWidget(splitter, 1)

        # 刷新定时器
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_display)
        self._timer.start(100)  # ESM 数据率较低，100ms 刷新

    def add_detection(self, detection: dict):
        """添加检测数据。

        Args:
            detection: {
                'timestamp': float,
                'azimuth': float (deg),
                'frequency': float (Hz),
                'pri': float (s),
                'pw': float (s),
                'modulation': str,
                'track_id': int,
                'confidence': float,
            }
        """
        self._data_buffer.append(detection)
        if len(self._data_buffer) > 500:
            self._data_buffer = self._data_buffer[-200:]

    def _refresh_display(self):
        if not self._data_buffer:
            return

        # 更新方位-时间图
        self._az_times = [d['timestamp'] for d in self._data_buffer]
        self._az_values = [d.get('azimuth', 0) for d in self._data_buffer]

        # 只显示最近 60 秒
        if self._az_times:
            t0 = self._az_times[-1]
            mask = [t > t0 - 60 for t in self._az_times]
            rel_times = [t - t0 for t in self._az_times if t > t0 - 60]
            rel_az = [a for i, a in enumerate(self._az_values) if mask[i]]
            self.az_time_curve.setData(rel_times, rel_az)

        # 更新频率-方位散点图
        self.freq_az_plot.clear()
        freqs_mhz = [d.get('frequency', 0) / 1e6 for d in self._data_buffer[-50:]]
        azs = [d.get('azimuth', 0) for d in self._data_buffer[-50:]]
        if freqs_mhz:
            scatter = pg.ScatterPlotItem(
                x=azs, y=freqs_mhz,
                brush=pg.mkBrush(255, 200, 50, 150),
                size=10,
            )
            self.freq_az_plot.addItem(scatter)

        # 更新辐射源参数表
        self._update_param_table()

        # 更新信息标签
        self.info_label.setText(
            f"截获: {len(self._data_buffer)} | 辐射源: {self.param_table.rowCount()}"
        )

    def _update_param_table(self):
        """更新辐射源参数表（去重后显示唯一的辐射源）。"""
        unique_emitters: dict[int, dict] = {}
        for d in self._data_buffer:
            tid = d.get('track_id', -1)
            if tid >= 0 and tid not in unique_emitters:
                unique_emitters[tid] = d

        self.param_table.setRowCount(0)
        for row, (tid, d) in enumerate(unique_emitters.items()):
            self.param_table.insertRow(row)
            self.param_table.setItem(row, 0, QTableWidgetItem(f"Emitter-{tid}"))

            # 频率
            freq = d.get('frequency', 0)
            freq_str = f"{freq/1e9:.3f} GHz" if freq > 1e9 else f"{freq/1e6:.1f} MHz"
            self.param_table.setItem(row, 1, QTableWidgetItem("频率"))
            self.param_table.setItem(row, 2, QTableWidgetItem(freq_str))

            row += 1
            self.param_table.insertRow(row)
            pri = d.get('pri', 0) * 1e6  # 转换为 μs
            self.param_table.setItem(row, 0, QTableWidgetItem(""))
            self.param_table.setItem(row, 1, QTableWidgetItem("PRI"))
            self.param_table.setItem(row, 2, QTableWidgetItem(f"{pri:.1f} μs"))

            row += 1
            self.param_table.insertRow(row)
            self.param_table.setItem(row, 0, QTableWidgetItem(""))
            self.param_table.setItem(row, 1, QTableWidgetItem("调制"))
            self.param_table.setItem(row, 2, QTableWidgetItem(d.get('modulation', '?')))

    def clear(self):
        self._data_buffer.clear()
        self.az_time_curve.setData([], [])
        self.freq_az_plot.clear()
        self.param_table.setRowCount(0)
