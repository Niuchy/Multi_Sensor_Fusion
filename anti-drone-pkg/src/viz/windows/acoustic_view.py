"""声探测传感器视图。

显示：
- 方位-频率瀑布图
- 声压级-时间图
- 粗略距离估计
- 目标分类
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt6.QtCore import QTimer, Qt


class AcousticView(QWidget):
    """声探测传感器视图。"""

    def __init__(self, sensor_id: str = "acoustic-01", parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self._data_buffer: list[dict] = []

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        info_layout = QHBoxLayout()
        self.title_label = QLabel(f"<b>声探测 — {self.sensor_id}</b>")
        self.info_label = QLabel("探测: 0 | 最近分类: -- | 距离估计: --m")
        info_layout.addWidget(self.title_label)
        info_layout.addStretch()
        info_layout.addWidget(self.info_label)
        layout.addLayout(info_layout)

        self.plot_widget = pg.GraphicsLayoutWidget()

        # 上：方位-时间图
        self.az_time_plot = self.plot_widget.addPlot(row=0, col=0)
        self.az_time_plot.setLabel('bottom', '时间 (s)')
        self.az_time_plot.setLabel('left', '方位角 (°)')
        self.az_time_plot.showGrid(x=True, y=True, alpha=0.3)
        self.az_time_plot.setXRange(-60, 0)
        self.az_time_plot.setYRange(-180, 180)
        self.az_time_curve = None
        self._az_times: list[float] = []
        self._az_values: list[float] = []
        self._no_data_text = pg.TextItem('等待声探测数据...', color=(150, 150, 150))
        self._no_data_text.setPos(-30, 0)
        self.az_time_plot.addItem(self._no_data_text)

        # 下：声压级-距离估计图
        self.spl_plot = self.plot_widget.addPlot(row=1, col=0)
        self.spl_plot.setLabel('bottom', '距离估计 (m)')
        self.spl_plot.setLabel('left', '声压级 (dB)')
        self.spl_plot.showGrid(x=True, y=True, alpha=0.3)
        self.spl_plot.setXRange(0, 1000)
        self.spl_plot.setYRange(0, 80)

        layout.addWidget(self.plot_widget, 1)

        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_display)
        self._timer.start(200)  # 5Hz sensor, 200ms refresh

    def add_detection(self, detection: dict):
        self._data_buffer.append(detection)
        if len(self._data_buffer) > 300:
            self._data_buffer = self._data_buffer[-200:]

    def _refresh_display(self):
        if not self._data_buffer:
            return

        # 首次收到数据，移除"等待数据"提示
        if hasattr(self, '_no_data_text') and self._no_data_text:
            self.az_time_plot.removeItem(self._no_data_text)
            self._no_data_text = None

        # 方位-时间
        t0 = self._data_buffer[-1].get('timestamp', 0)
        recent = [d for d in self._data_buffer if d.get('timestamp', 0) > t0 - 60]

        self._az_times = [d['timestamp'] - t0 for d in recent]
        self._az_values = [d.get('azimuth', 0) for d in recent]

        if self.az_time_curve:
            self.az_time_plot.removeItem(self.az_time_curve)
        if self._az_times:
            self.az_time_curve = self.az_time_plot.plot(
                self._az_times, self._az_values,
                pen=pg.mkPen(color=(100, 255, 180, 200), width=2),
                symbol='o', symbolSize=6, symbolBrush=(100, 255, 180),
            )

        # 声压级-距离
        self.spl_plot.clear()
        spl_values = [d.get('sound_level', 0) for d in recent]
        range_est = [d.get('range_estimate', 0) for d in recent]
        types_set = set()
        for d in recent:
            cls = d.get('classification', '')
            if cls:
                types_set.add(cls)

        if spl_values:
            colors = {t: pg.mkBrush(
                (100, 255, 180, 150) if i == 0 else (255, 200, 100, 150)
            ) for i, t in enumerate(types_set)}
            for d in recent:
                cls = d.get('classification', '')
                c = colors.get(cls, pg.mkBrush((150, 150, 150, 100)))
                scatter = pg.ScatterPlotItem(
                    x=[d.get('range_estimate', 0)], y=[d.get('sound_level', 0)],
                    brush=c, size=10,
                )
                self.spl_plot.addItem(scatter)

        # 信息
        latest = self._data_buffer[-1] if self._data_buffer else {}
        self.info_label.setText(
            f"探测: {len(recent)} | "
            f"分类: {latest.get('classification', '-')} | "
            f"距离估计: {latest.get('range_estimate', 0):.0f}m | "
            f"SPL: {latest.get('sound_level', 0):.0f}dB"
        )

    def clear(self):
        self._data_buffer.clear()
