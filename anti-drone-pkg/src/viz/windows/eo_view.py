"""光电传感器视场视图 — 动态FOV。

显示内容：
- 虚线矩形：光电窄视场（跟随转塔指向动态移动）
- 十字：目标检测位置
- 圆点：雷达引导方向（cue）
- 轨迹点：目标历史路径
- 状态标签：idle/slewing/acquiring/locked/tracking
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt6.QtCore import QTimer, Qt


class EOView(QWidget):
    """光电传感器动态视场视图。"""

    def __init__(self, sensor_id: str = "eo-01", parent=None):
        super().__init__(parent)
        self.sensor_id = sensor_id
        self._data_buffer: list[dict] = []
        self._max_buffer = 500

        # 传感器状态（从控制器同步）
        self._eo_state: dict = {
            'state': 'idle', 'current_az': 90.0, 'current_el': 5.0,
            'cue_az': None, 'cue_el': None, 'n_tracked': 0, 'n_locked': 0,
        }

        # 跟踪轨迹历史
        self._track_history: list[tuple[float, float]] = []

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(2, 2, 2, 2)

        # 信息栏
        info_layout = QHBoxLayout()
        self.title_label = QLabel(f"<b>光电跟踪 — {self.sensor_id}</b>")
        self.state_label = QLabel("")
        self.info_label = QLabel("")
        info_layout.addWidget(self.title_label)
        info_layout.addStretch()
        info_layout.addWidget(self.state_label)
        info_layout.addWidget(self.info_label)
        layout.addLayout(info_layout)

        # 绘图区
        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_item = self.plot_widget.addPlot()
        self.plot_item.setLabel('bottom', '方位角 (°)')
        self.plot_item.setLabel('left', '俯仰角 (°)')
        self.plot_item.showGrid(x=True, y=True, alpha=0.3)
        layout.addWidget(self.plot_widget, 1)

        # 动态元素（统一管理，刷新时先清后画）
        self._dynamic_items: list = []

        # 刷新定时器 (20fps)
        self._timer = QTimer()
        self._timer.timeout.connect(self._refresh_display)
        self._timer.start(50)

    def set_state(self, state: dict):
        """同步传感器状态（从控制器调用）。

        Args:
            state: {
                'state': 'idle'|'slewing'|'acquiring'|'locked'|'tracking',
                'current_az': float, 'current_el': float,
                'cue_az': float|None, 'cue_el': float|None,
                'n_tracked': int, 'n_locked': int,
            }
        """
        self._eo_state = state

    def add_detection(self, detection: dict):
        """添加检测数据。"""
        self._data_buffer.append(detection)
        if len(self._data_buffer) > self._max_buffer:
            self._data_buffer = self._data_buffer[-self._max_buffer:]

    def _refresh_display(self):
        # ── 清除上一帧的动态元素 ──
        for item in self._dynamic_items:
            self.plot_item.removeItem(item)
        self._dynamic_items.clear()

        state = self._eo_state
        current_az = state.get('current_az', 0)
        current_el = state.get('current_el', 5)
        cue_az = state.get('cue_az')
        cue_el = state.get('cue_el')
        eo_state = state.get('state', 'idle')

        # ── FOV 框居中于当前指向 ──
        fov_az_half, fov_el_half = 4.0, 3.0
        az_min, az_max = current_az - fov_az_half, current_az + fov_az_half
        el_min, el_max = current_el - fov_el_half, current_el + fov_el_half

        if eo_state in ('locked', 'tracking'):
            fov_color = (0, 220, 0, 180)
        elif eo_state == 'slewing':
            fov_color = (255, 200, 50, 150)
        elif eo_state == 'acquiring':
            fov_color = (255, 150, 50, 160)
        else:
            fov_color = (100, 180, 255, 120)

        x = [az_min, az_max, az_max, az_min, az_min]
        y = [el_min, el_min, el_max, el_max, el_min]
        self._dynamic_items.append(
            self.plot_item.plot(x, y, pen=pg.mkPen(color=fov_color, width=2, style=Qt.PenStyle.DashLine))
        )

        # ── 十字线标记当前指向中心 ──
        self._dynamic_items.append(
            self.plot_item.plot(
                [current_az - 1.5, current_az + 1.5], [current_el, current_el],
                pen=pg.mkPen(color=fov_color, width=1, style=Qt.PenStyle.DotLine))
        )
        self._dynamic_items.append(
            self.plot_item.plot(
                [current_az, current_az], [current_el - 1.0, current_el + 1.0],
                pen=pg.mkPen(color=fov_color, width=1, style=Qt.PenStyle.DotLine))
        )

        # ── 雷达 cue 方向（红色圆点） ──
        if cue_az is not None:
            self._dynamic_items.append(
                self.plot_item.plot(
                    [cue_az], [cue_el or current_el],
                    pen=None, symbol='o', symbolBrush=(255, 80, 80, 200),
                    symbolSize=11, symbolPen=pg.mkPen(width=2, color=(255, 80, 80)),
                )
            )
            # cue 和当前指向之间的虚线
            self._dynamic_items.append(
                self.plot_item.plot(
                    [current_az, cue_az], [current_el, cue_el or current_el],
                    pen=pg.mkPen(color=(255, 150, 100, 80), width=1, style=Qt.PenStyle.DashLine)
                )
            )

        # ── 转塔转动箭头（slewing 时） ──
        if eo_state == 'slewing' and cue_az is not None:
            dx = cue_az - current_az
            dy = (cue_el or current_el) - current_el
            arr_len = np.sqrt(dx**2 + dy**2)
            if arr_len > 0.1:
                arrow = pg.ArrowItem(
                    angle=0, headLen=0.8, tipAngle=30, tailLen=1.2,
                    brush=(255, 200, 50, 150),
                    pen=pg.mkPen(color=(255, 200, 50), width=1.5),
                )
                arrow.setPos(current_az, current_el)
                arrow.setRotation(np.degrees(np.arctan2(dy, dx)))
                self.plot_item.addItem(arrow)
                self._dynamic_items.append(arrow)

        # ── 目标检测（最新一条） ──
        recent = self._data_buffer[-10:]
        if recent:
            latest = recent[-1]
            t_az = latest.get('azimuth', 0)
            t_el = latest.get('elevation', 0)
            locked = latest.get('locked', True)
            color = (0, 255, 0, 220) if locked else (255, 220, 50, 180)
            self._dynamic_items.append(
                self.plot_item.plot(
                    [t_az], [t_el],
                    pen=None, symbol='+', symbolBrush=pg.mkBrush(color=color),
                    symbolSize=16, symbolPen=pg.mkPen(width=2, color=color),
                )
            )
            # 不确定椭圆
            err_az = latest.get('error_az', 0.005)
            err_el = latest.get('error_el', 0.005)
            from PyQt6.QtWidgets import QGraphicsEllipseItem
            from PyQt6.QtCore import QRectF
            rect = QGraphicsEllipseItem(QRectF(
                t_az - 3 * err_az, t_el - 3 * err_el,
                6 * err_az, 6 * err_el,
            ))
            rect.setPen(pg.mkPen(color=color, width=1))
            self.plot_item.addItem(rect)
            self._dynamic_items.append(rect)

            # 更新轨迹历史
            self._track_history.append((t_az, t_el))
            if len(self._track_history) > 200:
                self._track_history = self._track_history[-200:]

        # ── 轨迹历史线 ──
        if len(self._track_history) > 1:
            hx = [p[0] for p in self._track_history]
            hy = [p[1] for p in self._track_history]
            self._dynamic_items.append(
                self.plot_item.plot(hx, hy, pen=pg.mkPen(color=(255, 255, 255, 80), width=1))
            )

        # ── 视图范围跟随 FOV ──
        margin = 3.0
        self.plot_item.setXRange(az_min - margin, az_max + margin)
        self.plot_item.setYRange(el_min - margin - 2, el_max + margin + 2)

        # ── 状态文字 ──
        state_names = {
            'idle': '待命', 'slewing': '转动中...', 'acquiring': '搜索中...',
            'locked': '已锁定', 'tracking': '跟踪中',
        }
        state_text = state_names.get(eo_state, eo_state)
        if eo_state in ('locked', 'tracking'):
            self.state_label.setText(f"<b style='color:green'>{state_text}</b>")
        elif eo_state == 'slewing':
            self.state_label.setText(f"<b style='color:#cc8800'>{state_text}</b>")
        elif eo_state == 'acquiring':
            self.state_label.setText(f"<b style='color:#ff6600'>{state_text}</b>")
        else:
            self.state_label.setText(f"<span style='color:#888'>{state_text}</span>")

        self.info_label.setText(
            f"指向: Az{current_az:.1f}° El{current_el:.1f}° | "
            f"锁定: {state.get('n_locked', 0)} | "
            f"跟踪: {state.get('n_tracked', 0)}"
        )

    def clear(self):
        self._data_buffer.clear()
        self._track_history.clear()
