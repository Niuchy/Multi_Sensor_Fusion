"""2D 俯视渲染器。

提供直角坐标和极坐标下的轨迹、检测点、误差椭圆绘制。
"""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt
from typing import Optional


# 颜色定义
COLORS = {
    'truth': (0, 255, 0, 200),          # 绿色 — 真值
    'radar': (255, 100, 100, 200),       # 红色 — 雷达
    'eo': (100, 200, 255, 200),          # 蓝色 — 光电
    'esm': (255, 200, 50, 200),          # 橙色 — 电侦
    'acoustic': (100, 255, 180, 200),    # 青色 — 声探测
    'clutter': (150, 150, 150, 100),     # 灰色 — 杂波
    'false_track': (255, 100, 255, 150), # 紫色 — 虚假航迹
    'grid': (80, 80, 80, 80),
}


class TopDownRenderer:
    """2D 俯视图渲染器。

    在 pyqtgraph PlotItem 上绘制目标轨迹、检测点、误差椭圆等。
    """

    def __init__(self, plot_item: pg.PlotItem):
        """
        Args:
            plot_item: pyqtgraph PlotItem 实例
        """
        self.plot = plot_item
        self.plot.setAspectLocked(True)
        self.plot.setLabel('left', 'Y (m)')
        self.plot.setLabel('bottom', 'X (m)')
        self.plot.showGrid(x=True, y=True, alpha=0.3)

        # 存储各曲线对象
        self._curves: dict[str, dict] = {}
        self._scatter_plots: dict[str, pg.ScatterPlotItem] = {}
        self._ellipse_items: list = []

    def init_target(self, target_id: int, label: str, color: tuple = COLORS['truth']):
        """初始化目标真值轨迹曲线。

        Args:
            target_id: 目标 ID
            label: 图例标签
            color: 颜色
        """
        key = f"truth_{target_id}"
        pen = pg.mkPen(color=color, width=2)
        curve = self.plot.plot([], [], pen=pen, name=f"{label} (真值)")
        self._curves[key] = {'curve': curve, 'x': [], 'y': []}

    def init_sensor_track(self, sensor_id: str, track_id: int, color: tuple):
        """初始化传感器航迹曲线。

        Args:
            sensor_id: 传感器标识
            track_id: 航迹号
            color: 颜色
        """
        key = f"{sensor_id}_{track_id}"
        style_map = {
            'radar': Qt.PenStyle.DashLine,
            'eo': Qt.PenStyle.DotLine,
            'esm': Qt.PenStyle.DashDotLine,
        }
        style = style_map.get(sensor_id, Qt.PenStyle.SolidLine)
        pen = pg.mkPen(color=color, width=1.5, style=style)
        curve = self.plot.plot([], [], pen=pen, name=f"{sensor_id} track {track_id}")
        self._curves[key] = {'curve': curve, 'x': [], 'y': []}

    def update_truth(self, target_id: int, x: float, y: float):
        """追加真值轨迹点。

        Args:
            target_id: 目标 ID
            x, y: 位置
        """
        key = f"truth_{target_id}"
        if key not in self._curves:
            self.init_target(target_id, f"Target-{target_id}")
        entry = self._curves[key]
        entry['x'].append(x)
        entry['y'].append(y)
        # 只保留最近 2000 个点
        if len(entry['x']) > 2000:
            entry['x'] = entry['x'][-2000:]
            entry['y'] = entry['y'][-2000:]
        entry['curve'].setData(entry['x'], entry['y'])

    def update_sensor_track(self, sensor_id: str, track_id: int, x: float, y: float):
        """追加传感器航迹点。

        Args:
            sensor_id: 传感器标识
            track_id: 航迹号
            x, y: 位置
        """
        key = f"{sensor_id}_{track_id}"
        if key not in self._curves:
            color = COLORS.get(sensor_id, (255, 255, 255, 200))
            self.init_sensor_track(sensor_id, track_id, color)
        entry = self._curves[key]
        entry['x'].append(x)
        entry['y'].append(y)
        if len(entry['x']) > 500:
            entry['x'] = entry['x'][-500:]
            entry['y'] = entry['y'][-500:]
        entry['curve'].setData(entry['x'], entry['y'])

    def plot_detections(self, sensor_id: str, positions: np.ndarray,
                        color: tuple | None = None, symbol: str = 'o', size: int = 6):
        """绘制检测散点。

        Args:
            sensor_id: 传感器标识（用于管理散点图层）
            positions: (N, 2) 位置数组
            color: 颜色
            symbol: 符号
            size: 大小
        """
        if color is None:
            color = COLORS.get(sensor_id, (255, 255, 255, 200))

        if sensor_id in self._scatter_plots:
            self.plot.removeItem(self._scatter_plots[sensor_id])

        brush = pg.mkBrush(color=color)
        scatter = pg.ScatterPlotItem(
            x=positions[:, 0], y=positions[:, 1],
            brush=brush, size=size, symbol=symbol,
        )
        self.plot.addItem(scatter)
        self._scatter_plots[sensor_id] = scatter

    def plot_clutter(self, positions: np.ndarray):
        """绘制杂波点。

        Args:
            positions: (N, 2) 杂波位置
        """
        color = COLORS['clutter']
        if 'clutter' in self._scatter_plots:
            self.plot.removeItem(self._scatter_plots['clutter'])

        brush = pg.mkBrush(color=color)
        scatter = pg.ScatterPlotItem(
            x=positions[:, 0], y=positions[:, 1],
            brush=brush, size=3, symbol='x',
        )
        self.plot.addItem(scatter)
        self._scatter_plots['clutter'] = scatter

    def plot_error_ellipse(self, x: float, y: float, cov: np.ndarray,
                           color: tuple = (255, 255, 255, 100)):
        """绘制误差椭圆。

        Args:
            x, y: 中心位置
            cov: 2x2 协方差矩阵（xy 平面）
            color: 颜色
        """
        # 特征值分解得到椭圆参数
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        if eigenvalues[0] <= 0 or eigenvalues[1] <= 0:
            return

        # 1-σ 椭圆（约 39% 置信度），缩放为 3-σ（约 99%）
        scale = 3.0
        width = 2 * scale * np.sqrt(eigenvalues[0])
        height = 2 * scale * np.sqrt(eigenvalues[1])
        angle = np.degrees(np.arctan2(eigenvectors[1, 0], eigenvectors[0, 0]))

        from PyQt6.QtWidgets import QGraphicsEllipseItem
        from PyQt6.QtGui import QPen, QBrush, QColor
        from PyQt6.QtCore import QRectF

        ellipse = QGraphicsEllipseItem(QRectF(
            x - width / 2, y - height / 2, width, height
        ))
        ellipse.setPen(pg.mkPen(color=color, width=1))
        ellipse.setBrush(pg.mkBrush(color=(color[0], color[1], color[2], 30)))
        ellipse.setRotation(angle)
        ellipse.setTransformOriginPoint(x, y)

        # 清除旧椭圆（简化处理）
        for old in self._ellipse_items:
            self.plot.removeItem(old)
        self._ellipse_items = []

        self.plot.addItem(ellipse)
        self._ellipse_items.append(ellipse)

    def clear_sensor(self, sensor_id: str):
        """清除传感器相关的所有图形。"""
        keys_to_remove = [k for k in self._curves if k.startswith(sensor_id)]
        for k in keys_to_remove:
            self.plot.removeItem(self._curves.pop(k)['curve'])
        if sensor_id in self._scatter_plots:
            self.plot.removeItem(self._scatter_plots.pop(sensor_id))

    def clear_all(self):
        """清除所有图形。"""
        for entry in self._curves.values():
            self.plot.removeItem(entry['curve'])
        self._curves.clear()
        for scatter in self._scatter_plots.values():
            self.plot.removeItem(scatter)
        self._scatter_plots.clear()
        for ellipse in self._ellipse_items:
            self.plot.removeItem(ellipse)
        self._ellipse_items.clear()

    def set_sensor_position(self, sensor_id: str, x: float, y: float, label: str = ""):
        """标记传感器位置。"""
        color = COLORS.get(sensor_id, (255, 255, 255, 200))
        key = f"sensor_pos_{sensor_id}"
        if key in self._scatter_plots:
            self.plot.removeItem(self._scatter_plots[key])
        scatter = pg.ScatterPlotItem(
            x=[x], y=[y],
            brush=pg.mkBrush(color=color),
            pen=pg.mkPen(color=(255, 255, 255), width=2),
            size=15, symbol='s',
        )
        self.plot.addItem(scatter)
        self._scatter_plots[key] = scatter

        # 添加文字标签
        text = pg.TextItem(label, anchor=(0, 1), color=color)
        text.setPos(x, y)
        self.plot.addItem(text)
