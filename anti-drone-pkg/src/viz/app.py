"""主可视化应用。

管理多窗口布局、仿真时钟、数据订阅和显示刷新。
"""

from __future__ import annotations

import sys
import time
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QSplitter, QTabWidget, QDockWidget, QMenuBar, QMenu, QStatusBar
)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QAction, QKeySequence

from .widgets.time_controls import TimeControls
from .windows.radar_view import RadarView
from .windows.eo_view import EOView
from .windows.esm_view import ESMView
from .windows.acoustic_view import AcousticView
from .windows.truth_view import TruthView


class MainWindow(QMainWindow):
    """主可视化窗口。

    布局：
    - 顶部：时间控制条
    - 中央：Tab 页 + 综合态势
    - 底部：状态栏
    """

    # 信号
    sim_time_change = pyqtSignal(float)
    sim_play = pyqtSignal()
    sim_pause = pyqtSignal()
    sim_stop = pyqtSignal()
    sim_speed_change = pyqtSignal(float)

    def __init__(self, duration: float = 300.0):
        super().__init__()
        self.setWindowTitle("反无人机数据生成 — 多传感器可视化")
        self.resize(1600, 1000)

        # 默认传感器 ID（可在运行时配置）
        self.radar_id = "radar-01"
        self.eo_id = "eo-01"
        self.esm_id = "esm-01"

        self._init_menu()
        self._init_ui(duration)
        self._init_status_bar()

    def _init_menu(self):
        menubar = self.menuBar()

        # 文件菜单
        file_menu = menubar.addMenu("文件")
        load_action = QAction("加载场景...", self)
        load_action.setShortcut(QKeySequence("Ctrl+O"))
        file_menu.addAction(load_action)

        export_action = QAction("导出数据...", self)
        export_action.setShortcut(QKeySequence("Ctrl+E"))
        file_menu.addAction(export_action)

        file_menu.addSeparator()
        quit_action = QAction("退出", self)
        quit_action.setShortcut(QKeySequence("Ctrl+Q"))
        quit_action.triggered.connect(self.close)
        file_menu.addAction(quit_action)

        # 视图菜单
        view_menu = menubar.addMenu("视图")
        self.reset_layout_action = QAction("重置布局", self)
        view_menu.addAction(self.reset_layout_action)
        self.fullscreen_action = QAction("全屏", self)
        self.fullscreen_action.setShortcut(QKeySequence("F11"))
        self.fullscreen_action.triggered.connect(
            lambda: self.showFullScreen() if not self.isFullScreen() else self.showNormal()
        )
        view_menu.addAction(self.fullscreen_action)

    def _init_ui(self, duration: float):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)

        # 时间控件
        self.time_controls = TimeControls(duration)
        main_layout.addWidget(self.time_controls)

        # 主内容区：水平分割
        main_splitter = QSplitter(Qt.Orientation.Horizontal)

        # 左侧：传感器视图 Tab
        self.sensor_tabs = QTabWidget()
        self.radar_view = RadarView(self.radar_id)
        self.eo_view = EOView(self.eo_id)
        self.esm_view = ESMView(self.esm_id)
        self.acoustic_view = AcousticView("acoustic-01")
        self.sensor_tabs.addTab(self.radar_view, "🔴 雷达")
        self.sensor_tabs.addTab(self.eo_view, "🔵 光电")
        self.sensor_tabs.addTab(self.esm_view, "🟡 电侦")
        self.sensor_tabs.addTab(self.acoustic_view, "🟢 声探测")
        main_splitter.addWidget(self.sensor_tabs)

        # 右侧：综合态势
        self.truth_view = TruthView()
        main_splitter.addWidget(self.truth_view)

        main_splitter.setSizes([800, 700])
        main_layout.addWidget(main_splitter, 1)

    def _init_status_bar(self):
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("就绪 — 请加载场景并开始仿真")

    def get_views(self):
        """返回所有视图的字典。"""
        return {
            'radar': self.radar_view,
            'eo': self.eo_view,
            'esm': self.esm_view,
            'acoustic': self.acoustic_view,
            'truth': self.truth_view,
        }

    def set_sim_time(self, t: float):
        """更新仿真时间显示。"""
        self.time_controls.set_time(t)

    def set_status(self, text: str):
        """更新状态栏。"""
        self.status_bar.showMessage(text)


def run_viz(duration: float = 300.0) -> MainWindow:
    """启动可视化应用。

    Args:
        duration: 仿真总时长

    Returns:
        MainWindow: 主窗口实例
    """
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow(duration)
    window.show()
    return window
