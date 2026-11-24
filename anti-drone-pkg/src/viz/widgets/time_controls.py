"""时间控制组件。"""

from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QPushButton, QSlider, QLabel, QComboBox
)
from PyQt6.QtCore import Qt, pyqtSignal


class TimeControls(QWidget):
    """仿真播放控制条。

    信号:
        play: 播放
        pause: 暂停
        stop: 停止
        time_changed(float): 时间被拖动
        speed_changed(float): 速度倍率改变
    """

    play_signal = pyqtSignal()
    pause_signal = pyqtSignal()
    stop_signal = pyqtSignal()
    time_changed = pyqtSignal(float)
    speed_changed = pyqtSignal(float)

    def __init__(self, total_duration: float = 300.0, parent=None):
        super().__init__(parent)
        self.total_duration = total_duration
        self._playing = False
        self._init_ui()

    def _init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 2, 5, 2)

        # 播放按钮
        self.btn_play = QPushButton("▶")
        self.btn_play.setFixedWidth(35)
        self.btn_play.clicked.connect(self._on_play)
        layout.addWidget(self.btn_play)

        # 暂停按钮
        self.btn_pause = QPushButton("⏸")
        self.btn_pause.setFixedWidth(35)
        self.btn_pause.clicked.connect(self._on_pause)
        layout.addWidget(self.btn_pause)

        # 停止按钮
        self.btn_stop = QPushButton("⏹")
        self.btn_stop.setFixedWidth(35)
        self.btn_stop.clicked.connect(self._on_stop)
        layout.addWidget(self.btn_stop)

        # 时间滑块
        self.time_slider = QSlider(Qt.Orientation.Horizontal)
        self.time_slider.setRange(0, int(self.total_duration * 1000))  # 毫秒精度
        self.time_slider.setValue(0)
        self.time_slider.sliderMoved.connect(self._on_slider_moved)
        layout.addWidget(self.time_slider, 1)

        # 时间标签
        self.time_label = QLabel("00:00.000 / 00:00.000")
        self.time_label.setFixedWidth(200)
        layout.addWidget(self.time_label)

        # 速度选择
        self.speed_combo = QComboBox()
        self.speed_combo.addItems(["0.25x", "0.5x", "1x", "2x", "5x", "10x"])
        self.speed_combo.setCurrentIndex(2)
        self.speed_combo.currentTextChanged.connect(self._on_speed_changed)
        layout.addWidget(self.speed_combo)

    def _on_play(self):
        self._playing = True
        self.play_signal.emit()

    def _on_pause(self):
        self._playing = False
        self.pause_signal.emit()

    def _on_stop(self):
        self._playing = False
        self.time_slider.setValue(0)
        self.stop_signal.emit()

    def _on_slider_moved(self, value: int):
        t = value / 1000.0
        self._update_time_label(t)
        self.time_changed.emit(t)

    def _on_speed_changed(self, text: str):
        speed = float(text.replace('x', ''))
        self.speed_changed.emit(speed)

    def _update_time_label(self, current: float):
        def fmt(t):
            minutes = int(t // 60)
            seconds = int(t % 60)
            ms = int((t % 1) * 1000)
            return f"{minutes:02d}:{seconds:02d}.{ms:03d}"

        self.time_label.setText(
            f"{fmt(current)} / {fmt(self.total_duration)}"
        )

    def set_time(self, t: float):
        """更新滑块位置和时间标签（程序化设置）。"""
        self.time_slider.blockSignals(True)
        self.time_slider.setValue(int(t * 1000))
        self.time_slider.blockSignals(False)
        self._update_time_label(t)

    def set_duration(self, duration: float):
        """设置总时长。"""
        self.total_duration = duration
        self.time_slider.setRange(0, int(duration * 1000))

    @property
    def speed(self) -> float:
        text = self.speed_combo.currentText()
        return float(text.replace('x', ''))

    @property
    def is_playing(self) -> bool:
        return self._playing
