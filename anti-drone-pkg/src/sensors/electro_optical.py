"""光电传感器模型。

雷达引导光电协同工作流程：
1. 雷达粗测目标方位 → 发送 cue 给光电
2. 光电转塔转动到 cue 方向（slew）
3. 光电在窄视场内搜索目标（acquire）
4. 捕获后锁定跟踪（locked→track）

特点：
- 仅角度测量（方位 + 俯仰），高精度（角秒级）
- 窄视场（±4°×±3°）
- 支持雷达引导和自主搜索两种模式
- 受天气/能见度影响
"""

from __future__ import annotations

import numpy as np
from enum import Enum

from sensors.base import SensorBase
from sensors.error_models import (
    GaussianNoise, Bias, OutlierModel, ErrorPipeline,
)
from truth.target import Target
from bus.message import Detection, DetectionFlag


class EOState(Enum):
    """光电传感器工作状态。"""
    IDLE = "idle"               # 待命，等待雷达 cue
    SLEWING = "slewing"         # 转塔转动中，指向 cue 方向
    ACQUIRING = "acquiring"     # 已指向，窄视场内搜索
    LOCKED = "locked"           # 已锁定，稳定跟踪输出
    TRACKING = "tracking"       # 持续跟踪中
    LOST = "lost"               # 目标丢失，返回 IDLE


class ElectroOpticalSensor(SensorBase):
    """光电传感器模拟器 — 支持雷达引导。

    测量量:
    - azimuth: 方位角 (deg)
    - elevation: 俯仰角 (deg)

    工作模式：
    - 自主模式：目标进入 FOV 后自动锁定
    - 引导模式：接收雷达 cue → 转塔转动 → 捕获 → 跟踪
    """

    def __init__(
        self,
        sensor_id: str = "eo-01",
        position: list[float] | None = None,
        orientation: list[float] | None = None,
        update_rate: float = 30.0,
        fov: dict | None = None,
        max_track_range: float = 8000.0,
        error_sigma: dict | None = None,
        pd: float = 0.95,
        lock_on_time: float = 0.5,
        max_track_targets: int = 3,
        handover_delay: float = 1.0,
        visibility_km: float = 10.0,
        # ── 转塔/引导参数 ──
        slew_rate_az: float = 60.0,       # 方位转动速率 (°/s)
        slew_rate_el: float = 30.0,       # 俯仰转动速率 (°/s)
        acquisition_time: float = 0.3,    # 指向后搜索捕获时间 (s)
        cue_timeout: float = 10.0,        # cue 超时时间 (s)，超时后放弃
    ):
        position = position or [100.0, 0.0, 10.0]
        orientation = orientation or [0.0, 5.0]
        fov = fov or {'azimuth': [-4.0, 4.0], 'elevation': [-3.0, 3.0]}

        super().__init__(sensor_id, 'eo', position, orientation, update_rate, fov)

        self.max_track_range = max_track_range
        self.error_sigma = error_sigma or {'azimuth': 0.005, 'elevation': 0.005}
        self.pd_nominal = pd
        self.lock_on_time = lock_on_time
        self.max_track_targets = max_track_targets
        self.handover_delay = handover_delay
        self.visibility_km = visibility_km

        # 转塔参数
        self.slew_rate_az = slew_rate_az
        self.slew_rate_el = slew_rate_el
        self.acquisition_time = acquisition_time
        self.cue_timeout = cue_timeout

        # ── 状态机 ──
        self._state: EOState = EOState.IDLE
        self._current_az = float(orientation[0])  # 当前指向方位角 (°)
        self._current_el = float(orientation[1])  # 当前指向俯仰角 (°)

        # 引导信息
        self._cue: dict | None = None  # {'az','el','target_id','time','confidence'}
        self._slew_start_time: float = 0.0
        self._acquire_start_time: float = 0.0

        # 跟踪状态
        self._tracked_targets: dict[int, dict] = {}
        self._targets: list[Target] = []

    def set_targets(self, targets: list[Target]):
        self._targets = targets

    def _setup_error_pipeline(self):
        self._error_pipeline.add_stage(GaussianNoise(self.error_sigma))

    # ── 雷达引导接口 ──

    def receive_cue(self, az_deg: float, el_deg: float, target_id: int = -1,
                    confidence: float = 0.5):
        """接收雷达引导指令。

        雷达粗测到目标后，发送 cue 给光电。
        光电转塔开始向该方向转动。

        Args:
            az_deg: 雷达测得的方位角 (°)，精度约 0.3°
            el_deg: 雷达测得的俯仰角 (°)，精度约 0.85°
            target_id: 目标 ID（可选）
            confidence: 引导置信度
        """
        # 只接受正在跟踪的目标数 < 最大容量
        n_locked = sum(1 for v in self._tracked_targets.values() if v.get('locked'))
        if n_locked >= self.max_track_targets:
            return  # 已满，忽略新 cue

        self._cue = {
            'az': az_deg,
            'el': el_deg,
            'target_id': target_id,
            'time': 0.0,  # 由 detect 填充
            'confidence': confidence,
        }
        self._state = EOState.SLEWING
        self._slew_start_time = 0.0  # 由 detect 填充

    def get_state(self) -> dict:
        """获取当前状态（用于可视化）。"""
        return {
            'state': self._state.value,
            'current_az': self._current_az,
            'current_el': self._current_el,
            'cue_az': self._cue['az'] if self._cue else None,
            'cue_el': self._cue['el'] if self._cue else None,
            'n_tracked': len(self._tracked_targets),
            'n_locked': sum(1 for v in self._tracked_targets.values() if v.get('locked')),
        }

    # ── 检测逻辑 ──

    def detect(self, sim_time: float) -> list[Detection]:
        """执行光电检测。

        流程：
        1. 状态机推进（slew→acquire→lock→track）
        2. 根据当前状态和指向，搜索目标
        3. 自主模式：目标进入窄 FOV 直接锁定
        4. 输出高精度角度测量
        """
        detections = []

        # ── 状态机推进 ──
        self._update_state(sim_time)

        # ── 搜索/跟踪目标 ──
        # 引导模式：优先处理 cued 目标
        cued_target_id = self._cue.get('target_id', -1) if self._cue else -1

        for target in self._targets:
            state = target.get_state(sim_time)
            if state is None:
                continue

            rel_pos = state.position - self.position
            r = float(np.linalg.norm(rel_pos))

            if r > self.max_track_range:
                if target.id in self._tracked_targets:
                    del self._tracked_targets[target.id]
                continue

            az_true = float(np.arctan2(rel_pos[1], rel_pos[0]))
            el_true = float(np.arctan2(rel_pos[2], np.linalg.norm(rel_pos[:2])))
            az_deg = np.degrees(az_true)
            el_deg = np.degrees(el_true)

            # ── 判断目标是否在 EO 当前视场内 ──
            rel_az = (az_deg - self._current_az + 180) % 360 - 180
            rel_el = el_deg - self._current_el
            in_fov = (
                self.fov['azimuth'][0] <= rel_az <= self.fov['azimuth'][1] and
                self.fov['elevation'][0] <= rel_el <= self.fov['elevation'][1]
            )

            # ── 引导模式：在 slewing/acquiring 阶段只关注 cued 目标 ──
            if self._state in (EOState.SLEWING, EOState.ACQUIRING):
                if cued_target_id >= 0 and target.id != cued_target_id:
                    continue  # 忽略非 cued 目标

            # ── 跟踪管理 ──
            tracked = self._tracked_targets.get(target.id)

            if tracked is None:
                # 新目标：必须在视场内
                if not in_fov:
                    continue

                if len(self._tracked_targets) >= self.max_track_targets:
                    continue

                self._tracked_targets[target.id] = {
                    'lock_start': sim_time,
                    'locked': False,
                }
                tracked = self._tracked_targets[target.id]

            # 锁定计时
            if not tracked.get('locked'):
                if sim_time - tracked['lock_start'] >= self.lock_on_time:
                    tracked['locked'] = True
                    if self._state in (EOState.SLEWING, EOState.ACQUIRING):
                        self._state = EOState.LOCKED
                else:
                    continue  # 锁定中，无输出

            # 移出视场 → 丢失
            if not in_fov:
                del self._tracked_targets[target.id]
                if not self._tracked_targets:
                    self._state = EOState.IDLE
                continue

            # ── 检测 ──
            weather_factor = min(1.0, self.visibility_km / 10.0)
            pd = self.pd_nominal * weather_factor

            if np.random.random() > pd:
                continue

            raw_meas = {
                'azimuth': az_deg,
                'elevation': el_deg,
            }
            meas = self._error_pipeline.process(raw_meas)

            det = self.make_detection(
                timestamp=sim_time,
                measurements=meas,
                covariance={
                    'azimuth': self.error_sigma['azimuth'] ** 2,
                    'elevation': self.error_sigma['elevation'] ** 2,
                },
                confidence=pd,
                flags={DetectionFlag.NORMAL},
            )
            det.track_id = target.id
            det.truth_position = state.position.copy()
            detections.append(det)

            # 更新指向（跟踪模式：实时跟随目标）
            if self._state in (EOState.LOCKED, EOState.TRACKING):
                self._current_az = az_deg
                self._current_el = el_deg

            tracked['last_pos'] = state.position.copy()

        return detections

    def _update_state(self, sim_time: float):
        """状态机推进。"""
        if self._state == EOState.IDLE:
            # 待命，不做任何事（等 cue）
            pass

        elif self._state == EOState.SLEWING:
            # 转塔转动中
            if self._cue is None:
                self._state = EOState.IDLE
                return

            target_az = self._cue['az']
            target_el = self._cue['el']

            if self._slew_start_time == 0.0:
                self._slew_start_time = sim_time

            dt = sim_time - self._slew_start_time

            # 计算需要的转动量
            delta_az = min(abs(target_az - self._current_az),
                           360 - abs(target_az - self._current_az))
            delta_el = abs(target_el - self._current_el)

            # 判断转动是否完成
            az_done = delta_az < 0.1  # 到达精度 0.1°
            el_done = delta_el < 0.1

            if az_done and el_done:
                # 转动完成 → 开始搜索
                self._state = EOState.ACQUIRING
                self._acquire_start_time = sim_time
            else:
                # 继续转动（简化：线性插值）
                if not az_done:
                    direction = 1 if ((target_az - self._current_az + 180) % 360 - 180) > 0 else -1
                    step = self.slew_rate_az * dt
                    self._current_az += direction * min(step, delta_az)
                if not el_done:
                    direction = 1 if target_el > self._current_el else -1
                    step = self.slew_rate_el * dt
                    self._current_el += direction * min(step, delta_el)

            # 指向到目标位置
            self._current_az = self._current_az % 360
            if self._current_el > 90:
                self._current_el = 90

            # 超时
            if dt > self.cue_timeout:
                self._state = EOState.IDLE
                self._cue = None

        elif self._state == EOState.ACQUIRING:
            # 搜索捕获中 → 由 detect() 中的锁定计时处理
            # 如果超时未捕获，返回 IDLE
            if sim_time - self._acquire_start_time > self.acquisition_time + self.lock_on_time + 2.0:
                self._state = EOState.IDLE

        elif self._state in (EOState.LOCKED, EOState.TRACKING):
            # 升级到持续跟踪
            if self._state == EOState.LOCKED:
                self._state = EOState.TRACKING

            # 所有目标丢失
            if not self._tracked_targets:
                self._state = EOState.IDLE
