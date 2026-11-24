"""
跟踪层(航迹级 / T2T 的前置):每路传感器先各自成航迹。

- 雷达:多目标 恒速(CV)EKF + 全局最近邻(GNN)关联 + M/N 起批、N_miss 删批。
        水平状态为 [x, y, vx, vy]，垂直状态为独立的 [z, vz] 卡尔曼滤波，
        共同构成融合层的"锚航迹"。
- 光电:方位/俯仰 + 激光斜距航迹,用角度和距离联合关联并保留量测误差。
- 射频/声学:测向航迹 —— 对方位角做 α-β 平滑(状态 [az, az_rate]),
        角度门帧间关联。无距离,故为 1D 角度航迹。
- 频谱:无几何,对发射源数量/类型做低通 → 计数航迹。

设计说明:严格 T2T 默认每个传感器都能独立产出完整笛卡尔航迹(如多部雷达),
本系统中的雷达和带激光测距的光电可独立定位;射频/声学仍为纯测向。这里产出
"异构维度航迹",在 fusion 层做类型感知的航迹间关联与三角交汇。
"""

import numpy as np
from scipy.optimize import linear_sum_assignment

from runtime_config import wrap
from vertical_filter import VerticalKalmanFilter


# ============================ 雷达:笛卡尔多目标 EKF ============================
class CVTrack:
    """恒速模型笛卡尔航迹，含水平状态和独立垂直状态。"""
    _next = 1

    def __init__(
        self,
        z_xy,
        dt,
        R,
        z_alt=None,
        z_sigma=15.0,
        meas=None,
    ):
        self.id = CVTrack._next
        CVTrack._next += 1
        self.dt = dt
        self.x = np.array([z_xy[0], z_xy[1], 0.0, 0.0], float)
        self.P = np.diag([R[0, 0], R[1, 1], 40.0 ** 2, 40.0 ** 2]).astype(float)
        self.vertical = VerticalKalmanFilter(
            z_alt,
            dt,
            altitude_sigma=z_sigma,
        )
        self.hits = 1
        self.misses = 0
        self.age = 1
        self.confirmed = False
        self.cls = None
        self.cls_conf = 0.0
        self._capture_class(meas)

    def _capture_class(self, meas):
        if meas and meas.get("cls"):
            self.cls = str(meas["cls"])
            self.cls_conf = float(meas.get("cls_conf", 0.0) or 0.0)

    def predict(self, q=6.0):
        dt = self.dt
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        # 离散白噪声加速度过程噪声
        G = np.array([[dt ** 2 / 2, 0], [0, dt ** 2 / 2], [dt, 0], [0, dt]], float)
        Q = G @ (q ** 2 * np.eye(2)) @ G.T
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.vertical.predict()
        self.age += 1

    def update(self, z_xy, R, z_alt=None, z_sigma=None, meas=None):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        y = z_xy - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.hits += 1
        self.misses = 0
        if self.hits >= 3:
            self.confirmed = True
        self.vertical.update(z_alt, z_sigma)
        self._capture_class(meas)

    @property
    def alt(self):
        return self.vertical.altitude

    def gate_dist(self, z_xy, R):
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        y = z_xy - H @ self.x
        S = H @ self.P @ H.T + R
        return float(y @ np.linalg.inv(S) @ y)  # 马氏距离平方


class RadarTracker:
    """雷达多目标跟踪器:GNN 关联 + M/N 管理。"""

    def __init__(self, dt, gate=16.0, n_miss_del=6,
                 sig_r=15.0, sig_az=np.deg2rad(0.3),
                 sig_el=np.deg2rad(0.5)):
        self.dt = dt
        self.gate = gate            # 马氏门限(chi2, dof=2)
        self.n_miss_del = n_miss_del
        # 这里表示跟踪器“声明/相信”的雷达量测误差。默认值与历史行为一致；
        # 实验可将其与仿真的真实误差分别设置，以区分传感器性能和标定失配。
        self.sig_r = float(sig_r)
        self.sig_az = float(sig_az)
        self.sig_el = float(sig_el)
        self.tracks = []

    def _meas_cov(self, m):
        """把极坐标量测噪声线性化到笛卡尔(简化)。"""
        r, az = max(m["r"], 1.0), m["az"]
        sr, sa = self.sig_r, self.sig_az
        # 雅可比 J = d(x,y)/d(r,az)
        J = np.array([[np.cos(az), -r * np.sin(az)], [np.sin(az), r * np.cos(az)]])
        Rp = np.diag([sr ** 2, sa ** 2])
        return J @ Rp @ J.T

    def _vertical_sigma(self, measurement):
        """由斜距/俯仰误差传播得到高度量测标准差。"""
        r = max(float(measurement.get("r", 0.0) or 0.0), 1.0)
        el = float(measurement.get("el", 0.0) or 0.0)
        variance = (
            (np.sin(el) * self.sig_r) ** 2
            + (r * np.cos(el) * self.sig_el) ** 2
        )
        return max(float(np.sqrt(max(variance, 0.0))), 1.0)

    def process(self, radar_meas):
        for t in self.tracks:
            t.predict()
        meas = list(radar_meas.get("meas", [])) + list(radar_meas.get("fa", []))
        covs = [self._meas_cov(m) for m in meas]
        assigned_tracks = set()
        used = set()
        if self.tracks and meas:
            C = np.full((len(self.tracks), len(meas)), 1e6)
            for ti, t in enumerate(self.tracks):
                for mi, m in enumerate(meas):
                    d = t.gate_dist(np.array([m["x"], m["y"]]), covs[mi])
                    if d < self.gate:
                        C[ti, mi] = d
            ri, ci = linear_sum_assignment(C)
            for r, c in zip(ri, ci):
                if C[r, c] < self.gate:
                    m = meas[c]
                    self.tracks[r].update(
                        np.array([m["x"], m["y"]]),
                        covs[c],
                        z_alt=m.get("z"),
                        z_sigma=self._vertical_sigma(m),
                        meas=m,
                    )
                    assigned_tracks.add(r)
                    used.add(c)
        # 未更新航迹 miss+1
        for ti, t in enumerate(self.tracks):
            if ti not in assigned_tracks:
                t.misses += 1
        # 起批:未关联量测(只用真实回波起批更稳,但允许虚警短暂起批后被删)
        for mi, m in enumerate(meas):
            if mi in used:
                continue
            self.tracks.append(CVTrack(
                np.array([m["x"], m["y"]]),
                self.dt,
                covs[mi],
                z_alt=m.get("z"),
                z_sigma=self._vertical_sigma(m),
                meas=m,
            ))
        # 删批
        self.tracks = [t for t in self.tracks if t.misses < self.n_miss_del]
        return used

    def export(self):
        out = []
        for t in self.tracks:
            if not t.confirmed:
                continue
            row = dict(
                id=int(t.id), x=float(t.x[0]), y=float(t.x[1]),
                vx=float(t.x[2]), vy=float(t.x[3]),
                hits=int(t.hits), misses=int(t.misses),
                fresh=bool(t.misses == 0),
                cls=t.cls, cls_conf=float(t.cls_conf),
                cov=[[float(t.P[0, 0]), float(t.P[0, 1])],
                     [float(t.P[1, 0]), float(t.P[1, 1])]],
            )
            if t.alt is not None:
                row["z"] = float(t.alt)
                row["z_sigma"] = float(t.vertical.altitude_sigma_estimate)
            if t.vertical.velocity_valid:
                row["vz"] = float(t.vertical.velocity)
                row["vz_sigma"] = float(t.vertical.velocity_sigma_estimate)
            out.append(row)
        return out


# ============================ 测向:方位 α-β 航迹 ============================
class BearingTrack:
    _next = 1

    def __init__(self, az, dt, meas=None):
        self.id = BearingTrack._next
        BearingTrack._next += 1
        self.az = az
        self.rate = 0.0
        self.r = None
        self.range_rate = 0.0
        self.range_age = None       # 距最近一次真实激光距离更新的帧数
        self.el = None
        self.sig_r = None
        self.sig_az = None
        self.sig_el = None
        self.range_is_horizontal = False
        self.dt = dt
        self.hits = 1
        self.misses = 0
        self.confirmed = False
        self.cls = None          # 光电视觉类别 / 射频协议
        self.cls_conf = 0.0
        self.extra = {}
        self._capture_measurement(meas, initialize=True)

    @staticmethod
    def _finite(value):
        try:
            return value is not None and np.isfinite(float(value))
        except (TypeError, ValueError):
            return False

    def _capture_measurement(self, meas, initialize=False):
        if not meas:
            return
        if self._finite(meas.get("r")) and float(meas["r"]) > 0:
            measured_r = float(meas["r"])
            if initialize or self.r is None:
                self.r = measured_r
                self.range_rate = 0.0
            self.range_age = 0
        elif "r" in meas:
            self.r, self.range_age, self.range_rate = None, None, 0.0
        if self._finite(meas.get("el")):
            self.el = float(meas["el"]) if initialize or self.el is None else self.el
        if meas.get("range_is_horizontal") is True:
            self.range_is_horizontal = True
        for key in ("sig_r", "sig_az", "sig_el"):
            if self._finite(meas.get(key)):
                setattr(self, key, float(meas[key]))
        if "cls" in meas:
            self.cls, self.cls_conf = meas["cls"], meas.get("cls_conf", 0.6)
        if "proto" in meas:
            # Keep the legacy protocol string for display/identity, not as a class.
            confidence = meas.get("confidence")
            self.cls = meas["proto"]
            self.cls_conf = (float(confidence) if self._finite(confidence)
                             and 0 <= float(confidence) <= 1 else 0.0)
        # Native RF classification is a per-report snapshot, never sticky metadata.
        # A missing/invalid classification cannot refresh the previous model proof.
        for key in ("rf_classification", "source_track_id", "timestamp_ms"):
            self.extra.pop(key, None)
        if isinstance(meas.get("rf_classification"), dict):
            evidence = dict(meas["rf_classification"])
            self.extra.update(rf_classification=evidence,
                              source_track_id=evidence["source_track_id"],
                              timestamp_ms=evidence["timestamp_ms"])
        for key in ("sn_id", "model"):
            value = meas.get(key)
            if value:
                self.extra[key] = str(value)
        freq = meas.get("rf_freq", meas.get("freq_hz"))
        if self._finite(freq) and float(freq) > 0:
            self.extra["rf_freq"] = float(freq)
        if self._finite(meas.get("confidence")):
            self.extra["rf_confidence"] = float(meas["confidence"])

    def predict(self):
        self.az = wrap(self.az + self.rate * self.dt)
        if self.r is not None:
            self.r = max(1.0, self.r + self.range_rate * self.dt)
            self.range_age = 1 if self.range_age is None else self.range_age + 1

    def update(self, az, alpha=0.5, beta=0.1, meas=None,
               range_alpha=0.6, range_beta=0.08, el_alpha=0.5):
        resid = wrap(az - self.az)
        self.az = wrap(self.az + alpha * resid)
        self.rate += beta * resid / self.dt
        if meas and self._finite(meas.get("r")) and float(meas["r"]) > 0:
            measured_r = float(meas["r"])
            if self.r is None:
                self.r = measured_r
                self.range_rate = 0.0
            else:
                range_resid = measured_r - self.r
                self.r += range_alpha * range_resid
                self.range_rate += range_beta * range_resid / self.dt
            self.range_age = 0
        if meas and self._finite(meas.get("el")):
            measured_el = float(meas["el"])
            self.el = measured_el if self.el is None else (
                self.el + el_alpha * (measured_el - self.el)
            )
        self.hits += 1
        self.misses = 0
        if self.hits >= 3:
            self.confirmed = True
        self._capture_measurement(meas)


class BearingTracker:
    """角度航迹跟踪；量测带激光斜距时自动使用角度+距离联合关联。"""

    RANGED_SIGMA_GATE = 3.0

    def __init__(self, dt, gate_deg=2.5, n_miss_del=8,
                 ranged_az_gate_deg=2.5, range_gate_m=35.0):
        self.dt = dt
        self.gate = np.deg2rad(gate_deg)
        self.ranged_az_gate = np.deg2rad(ranged_az_gate_deg)
        self.range_gate = float(range_gate_m)
        self.n_miss_del = n_miss_del
        self.tracks = []

    @staticmethod
    def _measured_range(meas):
        value = meas.get("r")
        try:
            value = float(value)
        except (TypeError, ValueError):
            return None
        return value if np.isfinite(value) and value > 0 else None

    @staticmethod
    def _measurement_sigma(meas, key):
        """读取量测声明的 1σ 误差；无效或缺失时不改变历史门限。"""
        value = meas.get(key)
        try:
            value = abs(float(value))
        except (TypeError, ValueError):
            return 0.0
        return value if np.isfinite(value) else 0.0

    def _adaptive_ranged_gates(self, meas, measured_r, predicted_r):
        """
        把量测声明的径向/角向误差投影到站心平面。

        小误差光电量测仍使用原有的方位门和 range_gate；5--10°
        测向误差或 5%R 测距误差则按 3σ 自适应放宽，避免连续量测
        因固定 35 m 门限被不断起成新航迹。
        """
        sig_r = self._measurement_sigma(meas, "sig_r")
        sig_az = self._measurement_sigma(meas, "sig_az")
        sigma_gate = self.RANGED_SIGMA_GATE
        az_gate = max(self.ranged_az_gate, sigma_gate * sig_az)
        reference_r = max(float(measured_r), float(predicted_r), 1.0)
        local_sigma = float(np.hypot(sig_r, reference_r * sig_az))
        range_gate = max(self.range_gate, sigma_gate * local_sigma)
        return az_gate, range_gate

    def process(self, meas_list):
        for t in self.tracks:
            t.predict()
            # 光学仍有角度、但目标已超出激光量程时，不能无限沿用历史距离。
            # 短时漏测允许按距离速度外推；超过删除窗口则降级回纯方位航迹。
            if t.r is not None and t.range_age is not None and t.range_age >= self.n_miss_del:
                t.r = None
                t.range_rate = 0.0
                t.range_age = None
        used = set()
        assigned = set()
        if self.tracks and meas_list:
            C = np.full((len(self.tracks), len(meas_list)), 1e6)
            for ti, t in enumerate(self.tracks):
                for mi, m in enumerate(meas_list):
                    # Different native RF target IDs must not exchange model labels
                    # merely because their bearings are close. Legacy EO/sim unchanged.
                    evidence = m.get("rf_classification")
                    previous = t.extra.get("rf_classification")
                    if (evidence is not None and previous is not None
                            and evidence["source_track_id"] != previous["source_track_id"]):
                        continue
                    d_az = abs(wrap(m["az"] - t.az))
                    measured_r = self._measured_range(m)
                    ranged = measured_r is not None and t.r is not None
                    if ranged:
                        az_gate, local_gate = self._adaptive_ranged_gates(
                            m, measured_r, t.r
                        )
                    else:
                        # 纯方位设备(如 5° 声学阵列)也必须按其声明
                        # 的角误差放宽。旧量测没有 sig_az 时仍为原门限。
                        sig_az = self._measurement_sigma(m, "sig_az")
                        az_gate = max(
                            self.gate, self.RANGED_SIGMA_GATE * sig_az
                        )
                        local_gate = self.range_gate
                    if d_az >= az_gate:
                        continue
                    if ranged:
                        # 极坐标下直接计算同一站心系的二维点间距离；相比把角差
                        # 和距离差用两个宽门拼接，它能自然随目标距离收紧角度门。
                        local_distance_sq = (
                            measured_r ** 2 + t.r ** 2 -
                            2.0 * measured_r * t.r * np.cos(d_az)
                        )
                        local_distance = float(np.sqrt(max(local_distance_sq, 0.0)))
                        if local_distance >= local_gate:
                            continue
                        C[ti, mi] = local_distance / max(local_gate, 1e-9)
                    else:
                        C[ti, mi] = d_az / max(self.gate, 1e-9)
            ri, ci = linear_sum_assignment(C)
            for r, c in zip(ri, ci):
                if C[r, c] < 1e6:
                    t = self.tracks[r]
                    t.update(meas_list[c]["az"], meas=meas_list[c])
                    used.add(c); assigned.add(r)
        for ti, t in enumerate(self.tracks):
            if ti not in assigned:
                t.misses += 1
        for mi, m in enumerate(meas_list):
            if mi in used:
                continue
            bt = BearingTrack(m["az"], self.dt, meas=m)
            self.tracks.append(bt)
        self.tracks = [t for t in self.tracks if t.misses < self.n_miss_del]

    def export(self):
        out = []
        for t in self.tracks:
            if not t.confirmed:
                continue
            row = dict(id=int(t.id), az=float(t.az), rate=float(t.rate),
                       cls=t.cls, cls_conf=float(t.cls_conf),
                       hits=int(t.hits), misses=int(t.misses),
                       fresh=bool(t.misses == 0))
            if t.el is not None:
                row["el"] = float(t.el)
            for key in ("sig_az", "sig_el"):
                value = getattr(t, key)
                if value is not None:
                    row[key] = float(value)
            if t.r is not None:
                row.update(r=float(t.r), range_rate=float(t.range_rate),
                           range_age=int(t.range_age or 0),
                           range_is_horizontal=bool(t.range_is_horizontal))
                for key in ("sig_r",):
                    value = getattr(t, key)
                    if value is not None:
                        row[key] = float(value)
            row.update(t.extra)
            out.append(row)
        return out


# ============================ 频谱:计数航迹 ============================
class SpectrumTracker:
    def __init__(self):
        self.count = 0.0
        self.classes = {}

    def process(self, spec):
        c = spec.get("emitters", 0)
        self.count = 0.6 * self.count + 0.4 * c
        for k, v in spec.get("classes", {}).items():
            self.classes[k] = 0.6 * self.classes.get(k, 0) + 0.4 * v

    def export(self):
        return dict(emitters=int(round(self.count)),
                    classes={k: round(v, 1) for k, v in self.classes.items()})
