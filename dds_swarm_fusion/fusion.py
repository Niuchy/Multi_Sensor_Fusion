"""
融合层(核心):类型感知的异构航迹间(T2T)融合。

本系统中的雷达和带激光测距的光电可产出二维位置约束,射频/声学仍为纯测向,
频谱无方位。这里采用分布式异构跟踪的常规做法:

  1) 预测所有融合航迹(CV-EKF)。
  2) 雷达航迹 → 融合航迹:位置门 GNN 关联;雷达为几何锚,匹配则用雷达位置
     更新融合航迹,未匹配雷达航迹起批新融合航迹(origin=radar)。
  3) 光电激光航迹 → 融合航迹:由斜距/方位/俯仰解算位置,位置门关联;
     未匹配量测可直接起批单站可观测航迹(origin=eo_laser)。
  4) 纯测向航迹 → 融合航迹:对每条方位航迹,算其相对各融合航迹的期望方位,
     角度门关联;命中则 (a) 用方位量测做 EKF 更新(收紧垂直视线方向的横向距离)
     (b) 叠加身份/类别证据。被消费的方位航迹移出待定池。
  5) 三角交汇起批:对"未被任何融合航迹消费"的无距离方位航迹,取不同节点的两两组合
     做视线交汇;持续性确认后在交点新生融合航迹(origin=triangulation)。
  6) 置信度与类别融合:置信随贡献传感器数量/多样性上升。
  7) 航迹管理:删除失联航迹、合并重复航迹。
  8) 频谱交叉校验:频谱发射源计数 与 已定位发射目标数 比对 → 未定位发射源告警。

指标:对真值做最近邻匹配,分别评估 雷达单独 / 光电激光测距单独 / 射频单独
(仅检测识别)/ 融合 的检测率、定位 RMSE、航迹数,量化融合增益。
"""

import numpy as np
from collections import deque
from scipy.optimize import linear_sum_assignment

from runtime_config import AREA, CLASSES, wrap
from vertical_filter import VerticalKalmanFilter
from eo_geometry import intersect


# ----------------------------- 几何:双视线交汇 -----------------------------
def triangulate(p1, a1, p2, a2, sig1, sig2):
    """两条视线 (传感器 p, 方位 a) 交汇。返回 (point[2], cov2x2) 或 None。"""
    u1 = np.array([np.cos(a1), np.sin(a1)])
    u2 = np.array([np.cos(a2), np.sin(a2)])
    A = np.column_stack([u1, -u2])
    det = np.linalg.det(A)
    if abs(det) < 0.12:            # 视线近平行 → GDOP 过大,丢弃
        return None
    t = np.linalg.solve(A, (p2 - p1))
    if t[0] <= 0 or t[1] <= 0:     # 必须在两传感器前方
        return None
    pt = p1 + t[0] * u1
    # 数值雅可比 d(pt)/d(a1,a2) → 交汇协方差
    eps = 1e-4
    def solve_pt(b1, b2):
        v1 = np.array([np.cos(b1), np.sin(b1)])
        v2 = np.array([np.cos(b2), np.sin(b2)])
        tt = np.linalg.solve(np.column_stack([v1, -v2]), (p2 - p1))
        return p1 + tt[0] * v1
    J = np.column_stack([
        (solve_pt(a1 + eps, a2) - solve_pt(a1 - eps, a2)) / (2 * eps),
        (solve_pt(a1, a2 + eps) - solve_pt(a1, a2 - eps)) / (2 * eps),
    ])
    cov = J @ np.diag([sig1 ** 2, sig2 ** 2]) @ J.T
    return pt, cov


# ----------------------------- 融合航迹 -----------------------------
SRC_W = {"radar": 1.0, "eo_w": 0.7, "eo_e": 0.7, "rf_df": 0.6, "acoustic": 0.4}
# 测向传感器角噪声，用于 EKF 更新和交汇。
SRC_SIG = {"eo_w": np.deg2rad(0.1), "eo_e": np.deg2rad(0.15),
           "rf_df": np.deg2rad(0.6), "acoustic": np.deg2rad(1.6)}
# 运行时增设的传感器没有标定 sid,按 kind 回退取置信权重 / 角噪声。
# 标定站(eo_w/eo_e 等)仍优先用上面的 SRC_*,确保默认场景指标不变。
KIND_W = {"radar": 1.0, "eo": 0.7, "rf_df": 0.6, "acoustic": 0.4, "spectrum": 0.0}
KIND_SIG = {"eo": np.deg2rad(0.12), "rf_df": np.deg2rad(0.6), "acoustic": np.deg2rad(1.6)}
DEF_SIG = np.deg2rad(1.0)
DEFAULT_LASER_SIG_R = 3.0
DEFAULT_LASER_SIG_EL = np.deg2rad(0.15)


def _as_pos3(pos):
    values = np.asarray(pos, float).ravel()
    out = np.zeros(3, float)
    out[:min(3, values.size)] = values[:3]
    return out


class FusedTrack:
    _next = 1

    def __init__(self, x, P, dt, origin, alt=None, vz=None, alt_sigma=15.0):
        self.id = FusedTrack._next
        FusedTrack._next += 1
        self.x = np.asarray(x, float)       # [px,py,vx,vy]
        self.P = np.asarray(P, float)
        self.dt = dt
        self.origin = origin                # 'radar' | 'eo_laser' | 'triangulation'
        self.vertical = VerticalKalmanFilter(
            alt,
            dt,
            velocity=vz,
            altitude_sigma=alt_sigma,
        )
        self.cls_logp = {c: 0.0 for c in CLASSES}   # 有效已知类别的对数似然
        self.class_updates = 0
        # RF model inference is a bounded fallback, not a vote against EO classes.
        self.rf_class_evidence = None
        self.rf_id = None
        self.rf_metadata = {}
        self.rf_metadata_tick = -1
        self.rf_metadata_confidence = -1.0
        self.sources_seen = {}              # src -> 最近贡献的 tick 衰减权重
        self.sensor_ids_seen = {}           # 真实设备 sid -> 最近贡献 tick
        self.position_sources = set()       # 本身可给位置的来源(radar/激光测距等)
        self.position_seen = {}
        self.last_geometry_tick = None
        self.source_bindings = {}
        self.bearing_seen = {}
        self.cross_cov = np.zeros((4, 2))
        self.last_tick = 0
        self.conf = 0.3
        self.conf_parts = {}                # src -> 置信度贡献分量(供技术面板)
        self.assoc = {}                     # 本帧贡献量测 + 关联诊断(门限/新息)
        self.radar_binding = None
        self.last_report_tick = None
        self.report_timestamp_ms = None
        self.last_measurement_ms = None
        self.source_track_id = None
        # Publication-quality metadata; independent of association/classification.
        self.position_update_count = 0
        self.last_position_update_ms = None
        self.created_at_s = None

    def record_position_update(self, stamp):
        """One point per new observation time, never prediction or redraw."""
        try:
            stamp = float(stamp)
        except (TypeError, ValueError):
            return
        if not np.isfinite(stamp) or (self.last_position_update_ms is not None
                                     and stamp <= self.last_position_update_ms):
            return
        self.position_update_count += 1
        self.last_position_update_ms = stamp

    def predict(self, q=6.0):
        dt = self.dt
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        G = np.array([[dt ** 2 / 2, 0], [0, dt ** 2 / 2], [dt, 0], [0, dt]], float)
        Q = G @ (q ** 2 * np.eye(2)) @ G.T
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        Fz = np.array([[1.0, dt], [0.0, 1.0]])
        self.cross_cov = F @ self.cross_cov @ Fz.T
        self.vertical.predict()
        self.assoc = {}

    def update_position(
        self,
        z_xy,
        R,
        alt=None,
        alt_sigma=None,
        vel=None,
        vel_sig=8.0,
    ):
        """雷达笛卡尔更新(锚)。带速度时做全状态更新,消除再滤波滞后。"""
        if vel is None:
            H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
            z = np.asarray(z_xy, float)
            Rm = R
        else:
            H = np.eye(4)
            z = np.array([z_xy[0], z_xy[1], vel[0], vel[1]], float)
            Rm = np.zeros((4, 4))
            Rm[:2, :2] = R
            Rm[2, 2] = Rm[3, 3] = vel_sig ** 2
        if self.vertical.x is not None:
            H = np.pad(H, ((0, 0), (0, 2)))
            if alt is not None and np.isfinite(float(alt)):
                H = np.vstack([H, [0, 0, 0, 0, 1, 0]])
                z = np.append(z, float(alt))
                Rm = np.pad(Rm, ((0, 1), (0, 1)))
                Rm[-1, -1] = max(float(alt_sigma or 15.0), 0.001) ** 2
                self.vertical.measurement_updates += 1
            self._linear_update(z - H @ self.joint_state()[0], H, Rm)
        else:
            self._linear_update(z - H @ self.x, H, Rm)
            self.vertical.update(alt, alt_sigma)

    @property
    def alt(self):
        return self.vertical.altitude

    @property
    def vz(self):
        return (
            self.vertical.velocity
            if self.vertical.velocity_valid
            else None
        )

    def joint_state(self):
        if self.vertical.x is None:
            return self.x.copy(), self.P.copy()
        return (np.r_[self.x, self.vertical.x],
                np.block([[self.P, self.cross_cov],
                          [self.cross_cov.T, self.vertical.P]]))

    def _linear_update(self, residual, H, R):
        x, P = self.joint_state()
        S = H @ P @ H.T + R
        K = np.linalg.solve(S, H @ P).T
        x = x + K @ residual
        A = np.eye(len(x)) - K @ H
        P = A @ P @ A.T + K @ R @ K.T
        P = (P + P.T) * 0.5
        self.x, self.P = x[:4], P[:4, :4]
        if len(x) == 6:
            self.vertical.x, self.vertical.P = x[4:], P[4:, 4:]
            self.cross_cov = P[:4, 4:]

    def bearing_innovation(self, sensor_pos, az_meas, sig, el=None, sig_el=None):
        dx = self.x[0] - sensor_pos[0]
        dy = self.x[1] - sensor_pos[1]
        d2 = dx * dx + dy * dy
        if d2 < 1.0:
            return None
        x, P = self.joint_state()
        H = np.zeros((1, len(x)))
        H[0, :2] = [-dy / d2, dx / d2]
        residual = [wrap(az_meas - np.arctan2(dy, dx))]
        variances = [max(abs(float(sig)), 1e-6) ** 2]
        if el is not None and len(x) == 6 and len(sensor_pos) >= 3:
            rho = np.sqrt(d2)
            dz = x[4] - sensor_pos[2]
            r2 = d2 + dz * dz
            h_el = np.zeros(6)
            h_el[[0, 1, 4]] = [-dz*dx/(rho*r2), -dz*dy/(rho*r2), rho/r2]
            H = np.vstack([H, h_el])
            residual.append(float(el) - np.arctan2(dz, rho))
            variances.append(max(abs(float(sig_el or sig)), 1e-6) ** 2)
        residual, R = np.array(residual), np.diag(variances)
        S = H @ P @ H.T + R
        return residual, H, R, float(residual @ np.linalg.solve(S, residual))

    def update_bearing(self, sensor_pos, az_meas, sig, el=None, sig_el=None):
        innovation = self.bearing_innovation(sensor_pos, az_meas, sig, el, sig_el)
        if innovation is not None:
            self._linear_update(*innovation[:3])

    def add_class(self, cls, conf):
        # unknown 表示设备没有分类结论，不是反对其他类别的证据。
        # 只跳过类别累计；位置更新、来源权重和航迹置信度照常处理。
        if cls == "unknown":
            return
        if cls in self.cls_logp:
            try:
                conf = float(conf)
            except (TypeError, ValueError):
                return
            if not np.isfinite(conf) or conf <= 0.0:
                return
            self.rf_class_evidence = None
            conf = min(0.999, max(0.001, conf))
            for c in CLASSES:
                self.cls_logp[c] += np.log(conf if c == cls else (1 - conf) / (len(CLASSES) - 1) + 1e-6)
            self.class_updates += 1

    def best_class(self):
        # 没有有效分类证据时不能用字典的首项(drone)作为默认类别。
        if not self.class_updates:
            return "drone" if self.rf_class_evidence else "unknown"
        c = max(self.cls_logp, key=self.cls_logp.get)
        return c

    def touch(self, src, tick, position_capable=False, sensor_id=None):
        self.sources_seen[src] = (tick, 1.0)
        concrete_sid = sensor_id or (src if src != "radar" else None)
        if concrete_sid:
            self.sensor_ids_seen[str(concrete_sid)] = int(tick)
        if position_capable:
            self.position_sources.add(src)
            self.position_seen[src] = tick
        self.last_tick = tick

    def update_rf_metadata(self, metadata, tick):
        """保留最近、同帧内置信度最高的一组 ESM 识别信息。"""
        if not metadata:
            return
        candidate = {}
        for key in ("sn_id", "model"):
            value = metadata.get(key)
            if value is not None and str(value).strip():
                candidate[key] = str(value).strip()
        freq = metadata.get("rf_freq", metadata.get("freq_hz"))
        try:
            freq = float(freq)
        except (TypeError, ValueError):
            freq = None
        if freq is not None and np.isfinite(freq) and freq > 0:
            candidate["rf_freq"] = freq
        if not candidate:
            return

        confidence = metadata.get(
            "rf_confidence", metadata.get("confidence", metadata.get("cls_conf", 0.0))
        )
        try:
            confidence = float(confidence)
        except (TypeError, ValueError):
            confidence = 0.0
        priority = (int(tick), confidence)
        current = (self.rf_metadata_tick, self.rf_metadata_confidence)
        if priority < current:
            return
        merged = dict(self.rf_metadata)
        merged.update(candidate)
        self.rf_metadata = merged
        self.rf_metadata_tick = int(tick)
        self.rf_metadata_confidence = confidence

    def update_confidence(self, tick, weights=SRC_W):
        score = 0.0
        parts = {}
        for src, (tk, _) in list(self.sources_seen.items()):
            # "radar" 为雷达锚的通用标签(与具体雷达 sid 解耦),恒取锚权重 1.0
            base = 1.0 if src == "radar" else weights.get(src, 0.3)
            c = base * (0.85 ** (tick - tk))         # 时间衰减
            parts[src] = c
            score += c
        self.conf_parts = parts
        self.conf = float(1 - np.exp(-0.7 * score))

    def active_sources(self, tick, horizon=12):
        return sorted([s for s, (tk, _) in self.sources_seen.items() if tick - tk <= horizon])

    def active_sensor_ids(self, tick, horizon=12):
        return sorted([
            sid for sid, seen_tick in self.sensor_ids_seen.items()
            if tick - seen_tick <= horizon
        ])

    def pos(self):
        return self.x[:2].copy()

    def pos3(self):
        """返回有真实高度支撑的三维位置；高度未知时不伪造三维坐标。"""
        if self.alt is None or not np.isfinite(float(self.alt)):
            return None
        position = np.array([self.x[0], self.x[1], float(self.alt)], float)
        return position if np.all(np.isfinite(position)) else None


# ----------------------------- 融合器 -----------------------------
class Fusion:
    def __init__(self, sensors, dt, pos_gate=20.0, az_gate_deg=3.0,
                 adaptive_bearing_gates=False, area=None, min_confidence=0.05):
        # Preserve RK3588 field policy; independent tests explicitly use 0.2.
        self.min_confidence = float(min_confidence)
        if not np.isfinite(self.min_confidence) or not 0 <= self.min_confidence <= 1:
            raise ValueError('min_confidence must be finite and within 0–1')
        self.dt = dt
        self.area = dict(area or AREA)
        self.pos3 = {sid: _as_pos3(s.pos) for sid, s in sensors.items()}
        self.pos = {sid: value[:2].copy() for sid, value in self.pos3.items()}
        # 按 kind 索引的元信息:供按种类(而非写死 sid)做关联/置信/交汇,
        # 从而支持运行时增设任意数量的同类传感器。
        self.kinds = {sid: s.kind for sid, s in sensors.items()}
        self.station_known = {sid: bool(getattr(s, "station_known", True))
                              for sid, s in sensors.items()}
        self.sig = {
            sid: (float(getattr(s, "sig_az"))
                  if getattr(s, "sig_az", None) is not None
                  else self._sig_for(sid, s.kind))
            for sid, s in sensors.items()
        }
        self.weight = {sid: self._w_for(sid, s.kind) for sid, s in sensors.items()}
        self.pos_gate = pos_gate          # 雷达→融合 马氏门
        self.az_gate = np.deg2rad(az_gate_deg)
        # 实装误差实验中的RF/声学标称角误差可达5°–10°，若仍使用固定4°
        # 绝对帽，结果主要反映门限失配而不是传感器误差。默认关闭以保持实时演示
        # 行为不变；实验可显式启用按每站声明1σ扩展到3σ的绝对帽。
        self.adaptive_bearing_gates = bool(adaptive_bearing_gates)
        self.tracks = []
        self.tick = 0
        self._pending = []                # 三角交汇候选缓冲 [dict(pt, hits, sources)]
        self.time_s = None
        self._track_clock_s = 0.0
        self.bearing_cache = {}
        self.pair_used = {}
        self.geometry_events = []
        self.geometry_history = deque(maxlen=30)
        self.eo_pending = {}
        self.direction_tracks = []
        self.rf_class_candidates = {}

    # 关联/起批门限(单一来源,供技术面板标注;改这里即同时改逻辑与展示)
    AZ_CHI2 = 9.0                          # 测向马氏门(χ², 1dof, ≈3σ)
    ABS_CAP_DEG = 4.0                      # 测向绝对角差兜底
    BLOCK_DEG = 2.2                        # 交汇池准入遮挡
    TRI_CONFIRM = 8                        # 三角交汇持续确认帧数
    MERGE_DIST_3D = 20.0                   # 三维近距合并阈值(m)
    RADAR_ALT_ASSOC_SCALE_M = 50.0         # 雷达T2T关联的高度归一化尺度(m)
    DELETE_HORIZON = 10                    # 失联删除/可观测性窗口(帧)
    # 3种子短网格(10/20/50m间距)在45/55m间对照后取55m：可显著减少50m
    # 场景中的雷达碎片重复轨；10m召回约损失1.4个百分点，报告需保留该权衡。
    LASER_POS_GATE_M = 55.0                # 激光位置航迹最小绝对门(m)

    @staticmethod
    def _sig_for(sid, kind):
        # 标定站优先用精确值;运行时增设的按 kind 回退。
        return SRC_SIG.get(sid) or KIND_SIG.get(kind, DEF_SIG)

    @staticmethod
    def _w_for(sid, kind):
        return SRC_W.get(sid) or KIND_W.get(kind, 0.3)

    @staticmethod
    def _track_is_fresh(track):
        """上游预测轨可用于显示，但不能冒充本帧新量测反复刷新融合航迹。

        旧DDS录包没有 ``fresh/misses`` 字段时按新鲜输入处理，保持协议兼容。
        """
        if "fresh" in track:
            return bool(track["fresh"])
        try:
            return int(track.get("misses", 0)) == 0
        except (TypeError, ValueError):
            return True

    def _apply_track_evidence(self, fused_track, source_track, kind):
        """把设备航迹的分类与 ESM 身份信息附到已关联融合航迹。"""
        if kind in {"eo", "acoustic"} and source_track.get("cls"):
            fused_track.add_class(
                source_track["cls"], source_track.get("cls_conf", 0.6)
            )
        elif kind == "rf_df":
            if source_track.get("cls"):
                fused_track.rf_id = source_track.get("cls")
            fused_track.update_rf_metadata(source_track, self.tick)

    def _rf_classification_step(self, ft, bt, sid, unique):
        """Only unambiguous NEW model reports may fill a radar's unknown class.

        Evidence is separate from cls_logp so even a later EO classification wins.
        No extra geometry updates, DDS publications or synthetic confidence here.
        """
        e = bt.get("rf_classification")
        if not isinstance(e, dict) or not self._track_is_fresh(bt):
            return "no_new_model_evidence"
        try:
            stamp = int(e["timestamp_ms"])
            target_id = int(e["source_track_id"])
            need, gap, age = (int(e[k]) for k in ("confirm_reports", "max_gap_ms", "max_age_ms"))
            confidence = float(e["confidence"]) if e.get("confidence") is not None else 0.0
        except (KeyError, ValueError, TypeError, OverflowError):
            return "invalid_evidence"
        if not (2 <= need <= 20 and 100 <= gap <= age <= 60000):
            return "invalid_policy"
        now = (self.time_s if self.time_s is not None else self.tick * .5) * 1000
        key = (sid, target_id)
        prev = self.rf_class_candidates.get(key)
        if prev and stamp <= prev["timestamp_ms"]:
            return "duplicate_or_old_model_report"
        state = dict(timestamp_ms=stamp, fused_id=None, model=e.get("model"), hits=0)
        # Invalid, ambiguous or newly reassigned reports retract this RF fallback.
        # A known non-RF class is never cleared by this path.
        for track in self.tracks:
            old = track.rf_class_evidence
            if old and (old["sensor_id"], old["source_track_id"]) == key:
                track.rf_class_evidence = None
        self.rf_class_candidates[key] = state
        if not 0 <= now - stamp <= gap:
            return "model_report_time_mismatch"
        if e.get("valid") is not True or e.get("cls") != "drone" or not e.get("model"):
            return e.get("reason", "invalid_model")
        if not np.isfinite(confidence) or not 0 < confidence <= 1:
            return "invalid_confidence"
        if ft is None or not unique:
            return "unmatched_or_ambiguous"
        if ft.radar_binding is None and "radar" not in ft.position_sources:
            return "no_radar_track"
        if ft.class_updates:
            return "known_class_preserved"
        state["fused_id"] = ft.id
        same = (prev and prev["fused_id"] == ft.id and prev["model"] == e["model"]
                and stamp - prev["timestamp_ms"] <= gap)
        state["hits"] = min(need, prev["hits"] + 1 if same else 1)
        if state["hits"] < need:
            return "confirming_model_association"
        ft.rf_class_evidence = dict(cls="drone", model=e["model"], sensor_id=sid,
            source_track_id=target_id, timestamp_ms=stamp, confidence=confidence,
            confirmation_reports=state["hits"], expires_at_ms=stamp + age)
        return "applied_unknown_only"

    def _record_position(self, fused_track, observation):
        stamp = observation.get("report_timestamp_ms")
        if stamp is None:
            key = "last_position_ms" if observation.get("mode") == "position" else "last_range_ms"
            stamp = observation.get(key)
        if stamp is None:
            stamp = observation.get("timestamp_ms")
        if stamp is None:
            stamp = (self.time_s if self.time_s is not None else self.tick*self.dt)*1000
        fused_track.record_position_update(stamp)

    def add_sensor(self, sid, pos, kind):
        """运行时增设一路传感器,登记其位置/种类/角噪声/权重。不影响现有航迹。"""
        self.pos3[sid] = _as_pos3(pos)
        self.pos[sid] = self.pos3[sid][:2].copy()
        self.kinds[sid] = kind
        self.sig[sid] = self._sig_for(sid, kind)
        self.weight[sid] = self._w_for(sid, kind)
        self.station_known[sid] = True

    def remove_sensor(self, sid):
        """运行时撤除一路传感器。其历史贡献会随时间衰减/可观测性删除自然退出。"""
        for d in (self.pos, self.pos3, self.kinds, self.sig, self.weight, self.station_known):
            d.pop(sid, None)

    # ---- 雷达关联/更新/起批 ----
    # T2T 相关性说明:参数扫描表明本场景雷达航迹误差以机动滞后为主,
    # 协方差膨胀会放大级联滞后(×3 时共同集RMSE 19→33m),故取 1.0;
    # 代价是融合协方差略乐观,README 中如实说明。
    TRK_INFLATE = 1.0

    def _radar_step(self, radar_tracks):
        radar_tracks = [track for track in radar_tracks if self._track_is_fresh(track)
                        or (track.get("input_mode") == "device_tracks" and track.get("new_report"))]
        for t in self.tracks:
            t.predict()
        assigned_f = set()
        used_r = set()
        if self.tracks and radar_tracks:
            C = np.full((len(self.tracks), len(radar_tracks)), 1e6)
            # C 只保存原有二维门控代价；A 在有效门内叠加高度一致性，
            # 用于匈牙利分配。接受/拒绝仍由 C 判断，避免高度噪声造成漏关联。
            A = np.full_like(C, 1e6)
            NIS = np.full_like(C, np.nan)
            ND = np.full_like(C, np.nan)
            G = np.full_like(C, np.nan)
            laser_rows = np.zeros(len(self.tracks), dtype=bool)
            for fi, ft in enumerate(self.tracks):
                has_recent_laser = any(
                    source != "radar" and source in ft.position_sources and
                    self.tick - ft.position_seen.get(source, -999)
                    <= self.DELETE_HORIZON
                    for source in ft.position_sources
                )
                laser_rows[fi] = has_recent_laser
                for ri, rt in enumerate(radar_tracks):
                    binding = (rt.get("sensor_id"), rt["id"])
                    if (rt.get("input_mode") == "device_tracks" and ft.radar_binding is not None
                            and ft.radar_binding[0] == binding[0]):
                        if ft.radar_binding != binding:
                            continue
                        C[fi, ri] = A[fi, ri] = 0.0
                        NIS[fi, ri] = 0.0
                        continue
                    R = np.array(rt["cov"]) * self.TRK_INFLATE
                    z = np.array([rt["x"], rt["y"]])
                    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
                    yv = z - H @ ft.x
                    S = H @ ft.P @ H.T + R
                    if has_recent_laser:
                        # 激光反复更新后融合协方差很小，直接按NIS会把正常雷达
                        # 偏差误判为不关联并重复起批。这里使用协方差自适应的
                        # 绝对位置门，仍由匈牙利算法保证一一匹配。
                        gate_m = max(
                            self.LASER_POS_GATE_M,
                            3.0 * np.sqrt(max(float(np.max(np.linalg.eigvalsh(S))), 1.0)),
                        )
                        distance = float(np.linalg.norm(yv))
                        G[fi, ri] = gate_m
                        if distance < gate_m:
                            ND[fi, ri] = distance / gate_m
                            # 两类分支都使用“门限占比的平方”作为匈牙利代价，
                            # 避免把 distance/gate 与原始 NIS 混在同一矩阵中。
                            C[fi, ri] = ND[fi, ri] ** 2
                    else:
                        d = float(yv @ np.linalg.inv(S) @ yv)
                        if d < self.pos_gate:
                            NIS[fi, ri] = d
                            C[fi, ri] = d / self.pos_gate
                    if C[fi, ri] < 1.0:
                        A[fi, ri] = C[fi, ri]
                        radar_alt = rt.get("z")
                        if (
                            ft.alt is not None and
                            radar_alt is not None and
                            np.isfinite(float(ft.alt)) and
                            np.isfinite(float(radar_alt))
                        ):
                            dz = (float(ft.alt) - float(radar_alt))
                            A[fi, ri] += (
                                dz / self.RADAR_ALT_ASSOC_SCALE_M
                            ) ** 2
            fi_idx, ri_idx = linear_sum_assignment(A)
            for fi, ri in zip(fi_idx, ri_idx):
                if C[fi, ri] < 1.0:
                    rt = radar_tracks[ri]
                    ft = self.tracks[fi]
                    if rt.get("input_mode") == "device_tracks":
                        self._device_report(ft, rt)
                        assigned_f.add(fi); used_r.add(ri)
                        continue
                    ft.update_position(np.array([rt["x"], rt["y"]]),
                                       np.array(rt["cov"]) * self.TRK_INFLATE,
                                       alt=rt.get("z"),
                                       alt_sigma=rt.get("z_sigma"))
                    self._record_position(ft, rt)
                    # 注:不融合雷达航迹速度——其速度由位置量测递推而来,
                    # 机动时滞后明显,扫描显示注入后共同集RMSE 19→28m。
                    ft.touch(
                        "radar",
                        self.tick,
                        position_capable=True,
                        sensor_id=rt.get("sensor_id"),
                    )
                    if rt.get("cls"):
                        ft.add_class(rt["cls"], rt.get("cls_conf", 0.6))
                    if laser_rows[fi]:
                        ft.assoc["radar"] = dict(
                            x=rt["x"], y=rt["y"],
                            normalized_distance=round(float(ND[fi, ri]), 3),
                            gate_m=round(float(G[fi, ri]), 1),
                            laser_aided_gate=True,
                        )
                    else:
                        ft.assoc["radar"] = dict(
                            x=rt["x"], y=rt["y"],
                            nis=round(float(NIS[fi, ri]), 2), gate=self.pos_gate,
                        )
                    assigned_f.add(fi); used_r.add(ri)
        # 未匹配雷达航迹 → 新生融合航迹
        for ri, rt in enumerate(radar_tracks):
            if ri in used_r:
                continue
            R = np.array(rt["cov"])
            x = np.array([rt["x"], rt["y"], rt["vx"], rt["vy"]])
            P = np.diag([R[0, 0], R[1, 1], 50.0 ** 2, 50.0 ** 2]).astype(float)
            P[:2, :2] = R
            ft = FusedTrack(
                x,
                P,
                self.dt,
                origin="radar",
                alt=rt.get("z"),
                vz=rt.get("vz"),
                alt_sigma=rt.get("z_sigma", 15.0),
            )
            if rt.get("input_mode") == "device_tracks":
                self._device_report(ft, rt)
                self.tracks.append(ft)
                continue
            ft.touch(
                "radar",
                self.tick,
                position_capable=True,
                sensor_id=rt.get("sensor_id"),
            )
            if rt.get("cls"):
                ft.add_class(rt["cls"], rt.get("cls_conf", 0.6))
            ft.assoc["radar"] = dict(x=rt["x"], y=rt["y"], born=True)
            self._record_position(ft, rt)
            self.tracks.append(ft)

    def _device_report(self, ft, rt):
        """Use a device L2 state once. Predicted reports never add real evidence."""
        ft.radar_binding = (rt.get("sensor_id"), rt["id"])
        ft.source_track_id = rt.get("source_track_id")
        ft.last_report_tick = self.tick
        ft.report_timestamp_ms = rt.get("report_timestamp_ms")
        ft.last_measurement_ms = rt.get("last_measurement_ms")
        real = bool(rt.get("source_fresh"))
        has_other_source = any(src != "radar" and self.tick - seen[0] <= self.DELETE_HORIZON
                               for src, seen in ft.sources_seen.items())
        if not has_other_source:
            # Same-source states are correlated. Avoid a second Kalman update
            # which would incorrectly tighten uncertainty on extrapolated data.
            ft.x = np.array([rt["x"], rt["y"], rt["vx"], rt["vy"]], float)
            ft.P = np.diag([1.0, 1.0, 50.0 ** 2, 50.0 ** 2])
            ft.P[:2, :2] = np.array(rt["cov"])
            ft.cross_cov[:] = 0
            if rt.get("z") is not None:
                ft.vertical = VerticalKalmanFilter(rt["z"], self.dt,
                    altitude_sigma=rt.get("z_sigma", 15.0))
        elif real:
            ft.update_position([rt["x"], rt["y"]], np.array(rt["cov"]),
                               alt=rt.get("z"), alt_sigma=rt.get("z_sigma"))
        if real:
            self._record_position(ft, rt)
            ft.touch("radar", self.tick, position_capable=True, sensor_id=rt.get("sensor_id"))
            if rt.get("cls"):
                ft.add_class(rt["cls"], rt.get("cls_conf", 0.0))
        ft.assoc["radar"] = dict(x=rt["x"], y=rt["y"], device_report=True,
                                 source_track_id=rt.get("source_track_id"),
                                 real_measurement=real)

    # ---- 光电激光测距:由 r/az/el 直接形成位置约束 ----
    def _laser_observation(self, sid, track):
        """把带距离的航迹转换为二维位置、协方差和可选高度。

        光电/声探测2的 ``r`` 是斜距；射频 ``5%R`` 的实验近似使用
        ``range_is_horizontal`` 标记二维水平距离，不能据此虚构高度量测。
        """
        if track.get("mode") == "position":
            try:
                xy = np.array([track["x"], track["y"]], float)
                sigma = max(float(track["position_sigma_m"]), 0.01)
            except (KeyError, ValueError, TypeError):
                return None
            if not track.get("position_valid") or not np.all(np.isfinite(xy)):
                return None
            return dict(xy=xy, cov=np.eye(2)*sigma**2, alt=track.get("z"),
                        alt_sigma=track.get("altitude_sigma_m"), velocity=np.zeros(2),
                        r=None, az=track.get("az"), el=track.get("el"),
                        track=track, range_is_horizontal=False, mode="position")
        if ("mode" in track and (track["mode"] != "ranged" or not track.get("range_valid"))):
            return None
        if track.get("range_age", 0) > 0 or not self.station_known.get(sid, False):
            return None
        try:
            measured_r = float(track["r"])
            az = float(track["az"])
            el = float(track.get("el", 0.0))
        except (KeyError, TypeError, ValueError):
            return None
        if not (np.isfinite(measured_r) and measured_r > 0 and
                np.isfinite(az) and np.isfinite(el)):
            return None
        station = self.pos3.get(sid)
        if station is None:
            return None

        cos_az, sin_az = np.cos(az), np.sin(az)
        range_is_horizontal = bool(track.get("range_is_horizontal", False))
        cos_el, sin_el = np.cos(el), np.sin(el)
        horizontal_r = measured_r if range_is_horizontal else measured_r * cos_el
        xy = station[:2] + horizontal_r * np.array([cos_az, sin_az])
        alt = None if range_is_horizontal else float(
            station[2] + measured_r * sin_el
        )

        sig_r = float(track.get("sig_r", DEFAULT_LASER_SIG_R))
        sig_az = float(track.get("sig_az", self.sig.get(sid, DEF_SIG)))
        sig_el = float(track.get("sig_el", DEFAULT_LASER_SIG_EL))
        sig_r = max(abs(sig_r), 1e-3)
        sig_az = max(abs(sig_az), 1e-7)
        sig_el = max(abs(sig_el), 1e-7)
        if range_is_horizontal:
            # J=d(x,y)/d(r,az)，二维RF定位没有俯仰/高度信息。
            J = np.array([
                [cos_az, -horizontal_r * sin_az],
                [sin_az, horizontal_r * cos_az],
            ])
            cov = J @ np.diag([sig_r ** 2, sig_az ** 2]) @ J.T
            alt_sigma = None
        else:
            # J=d(x,y)/d(r,az,el)，斜距不能直接当水平距离。
            J = np.array([
                [cos_el * cos_az, -horizontal_r * sin_az,
                 -measured_r * sin_el * cos_az],
                [cos_el * sin_az, horizontal_r * cos_az,
                 -measured_r * sin_el * sin_az],
            ])
            cov = J @ np.diag([sig_r ** 2, sig_az ** 2, sig_el ** 2]) @ J.T
            alt_sigma = float(np.hypot(
                sin_el * sig_r,
                measured_r * cos_el * sig_el,
            ))
        cov += np.eye(2) * 1e-6

        range_rate = float(track.get("range_rate", 0.0) or 0.0)
        az_rate = float(track.get("rate", 0.0) or 0.0)
        horizontal_rate = range_rate if range_is_horizontal else range_rate * cos_el
        velocity = np.array([
            horizontal_rate * cos_az - horizontal_r * az_rate * sin_az,
            horizontal_rate * sin_az + horizontal_r * az_rate * cos_az,
        ])
        if not np.all(np.isfinite(velocity)) or np.linalg.norm(velocity) > 120.0:
            velocity = np.zeros(2, float)
        return dict(
            xy=xy, cov=cov, alt=alt, alt_sigma=alt_sigma, velocity=velocity,
            r=measured_r, az=az, el=el, track=track,
            range_is_horizontal=range_is_horizontal,
        )

    def _ranged_step(self, bearing_tracks):
        """处理带距离的测向航迹，返回仍需走纯方位流程的航迹。"""
        bearing_only = {}
        ranged_sids = set()
        for sid, tracks in bearing_tracks.items():
            observations = []
            plain = []
            for track in tracks:
                # BearingTracker会短期导出漏测后的预测轨。融合层已有自己的预测，
                # 若再次把该预测当成激光/方位新量测，会重复起批并无限刷新来源。
                if not self._track_is_fresh(track):
                    continue
                observation = self._laser_observation(sid, track) if (
                    "r" in track or track.get("mode") == "position") else None
                if observation is None:
                    if track.get("az") is not None and track.get("angle_valid", True):
                        plain.append(track)
                else:
                    observations.append(observation)
            bearing_only[sid] = plain
            if not observations:
                continue
            ranged_sids.add(sid)
            kind = self.kinds.get(sid, "")
            used = set()
            if self.tracks:
                C = np.full((len(self.tracks), len(observations)), 1e6)
                G = np.zeros((len(self.tracks), len(observations)), float)
                for fi, fused_track in enumerate(self.tracks):
                    for oi, observation in enumerate(observations):
                        source_id = observation["track"].get("source_track_id", observation["track"]["id"])
                        bound = fused_track.source_bindings.get(sid)
                        if (bound is not None and bound != source_id
                                and self.tick-fused_track.sources_seen.get(sid, (-1e30,))[0] <= 4):
                            continue
                        residual = observation["xy"] - fused_track.x[:2]
                        S = fused_track.P[:2, :2] + observation["cov"]
                        gate_m = max(
                            self.LASER_POS_GATE_M,
                            3.0 * np.sqrt(max(float(np.max(np.linalg.eigvalsh(S))), 1.0)),
                        )
                        distance = float(np.linalg.norm(residual))
                        G[fi, oi] = gate_m
                        if distance < gate_m:
                            # Keep identity preference within a valid spatial gate.
                            C[fi, oi] = distance / gate_m + (0 if bound == source_id else 0.05)
                fi_idx, oi_idx = linear_sum_assignment(C)
                for fi, oi in zip(fi_idx, oi_idx):
                    if C[fi, oi] >= 1.0:
                        continue
                    fused_track = self.tracks[fi]
                    observation = observations[oi]
                    fused_track.update_position(
                        observation["xy"],
                        observation["cov"],
                        alt=observation["alt"],
                        alt_sigma=observation["alt_sigma"],
                    )
                    fused_track.touch(sid, self.tick, position_capable=True)
                    self._record_position(fused_track, observation["track"])
                    fused_track.source_bindings[sid] = observation["track"].get(
                        "source_track_id", observation["track"]["id"])
                    fused_track.assoc[sid] = dict(
                        x=float(observation["xy"][0]),
                        y=float(observation["xy"][1]),
                        r=observation["r"],
                        az=observation["az"],
                        mode=observation.get("mode", "ranged"),
                        normalized_distance=round(float(C[fi, oi]), 3),
                        gate_m=round(float(G[fi, oi]), 1),
                        laser=True,
                    )
                    source_track = observation["track"]
                    self._apply_track_evidence(
                        fused_track, source_track, kind
                    )
                    used.add(oi)

            for oi, observation in enumerate(observations):
                if oi in used:
                    continue
                source_track = observation["track"]
                source_id = source_track.get("source_track_id", source_track["id"])
                if "mode" in source_track and any(
                        t.source_bindings.get(sid) == source_id for t in self.tracks):
                    if len(self.geometry_events) < 30:
                        self.geometry_events.append(dict(devices=[sid], targets=[source_id],
                                                         reason="bound_position_rejected"))
                    continue
                x = np.array([
                    observation["xy"][0], observation["xy"][1],
                    observation["velocity"][0], observation["velocity"][1],
                ])
                P = np.diag([1.0, 1.0, 35.0 ** 2, 35.0 ** 2]).astype(float)
                P[:2, :2] = observation["cov"]
                origin = ("eo_position" if observation.get("mode") == "position"
                          else "eo_laser" if kind == "eo" else "ranged_sensor")
                fused_track = FusedTrack(
                    x,
                    P,
                    self.dt,
                    origin=origin,
                    alt=observation["alt"],
                    alt_sigma=observation["alt_sigma"] or 15.0,
                )
                fused_track.touch(sid, self.tick, position_capable=True)
                self._record_position(fused_track, source_track)
                fused_track.source_bindings[sid] = source_track.get("source_track_id", source_track["id"])
                fused_track.assoc[sid] = dict(
                    x=float(observation["xy"][0]),
                    y=float(observation["xy"][1]),
                    r=observation["r"],
                    az=observation["az"],
                    mode=observation.get("mode", "ranged"),
                    born=True,
                    laser=True,
                )
                self._apply_track_evidence(
                    fused_track, source_track, kind
                )
                self.tracks.append(fused_track)

        if ranged_sids:
            # 运行时由旧的纯测向录包切换到激光量测时，清掉这些站的历史交汇候选，
            # 避免同一目标同时按激光位置和旧pending交汇重复起批。
            self._pending = [
                candidate for candidate in self._pending
                if not (set(candidate.get("sources", ())) & ranged_sids)
            ]
        return bearing_only

    # ---- 测向关联/更新(马氏门:归一化新息,随航迹协方差自适应) ----
    def _bearing_step(self, bearing_tracks):
        """bearing_tracks: {sensor_id: [ {az, rate, cls, cls_conf}, ... ]}
        返回未被任何融合航迹消费的方位航迹(供三角交汇)。
        门限:chi2(1dof) < 9 即 3σ,且绝对角差 < 4° 兜底,
        雷达锚定好的航迹 S 小 → 门自动收紧,邻居方位进不来;
        三角新生航迹 P 大 → 门自动放宽,容忍初期误差。"""
        leftover = {}
        CHI2 = self.AZ_CHI2
        for sid, bts in bearing_tracks.items():
            if not self.station_known.get(sid, False):
                leftover[sid] = []
                continue
            bts = [b for b in bts if self._track_is_fresh(b)
                   and b.get("az") is not None and b.get("angle_valid", True)]
            sp = self.pos3[sid]
            sig = self.sig.get(sid, DEF_SIG)
            abs_cap = np.deg2rad(self.ABS_CAP_DEG)
            if self.adaptive_bearing_gates:
                abs_cap = max(abs_cap, 3.0 * abs(float(sig)))
            kind = self.kinds.get(sid, "")
            consumed = set()
            if self.tracks and bts:
                C = np.full((len(self.tracks), len(bts)), 1e6)
                # Classification ambiguity is checked BEFORE identity bindings:
                # a previous association cannot hide another same-bearing target.
                geometry_matches = np.zeros(C.shape, dtype=bool)
                for fi, ft in enumerate(self.tracks):
                    dx = ft.x[0] - sp[0]
                    dy = ft.x[1] - sp[1]
                    d2 = dx * dx + dy * dy
                    if d2 < 1.0:
                        continue
                    exp_az = np.arctan2(dy, dx)
                    H = np.array([-dy / d2, dx / d2, 0.0, 0.0])
                    S = float(H @ ft.P @ H) + sig ** 2
                    for bi, bt in enumerate(bts):
                        source_id = bt.get("source_track_id", bt["id"])
                        bound = ft.source_bindings.get(sid)
                        local_sig = max(float(bt.get("sig_az", sig)), 1e-6)
                        resid = wrap(bt["az"] - exp_az)
                        if abs(resid) > max(abs_cap, 3*local_sig):
                            continue
                        innovation = ft.bearing_innovation(
                            sp, bt["az"], local_sig, bt.get("el"), bt.get("sig_el"))
                        if innovation is None:
                            continue
                        nis = innovation[3]
                        gate = 11.83 if len(innovation[0]) == 2 else CHI2
                        if nis < gate:
                            geometry_matches[fi, bi] = True
                            if (bound is not None and bound != source_id
                                    and self.tick-ft.sources_seen.get(sid, (-1e30,))[0] <= 4):
                                continue
                            C[fi, bi] = nis/gate + (0 if bound == source_id else 0.05)
                fi_idx, bi_idx = linear_sum_assignment(C)
                for fi, bi in zip(fi_idx, bi_idx):
                    if C[fi, bi] < 1.0:
                        ft = self.tracks[fi]; bt = bts[bi]
                        # 关联诊断(用预测态,与代价矩阵同口径):角残差/新息方差/归一化新息
                        dxa = ft.x[0] - sp[0]; dya = ft.x[1] - sp[1]
                        d2a = dxa * dxa + dya * dya
                        exp_az = np.arctan2(dya, dxa)
                        Hm = np.array([-dya / d2a, dxa / d2a, 0.0, 0.0])
                        Sv = float(Hm @ ft.P @ Hm) + sig ** 2
                        resid = wrap(bt["az"] - exp_az)
                        local_sig = max(float(bt.get("sig_az", sig)), 1e-6)
                        ft.update_bearing(sp, bt["az"], local_sig, bt.get("el"), bt.get("sig_el"))
                        ft.touch(sid, self.tick)
                        ft.source_bindings[sid] = bt.get("source_track_id", bt["id"])
                        ft.bearing_seen[sid] = float(bt.get("timestamp_ms",
                            (self.time_s if self.time_s is not None else self.tick*0.5)*1000))/1000
                        ft.assoc[sid] = dict(az=bt["az"],
                                             resid_deg=round(float(np.degrees(resid)), 3),
                                             normalized_cost=round(float(C[fi, bi]), 2),
                                             mode="bearing", el=bt.get("el"),
                                             S=Sv, sig_deg=round(float(np.degrees(sig)), 3))
                        self._apply_track_evidence(ft, bt, kind)
                        if kind == "rf_df":
                            unique = (np.count_nonzero(geometry_matches[:, bi]) == 1
                                      and np.count_nonzero(geometry_matches[fi, :]) == 1)
                            ft.assoc[sid]["rf_class_status"] = self._rf_classification_step(
                                ft, bt, sid, unique)
                        consumed.add(bi)
            if kind == "rf_df":
                for bi, bt in enumerate(bts):
                    if bi not in consumed:
                        self._rf_classification_step(None, bt, sid, False)
            # 三角交汇池准入:仅"无法被任何现有航迹解释"的方位才算 leftover。
            # 仅靠"未被消费"不够——密集区里输掉匈牙利竞争的方位会涌入交汇池造鬼影。
            BLOCK = np.deg2rad(self.BLOCK_DEG)
            blocked = set()
            if self.tracks and bts:
                for bi, bt in enumerate(bts):
                    for ft in self.tracks:
                        ea = np.arctan2(ft.x[1] - sp[1], ft.x[0] - sp[0])
                        if abs(wrap(bt["az"] - ea)) < BLOCK:
                            blocked.add(bi)
                            break
            leftover[sid] = [bt for bi, bt in enumerate(bts)
                             if bi not in consumed and bi not in blocked]
        return leftover

    def _eo_geometry_step(self, bearing_tracks, leftover):
        """Timestamped two-EO geometry, independent of their packet rates.

        A pair is new evidence only after BOTH reports advance. Consumed reports
        can check observability of an existing track, but cannot also create a
        second track or supply a second position update.
        """
        now = self.time_s if self.time_s is not None else self.tick * 0.5
        unassigned = {(sid, b.get("source_track_id", b["id"]))
                      for sid, rows in leftover.items() for b in rows}
        for sid, rows in bearing_tracks.items():
            if self.kinds.get(sid) != "eo":
                continue
            for b in rows:
                if (not self._track_is_fresh(b) or not b.get("angle_valid", True)
                        or b.get("az") is None or not self.station_known.get(sid, False)):
                    continue
                key = (sid, b.get("source_track_id", b["id"]))
                stamp = float(b.get("timestamp_ms", now*1000)) / 1000
                self.bearing_cache[key] = {**b, "sid": sid, "stamp": stamp,
                                          "sig_az": float(b.get("sig_az", self.sig[sid])),
                                          "unassigned": key in unassigned}
        self.bearing_cache = {k: b for k, b in self.bearing_cache.items()
                              if 0 <= now-b["stamp"] <= 0.5 and b["sid"] in self.pos3}
        self.direction_tracks = [
            dict(sensor_id=k[0], source_track_id=k[1], az=b["az"], el=b.get("el"),
                 timestamp_ms=round(b["stamp"]*1000), age_s=now-b["stamp"],
                 fused_ids=[t.id for t in self.tracks if t.source_bindings.get(k[0]) == k[1]])
            for k, b in self.bearing_cache.items()
        ][:100]
        items = list(self.bearing_cache.items())[-100:]
        active_pairs = set()
        for i, (ka, a) in enumerate(items):
            for kb, b in items[i+1:]:
                if ka[0] == kb[0]:
                    continue
                pair = tuple(sorted((ka, kb)))
                skew = abs(a["stamp"]-b["stamp"])
                if skew > 0.3:
                    continue
                active_pairs.add(pair)
                stamps = {ka: a["stamp"], kb: b["stamp"]}
                previous = self.pair_used.get(pair, {})
                if any(stamps[k] <= previous.get(k, -1e30) for k in stamps):
                    continue
                self.pair_used[pair] = stamps
                result, reason = intersect(self.pos3[ka[0]], a, self.pos3[kb[0]], b, skew)
                if len(self.geometry_events) < 30:
                    self.geometry_events.append(dict(devices=[ka[0], kb[0]],
                                                     targets=[ka[1], kb[1]], reason=reason,
                                                     skew_s=skew))
                if result is None:
                    self.eo_pending.pop(pair, None)
                    continue
                owners = [t for t in self.tracks
                          if t.source_bindings.get(ka[0]) == ka[1]
                          and t.source_bindings.get(kb[0]) == kb[1]
                          and t.bearing_seen.get(ka[0]) == a["stamp"]
                          and t.bearing_seen.get(kb[0]) == b["stamp"]]
                if owners:
                    for t in owners:
                        # Geometric support, NOT another Kalman measurement.
                        t.last_geometry_tick = self.tick
                        t.record_position_update(max(a["stamp"], b["stamp"])*1000)
                    self.eo_pending.pop(pair, None)
                    continue
                if not a["unassigned"] or not b["unassigned"]:
                    continue
                if any(t.source_bindings.get(k[0]) == k[1]
                       for t in self.tracks for k in (ka, kb)):
                    continue
                # The scenario area is a display/demo viewport, not an EO
                # operating fence. Real stations can be far from that origin;
                # use the station-relative geometry/range gates above instead.
                p = self.eo_pending.get(pair)
                if p is None or np.linalg.norm(p["pt"]-result["pt"]) > 100:
                    p = dict(hits=0, first=now)
                p.update(result, hits=p["hits"]+1, seen=now)
                self.eo_pending[pair] = p
                if p["hits"] < self.TRI_CONFIRM or now-p["first"] < 0.5:
                    continue
                P = np.diag([1., 1., 60.**2, 60.**2])
                P[:2, :2] = p["cov"]
                ft = FusedTrack([*p["pt"], 0., 0.], P, self.dt, "eo_triangulation",
                                alt=p["alt"], alt_sigma=p["alt_sigma"] or 20.)
                if p.get("cross_xy_alt") is not None:
                    ft.cross_cov[:2, 0] = p["cross_xy_alt"]
                ft.last_geometry_tick = self.tick
                ft.record_position_update(max(a["stamp"], b["stamp"])*1000)
                for key, obs in ((ka, a), (kb, b)):
                    ft.touch(key[0], self.tick)
                    ft.source_bindings[key[0]] = key[1]
                    ft.assoc[key[0]] = dict(mode="bearing", triangulated=True, az=obs["az"])
                    self._apply_track_evidence(ft, obs, "eo")
                self.tracks.append(ft)
                self.eo_pending.pop(pair, None)
        self.pair_used = {p: v for p, v in self.pair_used.items() if p in active_pairs}
        self.eo_pending = {p: v for p, v in self.eo_pending.items()
                           if now-v["seen"] <= 1.0}

    # ---- 三角交汇起批(持续性确认) ----
    def _triangulate_birth(self, leftover):
        cand = []
        keys = list(leftover.keys())
        for i in range(len(keys)):
            for j in range(i + 1, len(keys)):
                s1, s2 = keys[i], keys[j]
                if self.kinds.get(s1) == "eo" and self.kinds.get(s2) == "eo":
                    continue  # EO pairs use the timestamped geometry path above.
                if np.linalg.norm(self.pos[s1] - self.pos[s2]) < 200:
                    continue   # 同址,无基线
                for b1 in leftover[s1]:
                    for b2 in leftover[s2]:
                        res = triangulate(self.pos[s1], b1["az"], self.pos[s2], b2["az"],
                                          self.sig.get(s1, DEF_SIG), self.sig.get(s2, DEF_SIG))
                        if res is None:
                            continue
                        pt, cov = res
                        if not (
                            self.area["xmin"] < pt[0] < self.area["xmax"]
                            and self.area["ymin"] < pt[1] < self.area["ymax"]
                        ):
                            continue
                        # 不能落在已有融合航迹附近(避免重复)
                        if any(np.linalg.norm(ft.pos() - pt) < 160 for ft in self.tracks):
                            continue
                        rf_track = None
                        if self.kinds.get(s1) == "rf_df":
                            rf_track = b1
                        elif self.kinds.get(s2) == "rf_df":
                            rf_track = b2
                        cand.append(dict(
                            pt=pt,
                            cov=cov,
                            sources={s1, s2},
                            cls=b1.get("cls") or b2.get("cls"),
                            rf_metadata=dict(rf_track) if rf_track else None,
                        ))
        # 净化:已能被现有融合航迹解释的 pending 直接清除。
        # 启动期雷达航迹尚未确认时,全蜂群方位会涌入交汇池形成候选洪峰;
        # 雷达航迹一旦确认,这些候选即被解释,必须清掉——否则缓冲截尾会把
        # 真正需要三角起批的候选(如静默低RCS独狼)从队首挤掉,令其饿死。
        self._pending = [p for p in self._pending
                         if not any(np.linalg.norm(ft.pos() - p["pt"]) < 160
                                    for ft in self.tracks)]
        # 候选与 pending 缓冲做就近匹配,累计命中帧数
        for p in self._pending:
            p["seen"] = False
        for c in cand:
            matched = None
            for p in self._pending:
                if np.linalg.norm(p["pt"] - c["pt"]) < 180:
                    matched = p; break
            if matched:
                # 每拍最多 +1 命中:同帧多个交叉候选落进同一 pending 不得重复加分
                if not matched["seen"]:
                    matched["pt"] = 0.5 * matched["pt"] + 0.5 * c["pt"]
                    matched["cov"] = c["cov"]
                    matched["hits"] += 1
                    matched["seen"] = True
                matched["sources"] |= c["sources"]
                matched["cls"] = matched["cls"] or c["cls"]
                if c.get("rf_metadata"):
                    matched["rf_metadata"] = c["rf_metadata"]
            else:
                c.update(hits=1, seen=True, stale=0)
                self._pending.append(c)
        # 起批:累计命中 >=8 帧(0.8s 持续性)的候选;长期未复现的候选老化删除。
        # 门槛设 8 让雷达锚航迹先建立(~0.3-0.5s 确认)并通过遮挡规则吸收
        # 蜂群方位,启动期的交叉鬼影洪峰便不会成批起批;而真正无锚的目标
        # (静默低RCS)~1.2s 起批,仍远早于雷达对其的偶发确认。
        born = []
        nxt = []
        for p in self._pending:
            if p["hits"] >= self.TRI_CONFIRM:
                x = np.array([p["pt"][0], p["pt"][1], 0.0, 0.0])
                P = np.diag([1.0, 1.0, 60.0 ** 2, 60.0 ** 2]).astype(float)
                P[:2, :2] = p["cov"]
                ft = FusedTrack(x, P, self.dt, origin="triangulation")
                self._record_position(ft, {})
                for s in p["sources"]:
                    ft.touch(s, self.tick)
                    ft.assoc[s] = dict(az=float(np.arctan2(p["pt"][1] - self.pos[s][1],
                                                           p["pt"][0] - self.pos[s][0])))
                if p["cls"]:
                    ft.add_class(p["cls"], 0.6)
                if p.get("rf_metadata"):
                    ft.rf_id = p["rf_metadata"].get("cls")
                    ft.update_rf_metadata(p["rf_metadata"], self.tick)
                self.tracks.append(ft)
                born.append(ft.id)
                continue
            p["stale"] = 0 if p.pop("seen", False) else p.get("stale", 0) + 1
            if p["stale"] <= 30:
                nxt.append(p)
        self._pending = nxt[-120:]
        return born

    # ---- 管理:合并 + 删除 ----
    def _manage(self):
        # 合并过近航迹
        merged = True
        while merged:
            merged = False
            for i in range(len(self.tracks)):
                for j in range(i + 1, len(self.tracks)):
                    a, b = self.tracks[i], self.tracks[j]
                    recent_laser = any(
                        source != "radar" and source in track.position_sources and
                    self.tick - track.position_seen.get(source, -999)
                        <= self.DELETE_HORIZON
                        for track in (a, b) for source in track.position_sources
                    )
                    # 仅在两条航迹都有真实高度时使用三维距离合并。高度缺失时
                    # 不回退二维合并，避免把同一水平位置、不同高度层的真实目标
                    # 压成一条。近期有激光位置支撑的航迹继续禁止自动近距合并。
                    pos3_a, pos3_b = a.pos3(), b.pos3()
                    merge_distance = (
                        0.0 if recent_laser else self.MERGE_DIST_3D
                    )
                    if (
                        not any(s in b.source_bindings and ident != b.source_bindings[s]
                                for s, ident in a.source_bindings.items()) and
                        not (a.radar_binding is not None and b.radar_binding is not None
                             and a.radar_binding != b.radar_binding) and
                        pos3_a is not None and
                        pos3_b is not None and
                        np.linalg.norm(pos3_a - pos3_b) < merge_distance
                    ):
                        keep, drop = (a, b) if a.conf >= b.conf else (b, a)
                        for s, v in drop.sources_seen.items():
                            if s not in keep.sources_seen or v[0] > keep.sources_seen[s][0]:
                                keep.sources_seen[s] = v
                        for sid, seen_tick in drop.sensor_ids_seen.items():
                            keep.sensor_ids_seen[sid] = max(
                                seen_tick, keep.sensor_ids_seen.get(sid, -1)
                            )
                        keep.position_sources |= drop.position_sources
                        # Duplicate histories must not inflate valid point counts.
                        keep.position_update_count = max(keep.position_update_count, drop.position_update_count)
                        stamps = [v for v in (keep.last_position_update_ms, drop.last_position_update_ms) if v is not None]
                        keep.last_position_update_ms = max(stamps) if stamps else None
                        births = [v for v in (keep.created_at_s, drop.created_at_s) if v is not None]
                        keep.created_at_s = min(births) if births else None
                        for source, seen in drop.position_seen.items():
                            keep.position_seen[source] = max(keep.position_seen.get(source, -1e30), seen)
                        keep.source_bindings.update(drop.source_bindings)
                        keep.last_tick = max(keep.last_tick, drop.last_tick)
                        if drop.class_updates:
                            for cls, score in drop.cls_logp.items():
                                keep.cls_logp[cls] += score
                            keep.class_updates += drop.class_updates
                        if keep.class_updates:
                            keep.rf_class_evidence = None
                        elif drop.rf_class_evidence and (not keep.rf_class_evidence or
                                drop.rf_class_evidence["timestamp_ms"] > keep.rf_class_evidence["timestamp_ms"]):
                            keep.rf_class_evidence = dict(drop.rf_class_evidence)
                        if keep.rf_id is None:
                            keep.rf_id = drop.rf_id
                        if drop.rf_metadata:
                            keep.update_rf_metadata(
                                {
                                    **drop.rf_metadata,
                                    "rf_confidence": drop.rf_metadata_confidence,
                                },
                                drop.rf_metadata_tick,
                            )
                        self.tracks.remove(drop)
                        merged = True
                        break
                if merged:
                    break
        # 删除:长时间无更新,或仅剩单一且不可独立定位的纯测向源支撑。
        # 单条视线不可定位(距离不可观测)——交叉鬼影会寄生在某条方位线上,
        # 靠该方位持续刷新 last_tick 长期存活,必须按可观测性原则清除。
        def _supported(t):
            if t.last_report_tick is not None and self.tick - t.last_report_tick <= self.DELETE_HORIZON:
                return True
            recent = [s for s, (tk, _) in t.sources_seen.items()
                      if self.tick - tk <= self.DELETE_HORIZON]
            has_position_source = any(self.tick-seen <= self.DELETE_HORIZON
                                      for seen in t.position_seen.values())
            geometry_recent = (t.last_geometry_tick is not None
                               and self.tick-t.last_geometry_tick <= self.DELETE_HORIZON)
            # Preserve non-EO legacy triangulation behaviour; two EO names alone
            # are no longer proof that range is observable.
            legacy_multi = t.origin == "triangulation" and len(recent) >= 2
            return has_position_source or geometry_recent or legacy_multi
        self.tracks = [t for t in self.tracks
                       if self.tick - max(t.last_tick, t.last_report_tick or t.last_tick) <= self.DELETE_HORIZON and _supported(t)]

    def step(self, radar_tracks, bearing_tracks, spectrum):
        self.geometry_events = []
        # Measurement-time age for live/replay; accumulated dt for the demo.
        if self.time_s is not None and np.isfinite(float(self.time_s)):
            self._track_clock_s = max(self._track_clock_s, float(self.time_s))
        elif np.isfinite(float(self.dt)):
            self._track_clock_s += max(0.0, float(self.dt))
        if self.time_s is None:
            self.tick += 1
        else:
            # Legacy weights are calibrated to a 0.5s reference tick. Advance
            # them by elapsed time, independent of packet/output frequency.
            self.tick = self.time_s / 0.5
            self.DELETE_HORIZON = 24  # 12 seconds / four nominal scan periods.
        now_ms = (self.time_s if self.time_s is not None else self.tick * .5) * 1000
        for track in self.tracks:
            if track.rf_class_evidence and now_ms > track.rf_class_evidence["expires_at_ms"]:
                track.rf_class_evidence = None
        self.rf_class_candidates = {k: v for k, v in self.rf_class_candidates.items()
                                    if 0 <= now_ms - v["timestamp_ms"] <= 60000}
        # Bound bookkeeping even if a faulty device floods new target identifiers.
        if len(self.rf_class_candidates) > 10000:
            self.rf_class_candidates = dict(list(self.rf_class_candidates.items())[-10000:])
        self._radar_step(radar_tracks)
        bearing_only = self._ranged_step(bearing_tracks)
        leftover = self._bearing_step(bearing_only)
        self._eo_geometry_step(bearing_only, leftover)
        self._triangulate_birth(leftover)
        for t in self.tracks:
            if t.created_at_s is None:
                t.created_at_s = self._track_clock_s
            t.update_confidence(self.tick, self.weight)
            if not t.sources_seen and t.last_report_tick is not None:
                t.conf = 0.15 * (0.85 ** max(0, self.tick - t.last_report_tick))
        self._manage()
        event_time = round(self.time_s * 1000) if self.time_s is not None else None
        self.geometry_history.extend(dict(e, timestamp_ms=event_time) for e in self.geometry_events)

        # 频谱交叉校验:已定位的"发射型"目标数
        localized_emit = sum(1 for t in self.tracks
                             if ("rf_df" in t.active_sources(self.tick)) or t.rf_id)
        spec_n = spectrum.get("emitters", 0)
        unlocalized_emit = max(0, spec_n - localized_emit)

        return dict(spec_emitters=spec_n, localized_emit=localized_emit,
                    unlocalized_emit=unlocalized_emit)

    def export(self, *, include_coasting=False):
        out = []
        for t in self.tracks:
            if not include_coasting and t.conf < self.min_confidence:
                continue
            row = dict(
                id=int(t.id), x=float(t.x[0]), y=float(t.x[1]),
                vx=float(t.x[2]), vy=float(t.x[3]),
                cov=[[float(t.P[0, 0]), float(t.P[0, 1])],
                     [float(t.P[1, 0]), float(t.P[1, 1])]],
                cls=t.best_class(), rf_id=t.rf_id, conf=round(float(t.conf), 3),
                position_update_count=t.position_update_count,
                last_position_update_ms=t.last_position_update_ms,
                track_age_s=(None if t.created_at_s is None else
                             round(max(0.0, self._track_clock_s-t.created_at_s), 6)),
                class_evidence=bool(t.class_updates or t.rf_class_evidence),
                class_basis=("sensor_class" if t.class_updates else
                             "rf_model" if t.rf_class_evidence else "none"),
                origin=t.origin, sources=t.active_sources(self.tick),
                sensor_ids=t.active_sensor_ids(self.tick),
                age_ticks=max(0, int(self.tick - t.last_tick)),
                position_sources=sorted(t.position_sources),
                position_age_s=(None if not t.position_seen else
                                max(0.0, (self.tick-max(t.position_seen.values()))*0.5)),
                localization_mode=("position" if any(self.tick == v for v in t.position_seen.values())
                                   else "bearing_geometry" if t.last_geometry_tick == self.tick
                                   else "predicted_position"),
                source_bindings=dict(t.source_bindings),
                geometry_age_s=(None if t.last_geometry_tick is None else
                                max(0.0, (self.tick-t.last_geometry_tick)*0.5)),
                assoc=t.assoc,
                conf_parts={k: round(float(v), 3) for k, v in t.conf_parts.items()},
            )
            if t.radar_binding is not None:
                row.update(source_track_id=t.source_track_id,
                    age_ticks=max(0, int(self.tick - (t.last_tick if t.sources_seen else t.last_report_tick))),
                    report_timestamp_ms=t.report_timestamp_ms,
                    last_measurement_ms=t.last_measurement_ms,
                    input_mode="device_tracks",
                    is_coasting=self.tick > t.last_tick or not t.sources_seen,
                    state_timestamp_ms=round(self.time_s * 1000) if self.time_s is not None else None)
                if t.last_report_tick is not None and self.tick - t.last_report_tick <= self.DELETE_HORIZON:
                    row["sources"] = sorted(set(row["sources"]) | {"radar"})
                    if t.radar_binding[0]:
                        row["sensor_ids"] = sorted(set(row["sensor_ids"]) | {t.radar_binding[0]})
            if t.alt is not None and np.isfinite(float(t.alt)):
                row["z"] = float(t.alt)
            if t.vz is not None and np.isfinite(float(t.vz)):
                row["vz"] = float(t.vz)
            row.update(t.rf_metadata)
            if t.rf_class_evidence:
                row["rf_classification"] = dict(t.rf_class_evidence)
            out.append(row)
        return out

    def export_pending(self):
        """三角交汇候选(尚未起批),供技术面板展示"持续确认 hits/N"的过程。"""
        out = []
        for p in self._pending:
            cov = p.get("cov")
            out.append(dict(
                x=float(p["pt"][0]), y=float(p["pt"][1]),
                hits=int(p["hits"]), need=int(self.TRI_CONFIRM),
                sources=sorted(p["sources"]),
                cov=[[float(cov[0, 0]), float(cov[0, 1])],
                     [float(cov[1, 0]), float(cov[1, 1])]] if cov is not None else None,
            ))
        return out

    def params(self):
        """关联/起批门限常量(供技术面板标注口径)。"""
        return dict(
            pos_gate=float(self.pos_gate), az_chi2=float(self.AZ_CHI2),
            output_min_confidence=self.min_confidence,
            laser_pos_gate_m=float(self.LASER_POS_GATE_M),
            az_abs_cap_deg=float(self.ABS_CAP_DEG), block_deg=float(self.BLOCK_DEG),
            adaptive_bearing_gates=bool(self.adaptive_bearing_gates),
            tri_confirm=int(self.TRI_CONFIRM),
            radar_alt_assoc_scale_m=float(self.RADAR_ALT_ASSOC_SCALE_M),
            merge_dist=float(self.MERGE_DIST_3D),
            merge_distance_dimension=3,
            merge_requires_altitude=True,
            delete_horizon=int(self.DELETE_HORIZON), conf_decay=0.85,
        )


# ----------------------------- 指标评估 -----------------------------
def _match_to_truth(points, truth, gate=200.0):
    """points: [(id, x, y)] → 与真值就近一一匹配。
    返回 {真值id: 定位误差};命中数 = len(返回值)。"""
    if not points or not truth:
        return {}
    T = np.array([[t["x"], t["y"]] for t in truth])
    P = np.array([[p[1], p[2]] for p in points])
    C = np.linalg.norm(P[:, None, :] - T[None, :, :], axis=2)
    C[C > gate] = 1e6
    ri, ci = linear_sum_assignment(C)
    err = {}
    for r, c in zip(ri, ci):
        if C[r, c] < gate:
            err[int(truth[c]["id"])] = float(C[r, c])
    return err


def evaluate(truth, radar_tracks, bearing_tracks, fused, sensors, fusion, eo_fused=None):
    n = len(truth)
    if n == 0:
        return dict(
            det_rate=dict(radar=0.0, eo=0.0, rf=0.0, fusion=0.0),
            rmse=dict(radar=None, eo=None, rf=None, fusion=None),
            rmse_common=dict(n=0, radar=None, fusion=None),
            track_count=dict(
                radar=len(radar_tracks),
                eo=len(eo_fused or []),
                fusion=len(fused),
                truth=0,
            ),
            gain=dict(
                det_vs_best_single=None,
                extra_targets=None,
                rmse_improve_pct=None,
            ),
            showcase=dict(
                truth_id=None,
                held_by_fusion=False,
                held_by_radar=False,
                held_by_eo=False,
            ),
        )

    # 融合
    f_err = _match_to_truth([(f["id"], f["x"], f["y"]) for f in fused], truth)

    # 雷达单独
    r_err = _match_to_truth([(r["id"], r["x"], r["y"]) for r in radar_tracks], truth)

    # 光电单独(双站):同一套融合管线、仅喂双光电角度+激光距离航迹；
    # 旧录包缺少 r 时才回退纯方位三角交汇。eo_fused 即该消融管线的 export()。
    if eo_fused is None:
        eo_fused = []
    e_err = _match_to_truth([(f["id"], f["x"], f["y"]) for f in eo_fused], truth)
    eo_cov_ids = set(e_err)
    eo_cov = len(e_err)

    # 射频单独:仅检测/识别(单站不可定位)。命中按"被覆盖的真值发射目标"去重,
    # 分母统一为全部目标 → 体现"对静默目标天然盲"这一上限。
    rf_sids = [sid for sid, s in sensors.items() if s.kind == "rf_df"]
    rf_ids = set()
    for sid in rf_sids:
        rf_pos = fusion.pos.get(sid)
        if rf_pos is None:
            continue
        for bt in bearing_tracks.get(sid, []):
            best, bid = 1e9, None
            for t in truth:
                if not t["emitting"]:
                    continue
                taz = np.arctan2(t["y"] - rf_pos[1], t["x"] - rf_pos[0])
                d = abs(wrap(bt["az"] - taz))
                if d < best:
                    best, bid = d, t["id"]
            if best < np.deg2rad(2.0):
                rf_ids.add(bid)
    rf_det = len(rf_ids) / max(n, 1)

    def rmse(es):
        es = list(es)
        return round(float(np.sqrt(np.mean(np.square(es)))), 1) if es else None

    det = dict(radar=round(len(r_err) / n, 3), eo=round(eo_cov / n, 3),
               rf=round(rf_det, 3), fusion=round(len(f_err) / n, 3))
    rms = dict(radar=rmse(r_err.values()), eo=rmse(e_err.values()), rf=None,
               fusion=rmse(f_err.values()))
    counts = dict(radar=len(radar_tracks), eo=eo_cov, fusion=len(fused), truth=n)

    # 公平口径:仅在"雷达与融合共同捕获"的目标集上比定位精度
    # (各方法各自的 rmse 分母是不同目标集,直接比是苹果比橘子)
    common = sorted(set(f_err) & set(r_err))
    rms_common = dict(
        n=len(common),
        radar=rmse([r_err[i] for i in common]),
        fusion=rmse([f_err[i] for i in common]),
    )

    best_single_det = max(det["radar"], det["eo"])
    gain = dict(
        det_vs_best_single=round(det["fusion"] - best_single_det, 3),
        extra_targets=len(set(f_err) - set(r_err)),  # 融合比雷达多捕获的目标数
        rmse_improve_pct=(round((rms_common["radar"] - rms_common["fusion"])
                                / rms_common["radar"] * 100, 1)
                          if rms_common["radar"] not in (None, 0)
                          and rms_common["fusion"] is not None else None),
    )

    # 重点展示目标:低RCS+静默 的独立侧翼渗透者
    show_id = next((t["id"] for t in truth if t.get("showcase")), None)
    show = dict(
        truth_id=show_id,
        held_by_fusion=show_id in f_err,
        held_by_radar=show_id in r_err,
        held_by_eo=show_id in eo_cov_ids,
    )
    if show_id in f_err:
        st = next(t for t in truth if t["id"] == show_id)
        ft = min(fused, key=lambda f: (f["x"] - st["x"]) ** 2 + (f["y"] - st["y"]) ** 2)
        show["fused_origin"] = ft["origin"]
        show["fused_sources"] = ft["sources"]
        show["fused_id"] = ft["id"]

    return dict(det_rate=det, rmse=rms, rmse_common=rms_common,
                track_count=counts, gain=gain, showcase=show)
