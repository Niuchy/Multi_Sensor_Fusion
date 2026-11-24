"""Official DDS publication policy only; never removes a fusion track.

The fusion process is the sole owner of live settings and persistence. All
commands are compare-and-swap against its instance/version, including saves.
"""
import copy
import fcntl
import hashlib
import math
import os
from pathlib import Path
import re
import stat
import tempfile
import time
import uuid

import yaml


DEFAULTS = dict(enabled=True, points_enabled=True, min_points=2,
                min_speed_enabled=True, min_speed_mps=1.0,
                max_speed_enabled=True, max_speed_mps=15.0,
                height_enabled=True, min_height_m=30.0,
                reference_radar_id="radar_3152")
BOOL_KEYS = ("enabled", "points_enabled", "min_speed_enabled", "max_speed_enabled", "height_enabled")
INITIAL_FILTER_WINDOW_S = 9.0
HEADER = """# DDS 输出过滤；仅影响正式 FusionMultiTarget，内部航迹和 WebSocket 保留。
# 点数=不同观测时刻的有效定位更新数，预测/重复/单站纯角度不计数。
# 速度=m/s，水平融合速度；高度=m，相对 reference_radar_id 的生效站址。
# 点数、速度上下限、相对高度均仅在融合航迹建立后 [0,9) 秒生效，满9秒不再拦截。
# enabled 为总开关；points_enabled/min_speed_enabled/max_speed_enabled/height_enabled 独立控制各项。
# true=开启该项过滤，false=关闭该项过滤；关闭仍保留阈值，网页“保存配置并应用”同步写入。
# 等于边界允许；启用的判据缺少有效数值时拦截。网页临时应用不自动写盘。
"""


def finite(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def validate(settings):
    if not isinstance(settings, dict) or set(settings) != set(DEFAULTS):
        raise ValueError("配置字段不完整或含未知字段")
    result = dict(settings)
    for key in BOOL_KEYS:
        if type(result[key]) is not bool:
            raise ValueError(f"{key} 必须为 true/false")
    if type(result["min_points"]) is not int or not 1 <= result["min_points"] <= 10000:
        raise ValueError("最少点数必须为 1～10000 的整数")
    for key, maximum in (("min_speed_mps", 1000), ("max_speed_mps", 1000), ("min_height_m", 30000)):
        if not isinstance(result[key], (int, float)) or finite(result[key]) is None or not 0 <= result[key] <= maximum:
            raise ValueError(f"{key} 必须为 0～{maximum} 的有限数值")
        result[key] = float(result[key])
    if result["min_speed_enabled"] and result["max_speed_enabled"] and result["min_speed_mps"] > result["max_speed_mps"]:
        raise ValueError("同时开启速度上下限时，最低速度不能大于最高速度")
    if not isinstance(result["reference_radar_id"], str) or not re.fullmatch(r"radar(?:_[A-Za-z0-9-]+)?", result["reference_radar_id"]):
        raise ValueError("雷达设备标识无效，例如 radar_3152")
    return result


class ConfigStore:
    def __init__(self, path=None):
        self.path = Path(path or os.environ.get("SB_OUTPUT_FILTER_CONFIG") or
                         Path(__file__).resolve().parents[1] / "configs/output_filter.yaml").resolve()

    def read(self):
        # Missing or malformed configuration fails startup, never silently disables filtering.
        data = self.path.read_bytes()
        parsed = yaml.safe_load(data)
        if not isinstance(parsed, dict):
            raise ValueError("输出过滤配置必须为 YAML 对象")
        settings = validate({**DEFAULTS, **parsed})
        return settings, hashlib.sha256(data).hexdigest()

    def save(self, settings, expected_hash):
        settings = validate(settings)
        with self.path.with_name(self.path.name + ".lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            old = self.path.read_bytes()
            if hashlib.sha256(old).hexdigest() != expected_hash:
                raise ValueError("配置文件已被其他操作修改，请刷新后重新确认")
            backup = self.path.with_name(self.path.name + ".bak-" + time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
            with backup.open("xb") as stream:
                stream.write(old)
                stream.flush()
                os.fsync(stream.fileno())
            encoded = (HEADER + yaml.safe_dump(settings, sort_keys=False, allow_unicode=True)).encode("utf-8")
            fd, tmp = tempfile.mkstemp(prefix=".output-filter-", dir=self.path.parent)
            try:
                with os.fdopen(fd, "wb") as stream:
                    os.fchmod(stream.fileno(), stat.S_IMODE(self.path.stat().st_mode))
                    stream.write(encoded)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(tmp, self.path)
            finally:
                if os.path.exists(tmp):
                    os.unlink(tmp)
            return settings, hashlib.sha256(encoded).hexdigest(), str(backup)


class OutputFilter:
    def __init__(self, path=None):
        self.store = ConfigStore(path)
        self.settings, self.saved_hash = self.store.read()
        self.saved_settings = dict(self.settings)
        self.instance_id = uuid.uuid4().hex
        self.revision = 0
        self.last_command = None
        self.reference = None
        self.last_decisions = []
        self.last_batch = None
        self.config_error = None

    def snapshot(self):
        return copy.deepcopy(dict(instance_id=self.instance_id, revision=self.revision,
            settings=self.settings, saved_settings=self.saved_settings, saved_hash=self.saved_hash,
            config_path=str(self.store.path), config_error=self.config_error,
            unsaved=self.settings != self.saved_settings, last_command=self.last_command,
            reference=self.reference, batch=self.last_batch, decisions=self.last_decisions[:100],
            filter_window_s=INITIAL_FILTER_WINDOW_S, filter_window_policy="track_initial",
            decision_limit=100))

    def refresh_saved(self):
        try:
            self.saved_settings, self.saved_hash = self.store.read()
            self.config_error = None
        except (OSError, ValueError, yaml.YAMLError) as exc:
            self.config_error = str(exc)

    def command(self, cmd):
        if cmd.get("cmd") != "output_filter_apply":
            return False
        request_id = cmd.get("request_id")
        if self.last_command and self.last_command["request_id"] == request_id:
            return True  # Retransmission is acknowledged, never saved twice.
        result = dict(request_id=request_id, ok=False, saved=False, at=time.time())
        try:
            if not isinstance(request_id, str) or not re.fullmatch(r"[a-f0-9]{32}", request_id):
                raise ValueError("请求标识无效")
            expiry = finite(cmd.get("expires_at"))
            if expiry is None or not 0 <= expiry-time.time() <= 15:
                raise ValueError("请求已过期，请刷新后重试")
            if cmd.get("instance_id") != self.instance_id:
                raise ValueError("融合进程已重启，请刷新参数")
            if type(cmd.get("expected_revision")) is not int or cmd["expected_revision"] != self.revision:
                raise ValueError("运行参数已被其他操作修改，请刷新后重新确认")
            if type(cmd.get("save")) is not bool:
                raise ValueError("save 必须为 true/false")
            settings = validate(cmd.get("settings"))
            if cmd["save"]:
                saved, digest, backup = self.store.save(settings, cmd.get("expected_saved_hash"))
                self.saved_settings, self.saved_hash = saved, digest
                self.config_error = None
                result.update(saved=True, backup_path=backup)
            # Failed validation/write must leave the live policy unchanged.
            self.settings = settings
            self.revision += 1
            result.update(ok=True, revision=self.revision)
        except (OSError, ValueError, TypeError, yaml.YAMLError) as exc:
            result["error"] = str(exc)
        self.last_command = result
        return True

    def select(self, all_tracks, eligible_tracks, sensors_info, timestamp_ms):
        s = self.settings
        sid = s["reference_radar_id"]
        station = sensors_info.get(sid, {})
        pos = station.get("pos", [])
        station_z = finite(pos[2]) if isinstance(pos, (list, tuple)) and len(pos) >= 3 else None
        if station.get("kind") != "radar" or station.get("station_known") is not True:
            station_z = None
        self.reference = dict(sensor_id=sid, known=station_z is not None,
                              station_z_enu_m=station_z, station_lla=station.get("station_lla"),
                              station_source=station.get("station_source"),
                              station_config=station.get("station_config"))
        eligible_ids = {r["id"] for r in eligible_tracks}
        accepted, decisions, reason_counts = [], [], {}
        for row in all_tracks:
            reasons = []
            age = finite(row.get("track_age_s"))
            if age is not None and age < 0:
                age = None
            # A missing/invalid age must not silently bypass all checks.
            within_window = age is None or age < INITIAL_FILTER_WINDOW_S
            phase = ("disabled" if not s["enabled"] else "age_unknown" if age is None
                     else "initial" if within_window else "complete")
            points = row.get("position_update_count")
            vx, vy, z = (finite(row.get(k)) for k in ("vx", "vy", "z"))
            speed = math.hypot(vx, vy) if vx is not None and vy is not None else None
            height = z - station_z if z is not None and station_z is not None else None
            if row["id"] not in eligible_ids:
                reasons.append("existing_output_gate")
            if s["enabled"] and within_window:
                if s["points_enabled"]:
                    if type(points) is not int or points < 0:
                        reasons.append("points_unavailable")
                    elif points < s["min_points"]:
                        reasons.append("too_few_points")
                if s["min_speed_enabled"] or s["max_speed_enabled"]:
                    if speed is None:
                        reasons.append("speed_unavailable")
                    else:
                        if s["min_speed_enabled"] and speed < s["min_speed_mps"]:
                            reasons.append("speed_below_min")
                        if s["max_speed_enabled"] and speed > s["max_speed_mps"]:
                            reasons.append("speed_above_max")
                if s["height_enabled"]:
                    if station_z is None:
                        reasons.append("radar_station_unavailable")
                    if z is None:
                        reasons.append("height_unavailable")
                    if height is not None and height < s["min_height_m"]:
                        reasons.append("height_below_min")
            if not reasons and len(accepted) >= 255:
                reasons.append("dds_capacity")  # Existing protocol writer has this limit.
            passed = not reasons
            if passed:
                accepted.append(row)
            for reason in reasons:
                reason_counts[reason] = reason_counts.get(reason, 0) + 1
            decisions.append(dict(id=row["id"], passed=passed, reasons=reasons,
                track_age_s=age, filter_phase=phase,
                initial_filter_active=bool(s["enabled"] and within_window),
                points=points, speed_mps=speed, height_above_radar_m=height,
                cls=row.get("cls"), x=row.get("x"), y=row.get("y"), is_coasting=row.get("is_coasting", False)))
        self.last_decisions = decisions
        self.last_batch = dict(timestamp_ms=timestamp_ms, revision=self.revision,
            total=len(all_tracks), eligible=len(eligible_tracks), passed=len(accepted),
            blocked=len(all_tracks)-len(accepted), reasons=reason_counts,
            publish_ok=False, publish_error=None)
        return accepted

    def published(self, error=None):
        if self.last_batch is not None:
            self.last_batch.update(publish_ok=error is None, publish_error=error,
                                   published_at=time.time())
