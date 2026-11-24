"""Quality-aware EO observations. No DDS I/O and no guessed installation sites."""
from pathlib import Path
import math
import os

import yaml
from device_station import select_station


DEFAULTS = {
    "station_enu": None, "station_lla": None,
    "use_reported_station": True,
    "angles_calibrated": False,
    "azimuth_convention": "north_cw",
    "azimuth_offset_deg": 0.0, "elevation_offset_deg": 0.0,
    "time_offset_ms": 0,
    "position_enabled": True, "altitude_enabled": False,
    # Positive distance alone is NOT proof that the laser succeeded.
    "range_policy": "disabled",
    "position_sigma_m": 30.0, "altitude_sigma_m": 20.0,
    "azimuth_sigma_deg": 1.0, "elevation_sigma_deg": 1.0,
    "elevation_enabled": False,
    "max_unchanged_s": 3.5, "min_update_interval_ms": 100,
    "max_position_speed_mps": 120.0,
    "max_range_m": 30000.0,
}


def finite(value):
    try:
        value = float(value)
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


class EOObservations:
    """Per-device configuration and bounded, per-target freshness bookkeeping.

    Unchanged fields are *suspected stale*, not diagnosed laser failures.
    Parameters are loaded once at process start; the alignment preview does not
    silently change production fusion.
    """

    def __init__(self, converter, root, config_path=None):
        self.converter = converter
        self.path = Path(config_path or os.environ.get(
            "SB_EO_CONFIG", str(Path(root) / "configs/eo_devices.yaml")))
        data = {}
        if self.path.is_file():
            with self.path.open(encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        self.defaults = {**DEFAULTS, **data.get("defaults", {})}
        self.devices = {str(k): v for k, v in data.get("devices", {}).items()}
        self.history = {}
        self.reorder_ms = int(data.get("reorder_window_ms", 200))
        if not 0 <= self.reorder_ms <= 2000:
            raise ValueError("EO reorder_window_ms must be between 0 and 2000")
        for key in [None, *self.devices]:
            p = self.profile(key)
            for name in ("use_reported_station", "angles_calibrated", "position_enabled",
                         "altitude_enabled", "elevation_enabled"):
                if not isinstance(p[name], bool):
                    raise ValueError(f"EO {name} must be YAML true/false, not a string")
            for name in ("azimuth_offset_deg", "elevation_offset_deg"):
                if finite(p[name]) is None:
                    raise ValueError(f"Invalid EO {name}")
            if not 0 <= int(p["min_update_interval_ms"]) <= 10000:
                raise ValueError("Invalid EO min_update_interval_ms")
            if p["azimuth_convention"] not in {"north_cw", "enu"}:
                raise ValueError("EO azimuth_convention must be north_cw or enu")
            if p["range_policy"] not in {"disabled", "reported_positive"}:
                raise ValueError("EO range_policy must be disabled or reported_positive")
            for name in ("position_sigma_m", "altitude_sigma_m",
                         "azimuth_sigma_deg", "elevation_sigma_deg",
                         "max_unchanged_s", "max_range_m", "max_position_speed_mps"):
                if finite(p[name]) is None or float(p[name]) <= 0:
                    raise ValueError(f"Invalid EO {name}")
            if finite(p["time_offset_ms"]) is None:
                raise ValueError("Invalid EO time_offset_ms")
            for name in ("station_enu", "station_lla"):
                value = p[name]
                if value is not None and (len(value) != 3 or any(finite(x) is None for x in value)):
                    raise ValueError(f"Invalid EO {name}")
            if p["station_enu"] is not None and p["station_lla"] is not None:
                raise ValueError("Configure one EO station coordinate representation")

    def profile(self, device_id):
        return {**self.defaults, **self.devices.get(str(device_id), {})}

    def station(self, obj, profile, fallback_enu=None, fallback_label=None):
        enu, lla = profile["station_enu"], profile["station_lla"]
        label = str(self.path.resolve())
        if enu is None and lla is None:
            enu, label = fallback_enu, fallback_label
        else:
            device = self.devices.get(str(obj.device_id), {})
            field = "station_lla" if lla is not None else "station_enu"
            section = f"devices.{obj.device_id}" if field in device else "defaults"
            label += f" [{section}.{field}]"
        # Legacy use_reported_station is no longer a priority switch: valid
        # installation GPS in the current packet always wins by user policy.
        return select_station(self.converter, "eo", obj, config_enu=enu,
                              config_lla=lla, config_label=label)

    def normalize(self, obj, timestamp_ms, *, fallback_enu=None, fallback_label=None):
        p = self.profile(obj.device_id)
        station_info = self.station(obj, p, fallback_enu, fallback_label)
        station = station_info["station_enu"]
        key = (int(obj.device_id), int(obj.target_id))
        h = self.history.setdefault(key, {})
        if len(self.history) > 10000:
            self.history.pop(next(iter(self.history)))
        reasons = []
        valid_state = (int(getattr(obj, "fault_status", 0)) == 0
                       and int(obj.track_status) == 1)
        if not valid_state:
            reasons.append("not_normal_tracking_or_fault")
        az_raw, el_raw, r_raw = (finite(getattr(obj, k, None)) for k in
                               ("target_azimuth_deg", "target_elevation_deg", "target_dis_m"))
        az = None if az_raw is None else math.radians(
            (90.0 - (az_raw + float(p["azimuth_offset_deg"])))
            if p["azimuth_convention"] == "north_cw"
            else az_raw + float(p["azimuth_offset_deg"]))
        if az is not None:
            az = (az + math.pi) % (2 * math.pi) - math.pi
        el = None if el_raw is None else math.radians(el_raw + float(p["elevation_offset_deg"]))
        angle_valid = valid_state and az is not None and station is not None and bool(p["angles_calibrated"])
        if station is None:
            reasons.append("station_unknown")
        if not p["angles_calibrated"]:
            reasons.append("angle_calibration_unconfirmed")
        el_valid = angle_valid and bool(p["elevation_enabled"]) and el is not None and abs(el) < math.pi / 2
        lon, lat, alt = (finite(getattr(obj, k, None)) for k in
                         ("target_lng_deg", "target_lat_deg", "target_alt_m"))
        coordinate_valid = (valid_state and bool(p["position_enabled"])
                            and lon is not None and lat is not None
                            and self.converter.valid(lon, lat))
        xyz = None
        if coordinate_valid:
            xyz = list(self.converter.to_enu(lon, lat, alt or 0.0))
            if not p["altitude_enabled"] or alt is None:
                xyz[2] = None
            previous = h.get("accepted_xy")
            elapsed = (timestamp_ms - h.get("position_stamp", timestamp_ms)) / 1000
            if previous is not None and elapsed > 0:
                speed = math.hypot(xyz[0]-previous[0], xyz[1]-previous[1]) / elapsed
                if speed > float(p["max_position_speed_mps"]):
                    coordinate_valid = False
                    reasons.append("position_jump")
            if not self._field_recent(h, "position", (lon, lat, alt), timestamp_ms, p):
                coordinate_valid = False
                reasons.append("position_unchanged_unverified")
        range_valid = (valid_state and angle_valid and el_valid
                       and p["range_policy"] == "reported_positive"
                       and r_raw is not None and 0 < r_raw <= float(p["max_range_m"]))
        if r_raw is not None and r_raw > 0 and p["range_policy"] == "disabled":
            reasons.append("range_validity_unconfirmed")
        if range_valid and not self._field_recent(h, "range", r_raw, timestamp_ms, p):
            range_valid = False
            reasons.append("range_unchanged_unverified")
        mode = ("position" if coordinate_valid else "ranged" if range_valid
                else "bearing" if angle_valid else "invalid")
        fresh = mode != "invalid"
        if timestamp_ms - h.get("last_used", -10**18) < int(p["min_update_interval_ms"]):
            fresh = False
            reasons.append("update_rate_limited")
        if fresh:
            h["last_used"] = timestamp_ms
            if coordinate_valid:
                h["accepted_xy"] = xyz[:2]
                h["position_stamp"] = timestamp_ms
            if angle_valid:
                h["angle_stamp"] = timestamp_ms
            if range_valid:
                h["range_stamp"] = timestamp_ms
        out = dict(
            track_id=int(obj.target_id), source_track_id=int(obj.target_id),
            mode=mode, fresh=fresh, timestamp_ms=timestamp_ms,
            raw_timestamp_ms=int(obj.timestamp_ms), track_status=int(obj.track_status),
            fault_status=int(getattr(obj, "fault_status", 0)),
            az=az, el=el if el_valid else None,
            angle_valid=angle_valid, elevation_valid=el_valid,
            position_valid=coordinate_valid, range_valid=range_valid,
            **station_info,
            quality_reasons=reasons, position_quality="reported_unverified",
            raw_azimuth_deg=az_raw, raw_elevation_deg=el_raw, raw_range_m=r_raw,
            ptz_zeroazi_deg=getattr(obj, "ptz_zeroazi_deg", None),
            lng_deg=lon, lat_deg=lat, alt_m=alt,
            sig_r=float(p["position_sigma_m"]),
            position_sigma_m=float(p["position_sigma_m"]),
            altitude_sigma_m=float(p["altitude_sigma_m"]),
            sig_az=math.radians(float(p["azimuth_sigma_deg"])),
            sig_el=math.radians(float(p["elevation_sigma_deg"])),
            last_position_ms=h.get("position_stamp"), last_angle_ms=h.get("angle_stamp"),
            last_range_ms=h.get("range_stamp"),
            # EO class codes differ from radar class codes.
            cls={0: "bird", 1: "drone", 2: "large_target", 3: "unknown",
                 4: "vehicle", 5: "person", 6: "other"}.get(int(obj.target_type), "unknown"),
            cls_conf=max(0.0, min(1.0, float(obj.confidence) / 100)),
        )
        if coordinate_valid:
            out.update(x=xyz[0], y=xyz[1], z=xyz[2])
        if range_valid and not coordinate_valid:
            out["r"] = r_raw
        return out, station

    @staticmethod
    def _field_recent(history, name, value, stamp, profile):
        if history.get(name + "_value") != value:
            history[name + "_value"] = value
            history[name + "_changed"] = stamp
        return (stamp - history[name + "_changed"]) / 1000 <= float(profile["max_unchanged_s"])


class EOTracker:
    """Keep device track identity and observation type; do not refilter L2 twice."""
    def __init__(self, dt=0.5, timeout_s=12.0):
        self.dt, self.timeout_s = dt, timeout_s
        self.entries = {}
        self.tracks = []  # compatibility with node dt propagation

    def process(self, measurements, timestamp_ms):
        for entry in self.entries.values():
            entry["fresh"] = False
        for m in measurements:
            if not m.get("fresh") or m.get("mode") == "invalid":
                continue
            key = int(m["track_id"])
            old = self.entries.get(key, {})
            hits = old.get("hits", 0) + 1
            self.entries[key] = {**m, "id": key, "hits": hits, "misses": 0}
        self.entries = {k: v for k, v in self.entries.items()
                        if timestamp_ms - v["timestamp_ms"] <= self.timeout_s * 1000}

    def export(self):
        return [dict(v) for v in self.entries.values() if v["hits"] >= 3]
