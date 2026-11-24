"""Company simulation DDS topics -> normalized fusion raw-measurement batches."""

from __future__ import annotations

import argparse
import logging
import math
import os
import struct
import time
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import yaml
from company_dds import COMPANY_INPUT_TOPICS, CompanyDdsBus, Topics
from company_protocol import Wgs84Enu, parse_company_packet, radians, workspace_root
from runtime_config import AREA
from diagnostics import NodeDiagnostics
from eo_observations import EOObservations
from rf_classification import RFClassification
from device_station import select_station, valid_lla, vector3, SOURCE_LABELS, REASON_LABELS


LOG = logging.getLogger("company_input")

FRAME_KIND = {
    0x1A01: "radar",
    0x1B01: "rf_df",
    0x1C01: "eo",
    0x1D01: "acoustic",
}
DEFAULT_DEVICE_ID = {
    "radar": 101,
    "eo": 102,
    "rf_df": 103,
    "acoustic": 104,
}
TARGET_CLASS = {
    0: "unknown",
    1: "bird",
    2: "drone",
    3: "vehicle",
    4: "person",
    5: "large_target",
    6: "other",
}


def load_company_scenario(scenario_path: Path):
    """Read YAML and normalize explicitly declared WGS84 sites to internal ENU.

    Legacy ``position`` entries without ``position_frame`` remain ENU metres.
    Keep original WGS84 and file provenance for per-device station selection.
    """

    def merge(base, override):
        result = dict(base or {})
        for key, value in (override or {}).items():
            if isinstance(value, dict) and isinstance(result.get(key), dict):
                result[key] = merge(result[key], value)
            else:
                result[key] = value
        return result

    def namespace(value):
        if isinstance(value, dict):
            return SimpleNamespace(
                **{str(key): namespace(item) for key, item in value.items()}
            )
        if isinstance(value, list):
            return [namespace(item) for item in value]
        return value

    with scenario_path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}

    data.setdefault("coordinates", {"origin": [120.0, 30.0, 0.0], "frame": "ENU"})
    converter = Wgs84Enu(data["coordinates"]["origin"])

    def station_entry(position, frame, label):
        frame = str(frame).upper()
        if frame not in {"ENU", "WGS84"}:
            raise ValueError(f"Invalid position_frame at {label}: use ENU or WGS84")
        p = vector3(position)
        if p is None or (frame == "WGS84" and not valid_lla(p)):
            raise ValueError(f"Invalid {frame} station position at {label}")
        enu = list(converter.to_enu(*p)) if frame == "WGS84" else p
        return dict(position=enu, lla=p if frame == "WGS84" else None,
                    frame=frame, source=label)

    sensors = data.get("sensors", {})
    station_config_sources = {}
    station_configs = {}
    for sensor_name in ("radar", "eo", "esm", "acoustic"):
        entry = dict(sensors.get(sensor_name, {}) or {})
        use_config = entry.pop("use_config", None)
        defaults = {}
        if use_config:
            config_path = scenario_path.parent.parent / use_config
            if not config_path.exists():
                config_path = workspace_root() / "anti-drone-pkg" / use_config
        else:
            config_path = (
                workspace_root()
                / "anti-drone-pkg/config/sensors"
                / f"{sensor_name}.yaml"
            )
        if config_path.exists():
            with config_path.open("r", encoding="utf-8") as stream:
                defaults = yaml.safe_load(stream) or {}
            defaults = defaults.get(sensor_name, defaults)
        merged = merge(defaults, entry)
        label = (
            f"{scenario_path} [sensors.{sensor_name}.position]" if "position" in entry
            else f"{config_path.resolve()} [{sensor_name}.position]")
        # A frame-only override would reinterpret inherited ENU numbers as GPS.
        if ("position_frame" in entry and "position" not in entry
                and str(entry["position_frame"]).upper() != str(defaults.get("position_frame", "ENU")).upper()):
            raise ValueError(f"Changing position_frame requires position: sensors.{sensor_name}")
        frame = merged.get("position_frame", "ENU")
        base_station = station_entry(merged.get("position"), frame, label)
        device_stations = {}
        normalized_devices = {}
        device_entries = merged.get("devices", {})
        if not isinstance(device_entries, dict):
            raise ValueError(f"sensors.{sensor_name}.devices must be a device-ID mapping")
        for device_id, profile in device_entries.items():
            if isinstance(device_id, bool) or not str(device_id).isdigit() or not 0 < int(device_id) <= 0xFFFFFFFF:
                raise ValueError(f"Invalid station device ID: {sensor_name}.{device_id}")
            key = str(int(device_id))
            if key in device_stations or not isinstance(profile, dict):
                raise ValueError(f"Duplicate ID or invalid station profile: {sensor_name}.{device_id}")
            if "position" not in profile:
                raise ValueError(f"Per-device station requires position: {sensor_name}.{device_id}")
            scenario_devices = entry.get("devices", {})
            in_scenario = "position" in scenario_devices.get(device_id, {})
            device_label = (f"{scenario_path} [sensors.{sensor_name}.devices.{device_id}.position]"
                            if in_scenario else
                            f"{config_path.resolve()} [{sensor_name}.devices.{device_id}.position]")
            selected = station_entry(profile["position"], profile.get("position_frame", frame), device_label)
            device_stations[key] = selected
            normalized_devices[key] = {**profile, "position": selected["position"], "position_frame": "ENU"}
        station_configs[sensor_name] = dict(default=base_station, devices=device_stations)
        station_config_sources[sensor_name] = label
        data[sensor_name] = {**merged, "position": base_station["position"],
                             "position_frame": "ENU", "devices": normalized_devices}

    data.setdefault("targets", [])
    config = namespace(data)
    config.station_config_sources = station_config_sources
    config.station_configs = station_configs
    return config


def scenario_area(config):
    xs, ys = [0.0], [0.0]
    for target in config.targets:
        for waypoint in target.waypoints:
            if len(waypoint) >= 2:
                xs.append(float(waypoint[0]))
                ys.append(float(waypoint[1]))
    for sensor in (config.radar, config.eo, config.esm, config.acoustic):
        if sensor is not None:
            xs.append(float(sensor.position[0]))
            ys.append(float(sensor.position[1]))
    for settings in config.station_configs.values():
        for station in settings["devices"].values():
            xs.append(station["position"][0])
            ys.append(station["position"][1])
    margin = 500.0
    return {
        "xmin": min(AREA["xmin"], math.floor((min(xs) - margin) / 500) * 500),
        "xmax": max(AREA["xmax"], math.ceil((max(xs) + margin) / 500) * 500),
        "ymin": min(AREA["ymin"], math.floor((min(ys) - margin) / 500) * 500),
        "ymax": max(AREA["ymax"], math.ceil((max(ys) + margin) / 500) * 500),
    }


class CompanyInputNode:
    def __init__(
        self,
        scenario,
        *,
        batch_period=0.5,
        settle_ms=40,
        bus=None,
        radar_input_mode=None,
        azimuth_convention=None,
    ):
        self.scenario_path = Path(scenario).resolve()
        self.config = load_company_scenario(self.scenario_path)
        self.bus = bus or CompanyDdsBus(source="company_input_node")
        self.diagnostics = NodeDiagnostics("input", self.bus)
        self.stream_id = self.diagnostics.instance_id
        self.radar_input_mode = radar_input_mode or os.environ.get("SB_RADAR_INPUT_MODE", "measurements")
        self.azimuth_convention = azimuth_convention or os.environ.get("SB_AZIMUTH_CONVENTION", "enu")
        if self.radar_input_mode not in {"measurements", "device_tracks"}:
            raise ValueError("SB_RADAR_INPUT_MODE must be measurements or device_tracks")
        if self.azimuth_convention not in {"enu", "north_cw"}:
            raise ValueError("SB_AZIMUTH_CONVENTION must be enu or north_cw")
        self.origin = list(self.config.coordinates.origin)
        self.converter = Wgs84Enu(self.origin)
        self.eo_observations = EOObservations(self.converter, workspace_root())
        self.rf_classification = RFClassification(workspace_root())
        LOG.info("频侦型号分类配置: %s；仅补充已关联雷达的未知类别；设备配置=%s",
                 self.rf_classification.path, sorted(self.rf_classification.devices))
        self.area = scenario_area(self.config)
        self.batch_period = float(batch_period)
        self.settle_seconds = float(settle_ms) / 1000.0
        self.reorder_seconds = self.eo_observations.reorder_ms / 1000.0
        self.sensors_info = self._initial_sensor_info()
        self._station_log_state = {}
        self.pending = {}
        self.tick = 0
        self.last_timestamp_ms = None
        self.paused = False
        self._last_info_publish = 0.0
        self.topic_stats = {topic: dict(packets=0, bytes=0, parsed=0, errors=0,
                            last_received_at=None, last_error=None)
                            for topic in COMPANY_INPUT_TOPICS}
        self.latest_samples = {}
        self.recent_samples = deque(maxlen=100)
        self.last_target_times = {}
        self.dropped_old = 0
        self.duplicates = 0
        self.rf_zero_azimuth_filtered = 0
        self._last_emit_monotonic = time.monotonic()
        self._last_source_monotonic = None
        self._last_source_timestamp_ms = None
        self.last_data_received_at = None

    def _initial_sensor_info(self):
        configs = {
            "radar": self.config.radar,
            "eo": self.config.eo,
            "rf_df": self.config.esm,
            "acoustic": self.config.acoustic,
        }
        out = {}
        for kind, cfg in configs.items():
            if cfg is None:
                continue
            sid = f"{kind}_{DEFAULT_DEVICE_ID[kind]}"
            info = {
                "id": sid,
                "kind": kind,
                "device_id": DEFAULT_DEVICE_ID[kind],
                "pos": [float(x) for x in cfg.position],
                "online": True,
                "received": False,
                "source_topic": {
                    "radar": Topics.COMPANY_RADAR.name,
                    "eo": Topics.COMPANY_EO.name,
                    "rf_df": Topics.COMPANY_ESM.name,
                    "acoustic": Topics.COMPANY_ACOUSTIC.name,
                }[kind],
            }
            error = getattr(cfg, "error", None)
            if error is not None and hasattr(error, "azimuth_sigma"):
                info["sig_az"] = math.radians(float(error.azimuth_sigma))
            if error is not None and hasattr(error, "elevation_sigma"):
                info["sig_el"] = math.radians(float(error.elevation_sigma))
            if kind == "radar" and error is not None:
                info["sig_r"] = float(getattr(error, "range_sigma", 5.0))
            elif kind == "eo":
                # dev_proto provides target distance but no declared range
                # sigma.  Keep a conservative live-interface default.
                info["sig_r"] = 5.0
            out[sid] = info
        self.sensor_templates = out
        equipment = getattr(self.config, "equipment", None)
        if getattr(equipment, "discover_only", False):
            return {}
        return out

    @staticmethod
    def _empty_measurement(info):
        out = {
            "pos": list(info.get("pos", [0.0, 0.0, 0.0])),
            "meas": [],
            "protocol": "company dev_proto.h",
            "source_topic": info.get("source_topic"),
        }
        if info.get("kind") == "radar":
            out["fa"] = []
        return out

    def _batch(self, timestamp_ms):
        batch = self.pending.get(timestamp_ms)
        if batch is None:
            batch = {
                "created": time.monotonic(),
                "updated": time.monotonic(),
                "measurements": {
                    sid: self._empty_measurement(info)
                    for sid, info in self.sensors_info.items()
                },
            }
            self.pending[timestamp_ms] = batch
        batch["updated"] = time.monotonic()
        return batch

    def _sensor(self, kind, device_id, position=None):
        sid = f"{kind}_{int(device_id or DEFAULT_DEVICE_ID[kind])}"
        info = self.sensors_info.get(sid)
        if info is None:
            template_sid = f"{kind}_{DEFAULT_DEVICE_ID[kind]}"
            info = dict(self.sensors_info.get(template_sid, self.sensor_templates.get(template_sid, {})))
            info.update(id=sid, kind=kind, device_id=int(device_id))
            self.sensors_info[sid] = info
        if position is not None:
            info["pos"] = [float(x) for x in position]
        return sid, info

    def _station_config(self, kind, device_id=None):
        name = {"radar": "radar", "eo": "eo", "rf_df": "esm", "acoustic": "acoustic"}[kind]
        settings = self.config.station_configs[name]
        return settings["devices"].get(str(device_id), settings["default"])

    def _station_fallback(self, kind, device_id=None):
        station = self._station_config(kind, device_id)
        return station["position"], station["source"]

    def _use_station(self, kind, obj, station_info=None):
        if station_info is None:
            station = self._station_config(kind, obj.device_id)
            station_info = select_station(self.converter, kind, obj,
                                          config_enu=station["position"] if station["lla"] is None else None,
                                          config_lla=station["lla"], config_label=station["source"])
        sid, info = self._sensor(kind, obj.device_id, position=station_info["station_enu"])
        info.update(station_info)
        # First report and source changes are explicit; GPS jitter is logged at
        # most once per second. Diagnostics always contain the latest values.
        lla = info["station_lla"]
        signature = (info["station_source"], info["station_reason"], info["station_config"],
                     tuple(round(x, 7 if i < 2 else 2) for i, x in enumerate(lla)) if lla else None)
        previous = self._station_log_state.get(sid)
        now = time.monotonic()
        if (previous is None or (signature != previous[0] and
                (signature[:3] != previous[0][:3] or now-previous[1] >= 1.0))):
            log = LOG.info if info["station_source"] == "reported_lla" else LOG.warning
            log("[设备自身站址] %s 来源=%s；原因=%s；使用经度=%s 纬度=%s 高度=%s m；"
                "ENU=%s m；配置=%s；参考原点=%s。%s",
                sid, SOURCE_LABELS[info["station_source"]], REASON_LABELS[info["station_reason"]],
                f"{lla[0]:.7f}" if lla else "未知", f"{lla[1]:.7f}" if lla else "未知",
                f"{lla[2]:.2f}" if lla else "未知", info["station_enu"],
                info["station_config"] or "不使用配置站址", info["station_reference_lla"],
                "配置站址并非设备实测 GPS，请核对现场配置" if info["station_config"] else "")
            self._station_log_state[sid] = (signature, now)
        return sid, info

    def _radar(self, obj):
        sid, info = self._use_station("radar", obj)
        az = radians(90.0 - obj.azimuth_deg) if self.azimuth_convention == "north_cw" else radians(obj.azimuth_deg)
        info["radar_input_mode"] = self.radar_input_mode
        info["scan_period_s"] = 3.0
        info["azimuth_convention"] = self.azimuth_convention
        sx, sy, sz = info["pos"]
        if self.converter.valid(obj.lng_deg, obj.lat_deg):
            x, y, z = self.converter.to_enu(
                obj.lng_deg, obj.lat_deg, obj.alt_m
            )
        else:
            ground_range = obj.range_m * math.cos(math.radians(obj.elevation_deg))
            x = sx + ground_range * math.cos(az)
            y = sy + ground_range * math.sin(az)
            z = sz + obj.range_m * math.sin(math.radians(obj.elevation_deg))
        return sid, {
            "track_id": int(obj.track_id),
            "x": x,
            "y": y,
            "z": z,
            "r": float(obj.range_m),
            "az": az,
            "azimuth_deg": float(obj.azimuth_deg),
            "lng_deg": float(obj.lng_deg), "lat_deg": float(obj.lat_deg),
            "alt_m": float(obj.alt_m),
            "el": math.radians(float(obj.elevation_deg)),
            "vr": float(obj.radial_vel_ms),
            "speed": float(obj.speed_ms),
            "course_deg": float(obj.course_deg),
            "fresh": bool(obj.extrap_flag),
            "extrap_flag": int(obj.extrap_flag),
            "input_mode": self.radar_input_mode,
            "target_type": int(obj.target_type),
            "confidence": float(obj.confidence) / 100.0,
            "cls": TARGET_CLASS.get(int(obj.target_type), "unknown"),
            "cls_conf": float(obj.confidence) / 100.0,
        }

    def _eo(self, obj, timestamp_ms=None):
        if timestamp_ms is None:
            timestamp_ms = int(obj.timestamp_ms) + int(
                self.eo_observations.profile(obj.device_id)["time_offset_ms"])
        fallback, label = self._station_fallback("eo", obj.device_id)
        measurement, _ = self.eo_observations.normalize(
            obj, timestamp_ms, fallback_enu=fallback, fallback_label=label)
        sid, info = self._use_station("eo", obj, {k: v for k, v in measurement.items()
                                                if k.startswith("station_")})
        info.update(sig_az=measurement["sig_az"], sig_el=measurement["sig_el"],
                    sig_r=measurement["sig_r"], eo_quality_aware=True)
        return sid, measurement

    def _esm(self, obj):
        sid, _ = self._use_station("rf_df", obj)
        band = f"{float(obj.freq_hz) / 1e9:.2f}GHz"
        raw_azimuth = float(obj.azimuth_deg)
        # Site policy: an RF wire angle of exactly 0 degrees is a placeholder,
        # not a direction measurement. Check BEFORE angle wrapping/rounding;
        # nonzero angles (including 0.01 degrees) keep the existing convention.
        angle_valid = raw_azimuth != 0.0
        return sid, {
            "track_id": int(obj.target_id),
            "az": radians(raw_azimuth) if angle_valid else None,
            "raw_azimuth_deg": raw_azimuth,
            "angle_valid": angle_valid,
            "fresh": angle_valid,
            "quality_reasons": [] if angle_valid else ["rf_zero_azimuth_placeholder"],
            "proto": obj.brand_model or band,
            "sn_id": obj.sn or None,
            "model": obj.brand_model or None,
            "rf_freq": float(obj.freq_hz) if int(obj.freq_hz) > 0 else None,
            "freq_hz": int(obj.freq_hz),
            "bandwidth_hz": int(obj.bandw_hz),
            "power_dbm": float(obj.power_dbm),
            "confidence": float(obj.confidence) / 100.0,
            "rf_classification": self.rf_classification.classify(obj),
        }

    def _acoustic(self, obj):
        sid, _ = self._use_station("acoustic", obj)
        return sid, {
            "track_id": int(obj.target_id),
            "az": radians(obj.target_yaw_deg),
            "el": math.radians(float(obj.target_pitch_deg)),
            "cls": TARGET_CLASS.get(int(obj.target_type), "unknown"),
            "cls_conf": float(obj.confidence) / 100.0,
        }

    def ingest(self, unit_tag, source_time, obj):
        kind = FRAME_KIND.get(unit_tag)
        if kind is None:
            return
        timestamp_ms = int(getattr(obj, "timestamp_ms", 0) or source_time)
        if kind == "eo":
            timestamp_ms += int(self.eo_observations.profile(obj.device_id)["time_offset_ms"])
        if timestamp_ms <= 0:
            raise ValueError("设备报文缺少有效毫秒时间戳")
        key = (kind, int(obj.device_id), int(getattr(obj, "track_id", getattr(obj, "target_id", 0))))
        previous = self.last_target_times.get(key)
        if previous is not None and timestamp_ms <= previous:
            self.duplicates += int(timestamp_ms == previous)
            self.dropped_old += int(timestamp_ms < previous)
            return
        if self.last_timestamp_ms is not None and timestamp_ms < self.last_timestamp_ms:
            self.dropped_old += 1
            return
        converter = {
            "radar": self._radar,
            "eo": self._eo,
            "rf_df": self._esm,
            "acoustic": self._acoustic,
        }[kind]
        sid, measurement = self._eo(obj, timestamp_ms) if kind == "eo" else converter(obj)
        if (self.radar_input_mode == "device_tracks" and kind == "radar") or (
                kind == "eo" and measurement.get("position_valid")):
            # Actual WGS84 tracks may be kilometres outside the demo viewport.
            # Expand display bounds; do not invent a radar installation position.
            for axis in ("x", "y"):
                value = float(measurement[axis])
                self.area[axis + "min"] = min(self.area[axis + "min"], math.floor((value - 500) / 500) * 500)
                self.area[axis + "max"] = max(self.area[axis + "max"], math.ceil((value + 500) / 500) * 500)
        measurement["timestamp_ms"] = timestamp_ms
        if kind == "rf_df":
            measurement["rf_classification"]["timestamp_ms"] = timestamp_ms
        measurement.update({k: v for k, v in self.sensors_info[sid].items()
                            if k.startswith("station_")})
        self.last_target_times[key] = timestamp_ms
        # Bounded dedup history; inactive IDs need not be retained indefinitely.
        if len(self.last_target_times) > 10000:
            self.last_target_times.pop(next(iter(self.last_target_times)))
        self.sensors_info[sid]["received"] = True
        self.sensors_info[sid]["last_received_at"] = time.time()
        self.latest_samples[sid] = {
            "sensor_id": sid, "received_at": time.time(), **measurement,
            "interval_s": None if previous is None else (timestamp_ms - previous) / 1000.0,
        }
        self.recent_samples.append(self.latest_samples[sid])
        self.last_data_received_at = time.time()
        self._last_source_monotonic = time.monotonic()
        self._last_source_timestamp_ms = max(timestamp_ms, self._last_source_timestamp_ms or timestamp_ms)
        batch = self._batch(timestamp_ms)
        if kind == "rf_df" and not measurement["angle_valid"]:
            # Retain receipt/timing/raw diagnostics, but never forward the
            # placeholder to drawing, bearing tracking or fusion. Do not mark
            # this sensor as updated or erase other valid RF targets in the
            # same batch: a placeholder is not evidence of their disappearance.
            self.rf_zero_azimuth_filtered += 1
            return
        batch["measurements"].setdefault(
            sid, self._empty_measurement(self.sensors_info[sid])
        )["meas"].append(measurement)

    def poll_company_topics(self):
        count = 0
        for topic in COMPANY_INPUT_TOPICS:
            for packet in self.bus.read_bytes(topic):
                stats = self.topic_stats[topic]
                stats["packets"] += 1
                stats["bytes"] += len(packet)
                stats["last_received_at"] = time.time()
                stats["head_hex"] = packet[:48].hex(" ")
                try:
                    units = parse_company_packet(packet)
                    if not units:
                        raise ValueError("报文中没有可解析的设备目标帧")
                except (ValueError, struct.error) as exc:
                    stats["errors"] += 1
                    stats["last_error"] = str(exc)
                    LOG.warning("丢弃无效公司 DDS 报文 topic=%s: %s", topic, exc)
                    continue
                for unit_tag, source_time, obj in units:
                    try:
                        self.ingest(unit_tag, source_time, obj)
                    except (ValueError, KeyError, TypeError, OverflowError) as exc:
                        stats["errors"] += 1
                        stats["last_error"] = str(exc)
                        LOG.warning("设备目标归一化失败 topic=%s: %s", topic, exc)
                        continue
                    stats["parsed"] += 1
                    count += 1
        return count

    def apply_controls(self):
        for cmd in self.bus.read(Topics.CONTROL_COMMAND.name):
            action = cmd.get("cmd")
            if action == "pause":
                self.paused = True
            elif action == "play":
                self.paused = False
            elif action == "reset":
                self.pending.clear()
                self.tick = 0
                self.last_timestamp_ms = None
                self.last_target_times.clear()
                self.eo_observations.history.clear()
                self._last_source_timestamp_ms = None
                self._last_source_monotonic = None
                self.stream_id = f"{self.diagnostics.instance_id}:{time.monotonic()}"
            elif action == "sensor":
                sid = cmd.get("id")
                if sid in self.sensors_info:
                    self.sensors_info[sid]["online"] = bool(cmd.get("on", True))
                    self.publish_sensor_info()

    def publish_sensor_info(self):
        self.bus.write(
            Topics.SENSOR_INFO.name,
            {
                "type": "sensor_info",
                "tick": self.tick,
                "dt": self.batch_period,
                "area": self.area,
                "coordinates": {
                    "frame": "ENU",
                    "origin": self.origin,
                },
                "sensors": self.sensors_info,
                "data_source": "anti-drone-pkg",
                "dds_implementation": "zy-anti-drone TopicAPI/FastDDS",
                "control_capabilities": {
                    "pause_forwarding": True,
                    "sensor_filter": True,
                    "move_sensor": False,
                    "add_remove_sensor": False,
                    "simulator_control": False,
                },
            },
            kind="sensor_info",
            tick=self.tick,
        )
        self._last_info_publish = time.monotonic()

    def publish_batch(self, timestamp_ms, batch):
        if self.paused:
            return
        if (
            self.last_timestamp_ms is not None
            and timestamp_ms > self.last_timestamp_ms
        ):
            dt = (timestamp_ms - self.last_timestamp_ms) / 1000.0
            dt = max(dt, 0.001)
        else:
            dt = 0.001 if self.last_timestamp_ms == timestamp_ms else self.batch_period
        if (
            self.last_timestamp_ms is not None
            and timestamp_ms < self.last_timestamp_ms
        ):
            self.dropped_old += 1
            return
        self.last_timestamp_ms = timestamp_ms
        self.tick += 1
        self._last_emit_monotonic = time.monotonic()

        measurements = {}
        for sid, info in self.sensors_info.items():
            current = batch["measurements"].get(
                sid, self._empty_measurement(info)
            )
            if not info.get("online", True):
                current = self._empty_measurement(info)
            current["pos"] = list(info["pos"])
            measurements[sid] = current

        raw = {
            "type": "raw_measurements",
            "tick": self.tick,
            "stream_id": self.stream_id,
            "heartbeat": bool(batch.get("heartbeat", False)),
            "updated_sensors": [sid for sid, value in measurements.items() if value.get("meas")],
            "live_timing": self.radar_input_mode == "device_tracks",
            "t": timestamp_ms / 1000.0,
            "source_timestamp_ms": timestamp_ms,
            "dt": dt,
            "sensors": self.sensors_info,
            "measurements": measurements,
            "truth": [],
            "coordinates": {"frame": "ENU", "origin": self.origin},
            "area": self.area,
            "data_source": "anti-drone-pkg/dev_proto",
        }
        self.bus.write(
            Topics.RAW_MEASUREMENTS.name,
            raw,
            kind="raw_measurements",
            tick=self.tick,
        )

    def flush_due(self, force=False):
        if not self.pending:
            return 0
        now = time.monotonic()
        # Hold a bounded window even when a faster device has already sent a
        # newer frame. Never release a newer frame past an earlier pending one.
        due = []
        for timestamp in sorted(self.pending):
            batch = self.pending[timestamp]
            if not force and now - batch["created"] < max(self.settle_seconds, self.reorder_seconds):
                break
            due.append(timestamp)
        for timestamp in due:
            self.publish_batch(timestamp, self.pending.pop(timestamp))
        return len(due)

    def run(self):
        LOG.info("订阅公司设备 Topics: %s", ", ".join(COMPANY_INPUT_TOPICS))
        self.publish_sensor_info()
        while True:
            self.apply_controls()
            did_work = self.poll_company_topics()
            did_work += self.flush_due()
            self.emit_idle_frame()
            self.publish_diagnostics()
            if time.monotonic() - self._last_info_publish >= 5.0:
                self.publish_sensor_info()
            if not did_work:
                time.sleep(0.005)

    def emit_idle_frame(self):
        # No synthetic measurements: an empty heartbeat only advances prediction
        # and expiration. Keep the source clock; never mix epoch and replay time.
        if self.radar_input_mode != "device_tracks" or self.paused or self.pending:
            return
        now = time.monotonic()
        if self._last_source_monotonic is None or now - self._last_emit_monotonic < 0.5:
            return
        # Grace interval prevents a normal 3s report with jitter from being late
        # merely because a heartbeat raced it. After silence, coast and expire.
        if now - self._last_source_monotonic < 3.5:
            return
        timestamp_ms = self._last_source_timestamp_ms + int((now - self._last_source_monotonic) * 1000) - 500
        if timestamp_ms > (self.last_timestamp_ms or 0):
            self.publish_batch(timestamp_ms, {"measurements": {}, "heartbeat": True})

    def publish_diagnostics(self, force=False):
        self.diagnostics.publish(force=force, topics=self.topic_stats,
            samples=list(self.latest_samples.values())[-100:],
            recent_samples=list(self.recent_samples),
            raw_batches=self.tick, pending_batches=len(self.pending),
            duplicates=self.duplicates, dropped_old=self.dropped_old,
            rf_zero_azimuth_filtered=self.rf_zero_azimuth_filtered,
            last_data_received_at=self.last_data_received_at,
            device_stations=[dict(sensor_id=sid, device_id=info["device_id"],
                                  last_received_at=info.get("last_received_at"),
                                  **{k: v for k, v in info.items() if k.startswith("station_")})
                             for sid, info in self.sensors_info.items() if info.get("received")][-100:],
            scenario_path=str(self.scenario_path), origin=self.origin,
            eo_config_path=str(self.eo_observations.path),
            rf_config_path=str(self.rf_classification.path),
            reorder_window_ms=self.eo_observations.reorder_ms,
            radar_input_mode=self.radar_input_mode,
            azimuth_convention=self.azimuth_convention, paused=self.paused)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        default=str(
            workspace_root() / "anti-drone-pkg/config/scenarios/multi_target.yaml"
        ),
    )
    parser.add_argument(
        "--batch-period",
        type=float,
        default=0.5,
        help="公司仿真协议发布周期，默认 0.5s/2Hz",
    )
    parser.add_argument("--settle-ms", type=int, default=40)
    args = parser.parse_args()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    CompanyInputNode(
        args.scenario,
        batch_period=args.batch_period,
        settle_ms=args.settle_ms,
    ).run()


if __name__ == "__main__":
    main()
