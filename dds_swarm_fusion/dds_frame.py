"""Frame and sensor helpers shared by DDS nodes."""

from dataclasses import dataclass
from typing import Dict, Mapping, Optional

import numpy as np

from runtime_config import AREA


DT = 0.5
BEARING_KINDS = ("eo", "rf_df", "acoustic")


@dataclass
class SensorStub:
    sid: str
    kind: str
    pos: np.ndarray
    online: bool = True
    sig_r: Optional[float] = None
    sig_az: Optional[float] = None
    sig_el: Optional[float] = None
    station_known: bool = True

    def info(self):
        out = {
            "id": self.sid,
            "kind": self.kind,
            "pos": self.pos.tolist(),
            "online": bool(self.online),
        }
        for key in ("sig_r", "sig_az", "sig_el"):
            value = getattr(self, key)
            if value is not None:
                out[key] = float(value)
        return out


def sensor_info_from_sensors(sensors):
    return {sid: sensor.info() for sid, sensor in sensors.items()}


def stubs_from_info(sensors_info: Mapping[str, Mapping]) -> Dict[str, SensorStub]:
    out = {}
    for sid, info in sensors_info.items():
        pos = info.get("pos", [0.0, 0.0, 0.0])
        out[sid] = SensorStub(
            sid=sid,
            kind=info.get("kind", "unknown"),
            pos=np.asarray(pos, float),
            online=bool(info.get("online", True)),
            sig_r=(float(info["sig_r"]) if info.get("sig_r") is not None else None),
            sig_az=(float(info["sig_az"]) if info.get("sig_az") is not None else None),
            sig_el=(float(info["sig_el"]) if info.get("sig_el") is not None else None),
            station_known=bool(info.get("station_known", True)),
        )
    return out


def make_init_msg(
    sensors_info: Mapping[str, Mapping],
    *,
    fusion_params: Optional[Mapping] = None,
    dt: float = DT,
    area: Optional[Mapping] = None,
    coordinates: Optional[Mapping] = None,
    data_source: str = "anti-drone-pkg",
    dds_implementation: str = "zy-anti-drone TopicAPI/FastDDS",
    control_capabilities: Optional[Mapping] = None,
):
    return {
        "type": "init",
        "area": dict(area or AREA),
        "dt": dt,
        "sensors": dict(sensors_info),
        "fusion_params": dict(fusion_params or {}),
        "coordinates": dict(coordinates or {}),
        "data_source": data_source,
        "dds_implementation": dds_implementation,
        "control_capabilities": dict(control_capabilities or {}),
    }


def flatten_radar_tracks(radar_tracks_by_sid):
    out = []
    for sid, tracks in radar_tracks_by_sid.items():
        for track in tracks:
            row = dict(track)
            row["sensor_id"] = sid
            out.append(row)
    return out


def aggregate_spectrum_from_measurements(sensors_info, measurements):
    emitters, classes = 0, {}
    for sid, info in sensors_info.items():
        if info.get("kind") != "spectrum":
            continue
        m = measurements.get(sid, {})
        emitters = max(emitters, int(m.get("emitters", 0)))
        for cls, count in m.get("classes", {}).items():
            classes[cls] = max(classes.get(cls, 0), count)
    return {"emitters": emitters, "classes": classes}


def build_sensors_dyn(sensors_info, measurements):
    sensors_dyn = {}
    for sid, info in sensors_info.items():
        kind = info.get("kind")
        m = measurements.get(sid, {})
        d = {"online": bool(info.get("online", True)), "kind": kind}
        if kind == "radar":
            d["meas"] = [
                {"x": round(x["x"], 1), "y": round(x["y"], 1)}
                for x in m.get("meas", [])
            ]
            d["fa"] = [
                {"x": round(x["x"], 1), "y": round(x["y"], 1)}
                for x in m.get("fa", [])
            ]
        elif kind == "spectrum":
            d["emitters"] = m.get("emitters", 0)
            d["classes"] = m.get("classes", {})
        elif kind == "eo":
            d["meas"] = [
                {
                    "az": round(x["az"], 4),
                    "el": round(x["el"], 4) if x.get("el") is not None else None,
                    **({"r": round(x["r"], 1)} if x.get("r") is not None else {}),
                    **({"x": round(x["x"], 1), "y": round(x["y"], 1)}
                       if x.get("x") is not None and x.get("y") is not None else {}),
                }
                for x in m.get("meas", []) if x.get("az") is not None
            ]
        else:
            d["meas"] = [{"az": round(x["az"], 4)} for x in m.get("meas", [])]
        sensors_dyn[sid] = d
    return sensors_dyn


def sync_fusion_sensors(fusion, previous_info, sensors_info):
    """Apply sensor layout changes to an existing Fusion instance."""
    previous_info = previous_info or {}
    old_sids = set(previous_info)
    new_sids = set(sensors_info)

    for sid in old_sids - new_sids:
        fusion.remove_sensor(sid)

    for sid in new_sids:
        info = sensors_info[sid]
        pos = np.asarray(info.get("pos", [0.0, 0.0, 0.0]), float)
        kind = info.get("kind", "unknown")
        if sid not in old_sids:
            fusion.add_sensor(sid, pos, kind)
        else:
            pos3 = np.zeros(3, float)
            pos3[:min(3, pos.size)] = pos[:3]
            fusion.pos3[sid] = pos3
            fusion.pos[sid] = pos3[:2].copy()
            fusion.kinds[sid] = kind
            fusion.sig[sid] = (
                float(info["sig_az"])
                if info.get("sig_az") is not None
                else fusion._sig_for(sid, kind)
            )
            fusion.weight[sid] = fusion._w_for(sid, kind)
        if sid in fusion.sig and info.get("sig_az") is not None:
            fusion.sig[sid] = float(info["sig_az"])
        fusion.station_known[sid] = bool(info.get("station_known", True))
