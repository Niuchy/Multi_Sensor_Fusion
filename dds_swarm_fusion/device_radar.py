"""Import device L2 tracks without re-confirming them as anonymous detections.

An arriving report is new transport data; tag=0 is still an extrapolated point.
These two notions of freshness must stay separate all the way to the UI/API.
"""

import math
import numpy as np

from tracking import CVTrack


class DeviceRadarTracker:
    def __init__(self, dt=0.5, timeout_s=12.0, sig_r=15.0, **kwargs):
        self.dt = dt
        self.timeout_s = timeout_s
        self.sig_r = sig_r
        self.entries = {}
        self.now_ms = 0
        self.rejected = 0
        self.deleted = 0

    def process(self, radar_meas, timestamp_ms):
        self.now_ms = timestamp_ms
        for entry in self.entries.values():
            entry["new_report"] = False
        for m in radar_meas.get("meas", []):
            key = int(m["track_id"])
            stamp = int(m.get("timestamp_ms", timestamp_ms))
            old = self.entries.get(key)
            if old is not None and stamp <= old["timestamp_ms"]:
                self.rejected += 1
                continue
            xy = np.array([m["x"], m["y"]], float)
            if not np.all(np.isfinite(xy)):
                self.rejected += 1
                continue
            velocity = np.zeros(2)
            if old is not None:
                elapsed = (stamp - old["timestamp_ms"]) / 1000
                velocity = (xy - old["xy"]) / elapsed
                # Reused ID / impossible jump starts a new local generation.
                if np.linalg.norm(velocity) > 150:
                    old = None
                    velocity = np.zeros(2)
            if old is None:
                local_id = CVTrack._next
                CVTrack._next += 1
            else:
                local_id = old["id"]
            real = m.get("fresh", True)
            self.entries[key] = dict(
                id=local_id, source_track_id=key, xy=xy, velocity=velocity,
                timestamp_ms=stamp, new_report=True, source_fresh=bool(real),
                last_measurement_ms=stamp if real else (old or {}).get("last_measurement_ms"),
                hits=(old or {}).get("hits", 0) + int(real),
                reports=(old or {}).get("reports", 0) + 1,
                measurement=m,
            )
        alive = {key: e for key, e in self.entries.items()
                 if (timestamp_ms - e["timestamp_ms"]) / 1000 <= self.timeout_s}
        self.deleted += len(self.entries) - len(alive)
        self.entries = alive

    def export(self):
        rows = []
        for entry in self.entries.values():
            m = entry["measurement"]
            age = max(0, (self.now_ms - entry["timestamp_ms"]) / 1000)
            xy = entry["xy"] + entry["velocity"] * age
            sigma = max(self.sig_r, m.get("r", 0) * math.radians(0.3))
            variance = sigma ** 2 * (1 if entry["source_fresh"] else 4) + (6 * age ** 2 / 2) ** 2
            row = dict(
                id=entry["id"], source_track_id=entry["source_track_id"],
                x=float(xy[0]), y=float(xy[1]),
                vx=float(entry["velocity"][0]), vy=float(entry["velocity"][1]),
                cov=[[variance, 0.0], [0.0, variance]],
                hits=entry["hits"], reports=entry["reports"],
                misses=int(age // 3), fresh=entry["new_report"] and entry["source_fresh"],
                new_report=entry["new_report"], source_fresh=entry["source_fresh"],
                input_mode="device_tracks", timestamp_ms=self.now_ms,
                report_timestamp_ms=entry["timestamp_ms"],
                last_measurement_ms=entry["last_measurement_ms"],
                is_coasting=not entry["source_fresh"] or age > 0,
                cls=m.get("cls"), cls_conf=m.get("cls_conf", 0),
            )
            if m.get("z") is not None:
                row.update(z=float(m["z"]), z_sigma=max(15, math.sqrt(variance)))
            rows.append(row)
        return rows

    def debug(self):
        return dict(mode="device_tracks", total=len(self.entries), tentative=0,
                    confirmed=sum(e["source_fresh"] for e in self.entries.values()),
                    coasting=sum(not e["source_fresh"] or self.now_ms > e["timestamp_ms"]
                                 for e in self.entries.values()),
                    rejected=self.rejected, deleted=self.deleted,
                    reason="按设备航迹号接续；tag=0 保留为外推，真实量测命中不增加")
