"""Company-DDS tracking node: normalized equipment frames -> sensor tracks."""

import argparse
import time
from eo_observations import EOTracker

from company_dds import CompanyDdsBus, Topics
from dds_frame import (
    BEARING_KINDS,
    DT,
    aggregate_spectrum_from_measurements,
    flatten_radar_tracks,
)
from tracking import BearingTracker, RadarTracker, SpectrumTracker
from device_radar import DeviceRadarTracker
from diagnostics import NodeDiagnostics


class TrackingDdsNode:
    def __init__(self, bus=None):
        self.bus = bus or CompanyDdsBus(source="tracking_node")
        self.sensors_info = {}
        self.radar_trks = {}
        self.bearing_trks = {}
        self.spec_trk = SpectrumTracker()
        self.last_tick = 0
        self.dt = DT
        self.diagnostics = NodeDiagnostics("tracking", self.bus)
        self.stream_id = None
        self.last_sensor_time = {}
        self.reset_count = 0

    def sync_trackers(self, sensors_info):
        old_info = self.sensors_info
        new_info = dict(sensors_info)
        live = set(new_info)
        for sid in list(self.radar_trks):
            if sid not in live or new_info[sid].get("kind") != "radar":
                self.radar_trks.pop(sid, None)
        for sid in list(self.bearing_trks):
            if sid not in live or new_info[sid].get("kind") not in BEARING_KINDS:
                self.bearing_trks.pop(sid, None)

        for sid, info in new_info.items():
            kind = info.get("kind")
            old = old_info.get(sid, {})
            moved = old.get("pos") != info.get("pos")
            tracker_class = DeviceRadarTracker if info.get("radar_input_mode") == "device_tracks" else RadarTracker
            if kind == "radar" and (sid not in self.radar_trks or moved or
                                    not isinstance(self.radar_trks[sid], tracker_class)):
                self.radar_trks[sid] = tracker_class(
                    self.dt,
                    sig_r=float(info.get("sig_r", 15.0)),
                    sig_az=float(info.get("sig_az", 0.005235987755982988)),
                    sig_el=float(info.get("sig_el", 0.008726646259971648)),
                )
            elif kind in BEARING_KINDS:
                cls = EOTracker if kind == "eo" and info.get("eo_quality_aware") else BearingTracker
                if (sid not in self.bearing_trks or not isinstance(self.bearing_trks[sid], cls)
                        or (moved and cls is not EOTracker)):
                    self.bearing_trks[sid] = cls(self.dt)
        self.sensors_info = new_info

    def reset_trackers(self):
        self.reset_count += 1
        self.last_sensor_time.clear()
        self.radar_trks = {}
        self.bearing_trks = {}
        self.spec_trk = SpectrumTracker()
        self.sync_trackers(self.sensors_info)

    def process_raw(self, raw):
        tick = int(raw.get("tick", 0))
        incoming_dt = float(raw.get("dt", self.dt) or self.dt)
        self.dt = max(incoming_dt, 0.001)
        stream_id = raw.get("stream_id")
        if stream_id and self.stream_id and stream_id != self.stream_id:
            self.reset_trackers()
        elif self.last_tick and tick <= self.last_tick:
            return  # DDS duplicate/late frame must not clear live tracks.
        self.stream_id = stream_id
        self.last_tick = tick

        sensors_info = raw.get("sensors", {})
        measurements = raw.get("measurements", {})
        self.sync_trackers(sensors_info)

        radar_tracks_by_sid = {}
        for sid, tracker in self.radar_trks.items():
            if isinstance(tracker, DeviceRadarTracker):
                tracker.process(measurements.get(sid, {}), int(raw.get("source_timestamp_ms", 0)))
            else:
                # Per-sensor elapsed time; an EO/RF packet is not a radar miss.
                updated = raw.get("updated_sensors")
                if updated is None or sid in updated:
                    self._set_dt(tracker, sid, raw)
                    tracker.process(measurements.get(sid, {}))
            radar_tracks_by_sid[sid] = tracker.export()
            if not isinstance(tracker, DeviceRadarTracker) and raw.get("updated_sensors") is not None and sid not in raw["updated_sensors"]:
                for row in radar_tracks_by_sid[sid]:
                    row["fresh"] = False

        bearing_tracks = {}
        for sid, tracker in self.bearing_trks.items():
            updated = raw.get("updated_sensors")
            if isinstance(tracker, EOTracker):
                tracker.process(measurements.get(sid, {}).get("meas", []),
                                int(raw.get("source_timestamp_ms", 0)))
            elif updated is None or sid in updated:
                self._set_dt(tracker, sid, raw)
                tracker.process(measurements.get(sid, {}).get("meas", []))
            bearing_tracks[sid] = tracker.export()
            if updated is not None and sid not in updated:
                for row in bearing_tracks[sid]:
                    row["fresh"] = False

        spec_agg = aggregate_spectrum_from_measurements(sensors_info, measurements)
        self.spec_trk.process(spec_agg)
        spectrum = self.spec_trk.export()

        msg = {
            "type": "sensor_tracks",
            "tick": tick,
            "stream_id": stream_id,
            "live_timing": raw.get("live_timing", False),
            "t": raw.get("t"),
            "source_timestamp_ms": raw.get("source_timestamp_ms", 0),
            "dt": self.dt,
            "sensors": sensors_info,
            "measurements": measurements,
            "truth": raw.get("truth", []),
            "coordinates": raw.get("coordinates", {}),
            "area": raw.get("area", {}),
            "data_source": raw.get("data_source", "anti-drone-pkg/dev_proto"),
            "radar_tracks_by_sid": radar_tracks_by_sid,
            "radar_tracks": flatten_radar_tracks(radar_tracks_by_sid),
            "bearing_tracks": bearing_tracks,
            "spectrum": spectrum,
            "sensor_tracks": {**radar_tracks_by_sid, **bearing_tracks},
        }
        self.bus.write(Topics.SENSOR_TRACKS.name, msg, kind="sensor_tracks", tick=tick)
        radar_debug = {sid: tracker.debug() if isinstance(tracker, DeviceRadarTracker) else {
            "total": len(tracker.tracks),
            "tentative": sum(not t.confirmed for t in tracker.tracks),
            "confirmed": sum(t.confirmed for t in tracker.tracks),
            "max_hits": max((t.hits for t in tracker.tracks), default=0),
            "reason": "累计 3 次命中确认",
        } for sid, tracker in self.radar_trks.items()}
        self.diagnostics.publish(last_processed_at=time.time(), tick=tick,
            radar=radar_debug, radar_tracks=msg["radar_tracks"][:100],
            reset_count=self.reset_count, dt=self.dt)

    def _set_dt(self, tracker, sid, raw):
        stamp = raw.get("source_timestamp_ms")
        previous = self.last_sensor_time.get(sid)
        dt = max(0.001, (stamp - previous) / 1000) if stamp and previous else self.dt
        if stamp:
            self.last_sensor_time[sid] = stamp
        tracker.dt = dt
        for track in tracker.tracks:
            track.dt = dt
            if hasattr(track, "vertical"):
                track.vertical.dt = dt

    def run(self):
        print("tracking_node: raw measurements -> sensor tracks")
        while True:
            did_work = False
            for raw in self.bus.read(Topics.RAW_MEASUREMENTS.name):
                self.process_raw(raw)
                did_work = True
            if not did_work:
                self.diagnostics.publish()
                time.sleep(0.01)


def main():
    argparse.ArgumentParser().parse_args()
    TrackingDdsNode().run()


if __name__ == "__main__":
    main()
