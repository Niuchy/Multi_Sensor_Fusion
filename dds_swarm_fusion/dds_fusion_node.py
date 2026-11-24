"""Company-DDS fusion node: sensor tracks -> fused tracks."""

import argparse
import os
import time

from company_dds import CompanyDdsBus, Topics
from dds_frame import (
    DT,
    build_sensors_dyn,
    stubs_from_info,
    sync_fusion_sensors,
)
from fusion import Fusion, evaluate
from diagnostics import NodeDiagnostics
from output_filter import OutputFilter


class FusionDdsNode:
    def __init__(self, bus=None, *, min_confidence=None, output_filter_path=None):
        self.min_confidence = float(os.environ.get("SB_FUSION_MIN_CONFIDENCE", "0.05")
                                    if min_confidence is None else min_confidence)
        if not 0 <= self.min_confidence <= 1:
            raise ValueError("SB_FUSION_MIN_CONFIDENCE must be finite and within 0–1")
        self.output_filter = OutputFilter(output_filter_path)
        self.bus = bus or CompanyDdsBus(source="fusion_node")
        self.sensors = {}
        self.sensors_info = {}
        self.fusion = None
        self.fusion_eo = None
        self.last_tick = 0
        self.dt = DT
        self.stream_id = None
        self.diagnostics = NodeDiagnostics("fusion", self.bus)
        self._last_filter_refresh = 0.0

    def process_controls(self):
        for cmd in self.bus.read(Topics.CONTROL_COMMAND.name):
            if isinstance(cmd, dict) and self.output_filter.command(cmd):
                self.diagnostics.publish(force=True, output_filter=self.output_filter.snapshot())
        if time.monotonic()-self._last_filter_refresh >= 1:
            self.output_filter.refresh_saved()
            self.diagnostics.publish(output_filter=self.output_filter.snapshot())
            self._last_filter_refresh = time.monotonic()

    def sync_fusion(self, sensors_info, area=None):
        if not sensors_info:
            return
        active_area = dict(area or {})
        if self.fusion is None:
            self.sensors_info = dict(sensors_info)
            self.sensors = stubs_from_info(self.sensors_info)
            self.fusion = Fusion(self.sensors, self.dt, area=active_area or None,
                                 min_confidence=self.min_confidence)
            self.fusion_eo = Fusion(self.sensors, self.dt, area=active_area or None,
                                 min_confidence=self.min_confidence)
            return

        if active_area:
            self.fusion.area = active_area
            self.fusion_eo.area = active_area
        sync_fusion_sensors(self.fusion, self.sensors_info, sensors_info)
        sync_fusion_sensors(self.fusion_eo, self.sensors_info, sensors_info)
        self.sensors_info = dict(sensors_info)
        self.sensors = stubs_from_info(self.sensors_info)

    def process_tracks(self, tracks_msg):
        tick = int(tracks_msg.get("tick", 0))
        incoming_dt = float(tracks_msg.get("dt", self.dt) or self.dt)
        self.dt = max(incoming_dt, 0.001)
        sensors_info = tracks_msg.get("sensors", {})
        stream_id = tracks_msg.get("stream_id")
        if stream_id and self.stream_id and stream_id != self.stream_id:
            self.fusion = None
            self.fusion_eo = None
            self.sensors_info = {}
        elif self.last_tick and tick <= self.last_tick:
            return
        self.stream_id = stream_id
        self.last_tick = tick
        self.sync_fusion(sensors_info, tracks_msg.get("area"))
        if self.fusion is None:
            return

        radar_tracks = tracks_msg.get("radar_tracks", [])
        bearing_tracks = tracks_msg.get("bearing_tracks", {})
        spectrum = tracks_msg.get("spectrum", {})

        for engine in (self.fusion, self.fusion_eo):
            engine.dt = self.dt
            for track in engine.tracks:
                track.dt = self.dt
                track.vertical.dt = self.dt
            if tracks_msg.get("live_timing"):
                engine.time_s = float(tracks_msg["source_timestamp_ms"]) / 1000
        xchk = self.fusion.step(radar_tracks, bearing_tracks, spectrum)
        fused = self.fusion.export()
        external_tracks = self.fusion.export(include_coasting=True)
        official_tracks = self.output_filter.select(external_tracks, fused, sensors_info,
                                                    tracks_msg.get("source_timestamp_ms", 0))

        eo_btracks = {
            sid: bearing_tracks[sid]
            for sid, info in sensors_info.items()
            if info.get("kind") == "eo" and sid in bearing_tracks
        }
        self.fusion_eo.step([], eo_btracks, {})

        truth = tracks_msg.get("truth", [])
        metrics = evaluate(
            truth,
            radar_tracks,
            bearing_tracks,
            fused,
            self.sensors,
            self.fusion,
            eo_fused=self.fusion_eo.export(),
        )
        metrics["spectrum"] = dict(spectrum, **xchk)
        metrics["ground_truth_available"] = bool(truth)
        metrics["mode"] = "company_live_data"

        fused_msg = {
            "type": "fused_tracks",
            "tick": tick,
            "t": tracks_msg.get("t"),
            "fused": fused,
            # 外部后端需要持续收到低置信度滑行航迹，直到融合器正式删除。
            # 实装页面也展示滑行；正式 FusionMultiTarget 在 fused 基础上执行输出过滤。
            "external_tracks": external_tracks,
            "pending": self.fusion.export_pending(),
            "metrics": metrics,
            "fusion_params": self.fusion.params(),
            "source_timestamp_ms": tracks_msg.get("source_timestamp_ms", 0),
            "coordinates": tracks_msg.get("coordinates", {}),
        }
        frame = {
            "type": "frame",
            "t": tracks_msg.get("t"),
            "tick": tick,
            "truth": truth,
            "sensors": build_sensors_dyn(sensors_info, tracks_msg.get("measurements", {})),
            "sensor_tracks": tracks_msg.get("sensor_tracks", {}),
            "sensors_raw": tracks_msg.get("measurements", {}),
            "fused": external_tracks if tracks_msg.get("live_timing") else fused,
            "metrics": metrics,
            "pending": self.fusion.export_pending(),
            "fusion_params": self.fusion.params(),
            "source_timestamp_ms": tracks_msg.get("source_timestamp_ms", 0),
            "coordinates": tracks_msg.get("coordinates", {}),
            "area": tracks_msg.get("area", {}),
            "data_source": tracks_msg.get(
                "data_source", "anti-drone-pkg/dev_proto"
            ),
        }

        self.bus.write(Topics.FUSED_TRACKS.name, fused_msg, kind="fused_tracks", tick=tick)
        self.bus.write(Topics.FUSED_FRAME.name, frame, kind="frame", tick=tick)
        origin = tracks_msg.get("coordinates", {}).get(
            "origin", [120.0, 30.0, 0.0]
        )
        try:
            self.bus.publish_official_fusion(
                official_tracks,
                origin=origin,
                sensors_info=sensors_info,
                timestamp_ms=int(tracks_msg.get("source_timestamp_ms", 0) or 0),
            )
        except Exception as exc:
            self.output_filter.published(f"{type(exc).__name__}: {exc}")
            self.diagnostics.publish(force=True, output_filter=self.output_filter.snapshot())
            raise
        self.output_filter.published()
        self.diagnostics.publish(last_processed_at=time.time(), tick=tick,
            output_filter=self.output_filter.snapshot(), official_count=len(official_tracks),
            radar_input=len(radar_tracks), fused_count=len(fused),
            external_count=len(external_tracks), tracks=external_tracks[:100],
            output_min_confidence=self.min_confidence,
            direction_tracks=self.fusion.direction_tracks,
            geometry_events=list(self.fusion.geometry_history),
            eo_pending=[dict(devices=[key[0][0], key[1][0]], hits=value["hits"],
                             sigma_max_m=value["sigma_max_m"])
                        for key, value in self.fusion.eo_pending.items()][:100],
            published_topics=[Topics.FUSED_TRACKS.name, Topics.FUSED_FRAME.name, Topics.OFFICIAL_FUSION.name],
            last_published_at=time.time())

    def run(self):
        print("fusion_node: sensor tracks -> fused tracks")
        print(f"[DDS 输出过滤] config={self.output_filter.store.path} "
              f"settings={self.output_filter.settings}", flush=True)
        while True:
            self.process_controls()
            did_work = False
            for msg in self.bus.read(Topics.SENSOR_TRACKS.name):
                self.process_tracks(msg)
                did_work = True
            if not did_work:
                self.diagnostics.publish()
                time.sleep(0.01)


def main():
    argparse.ArgumentParser().parse_args()
    FusionDdsNode().run()


if __name__ == "__main__":
    main()
