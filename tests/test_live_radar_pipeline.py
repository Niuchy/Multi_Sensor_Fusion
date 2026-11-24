import asyncio
import math
import sys
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
sys.path.insert(0, str(ROOT / "tools"))
from replay_radar_log import MemoryBus
from company_input_node import CompanyInputNode
from company_dds import Topics
from dds_tracking_node import TrackingDdsNode
from dds_fusion_node import FusionDdsNode
from fused_tracks_ws_sender import format_track, TRACK_COASTING
from device_radar import DeviceRadarTracker
from company_protocol import parse_company_packet, PXSB_HEADER_FORMAT


def report(timestamp=48265852, fresh=False, **overrides):
    values = dict(device_id=3152, track_id=89, timestamp_ms=timestamp,
        range_m=579.0, azimuth_deg=104.32, elevation_deg=6.25,
        height_m=63.0, radial_vel_ms=0.0, lng_deg=120.1467056,
        lat_deg=30.1104393, alt_m=87.0, speed_ms=5.4, course_deg=0.0,
        extrap_flag=int(fresh), target_type=0, confidence=80)
    values.update(overrides)
    return SimpleNamespace(**values)


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.bus = MemoryBus()
        self.input = CompanyInputNode(ROOT / "anti-drone-pkg/config/scenarios/multi_target.yaml",
            bus=self.bus, radar_input_mode="device_tracks", azimuth_convention="north_cw")
        self.tracking = TrackingDdsNode(bus=self.bus)
        # This fixture asserts the strict-confirmation policy. RK3588 field
        # default 0.05 is separately covered by test_rk3588_sync.py.
        self.fusion = FusionDdsNode(bus=self.bus, min_confidence=0.2)

    def send(self, obj):
        self.input.ingest(0x1A01, obj.timestamp_ms, obj)
        self.input.flush_due(force=True)
        return self.advance()

    def advance(self):
        self.tracking.process_raw(self.bus.messages[Topics.RAW_MEASUREMENTS.name])
        self.fusion.process_tracks(self.bus.messages[Topics.SENSOR_TRACKS.name])
        return self.bus.messages[Topics.FUSED_TRACKS.name]

    def test_logged_jitter_no_longer_resets_detection_tracker(self):
        self.input.radar_input_mode = "measurements"
        ids = []
        for stamp in (48265852, 48268832, 48271832, 48274833, 48277830, 48280831):
            msg = self.send(report(stamp, fresh=True))
            ids += [t["id"] for t in msg["fused"]]
        self.assertEqual(len(ids), 4)
        self.assertEqual(len(set(ids)), 1)
        self.assertEqual(self.tracking.reset_count, 0)

    def test_device_extrapolation_is_output_without_becoming_real_evidence(self):
        ids = []
        for stamp in (48265852, 48268832, 48271832, 48274833):
            msg = self.send(report(stamp))
            self.assertEqual(msg["fused"], [])
            self.assertEqual(len(msg["external_tracks"]), 1)
            track = msg["external_tracks"][0]
            ids.append(track["id"])
            self.assertTrue(track["is_coasting"])
            self.assertIsNone(track["last_measurement_ms"])
            self.assertLessEqual(track["conf"], 0.15)
            self.assertEqual(track["sensor_ids"], ["radar_3152"])
            row = format_track(track, origin=self.input.origin, timestamp_ms=stamp)
            self.assertEqual(row["status"], TRACK_COASTING)
        self.assertEqual(len(set(ids)), 1)
        self.assertEqual(self.tracking.radar_trks["radar_3152"].entries[89]["hits"], 0)

    def test_real_report_promotes_same_fused_id(self):
        old = self.send(report())["external_tracks"][0]["id"]
        new = self.send(report(48268832, fresh=True))["fused"][0]
        self.assertEqual(old, new["id"])
        self.assertFalse(new["is_coasting"])
        self.assertEqual(new["last_measurement_ms"], 48268832)

    def test_silence_expires_tracks_on_empty_heartbeat(self):
        self.send(report())
        self.input.publish_batch(48265852 + 13000, {"measurements": {}, "heartbeat": True})
        msg = self.advance()
        self.assertEqual(msg["external_tracks"], [])

    def test_duplicates_and_old_reports_do_not_reset_stream(self):
        self.send(report())
        tick = self.input.tick
        self.input.ingest(0x1A01, 0, report())
        self.input.ingest(0x1A01, 0, report(48265851))
        self.input.flush_due(force=True)
        self.assertEqual(self.input.tick, tick)
        self.assertEqual(self.input.duplicates, 1)
        self.assertEqual(self.input.dropped_old, 1)

    def test_device_ids_are_distinct_even_at_same_position(self):
        self.input.ingest(0x1A01, 0, report(track_id=89))
        self.input.ingest(0x1A01, 0, report(track_id=90))
        self.input.flush_due(force=True)
        msg = self.advance()
        self.assertEqual(len(msg["external_tracks"]), 2)

    def test_north_clockwise_converts_to_enu(self):
        sid, m = self.input._radar(report(lng_deg=0, lat_deg=0, azimuth_deg=0, elevation_deg=0, range_m=100))
        pos = self.input.sensors_info[sid]["pos"]
        self.assertAlmostEqual(m["x"], pos[0])
        self.assertAlmostEqual(m["y"], pos[1]+100)
        self.assertAlmostEqual(m["az"], math.pi/2)

    def test_actual_position_extends_demo_viewport(self):
        self.send(report())
        m = self.input.latest_samples["radar_3152"]
        self.assertGreater(self.input.area["xmax"], m["x"])
        self.assertGreater(self.input.area["ymax"], m["y"])

    def test_wire_report_with_both_supported_unit_headers(self):
        # Reconstructed from the first decoded log record. Not original bytes.
        stamp = 48265852
        frame = struct.pack("<IBHHHIQ", 0xA55AA55A, 1, 0x1A01, 1, 66, 3152, stamp)
        frame += struct.pack("<IIhhIhiiIIHBBB", 89, 5790, 10432, 625,
            630, 0, 1201467056, 301104393, 870, 54, 0, 0, 0, 80)
        frame += struct.pack("<I", 0)
        for with_time in (True, False):
            unit = struct.pack("<IH", 66, 0x1A01) + (struct.pack("<Q", stamp) if with_time else b"")
            header = struct.pack(PXSB_HEADER_FORMAT, 0x7E, 1, 3152, 0,
                                 len(unit)+len(frame), 0, 784, 0, 0, stamp)
            obj = parse_company_packet(header+unit+frame)[0][2]
            self.assertEqual(obj.track_id, 89)
            self.assertEqual(obj.extrap_flag, 0)
            self.assertAlmostEqual(obj.lng_deg, 120.1467056)
            broken = frame[:9] + struct.pack("<H", 65) + frame[11:]
            with self.assertRaises(ValueError):
                parse_company_packet(header+unit+broken)

    def test_other_target_reports_do_not_count_as_missing_scan(self):
        self.send(report())
        for index in range(1, 15):
            msg = self.send(report(48265852+index*100, track_id=90, lng_deg=120.15))
        self.assertEqual(len(msg["external_tracks"]), 2)

    def test_diagnostics_survive_parse_failure_and_no_fused_output(self):
        packets = [b"bad packet"]
        self.bus.read_bytes = lambda topic: packets if topic == "RadarTarget" else []
        self.input.poll_company_topics()
        self.input.publish_diagnostics(force=True)
        status = self.bus.messages[Topics.DIAGNOSTICS.name]
        self.assertEqual(status["topics"]["RadarTarget"]["packets"], 1)
        self.assertEqual(status["topics"]["RadarTarget"]["errors"], 1)
        self.assertEqual(status["raw_batches"], 0)
        from dds_ws_gateway import Gateway
        self.bus.read = lambda topic: [status] if topic == Topics.DIAGNOSTICS.name else []
        gateway = Gateway(bus=self.bus)
        asyncio.run(gateway.poll_once())
        self.assertTrue(gateway.diagnostics_snapshot()["nodes"]["input"]["alive"])
        self.assertIsNone(gateway.latest_frame)


if __name__ == "__main__":
    unittest.main()
