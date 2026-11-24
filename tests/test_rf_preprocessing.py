"""RF zero-angle site policy; in-memory only, no native DDS or services."""
import math
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
sys.path.insert(0, str(ROOT / "tools"))
from company_dds import Topics
from company_input_node import CompanyInputNode
from company_protocol import PXSB_HEADER_FORMAT
from dds_tracking_node import TrackingDdsNode
from dds_fusion_node import FusionDdsNode
from replay_radar_log import MemoryBus
from test_live_radar_pipeline import report as radar_report
from test_eo_fusion import obj as eo_report


def report(stamp=1000, angle=0.0, **overrides):
    values = dict(device_id=4001, target_id=0, timestamp_ms=stamp,
                  azimuth_deg=angle, freq_hz=0, bandw_hz=0, power_dbm=0.0,
                  sn="12345", brand_model="FanShuang-FreqDetect", confidence=0)
    values.update(overrides)
    return SimpleNamespace(**values)


def wire_packet():
    body = struct.pack("<I 16s 32s i i I I I I i i I I h H Q B 7s",
                       0, b"12345", b"FanShuang-FreqDetect",
                       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, bytes(7))
    size = 23 + len(body) + 4
    frame = struct.pack("<IBHHHIQ", 0xB44BB44B, 1, 0x1B01, 1,
                        size, 4001, 1000) + body + bytes(4)
    unit = struct.pack("<IHQ", len(frame), 0x1B01, 1000) + frame
    return struct.pack(PXSB_HEADER_FORMAT, 0x7E, 1, 4001, 0, len(unit),
                       0, 1, 0, 0, 1000) + unit


class RfPreprocessingTests(unittest.TestCase):
    def setUp(self):
        self.bus = MemoryBus()
        self.node = CompanyInputNode(ROOT / "configs/equipment.yaml", bus=self.bus,
                                     radar_input_mode="device_tracks")
        self.tracking = TrackingDdsNode(bus=self.bus)
        self.fusion = FusionDdsNode(bus=self.bus)
        log = patch("company_input_node.LOG")
        log.start()
        self.addCleanup(log.stop)

    def ingest(self, obj):
        self.node.ingest(0x1B01, obj.timestamp_ms, obj)

    def flush(self):
        self.node.flush_due(force=True)
        return self.bus.messages[Topics.RAW_MEASUREMENTS.name]

    def advance(self):
        self.tracking.process_raw(self.flush())
        self.fusion.process_tracks(self.bus.messages[Topics.SENSOR_TRACKS.name])
        return self.bus.messages[Topics.FUSED_FRAME.name]

    def test_zero_report_is_diagnostic_only_with_explicit_reason(self):
        self.ingest(report())
        raw = self.flush()
        self.assertEqual(raw["measurements"]["rf_df_4001"]["meas"], [])
        self.assertNotIn("rf_df_4001", raw["updated_sensors"])
        self.assertTrue(raw["sensors"]["rf_df_4001"]["received"])
        sample = self.node.latest_samples["rf_df_4001"]
        self.assertEqual(sample["raw_azimuth_deg"], 0)
        self.assertIsNone(sample["az"])
        self.assertFalse(sample["angle_valid"])
        self.assertFalse(sample["fresh"])
        self.assertEqual(sample["quality_reasons"], ["rf_zero_azimuth_placeholder"])
        self.node.publish_diagnostics(force=True)
        d = self.bus.messages[Topics.DIAGNOSTICS.name]
        self.assertEqual(d["rf_zero_azimuth_filtered"], 1)
        self.assertEqual(len(d["recent_samples"]), 1)
        self.assertEqual(d["device_stations"][0]["device_id"], 4001)

    def test_zero_rule_does_not_depend_on_id_sn_or_confidence(self):
        for i, angle in enumerate([0.0, -0.0]):
            self.ingest(report(1000 + i, angle, target_id=231, sn="real-sn", confidence=99))
            self.assertEqual(self.flush()["measurements"]["rf_df_4001"]["meas"], [])
        self.assertEqual(self.node.rf_zero_azimuth_filtered, 2)

    def test_small_and_other_nonzero_angles_keep_existing_convention(self):
        for i, angle in enumerate([0.01, 0.02, 52.0, 90.0, 180.0, 286.0, 359.99]):
            with self.subTest(angle=angle):
                self.ingest(report(1000 + i, angle))
                raw = self.flush()
                m = raw["measurements"]["rf_df_4001"]["meas"][0]
                self.assertTrue(m["angle_valid"])
                self.assertTrue(m["fresh"])
                self.assertAlmostEqual(m["az"], (math.radians(angle) + math.pi) % (2 * math.pi) - math.pi)
                self.assertIn("rf_df_4001", raw["updated_sensors"])
        self.assertEqual(self.node.rf_zero_azimuth_filtered, 0)

    def test_mixed_targets_same_batch_keep_valid_reports_in_either_order(self):
        for i, order in enumerate([(0, 52), (52, 0)]):
            stamp = 1000 + i
            for angle in order:
                self.ingest(report(stamp, angle, target_id=0 if angle == 0 else 231))
            raw = self.flush()
            meas = raw["measurements"]["rf_df_4001"]["meas"]
            self.assertEqual(len(meas), 1)
            self.assertEqual(meas[0]["track_id"], 231)
            self.assertIn("rf_df_4001", raw["updated_sensors"])

    def test_zero_does_not_disable_other_rf_devices(self):
        self.ingest(report())
        self.ingest(report(angle=60, device_id=4002))
        raw = self.flush()
        self.assertEqual(raw["measurements"]["rf_df_4001"]["meas"], [])
        self.assertEqual(len(raw["measurements"]["rf_df_4002"]["meas"]), 1)

    def test_zero_packets_are_received_and_parsed_not_protocol_errors(self):
        self.bus.read_bytes = lambda topic: [wire_packet()] if topic == "ESMTarget" else []
        self.assertEqual(self.node.poll_company_topics(), 1)
        self.flush()
        self.node.publish_diagnostics(force=True)
        d = self.bus.messages[Topics.DIAGNOSTICS.name]
        self.assertEqual(d["topics"]["ESMTarget"]["packets"], 1)
        self.assertEqual(d["topics"]["ESMTarget"]["parsed"], 1)
        self.assertEqual(d["topics"]["ESMTarget"]["errors"], 0)
        self.assertEqual(d["rf_zero_azimuth_filtered"], 1)

    def test_duplicate_zero_is_not_counted_twice(self):
        self.ingest(report())
        self.ingest(report())
        self.assertEqual(self.node.duplicates, 1)
        self.assertEqual(self.node.rf_zero_azimuth_filtered, 1)

    def test_placeholder_only_pipeline_has_no_lines_tracks_or_fusion(self):
        for i in range(8):
            self.ingest(report(1000 + i * 500))
            frame = self.advance()
            self.assertEqual(frame["sensors_raw"]["rf_df_4001"]["meas"], [])
            self.assertEqual(frame["sensor_tracks"]["rf_df_4001"], [])
            self.assertEqual(frame["fused"], [])
            self.assertEqual(self.tracking.bearing_trks["rf_df_4001"].tracks, [])

    def test_placeholder_does_not_refresh_or_erase_existing_nonzero_track(self):
        for i in range(4):
            self.ingest(report(1000 + i * 500, 52, target_id=231, sn="", confidence=89))
            self.advance()
        tracker = self.tracking.bearing_trks["rf_df_4001"]
        self.assertEqual(len(tracker.tracks), 1)
        hits, ident = tracker.tracks[0].hits, tracker.tracks[0].id
        self.ingest(report(3000))
        frame = self.advance()
        self.assertEqual(tracker.tracks[0].hits, hits)
        self.assertEqual(tracker.tracks[0].id, ident)
        self.assertFalse(frame["sensor_tracks"]["rf_df_4001"][0]["fresh"])
        self.assertEqual(frame["sensors_raw"]["rf_df_4001"]["meas"], [])
        self.ingest(report(3500, 52, target_id=231, sn="", confidence=89))
        frame = self.advance()
        self.assertTrue(frame["sensor_tracks"]["rf_df_4001"][0]["fresh"])
        self.assertEqual(tracker.tracks[0].id, ident)

    def test_zero_radar_eo_and_acoustic_angles_are_not_rf_placeholders(self):
        radar = radar_report(timestamp=1000, azimuth_deg=0)
        self.node.ingest(0x1A01, 1000, radar)
        eo = eo_report(1000, target_azimuth_deg=0)
        self.node.ingest(0x1C01, 1000, eo)
        ac = SimpleNamespace(device_id=4201, target_id=4, timestamp_ms=1000,
                             target_yaw_deg=0, target_pitch_deg=0, target_type=1,
                             confidence=90)
        self.node.ingest(0x1D01, 1000, ac)
        raw = self.flush()
        for sid in ["radar_3152", "eo_201", "acoustic_4201"]:
            self.assertEqual(len(raw["measurements"][sid]["meas"]), 1)
        self.assertEqual(self.node.rf_zero_azimuth_filtered, 0)


if __name__ == "__main__":
    unittest.main()
