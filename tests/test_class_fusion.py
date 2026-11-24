"""Category evidence regressions; in-memory pipeline, never publishes DDS."""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
sys.path.insert(0, str(ROOT / "tools"))

from company_dds import Topics, _fusion_target_type
from company_input_node import CompanyInputNode
from dds_fusion_node import FusionDdsNode
from dds_tracking_node import TrackingDdsNode
from eo_observations import DEFAULTS
from fusion import FusedTrack, Fusion
from replay_radar_log import MemoryBus
from test_eo_fusion import obj
from test_live_radar_pipeline import report


class ClassEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.track = FusedTrack([500, 500, 0, 0], np.eye(4), .1, "radar")

    def test_no_evidence_exports_unknown_even_without_device_binding(self):
        fusion = Fusion({}, .1)
        fusion.tracks = [self.track]
        row = fusion.export(include_coasting=True)[0]
        self.assertEqual(row["cls"], "unknown")
        self.assertFalse(row["class_evidence"])
        self.assertEqual(_fusion_target_type(row["cls"]), 255)

    def test_unknown_never_adds_evidence_even_with_high_confidence(self):
        before = dict(self.track.cls_logp)
        for _ in range(100):
            self.track.add_class("unknown", .99)
        self.assertEqual(self.track.cls_logp, before)
        self.assertEqual(self.track.class_updates, 0)
        self.assertEqual(self.track.best_class(), "unknown")

    def test_unknown_does_not_overturn_valid_classification(self):
        self.track.add_class("drone", .9)
        before = dict(self.track.cls_logp)
        for _ in range(100):
            self.track.add_class("unknown", .99)
        self.assertEqual(self.track.cls_logp, before)
        self.assertEqual(self.track.class_updates, 1)
        self.assertEqual(self.track.best_class(), "drone")

    def test_invalid_category_and_confidence_are_not_evidence(self):
        for label, conf in [(None, .9), ("unsupported", .9), ("", .9)]:
            self.track.add_class(label, conf)
        for conf in (0, -1, None, "invalid", float("nan"), float("inf")):
            self.track.add_class("drone", conf)
        self.assertEqual(self.track.class_updates, 0)
        self.assertEqual(self.track.best_class(), "unknown")

    def test_known_conflicting_classes_keep_existing_accumulation(self):
        self.track.add_class("bird", .9)
        self.track.add_class("bird", .9)
        self.track.add_class("drone", .9)
        self.assertEqual(self.track.best_class(), "bird")
        self.track.add_class("drone", .9)
        self.track.add_class("drone", .9)
        self.assertEqual(self.track.best_class(), "drone")
        self.assertEqual(self.track.class_updates, 5)

    def test_other_is_a_real_class_and_dds_mapping_is_unchanged(self):
        self.track.add_class("other", .8)
        self.assertEqual(self.track.best_class(), "other")
        self.assertEqual(self.track.class_updates, 1)
        expected = {"drone": 0, "quad": 0, "fixed_wing": 0, "heli": 0,
                    "bird": 1, "large_target": 2, "vehicle": 3,
                    "person": 4, "other": 5, "unknown": 255}
        for label, value in expected.items():
            self.assertEqual(_fusion_target_type(label), value)


class ClassPipelineTests(unittest.TestCase):
    def setUp(self):
        self.bus = MemoryBus()
        self.input = CompanyInputNode(ROOT / "configs/equipment.yaml", bus=self.bus,
            radar_input_mode="device_tracks", azimuth_convention="north_cw")
        # Isolate classification tests from field-site calibration edits.
        self.input.eo_observations.defaults = dict(DEFAULTS)
        self.input.eo_observations.devices = {"201": dict(
            station_enu=[0., 0., 0.], angles_calibrated=True)}
        self.tracking = TrackingDdsNode(bus=self.bus)
        self.fusion = FusionDdsNode(bus=self.bus, min_confidence=.2)
        lng, lat, alt = self.input.origin
        self.target = (lng + 500 / self.input.converter.meters_per_deg_lng,
                       lat + 500 / self.input.converter.METERS_PER_DEG_LAT, alt + 100)
        x, y, _ = self.input.converter.to_enu(*self.target)
        self.azimuth = math.degrees(math.atan2(x, y))

    def send(self, tag, observation):
        self.input.ingest(tag, observation.timestamp_ms, observation)
        self.input.flush_due(force=True)
        self.tracking.process_raw(self.bus.messages[Topics.RAW_MEASUREMENTS.name])
        self.fusion.process_tracks(self.bus.messages[Topics.SENSOR_TRACKS.name])
        return self.bus.messages[Topics.FUSED_TRACKS.name]["fused"]

    def radar(self, stamp, **kwargs):
        lng, lat, alt = self.target
        return self.send(0x1A01, report(stamp, fresh=True, lng_deg=lng,
            lat_deg=lat, alt_m=alt, speed_ms=0., **kwargs))

    def eo(self, stamp, *, mode="position", **kwargs):
        values = dict(target_dis_m=0., target_azimuth_deg=self.azimuth, confidence=90)
        if mode == "position":
            values.update(zip(("target_lng_deg", "target_lat_deg", "target_alt_m"), self.target))
        values.update(kwargs)
        return self.send(0x1C01, obj(stamp, **values))

    def radar_history(self):
        ids = []
        for i in range(10):
            rows = self.radar(1000 + 3000 * i)
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["cls"], "unknown")
            ids.append(rows[0]["id"])
        self.assertEqual(len(set(ids)), 1)
        return ids[0]

    def test_radar_unknown_keeps_geometry_but_no_category_evidence(self):
        self.radar_history()
        row = self.bus.messages[Topics.FUSED_TRACKS.name]["fused"][0]
        self.assertFalse(row["class_evidence"])
        self.assertIn("radar", row["sources"])
        self.assertFalse(row["is_coasting"])
        x, y, _ = self.input.converter.to_enu(*self.target)
        self.assertAlmostEqual(row["x"], x, delta=1.)
        self.assertAlmostEqual(row["y"], y, delta=1.)

    def test_associated_eo_position_or_bearing_classifies_after_confirmation(self):
        for mode in ("position", "bearing"):
            with self.subTest(mode=mode):
                self.setUp()
                ident = self.radar_history()
                for i in range(3):
                    rows = self.eo(28100 + i * 100, mode=mode)
                    self.assertEqual(len(rows), 1)
                    self.assertEqual(rows[0]["id"], ident)
                    # EO's normal three-hit track confirmation is not bypassed.
                    self.assertEqual(rows[0]["cls"], "unknown" if i < 2 else "drone")
                self.assertTrue(rows[0]["class_evidence"])
                self.assertEqual(rows[0]["source_bindings"]["eo_201"], 7)
                self.assertEqual(_fusion_target_type(rows[0]["cls"]), 0)
                rows = self.radar(31000)
                self.assertEqual(rows[0]["id"], ident)
                self.assertEqual(rows[0]["cls"], "drone")

    def test_unassociated_bearing_does_not_transfer_eo_class(self):
        ident = self.radar_history()
        for i in range(4):
            rows = self.eo(28100 + i * 100, mode="bearing",
                           target_azimuth_deg=self.azimuth + 90.)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["id"], ident)
        self.assertEqual(rows[0]["cls"], "unknown")
        self.assertFalse(rows[0]["class_evidence"])
        self.assertNotIn("eo_201", rows[0]["source_bindings"])

    def test_unknown_zero_confidence_or_invalid_eo_cannot_classify(self):
        for options in ({"target_type": 3}, {"confidence": 0}, {"track_status": 0},
                        {"fault_status": 1}):
            with self.subTest(options=options):
                self.setUp()
                ident = self.radar_history()
                for i in range(4):
                    rows = self.eo(28100 + i * 100, **options)
                self.assertEqual(len(rows), 1)
                self.assertEqual(rows[0]["id"], ident)
                self.assertEqual(rows[0]["cls"], "unknown")
                self.assertFalse(rows[0]["class_evidence"])

    def test_known_radar_class_still_contributes(self):
        rows = self.radar(1000, target_type=3)
        self.assertEqual(rows[0]["cls"], "vehicle")
        self.assertTrue(rows[0]["class_evidence"])


if __name__ == "__main__":
    unittest.main()
