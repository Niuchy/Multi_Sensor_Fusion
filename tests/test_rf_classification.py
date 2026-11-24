"""RF model-only class fallback; offline tests, never publishes native DDS."""
import math
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
sys.path.insert(0, str(ROOT / "tools"))
from company_dds import Topics, _fusion_target_type
from dds_frame import SensorStub
from fusion import Fusion
from rf_classification import RFClassification
from tracking import BearingTracker
import test_class_fusion as class_fixtures
from test_rf_preprocessing import report

MODEL = "Dji_Mavic3_T/Dji_Air3"


def rf(stamp=1000, **kw):
    return report(stamp, angle=45., target_id=231, brand_model=MODEL,
                  confidence=89, **kw)


class ModelAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.policy = RFClassification(ROOT)

    def test_observed_aircraft_models_are_explicitly_allowlisted(self):
        for name in self.policy.profile(4001)["drone_models"]:
            observation = rf()
            observation.brand_model = name
            e = self.policy.classify(observation)
            self.assertTrue(e["valid"], name)
            self.assertEqual(e["cls"], "drone")
            self.assertEqual(e["confidence"], .89)

    def test_controller_placeholder_band_and_partial_models_are_not_aircraft(self):
        for name in ("", "FanShuang-FreqDetect", "12345", "DJI", "Dji_control",
                     "Air3_L_control/Mini4_Pro_controlDJI_CONTROL", "2.40GHz",
                     "Mini4", "DJI-NewUnknownModel", "drone", MODEL + "_unknown"):
            observation = rf()
            observation.brand_model = name
            self.assertFalse(self.policy.classify(observation)["valid"], name)

    def test_case_and_padding_only_are_normalized(self):
        observation = rf()
        observation.brand_model = "  DJI_MAVIC3_T/DJI_AIR3\0 "
        self.assertTrue(self.policy.classify(observation)["valid"])

    def test_missing_or_invalid_confidence_never_becomes_ninety_percent(self):
        for confidence in (None, "bad", -1, 0, 49, 101, float("nan"), float("inf")):
            observation = rf()
            observation.confidence = confidence
            self.assertFalse(self.policy.classify(observation)["valid"])

    def test_unconfigured_device_is_not_enabled_by_another_devices_models(self):
        observation = rf(device_id=4002)
        self.assertFalse(self.policy.classify(observation)["valid"])

    def test_full_width_wire_model_is_rejected_even_if_allowlisted(self):
        name = "D" * 32
        self.policy.devices["4001"]["drone_models"] = [name]
        observation = rf()
        observation.brand_model = name
        self.assertEqual(self.policy.classify(observation)["reason"], "model_may_be_truncated")

    def test_explicit_configuration_is_loaded_and_bad_policy_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "rf.yaml"
            data = dict(devices={"4002": dict(drone_models=["SITE_VERIFIED_AIRCRAFT"])})
            path.write_text(yaml.safe_dump(data))
            with patch.dict(os.environ, {"SB_RF_CONFIG": str(path)}):
                p = RFClassification(ROOT)
            observation = rf(device_id=4002)
            observation.brand_model = "SITE_VERIFIED_AIRCRAFT"
            self.assertTrue(p.classify(observation)["valid"])
            self.assertEqual(p.path, path.resolve())
            for values in ({"enabled": "false"}, {"min_confidence": 0},
                           {"confirm_reports": 1}, {"drone_models": "DJI"},
                           {"drone_models": ["DJI*"]}, {"max_gap_ms": -1}):
                path.write_text(yaml.safe_dump(dict(defaults=values)))
                with self.subTest(values=values), self.assertRaises(ValueError):
                    RFClassification(ROOT, path)
            with self.assertRaises(ValueError):
                RFClassification(ROOT, Path(directory) / "missing.yaml")


class ClassGeometryTests(unittest.TestCase):
    def setUp(self):
        self.policy = RFClassification(ROOT)
        self.f = Fusion({"rf_df_4001": SensorStub("rf_df_4001", "rf_df", [0, 0, 0])}, .5)

    def radar(self, ident=1, xy=(500, 500)):
        return dict(id=ident, x=xy[0], y=xy[1], z=100., vx=0., vy=0.,
                    cov=[[25., 0.], [0., 25.]], fresh=True, cls="unknown", cls_conf=.8,
                    sensor_id="radar_3152")

    def bearing(self, stamp, ident=231, model=MODEL, angle=45., **kw):
        observation = rf(stamp)
        observation.target_id, observation.brand_model = ident, model
        e = self.policy.classify(observation)
        return dict(id=ident, source_track_id=ident, timestamp_ms=stamp, az=math.radians(angle),
                    cls=model, model=model, fresh=True, angle_valid=True,
                    rf_classification=e, **kw)

    def step(self, stamp, bts=None, radars=None):
        self.f.time_s = stamp / 1000
        self.f.step([self.radar()] if radars is None else radars,
                    {"rf_df_4001": bts if bts is not None else [self.bearing(stamp)]}, {})
        return self.f.export(include_coasting=True)

    def establish(self):
        for stamp in (1000, 1500, 2000):
            rows = self.step(stamp)
        self.assertEqual(rows[0]["cls"], "drone")
        return rows[0]

    def test_three_unique_reports_then_unknown_only_fallback(self):
        for i in range(3):
            row = self.step(1000 + i * 500)[0]
            self.assertEqual(row["cls"], "drone" if i == 2 else "unknown")
        self.assertEqual(row["class_basis"], "rf_model")
        self.assertTrue(row["class_evidence"])
        self.assertEqual(row["rf_classification"]["confirmation_reports"], 3)
        self.assertEqual(self.f.tracks[0].class_updates, 0)
        self.assertEqual(_fusion_target_type(row["cls"]), 0)

    def test_duplicates_cannot_confirm_or_extend_expiry(self):
        for stamp in (1000, 1100, 1200, 1300):
            rows = self.step(stamp, [self.bearing(1000)])
        self.assertEqual(rows[0]["cls"], "unknown")
        self.step(1500)
        row = self.step(2000)[0]
        expiry = row["rf_classification"]["expires_at_ms"]
        row = self.step(2500, [self.bearing(2000)])[0]
        self.assertEqual(row["rf_classification"]["expires_at_ms"], expiry)

    def test_late_future_and_predicted_reports_cannot_classify(self):
        for offset, fresh in ((-6000, True), (1000, True), (0, False)):
            self.setUp()
            for stamp in (10000, 10500, 11000):
                bt = self.bearing(stamp + offset)
                bt["fresh"] = fresh
                rows = self.step(stamp, [bt])
            self.assertEqual(rows[0]["cls"], "unknown")

    def test_gap_or_model_change_requires_new_confirmation(self):
        self.step(1000)
        self.step(1500)
        self.assertEqual(self.step(7000)[0]["cls"], "unknown")
        self.step(7500)
        self.assertEqual(self.step(8000, [self.bearing(8000, model="Mini4Pro_L")])[0]["cls"], "unknown")
        self.step(8500, [self.bearing(8500, model="Mini4Pro_L")])
        self.assertEqual(self.step(9000, [self.bearing(9000, model="Mini4Pro_L")])[0]["cls"], "drone")

    def test_same_bearing_two_radar_targets_does_not_label_either(self):
        radars = [self.radar(1), self.radar(2, (900, 900))]
        for stamp in (1000, 1500, 2000, 2500):
            rows = self.step(stamp, radars=radars)
        self.assertEqual(len(rows), 2)
        self.assertEqual({r["cls"] for r in rows}, {"unknown"})

    def test_two_rf_targets_one_radar_is_ambiguous(self):
        for stamp in (1000, 1500, 2000):
            rows = self.step(stamp, [self.bearing(stamp), self.bearing(stamp, ident=232)])
        self.assertEqual(rows[0]["cls"], "unknown")

    def test_old_binding_does_not_hide_new_ambiguous_target(self):
        self.establish()
        rows = self.step(2500, radars=[self.radar(1), self.radar(2, (900, 900))])
        self.assertEqual({r["cls"] for r in rows}, {"unknown"})

    def test_bad_model_or_unmatched_report_retracts_fallback(self):
        for bt in (self.bearing(2500, model="Dji_control"), self.bearing(2500, angle=130.)):
            self.setUp()
            self.establish()
            row = self.step(2500, [bt])[0]
            self.assertEqual(row["cls"], "unknown")
            self.assertFalse(row["class_evidence"])

    def test_expiry_does_not_delete_radar_geometry(self):
        row = self.establish()
        ident = row["id"]
        for stamp in range(2500, 14501, 500):
            row = self.step(stamp, [])[0]
        self.assertEqual(row["id"], ident)
        self.assertEqual(row["cls"], "unknown")
        self.assertIn("radar", row["sources"])

    def test_known_class_before_or_after_rf_always_has_priority(self):
        self.establish()
        self.f.tracks[0].add_class("bird", .6)
        for stamp in (2500, 3000, 3500):
            row = self.step(stamp)[0]
            self.assertEqual(row["cls"], "bird")
            self.assertEqual(row["class_basis"], "sensor_class")
        self.assertEqual(self.f.tracks[0].class_updates, 1)

    def test_no_association_or_no_radar_never_generates_classified_target(self):
        for stamp in (1000, 1500, 2000):
            rows = self.step(stamp, radars=[])
        self.assertEqual(rows, [])
        self.setUp()
        for stamp in (1000, 1500, 2000):
            rows = self.step(stamp, [self.bearing(stamp, angle=130.)])
        self.assertEqual(rows[0]["cls"], "unknown")


class NativePipelineTests(unittest.TestCase):
    def setUp(self):
        # Reuse fixtures, not the other suite's test methods.
        self.p = class_fixtures.ClassPipelineTests()
        self.p.setUp()

    def send_rf(self, stamp, **overrides):
        observation = rf(stamp)
        for key, value in overrides.items():
            setattr(observation, key, value)
        return self.p.send(0x1B01, observation)

    def test_full_input_tracking_fusion_preserves_actual_confidence(self):
        ident = self.p.radar(1000)[0]["id"]
        for i in range(5):
            row = self.send_rf(1100 + i * 500)[0]
            self.assertEqual(row["cls"], "drone" if i == 4 else "unknown")
        self.assertEqual(row["id"], ident)
        self.assertAlmostEqual(row["rf_classification"]["confidence"], .89)
        self.assertEqual(row["source_bindings"]["rf_df_4001"], 231)
        raw = self.p.bus.messages[Topics.SENSOR_TRACKS.name]["bearing_tracks"]["rf_df_4001"][0]
        self.assertEqual(raw["cls"], MODEL)  # legacy identity display unchanged
        self.assertEqual(raw["cls_conf"], .89)
        self.assertEqual(raw["timestamp_ms"], 3100)
        self.p.input.publish_diagnostics(force=True)
        self.assertTrue(self.p.bus.messages[Topics.DIAGNOSTICS.name]["rf_config_path"].endswith("rf_devices.yaml"))

    def test_zero_angle_remains_filtered_even_with_valid_aircraft_model(self):
        self.p.radar(1000)
        for i in range(6):
            row = self.send_rf(1100 + i * 500, azimuth_deg=0.)[0]
            self.assertEqual(row["cls"], "unknown")
        self.assertEqual(self.p.input.rf_zero_azimuth_filtered, 6)

    def test_eo_class_overrides_prior_rf_without_accumulating_against_it(self):
        self.p.radar(1000)
        for i in range(5):
            row = self.send_rf(1100 + i * 500)[0]
        self.assertEqual(row["cls"], "drone")
        for i in range(3):
            row = self.p.eo(3200 + i * 100, target_type=0)[0]  # EO bird
        self.assertEqual(row["cls"], "bird")
        row = self.send_rf(3600)[0]
        self.assertEqual(row["cls"], "bird")

    def test_missing_new_classification_clears_snapshot_in_tracker(self):
        policy = RFClassification(ROOT)
        measurement = dict(az=.5, proto=MODEL, confidence=.89,
                           rf_classification=policy.classify(rf()))
        tracker = BearingTracker(.5)
        for _ in range(3):
            tracker.process([measurement])
        self.assertTrue(tracker.export()[0]["rf_classification"]["valid"])
        tracker.process([dict(az=.5, proto="2.4GHz")])
        row = tracker.export()[0]
        self.assertNotIn("rf_classification", row)
        self.assertEqual(row["cls_conf"], 0)

    def test_distinct_device_target_ids_do_not_exchange_model_evidence(self):
        policy = RFClassification(ROOT)
        a, b = rf(), rf()
        b.target_id, b.brand_model = 232, "Dji_control"
        tracker = BearingTracker(.5)
        for i in range(4):
            measurements = [dict(az=.5, proto=o.brand_model, confidence=.89,
                                 rf_classification=policy.classify(o)) for o in (a, b)]
            tracker.process(measurements if i % 2 else list(reversed(measurements)))
        rows = {r["source_track_id"]: r for r in tracker.export()}
        self.assertEqual(set(rows), {231, 232})
        self.assertTrue(rows[231]["rf_classification"]["valid"])
        self.assertFalse(rows[232]["rf_classification"]["valid"])


if __name__ == "__main__":
    unittest.main()
