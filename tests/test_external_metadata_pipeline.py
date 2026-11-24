import sys
import unittest
from pathlib import Path

import numpy as np


DDS_DIR = Path(__file__).resolve().parents[1] / "dds_swarm_fusion"
if str(DDS_DIR) not in sys.path:
    sys.path.insert(0, str(DDS_DIR))

from dds_frame import flatten_radar_tracks  # noqa: E402
from fusion import FusedTrack, Fusion  # noqa: E402
from tracking import BearingTracker, RadarTracker  # noqa: E402


class ExternalMetadataPipelineTests(unittest.TestCase):
    def test_flatten_radar_tracks_preserves_concrete_sensor_id(self):
        rows = flatten_radar_tracks({
            "radar_101": [{"id": 1}],
            "radar_202": [{"id": 2}],
        })
        self.assertEqual(
            [(row["id"], row["sensor_id"]) for row in rows],
            [(1, "radar_101"), (2, "radar_202")],
        )

    def test_sensor_trackers_preserve_class_and_esm_metadata(self):
        radar = RadarTracker(0.5)
        radar_measurement = {
            "x": 100.0,
            "y": 200.0,
            "z": 50.0,
            "r": 223.6,
            "az": math_atan2(200.0, 100.0),
            "cls": "drone",
            "cls_conf": 0.8,
        }
        for _ in range(3):
            radar.process({"meas": [radar_measurement], "fa": []})
        radar_rows = radar.export()
        self.assertEqual(len(radar_rows), 1)
        self.assertEqual(radar_rows[0]["cls"], "drone")
        self.assertAlmostEqual(radar_rows[0]["cls_conf"], 0.8)

        esm = BearingTracker(0.5)
        esm_measurement = {
            "az": 0.3,
            "proto": "DJI-M300",
            "sn_id": "SN123456",
            "model": "DJI-M300",
            "rf_freq": 2_400_000_000.0,
            "confidence": 0.85,
        }
        for _ in range(3):
            esm.process([esm_measurement])
        esm_rows = esm.export()
        self.assertEqual(len(esm_rows), 1)
        self.assertEqual(esm_rows[0]["sn_id"], "SN123456")
        self.assertEqual(esm_rows[0]["model"], "DJI-M300")
        self.assertEqual(esm_rows[0]["rf_freq"], 2_400_000_000.0)
        self.assertAlmostEqual(esm_rows[0]["rf_confidence"], 0.85)

    def test_fusion_external_export_includes_age_sources_and_rf(self):
        fused_track = FusedTrack(
            np.array([10.0, 20.0, 2.0, 3.0]),
            np.eye(4),
            0.5,
            origin="radar",
            alt=30.0,
        )
        fused_track.touch(
            "radar", 4, position_capable=True, sensor_id="radar_101"
        )
        fused_track.touch("rf_df_103", 4)
        fused_track.add_class("drone", 0.8)
        fused_track.update_rf_metadata({
            "rf_freq": 2_400_000_000,
            "sn_id": "SN123456",
            "model": "DJI-M300",
            "rf_confidence": 0.85,
        }, 4)

        fusion = Fusion({}, 0.5)
        fusion.tick = 6
        fusion.tracks = [fused_track]
        row = fusion.export(include_coasting=True)[0]

        self.assertEqual(row["age_ticks"], 2)
        self.assertEqual(row["sensor_ids"], ["radar_101", "rf_df_103"])
        self.assertTrue(row["class_evidence"])
        self.assertEqual(row["rf_freq"], 2_400_000_000.0)
        self.assertEqual(row["sn_id"], "SN123456")
        self.assertEqual(row["model"], "DJI-M300")

    def test_vertical_velocity_flows_from_radar_to_fusion(self):
        radar = RadarTracker(
            0.5,
            sig_r=0.5,
            sig_el=np.deg2rad(0.01),
        )
        for index in range(20):
            measurement = {
                "x": 100.0,
                "y": 200.0,
                "z": 50.0 + index,
                "r": 230.0,
                "az": math_atan2(200.0, 100.0),
                "el": math_atan2(50.0 + index, 223.6),
            }
            radar.process({"meas": [measurement], "fa": []})

        radar_row = radar.export()[0]
        self.assertAlmostEqual(radar_row["vz"], 2.0, delta=0.2)

        fusion = Fusion({}, 0.5)
        fusion.step([radar_row], {}, {})
        fused_row = fusion.export(include_coasting=True)[0]
        self.assertAlmostEqual(fused_row["vz"], 2.0, delta=0.2)


def math_atan2(y, x):
    return float(np.arctan2(y, x))


if __name__ == "__main__":
    unittest.main()
