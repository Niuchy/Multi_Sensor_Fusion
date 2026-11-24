import sys
import unittest
from pathlib import Path


DDS_DIR = Path(__file__).resolve().parents[1] / "dds_swarm_fusion"
if str(DDS_DIR) not in sys.path:
    sys.path.insert(0, str(DDS_DIR))

from vertical_filter import VerticalKalmanFilter  # noqa: E402


class VerticalKalmanFilterTests(unittest.TestCase):
    def test_estimates_constant_climb_rate(self):
        tracker = VerticalKalmanFilter(
            100.0,
            0.5,
            altitude_sigma=0.5,
            acceleration_sigma=0.2,
        )
        for index in range(1, 21):
            tracker.predict()
            tracker.update(100.0 + index, altitude_sigma=0.5)

        self.assertTrue(tracker.velocity_valid)
        self.assertAlmostEqual(tracker.altitude, 120.0, delta=0.2)
        self.assertAlmostEqual(tracker.velocity, 2.0, delta=0.1)

    def test_velocity_is_invalid_until_enough_height_updates(self):
        tracker = VerticalKalmanFilter(100.0, 0.5)
        self.assertFalse(tracker.velocity_valid)
        tracker.predict()
        tracker.update(101.0)
        self.assertFalse(tracker.velocity_valid)
        tracker.predict()
        tracker.update(102.0)
        self.assertTrue(tracker.velocity_valid)

    def test_missing_height_does_not_create_a_fake_velocity(self):
        tracker = VerticalKalmanFilter(None, 0.5)
        tracker.predict()
        tracker.update(None)
        self.assertIsNone(tracker.altitude)
        self.assertIsNone(tracker.velocity)
        self.assertFalse(tracker.velocity_valid)


if __name__ == "__main__":
    unittest.main()
