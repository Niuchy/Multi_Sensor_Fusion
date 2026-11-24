"""Runtime constants shared by the live company-data pipeline."""

import math


AREA = {"xmin": -5000.0, "xmax": 5000.0, "ymin": -5000.0, "ymax": 8000.0}
CLASSES = [
    "drone",
    "quad",
    "fixed_wing",
    "heli",
    "bird",
    "vehicle",
    "person",
    "large_target",
    "other",
    "unknown",
]


def wrap(angle):
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
