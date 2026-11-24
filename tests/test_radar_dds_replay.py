import struct
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from replay_radar_log import read_radar_log
from replay_radar_dds import encode_radar_packet, make_schedule, inspect_official_packet
from company_protocol import PXSB_HEADER_FORMAT, parse_company_packet

RECORD = """
╔══ Radar #1 ═══════════════════════════════════
  [PXSB] src=0x0C50 dst=0x0000 msgID=784 units=1 unitTag=0x1A01 unitLen=66
  [Frame] magic=0xA55AA55A(Radar) ver=1 ftype=0x1A01 dtype=1 len=66 dev=3152 ts=13:24:25.852
  [Body] track_id=89
  L1 量测: dis=579.0m azi=104.32° ele=6.25° height=63.0m radial_vel=0.0m/s
  L2 航迹: lng=120.1467056° lat=30.1104393° alt=87.0m speed=5.4m/s course=0.00° tag=外推点
  L3 识别: type=未知 conf=80%
  [Tail] crc32=0x00000000
"""


class RadarDdsReplayTests(unittest.TestCase):
    def parse(self, text):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "radar.log"
            path.write_text(text, encoding="utf-8")
            return read_radar_log(path)

    def test_binary_round_trip_preserves_logged_fields(self):
        row = self.parse(RECORD)[0]
        packet = encode_radar_packet(row, 1_790_000_000_123)
        self.assertEqual(len(packet), 106)
        header = struct.unpack_from(PXSB_HEADER_FORMAT, packet)
        self.assertEqual(header[2:7], (3152, 0, 80, 0, 784))
        self.assertEqual(packet[-4:], b"\0" * 4)
        _, source_stamp, parsed = parse_company_packet(packet)[0]
        self.assertEqual(source_stamp, 1_790_000_000_123)
        self.assertEqual(parsed.timestamp_ms, source_stamp)
        for field in ("device_id", "track_id", "range_m", "azimuth_deg", "elevation_deg",
                      "height_m", "radial_vel_ms", "lng_deg", "lat_deg", "alt_m",
                      "speed_ms", "course_deg", "extrap_flag", "target_type", "confidence"):
            self.assertAlmostEqual(getattr(parsed, field), getattr(row, field), msg=field)

    def test_timestamp_rebase_preserves_jitter(self):
        rows = self.parse(RECORD + RECORD.replace("13:24:25.852", "13:24:28.832"))
        schedule = make_schedule(rows, 1_790_000_000_000)
        self.assertEqual([r[0] for r in schedule], [0, 2980])
        self.assertEqual(schedule[1][1] - schedule[0][1], 2980)

    def test_midnight_rollover(self):
        rows = self.parse(RECORD.replace("13:24:25.852", "23:59:59.500") +
                          RECORD.replace("13:24:25.852", "00:00:02.501"))
        self.assertEqual(rows[1].timestamp_ms - rows[0].timestamp_ms, 3001)

    def test_real_flag_and_class_are_not_overridden(self):
        row = self.parse(RECORD.replace("外推点", "真实点").replace("type=未知", "type=鸟"))[0]
        parsed = parse_company_packet(encode_radar_packet(row, 123456))[0][2]
        self.assertEqual(parsed.extrap_flag, 1)
        self.assertEqual(parsed.target_type, 1)

    def test_missing_semantics_fail_closed(self):
        with self.assertRaises(ValueError):
            self.parse(RECORD.replace("tag=外推点", "tag=?"))

    def test_out_of_order_log_not_published(self):
        rows = self.parse(RECORD + RECORD.replace("13:24:25.852", "13:24:22.832"))
        with self.assertRaises(ValueError):
            make_schedule(rows, 1_790_000_000_000)

    def test_official_packet_audit_detects_extra_timestamp(self):
        stamp = 1_790_000_000_123
        header = struct.pack(PXSB_HEADER_FORMAT, 0x7E, 1, 201, 0xFFFF, 15, 0, 1, 0, 0, stamp)
        unit = struct.pack("<IHQ", 1, 1, stamp)
        good = header + unit + b"\0"
        bad = header + unit + struct.pack("<Q", stamp) + b"\0"
        self.assertTrue(inspect_official_packet(good)["valid"])
        self.assertEqual(inspect_official_packet(good)["target_count"], 0)
        self.assertFalse(inspect_official_packet(bad)["valid"])


if __name__ == "__main__":
    unittest.main()
