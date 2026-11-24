"""Offline regression: no native DDS, network publication or truth-fed filtering."""
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'dds_swarm_fusion'))
sys.path.insert(0, str(ROOT / 'tools'))
from eo_observations import EOObservations, EOTracker
from eo_geometry import intersect
from company_protocol import Wgs84Enu
from company_input_node import CompanyInputNode
from company_dds import Topics
from dds_frame import SensorStub, stubs_from_info
from dds_tracking_node import TrackingDdsNode
from dds_fusion_node import FusionDdsNode
from fusion import FusedTrack, Fusion
from tracking import BearingTrack
from replay_radar_log import MemoryBus


def obj(stamp=1000, **kw):
    values = dict(device_id=201, target_id=7, timestamp_ms=stamp,
                  fault_status=0, track_status=1, target_azimuth_deg=45.,
                  target_elevation_deg=10., target_dis_m=500.,
                  target_lng_deg=0., target_lat_deg=0., target_alt_m=0.,
                  ptz_lng_deg=0., ptz_lat_deg=0., ptz_alt_m=0.,
                  target_type=1, confidence=80, ptz_zeroazi_deg=0.)
    values.update(kw)
    return SimpleNamespace(**values)


def bearing(sid, stamp, pos=(500., 500., 100.), target=7, elevation=True):
    station = STATIONS[sid]
    d = np.array(pos)-station
    return dict(id=target, source_track_id=target, mode='bearing', fresh=True,
                timestamp_ms=round(stamp*1000), angle_valid=True,
                az=math.atan2(d[1], d[0]),
                el=math.atan2(d[2], math.hypot(d[0], d[1])) if elevation else None,
                sig_az=math.radians(.2), sig_el=math.radians(.2))


def position(stamp, xy=(500., 500.), target=7):
    return dict(id=target, source_track_id=target, mode='position', fresh=True,
                timestamp_ms=round(stamp*1000), position_valid=True,
                x=xy[0], y=xy[1], z=100., position_sigma_m=10., altitude_sigma_m=10.,
                az=None, el=None)


STATIONS = {'eo_201': np.array([0., 0., 0.]), 'eo_202': np.array([1000., 0., 0.])}


class QualityTests(unittest.TestCase):
    def setUp(self):
        self.q = EOObservations(Wgs84Enu([120., 30., 0.]), ROOT,
                                config_path='/nonexistent-eo-config')

    def calibrated(self):
        self.q.defaults.update(station_enu=[0, 0, 0], angles_calibrated=True,
                               elevation_enabled=True)

    def test_unknown_site_does_not_use_demo_position(self):
        m, site = self.q.normalize(obj(), 1000)
        self.assertIsNone(site)
        self.assertEqual(m['mode'], 'invalid')
        self.assertIn('station_unknown', m['quality_reasons'])

    def test_coordinate_does_not_require_station_or_synthetic_range(self):
        m, _ = self.q.normalize(obj(target_lng_deg=120.005, target_lat_deg=30.005), 1000)
        self.assertEqual(m['mode'], 'position')
        self.assertNotIn('r', m)
        self.assertIsNone(m['z'])
        self.assertEqual(m['cls'], 'drone')

    def test_positive_range_not_automatically_laser_success(self):
        self.calibrated()
        m, _ = self.q.normalize(obj(), 1000)
        self.assertEqual(m['mode'], 'bearing')
        self.assertNotIn('r', m)

    def test_range_zero_negative_and_nan_are_not_one_metre(self):
        self.calibrated()
        self.q.defaults['range_policy'] = 'reported_positive'
        for r in (0, -1, float('nan')):
            with self.subTest(r=r):
                m, _ = self.q.normalize(obj(target_dis_m=r), 1000)
                self.assertEqual(m['mode'], 'bearing')
                self.assertNotIn('r', m)
                bt = BearingTrack(0., .1, {'az': 0., 'r': r})
                self.assertIsNone(bt.r)

    def test_old_distance_expires_while_angles_continue(self):
        self.calibrated()
        self.q.defaults['range_policy'] = 'reported_positive'
        a, _ = self.q.normalize(obj(), 1000)
        b, _ = self.q.normalize(obj(5000, target_azimuth_deg=46), 5000)
        self.assertEqual(a['mode'], 'ranged')
        self.assertEqual(b['mode'], 'bearing')
        self.assertEqual(b['last_range_ms'], 1000)
        self.assertEqual(b['last_angle_ms'], 5000)

    def test_fault_memory_loss_are_not_measurements(self):
        self.calibrated()
        for status in (0, 2, 3):
            m, _ = self.q.normalize(obj(track_status=status), 1000)
            self.assertFalse(m['fresh'])
        m, _ = self.q.normalize(obj(fault_status=1), 1000)
        self.assertEqual(m['mode'], 'invalid')

    def test_separate_device_sites_and_corrections(self):
        self.q.devices['201'] = dict(station_enu=[1, 2, 3], angles_calibrated=True,
                                     azimuth_offset_deg=5)
        self.q.devices['202'] = dict(station_enu=[1000, 2, 3], angles_calibrated=True,
                                     azimuth_offset_deg=-5)
        a, sa = self.q.normalize(obj(), 1000)
        b, sb = self.q.normalize(obj(device_id=202), 1000)
        self.assertNotEqual(sa, sb)
        self.assertAlmostEqual(math.degrees(b['az']-a['az']), 10)

    def test_coordinate_jump_falls_back_to_angle(self):
        self.calibrated()
        a, _ = self.q.normalize(obj(target_lng_deg=120.005, target_lat_deg=30.005), 1000)
        b, _ = self.q.normalize(obj(1100, target_lng_deg=120.015, target_lat_deg=30.005), 1100)
        self.assertEqual(a['mode'], 'position')
        self.assertEqual(b['mode'], 'bearing')
        self.assertIn('position_jump', b['quality_reasons'])

    def test_high_rate_and_coordinate_repetition_are_explicit(self):
        a, _ = self.q.normalize(obj(target_lng_deg=120.005, target_lat_deg=30.005), 1000)
        b, _ = self.q.normalize(obj(1010, target_lng_deg=120.005, target_lat_deg=30.005), 1010)
        c, _ = self.q.normalize(obj(5000, target_lng_deg=120.005, target_lat_deg=30.005), 5000)
        self.assertTrue(a['fresh'])
        self.assertFalse(b['fresh'])
        self.assertIn('position_unchanged_unverified', c['quality_reasons'])


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.f = Fusion({s: SensorStub(s, 'eo', p, sig_az=math.radians(.2))
                         for s, p in STATIONS.items()}, .1,
                        area=dict(xmin=-2000, xmax=2000, ymin=-2000, ymax=2000))

    def step(self, stamp, rows, radar=None):
        self.f.dt = .1
        for t in self.f.tracks:
            t.dt = .1
            t.vertical.dt = .1
        self.f.time_s = stamp
        self.f.step(radar or [], rows, {})
        return self.f.export(include_coasting=True)

    def test_position_angle_position_retains_fused_id(self):
        a = self.step(1, {'eo_201': [position(1)]})
        b = self.step(1.1, {'eo_201': [bearing('eo_201', 1.1)]})
        c = self.step(1.2, {'eo_201': [position(1.2, (501, 500))]})
        self.assertEqual([a[0]['id']], [r['id'] for r in b])
        self.assertEqual(a[0]['id'], c[0]['id'])
        self.assertAlmostEqual(b[0]['position_age_s'], .1)

    def test_returning_bad_coordinate_not_duplicate_birth(self):
        a = self.step(1, {'eo_201': [position(1)]})
        b = self.step(1.1, {'eo_201': [position(1.1, (1500, 1500))]})
        self.assertEqual(len(b), 1)
        self.assertEqual(a[0]['id'], b[0]['id'])
        self.assertEqual(self.f.geometry_events[0]['reason'], 'bound_position_rejected')

    def test_two_coordinate_sources_same_target_one_fused_track(self):
        rows = self.step(1, {'eo_201': [position(1)], 'eo_202': [position(1, (501, 501), 99)]})
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['source_bindings'], {'eo_201': 7, 'eo_202': 99})

    def test_coordinate_plus_other_device_bearing(self):
        rows = self.step(1, {'eo_201': [position(1)], 'eo_202': [bearing('eo_202', 1, target=99)]})
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['source_bindings']['eo_202'], 99)

    def test_radar_with_one_or_two_eo_bearings(self):
        for stations in (['eo_201'], list(STATIONS)):
            self.setUp()
            radar = dict(id=55, x=500., y=500., z=100., vx=0., vy=0.,
                         cov=[[100., 0.], [0., 100.]], fresh=True, sensor_id='radar_3152')
            rows = self.step(1, {s: [bearing(s, 1)] for s in stations}, [radar])
            self.assertEqual(len(rows), 1)
            self.assertEqual(set(rows[0]['source_bindings']), set(stations))

    def test_distinct_targets_same_device_not_merged(self):
        rows = self.step(1, {'eo_201': [position(1, target=7), position(1, (505, 505), target=8)]})
        self.assertEqual(len(rows), 2)

    def test_device_target_id_can_change_after_gap(self):
        old = self.step(1, {'eo_201': [position(1)]})[0]['id']
        rows = self.step(4, {'eo_201': [position(4, target=99)]})
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['id'], old)
        self.assertEqual(rows[0]['source_bindings']['eo_201'], 99)

    def test_single_angle_does_not_invent_position(self):
        for i in range(20):
            rows = self.step(1+i*.1, {'eo_201': [bearing('eo_201', 1+i*.1)]})
        self.assertEqual(rows, [])
        self.assertEqual(len(self.f.direction_tracks), 1)

    def test_dual_angle_birth_2d_and_3d(self):
        for elevation in (False, True):
            with self.subTest(elevation=elevation):
                self.setUp()
                for i in range(12):
                    stamp = 1+i*.1
                    rows = self.step(stamp, {s: [bearing(s, stamp, elevation=elevation)] for s in STATIONS})
                self.assertEqual(len(rows), 1)
                self.assertAlmostEqual(rows[0]['x'], 500, delta=.1)
                self.assertAlmostEqual(rows[0]['y'], 500, delta=.1)
                if elevation:
                    self.assertAlmostEqual(rows[0]['z'], 100, delta=.1)
                else:
                    self.assertNotIn('z', rows[0])

    def test_asynchronous_dual_angle_and_duplicate_pair(self):
        for i in range(12):
            stamp = 1+i*.1
            self.step(stamp, {'eo_201': [bearing('eo_201', stamp)]})
            rows = self.step(stamp+.02, {'eo_202': [bearing('eo_202', stamp+.02)]})
        self.assertEqual(len(rows), 1)
        self.setUp()
        a, b = bearing('eo_201', 1), bearing('eo_202', 1)
        self.step(1, {'eo_201': [a], 'eo_202': [b]})
        for i in range(1, 4):
            self.step(1+i*.1, {'eo_201': [bearing('eo_201', 1+i*.1)]})
        self.assertEqual(max(p['hits'] for p in self.f.eo_pending.values()), 1)

    def test_valid_intersection_outside_demo_viewport_is_not_dropped(self):
        shift = np.array([20000., 20000., 0.])
        self.f = Fusion({s: SensorStub(s, 'eo', p+shift, sig_az=math.radians(.2))
                         for s, p in STATIONS.items()}, .1,
                        area=dict(xmin=-2000, xmax=2000, ymin=-2000, ymax=2000))
        for i in range(12):
            stamp = 1+i*.1
            # Translating stations and target equally leaves their angles unchanged.
            rows = self.step(stamp, {s: [bearing(s, stamp)] for s in STATIONS})
        self.assertEqual(len(rows), 1)
        self.assertAlmostEqual(rows[0]['x'], 20500., delta=.1)
        self.assertAlmostEqual(rows[0]['y'], 20500., delta=.1)

    def test_bad_geometry_rejected(self):
        a, b = bearing('eo_201', 1), bearing('eo_202', 1)
        result, reason = intersect([0, 0, 0], a, [2, 0, 0], b)
        self.assertIsNone(result)
        self.assertEqual(reason, 'baseline_too_short')
        b['az'], b['el'] = a['az'], a['el']
        result, reason = intersect(STATIONS['eo_201'], a, STATIONS['eo_202'], b)
        self.assertIsNone(result)
        self.assertEqual(reason, 'intersection_angle_too_small')

    def test_position_support_not_refreshed_by_angle(self):
        self.step(1, {'eo_201': [position(1)]})
        for i in range(1, 131):
            stamp = 1+i*.1
            rows = self.step(stamp, {'eo_201': [bearing('eo_201', stamp)]})
        self.assertEqual(rows, [])
        self.assertEqual(len(self.f.direction_tracks), 1)

    def test_rejected_angles_do_not_refresh_geometry_support(self):
        original_id = self.step(1, {'eo_201': [position(1)], 'eo_202': [bearing('eo_202', 1)]})[0]['id']
        for i in range(1, 131):
            stamp = 1+i*.1
            rows = self.step(stamp, {s: [bearing(s, stamp, pos=(100, -800, 100))] for s in STATIONS})
        self.assertNotIn(original_id, [r['id'] for r in rows])
        self.assertTrue(all(r['origin'] == 'eo_triangulation' for r in rows))

    def test_joint_filter_covariance_psd(self):
        self.step(1, {'eo_201': [position(1)]})
        for i in range(1, 100):
            stamp = 1+i*.1
            rows = {s: [bearing(s, stamp)] for s in STATIONS}
            if i % 5 == 0:
                rows['eo_201'] = [position(stamp)]
            self.step(stamp, rows)
            for t in self.f.tracks:
                _, P = t.joint_state()
                self.assertTrue(np.all(np.isfinite(P)))
                self.assertGreaterEqual(np.linalg.eigvalsh(P).min(), -1e-8)

    def test_old_predicted_range_not_position_measurement(self):
        self.step(1, {'eo_201': [position(1)]})
        b = bearing('eo_201', 1.1)
        b.pop('mode')
        b.update(r=math.sqrt(510000), range_age=1)
        rows = self.step(1.1, {'eo_201': [b]})
        self.assertAlmostEqual(rows[0]['position_age_s'], .1)


class PipelineTests(unittest.TestCase):
    def setUp(self):
        self.bus = MemoryBus()
        self.node = CompanyInputNode(ROOT/'anti-drone-pkg/config/scenarios/multi_target.yaml',
                                    bus=self.bus, radar_input_mode='device_tracks')
        self.node.eo_observations.defaults.update(station_enu=[0, 0, 0], angles_calibrated=True)
        self.tracker = TrackingDdsNode(self.bus)
        self.fuser = FusionDdsNode(self.bus)

    def test_per_device_identity_and_mode_transition_pipeline(self):
        ids = []
        for i in range(6):
            coord = dict(target_lng_deg=120.005+i*.000001, target_lat_deg=30.005) if i != 4 else {}
            report = obj(1000+i*100, **coord)
            self.node.ingest(0x1C01, 0, report)
            self.node.flush_due(force=True)
            self.tracker.process_raw(self.bus.messages[Topics.RAW_MEASUREMENTS.name])
            self.fuser.process_tracks(self.bus.messages[Topics.SENSOR_TRACKS.name])
            ids += [r['id'] for r in self.bus.messages[Topics.FUSED_TRACKS.name]['external_tracks']]
        self.assertEqual(len(set(ids)), 1)

    def test_reorder_window_holds_fast_device_and_sorts(self):
        self.node.ingest(0x1C01, 0, obj(1100, device_id=202))
        self.node.ingest(0x1C01, 0, obj(1000, device_id=201))
        self.assertEqual(self.node.flush_due(), 0)
        order = []
        original = self.node.publish_batch
        def publish(stamp, batch):
            order.append(stamp)
            original(stamp, batch)
        self.node.publish_batch = publish
        self.node.flush_due(force=True)
        self.assertEqual(order, [1000, 1100])

    def test_sensor_station_explicit_scenario_fallback_propagates(self):
        self.node.eo_observations.defaults['station_enu'] = None
        self.node.ingest(0x1C01, 0, obj())
        self.assertTrue(stubs_from_info(self.node.sensors_info)['eo_201'].station_known)
        self.assertEqual(self.node.latest_samples['eo_201']['station_source'], 'configured_enu')

    def test_reported_station_jitter_does_not_reset_eo_identity(self):
        self.node.eo_observations.defaults['station_enu'] = None
        for i in range(5):
            report = obj(1000+i*100, ptz_lng_deg=120.+i*.0000001, ptz_lat_deg=30.,
                         target_lng_deg=120.005, target_lat_deg=30.005)
            self.node.ingest(0x1C01, 0, report)
            self.node.flush_due(force=True)
            self.tracker.process_raw(self.bus.messages[Topics.RAW_MEASUREMENTS.name])
        self.assertEqual(self.tracker.bearing_trks['eo_201'].entries[7]['hits'], 5)

    def test_source_timestamp_fallback_and_device_offset(self):
        self.node.eo_observations.devices['201'] = {'time_offset_ms': 200}
        self.node.ingest(0x1C01, 1000, obj(0))
        m = self.node.latest_samples['eo_201']
        self.assertEqual(m['timestamp_ms'], 1200)
        self.assertEqual(m['last_angle_ms'], 1200)


if __name__ == '__main__':
    unittest.main()
