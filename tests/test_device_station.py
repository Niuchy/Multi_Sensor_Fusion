"""Station selection/logging only: no DDS network, no service start."""
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'dds_swarm_fusion'))
sys.path.insert(0, str(ROOT/'tools'))
from company_input_node import CompanyInputNode, load_company_scenario
from company_protocol import Wgs84Enu
from company_dds import Topics
from device_station import select_station, enu_to_lla
from replay_radar_log import MemoryBus
from test_eo_fusion import obj as eo


class StationSelectionTests(unittest.TestCase):
    def setUp(self):
        self.c = Wgs84Enu([120., 30., 10.])

    def select(self, kind, obj, **kw):
        return select_station(self.c, kind, obj, config_label='sensor.yaml [position]', **kw)

    def test_eo_report_overrides_config(self):
        s = self.select('eo', eo(ptz_lng_deg=120.001, ptz_lat_deg=30.002, ptz_alt_m=30),
                        config_lla=[120.1, 30.1, 99])
        self.assertEqual(s['station_source'], 'reported_lla')
        self.assertEqual(s['station_lla'], [120.001, 30.002, 30])
        self.assertIsNone(s['station_config'])
        self.assertEqual(s['station_enu'], list(self.c.to_enu(120.001, 30.002, 30)))

    def test_acoustic_report_overrides_config(self):
        s = self.select('acoustic', SimpleNamespace(device_lng_deg=120.003,
                        device_lat_deg=30.004, device_alt_m=0), config_enu=[9, 9, 9])
        self.assertEqual(s['station_source'], 'reported_lla')
        self.assertEqual(s['station_lla'][2], 0)  # sea-level altitude is valid

    def test_zero_gps_explicit_config_enu(self):
        s = self.select('eo', eo(), config_enu=[100, 200, 3])
        self.assertEqual(s['station_source'], 'configured_enu')
        self.assertEqual(s['station_reason'], 'device_gps_invalid')
        self.assertEqual(s['station_config'], 'sensor.yaml [position]')
        self.assertEqual(s['station_lla'][2], 13)
        self.assertAlmostEqual(s['station_lla'][0], 120+100/self.c.meters_per_deg_lng)

    def test_invalid_report_never_nan_or_overrides_config(self):
        for lon, lat, alt in [(math.nan,30,10),(120,math.inf,10),(181,30,10),
                              (120,91,10),(120,30,math.nan),(0,0,10)]:
            with self.subTest(lla=(lon,lat,alt)):
                s = self.select('eo', eo(ptz_lng_deg=lon,ptz_lat_deg=lat,ptz_alt_m=alt),
                                config_lla=[120.01,30.01,12])
                self.assertEqual(s['station_source'], 'configured_lla')
                self.assertTrue(all(math.isfinite(x) for x in s['station_lla']))

    def test_missing_report_and_missing_config_are_explicit(self):
        s = self.select('eo', SimpleNamespace(), config_enu=[0,0,0])
        self.assertEqual(s['station_reason'], 'device_gps_missing')
        self.assertEqual(s['station_source'], 'configured_enu')
        s = self.select('eo', SimpleNamespace())
        self.assertFalse(s['station_known'])
        self.assertIsNone(s['station_lla'])

    def test_radar_and_rf_target_remote_controller_gps_never_station(self):
        for kind in ('radar','rf_df'):
            s = self.select(kind, SimpleNamespace(lng_deg=120.5, lat_deg=30.5, alt_m=100,
                rctrl_lng_deg=120.6,rctrl_lat_deg=30.6), config_enu=[-50,0,8])
            self.assertEqual(s['station_source'], 'configured_enu')
            self.assertEqual(s['station_reason'], 'protocol_has_no_device_gps')
            self.assertEqual(s['station_enu'], [-50,0,8])
            self.assertNotEqual(s['station_lla'][0], 120.5)

    def test_invalid_config_is_error_not_guessed_position(self):
        for kw in ({'config_enu':[1,math.nan,3]}, {'config_lla':[181,30,2]},
                   {'config_enu':[1,2]}):
            with self.assertRaises(ValueError):
                self.select('eo', eo(), **kw)

    def test_inverse_enu_uses_same_reference(self):
        p = [120.012345,30.012345,123.45]
        for a,b in zip(p, enu_to_lla(self.c,self.c.to_enu(*p))):
            self.assertAlmostEqual(a,b,places=9)


class StationPipelineTests(unittest.TestCase):
    def setUp(self):
        self.bus = MemoryBus()
        self.node = CompanyInputNode(ROOT/'anti-drone-pkg/config/scenarios/multi_target.yaml',
                                    bus=self.bus,radar_input_mode='device_tracks')
        self.logging = patch('company_input_node.LOG')
        self.log = self.logging.start()
        self.addCleanup(self.logging.stop)

    def ingest(self, report):
        self.node.ingest(0x1C01, report.timestamp_ms, report)
        self.node.flush_due(force=True)
        return self.node.latest_samples[f'eo_{report.device_id}']

    def test_profile_not_override_report_even_legacy_switch_false(self):
        self.node.eo_observations.devices['201'] = dict(station_lla=[120.1,30.1,100],
                                                       use_reported_station=False)
        m = self.ingest(eo(ptz_lng_deg=120.002,ptz_lat_deg=30.002,ptz_alt_m=20))
        self.assertEqual(m['station_source'],'reported_lla')
        self.assertEqual(m['station_lla'],[120.002,30.002,20])
        self.assertEqual(self.node.sensors_info['eo_201']['pos'],m['station_enu'])

    def test_report_loss_reverts_to_config_not_previous_gps(self):
        self.node.eo_observations.devices['201'] = dict(station_enu=[10,20,3])
        self.ingest(eo(ptz_lng_deg=120.002,ptz_lat_deg=30.002,ptz_alt_m=20))
        m = self.ingest(eo(1100))
        self.assertEqual(m['station_source'],'configured_enu')
        self.assertEqual(self.node.sensors_info['eo_201']['pos'],[10,20,3])
        self.assertIn('devices.201.station_enu',m['station_config'])
        self.ingest(eo(1200,ptz_lng_deg=120.003,ptz_lat_deg=30.003,ptz_alt_m=21))
        self.assertEqual(self.node.sensors_info['eo_201']['station_source'],'reported_lla')
        self.assertEqual(self.log.info.call_count,2)
        self.assertEqual(self.log.warning.call_count,1)

    def test_scenario_fallback_not_implicitly_calibrated(self):
        m = self.ingest(eo())
        self.assertEqual(m['station_source'],'configured_enu')
        self.assertEqual(m['station_enu'],self.node.config.eo.position)
        self.assertIn('eo.yaml',m['station_config'])
        self.assertFalse(m['angle_valid'])

    def test_two_devices_independent_config_and_no_station_leak(self):
        self.node.eo_observations.devices.update({'201':{'station_enu':[10,20,3]},
                                                  '202':{'station_enu':[1000,20,3]}})
        self.ingest(eo(device_id=201,ptz_lng_deg=120.003,ptz_lat_deg=30.003,ptz_alt_m=20))
        m = self.ingest(eo(device_id=202))
        self.assertEqual(m['station_enu'],[1000,20,3])

    def test_raw_frame_and_diagnostics_use_identical_station(self):
        m = self.ingest(eo(ptz_lng_deg=120.001,ptz_lat_deg=30.002,ptz_alt_m=13))
        self.node.publish_diagnostics(force=True)
        d = self.bus.messages[Topics.DIAGNOSTICS.name]['device_stations']
        self.assertEqual(len(d),1)  # no demo placeholders represented as received
        self.assertEqual(d[0]['station_lla'],m['station_lla'])
        raw = self.bus.messages[Topics.RAW_MEASUREMENTS.name]
        self.assertEqual(raw['sensors']['eo_201']['pos'],m['station_enu'])
        self.assertEqual(raw['measurements']['eo_201']['meas'][0]['station_source'],'reported_lla')

    def test_log_coordinates_and_source_not_per_packet_spam(self):
        self.ingest(eo())
        for stamp in (1100,1200,1300):
            self.ingest(eo(stamp))
        self.assertEqual(self.log.warning.call_count,1)
        args = self.log.warning.call_args.args
        line = args[0] % args[1:]
        for text in ('设备自身站址','config ENU 换算经纬高','使用经度=','纬度=','高度=',
                     'eo.yaml','参考原点=','并非设备实测 GPS'):
            self.assertIn(text,line)

    def test_acoustic_valid_then_invalid_reverts_to_immutable_config(self):
        valid = SimpleNamespace(device_id=104,device_lng_deg=120.001,
                                device_lat_deg=30.001,device_alt_m=100)
        self.node._use_station('acoustic',valid)
        _,info = self.node._use_station('acoustic',SimpleNamespace(device_id=104))
        self.assertEqual(info['station_source'],'configured_enu')
        self.assertEqual(info['pos'],self.node.config.acoustic.position)


class StationConfigTests(unittest.TestCase):
    """New WGS84/per-device settings, using only an in-memory bus."""

    def scenario(self, sensors):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name)/'site.yaml'
        path.write_text(yaml.safe_dump(dict(
            coordinates=dict(frame='ENU', origin=[120.,30.,10.]),
            equipment=dict(discover_only=True), targets=[], sensors=sensors)), encoding='utf-8')
        return path

    def two_stations(self):
        return dict(acoustic=dict(position_frame='WGS84', position=[120.,30.,10.], devices={
            '4201': dict(position=[120.001,30.002,13.]),
            4202: dict(position=[120.01,30.02,17.])}))

    def test_equipment_two_acoustic_ids_and_same_site_explicitly_lla(self):
        node = CompanyInputNode(ROOT/'configs/equipment.yaml', bus=MemoryBus())
        self.assertEqual(node.sensors_info, {})  # no invented/preseeded devices
        self.assertEqual(set(node.config.station_configs['acoustic']['devices']), {'4201','4202'})
        for kind, device in [('radar',3152),('rf_df',4001),('acoustic',4201),('acoustic',4202)]:
            _, info = node._use_station(kind, SimpleNamespace(device_id=device))
            self.assertEqual(info['station_source'], 'configured_lla')
            self.assertEqual(info['station_lla'], [120.140914917,30.111728668,22.])
            self.assertEqual(info['pos'], [0.,0.,0.])
            self.assertIn('equipment.yaml', info['station_config'])
            if kind == 'acoustic':
                self.assertIn(f'devices.{device}.position', info['station_config'])

    def test_different_device_sites_converted_once_not_treated_as_metres(self):
        path = self.scenario(self.two_stations())
        node = CompanyInputNode(path, bus=MemoryBus())
        for device, lla in [(4201,[120.001,30.002,13.]),(4202,[120.01,30.02,17.])]:
            _, info = node._use_station('acoustic', SimpleNamespace(device_id=device))
            self.assertEqual(info['station_lla'], lla)
            self.assertEqual(info['pos'], list(node.converter.to_enu(*lla)))
            self.assertIn(f'{path} [sensors.acoustic.devices.{device}.position]', info['station_config'])
        self.assertNotEqual(node.sensors_info['acoustic_4201']['pos'], node.sensors_info['acoustic_4202']['pos'])

    def test_equipment_eo1112_is_near_radar_not_18km_from_demo_origin(self):
        node = CompanyInputNode(ROOT/'configs/equipment.yaml', bus=MemoryBus())
        _, radar = node._use_station('radar', SimpleNamespace(device_id=3152))
        measurement = node._eo(eo(device_id=1112))[1]
        distance = math.hypot(measurement['station_enu'][0]-radar['pos'][0],
                              measurement['station_enu'][1]-radar['pos'][1])
        self.assertLess(distance, 3.0)
        self.assertEqual(measurement['station_source'], 'configured_lla')
        self.assertIn('devices.1112.station_lla', measurement['station_config'])
        self.assertEqual(node.config.eo.position, [0., 0., 0.])

    def test_equipment_config_path_and_origin_are_in_diagnostics(self):
        bus = MemoryBus()
        node = CompanyInputNode(ROOT/'configs/equipment.yaml', bus=bus)
        node.publish_diagnostics(force=True)
        report = bus.messages[Topics.DIAGNOSTICS.name]
        self.assertEqual(report['scenario_path'], str((ROOT/'configs/equipment.yaml').resolve()))
        self.assertEqual(report['origin'], [120.140914917,30.111728668,22.])

    def test_reported_site_wins_and_loss_reverts_to_own_device_not_other(self):
        node = CompanyInputNode(self.scenario(self.two_stations()), bus=MemoryBus())
        reported = SimpleNamespace(device_id=4201,device_lng_deg=120.2,
                                   device_lat_deg=30.3,device_alt_m=25.)
        _, info = node._use_station('acoustic', reported)
        self.assertEqual(info['station_source'],'reported_lla')
        self.assertEqual(info['station_lla'],[120.2,30.3,25.])
        _, other = node._use_station('acoustic', SimpleNamespace(device_id=4202))
        self.assertEqual(other['station_lla'],[120.01,30.02,17.])
        _, info = node._use_station('acoustic', SimpleNamespace(device_id=4201,
                          device_lng_deg=0.,device_lat_deg=0.,device_alt_m=0.))
        self.assertEqual(info['station_source'],'configured_lla')
        self.assertEqual(info['station_lla'],[120.001,30.002,13.])

    def test_unlisted_id_uses_type_default_not_first_device(self):
        node = CompanyInputNode(self.scenario(self.two_stations()), bus=MemoryBus())
        node._use_station('acoustic',SimpleNamespace(device_id=4201))
        _, info = node._use_station('acoustic',SimpleNamespace(device_id=4299))
        self.assertEqual(info['station_lla'],[120.,30.,10.])
        self.assertNotIn('devices.',info['station_config'])

    def test_legacy_scene_and_explicit_mixed_frame_stay_enu(self):
        old = load_company_scenario(ROOT/'anti-drone-pkg/config/scenarios/multi_target.yaml')
        self.assertEqual(old.acoustic.position,[-30.,0.,2.])
        self.assertEqual(old.station_configs['acoustic']['default']['frame'],'ENU')
        sensors = self.two_stations()
        sensors['acoustic']['devices'][4202] = dict(position_frame='ENU',position=[100.,200.,3.])
        node = CompanyInputNode(self.scenario(sensors),bus=MemoryBus())
        _, info = node._use_station('acoustic',SimpleNamespace(device_id=4202))
        self.assertEqual(info['station_source'],'configured_enu')
        self.assertEqual(info['pos'],[100.,200.,3.])

    def test_invalid_frame_or_lla_fails_at_load(self):
        for profile in [dict(position_frame='GPS',position=[120,30,10]),
                        dict(position_frame='WGS84',position=[181,30,10]),
                        dict(position_frame='WGS84',position=[120,91,10]),
                        dict(position_frame='WGS84',position=[0,0,0]),
                        dict(position_frame='WGS84',position=[120,30,float('nan')]),
                        dict(position_frame='WGS84',position=[120,30]),
                        dict(position_frame='WGS84')]:
            with self.subTest(profile=profile), self.assertRaises(ValueError):
                load_company_scenario(self.scenario(dict(acoustic=profile)))

    def test_invalid_device_profile_fails_at_load(self):
        for devices in [{'34BD0017':dict(position=[1,2,3])},
                        {'4201':dict(position_frame='WGS84')},
                        {'4201':dict(position=[1,2,3]),4201:dict(position=[4,5,6])},
                        {'4201':dict(position_frame='WGS84',position=[120,100,20])}]:
            with self.subTest(devices=devices),self.assertRaises(ValueError):
                load_company_scenario(self.scenario(dict(acoustic=dict(devices=devices))))

    def test_per_device_sites_reach_raw_frame_and_diagnostics(self):
        bus=MemoryBus()
        node=CompanyInputNode(self.scenario(self.two_stations()),bus=bus)
        with patch('company_input_node.LOG') as log:
            for device in (4201,4202):
                obj=SimpleNamespace(device_id=device,target_id=7,timestamp_ms=1000,
                    target_yaw_deg=10.,target_pitch_deg=5.,target_type=2,confidence=90)
                node.ingest(0x1D01,1000,obj)
            node.flush_due(force=True)
            node.publish_diagnostics(force=True)
        self.assertEqual(log.warning.call_count,2)
        raw=bus.messages[Topics.RAW_MEASUREMENTS.name]
        diagnostics=bus.messages[Topics.DIAGNOSTICS.name]['device_stations']
        self.assertEqual(len(diagnostics),2)
        for station in diagnostics:
            sid=station['sensor_id']
            measurement=raw['measurements'][sid]['meas'][0]
            self.assertEqual(measurement['station_lla'],station['station_lla'])
            self.assertEqual(raw['sensors'][sid]['pos'],station['station_enu'])
            self.assertEqual(station['station_source'],'configured_lla')

    def test_area_includes_device_specific_sites(self):
        sensors=self.two_stations()
        sensors['acoustic']['devices']['4201']['position']=[120.2,30.2,15.]
        node=CompanyInputNode(self.scenario(sensors),bus=MemoryBus())
        x,y,_=node.converter.to_enu(120.2,30.2,15.)
        self.assertGreater(node.area['xmax'],x)
        self.assertGreater(node.area['ymax'],y)


if __name__ == '__main__':
    unittest.main()
