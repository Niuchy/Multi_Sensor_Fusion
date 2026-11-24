"""Independent simulator regressions; no DDS library or live service is opened."""
import ast
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'dds_swarm_fusion'))
import field_simulator as sim
from company_protocol import parse_company_packet, Wgs84Enu
from dds_data_tool import prepare, HEADER_SIZE
from evaluate_field_sim import evaluate


def decoded(event):
    return parse_company_packet(event['packet'])[0][2]


class GeneratorTests(unittest.TestCase):
    def cfg(self,**kw): return sim.load_settings(duration_s=15,**kw)

    def test_reproducible_packets_with_nonzero_noise(self):
        c=self.cfg();a=sim.Simulation(c,'baseline').generate()
        b=sim.Simulation(c,'baseline').generate()
        self.assertEqual(a,b)
        x=next(e for e in a if e['topic']=='EOTarget');o=decoded(x)
        xyz=Wgs84Enu(c['origin']).to_enu(o.target_lng_deg,o.target_lat_deg,o.target_alt_m)
        truth=sim.truth_state(x['source_time_s'],'baseline',15)[0]
        self.assertGreater(sim.distance(xyz,truth),.1)

    def test_all_cases_original_packet_parser_roundtrip(self):
        sizes={'RadarTarget':106,'EOTarget':119,'ESMTarget':179}
        for case in sim.CASES:
            with self.subTest(case=case):
                events=sim.Simulation(self.cfg(),case).generate()
                self.assertTrue(events)
                self.assertEqual(events,sorted(events,key=lambda x:x['arrival_s']))
                for e in events:
                    o=decoded(e);self.assertEqual(o.device_id,e['device_id'])
                    self.assertEqual(len(e['packet']),sizes[e['topic']])
                    head=struct.unpack_from(sim.PXSB,e['packet'])
                    self.assertEqual(head[4],len(e['packet'])-HEADER_SIZE)
                    self.assertEqual(head[-1],o.timestamp_ms)
                    self.assertEqual(e['packet'][-4:],b'\0'*4)
                    if e['topic']=='EOTarget': self.assertEqual(o.target_type,1)

    def test_radar_scan_intervals_no_extra_real_points(self):
        c=self.cfg(radar_pd=1.)
        rows=[e for e in sim.Simulation(c,'baseline').generate() if e['topic']=='RadarTarget']
        self.assertEqual(len(rows),5)
        for a,b in zip(rows,rows[1:]):
            self.assertTrue(2.98<=b['source_time_s']-a['source_time_s']<=3.02)
        self.assertTrue(all(decoded(e).extrap_flag==1 for e in rows))

    def test_extrapolation_not_truth_or_real_measurement(self):
        rows=sim.Simulation(self.cfg(),'radar-extrapolation').generate()
        self.assertGreaterEqual(len(rows),2)
        self.assertTrue(all(decoded(e).extrap_flag==0 for e in rows))
        self.assertTrue(all(e['source_time_s']<=6 for e in rows))
        positions=[(decoded(e).lng_deg,decoded(e).lat_deg) for e in rows]
        self.assertEqual(len(set(positions)),1)  # stationary noisy prior, truth is moving

    def test_bearing_only_zero_gps_distance_normal_status(self):
        for case in ('single-bearing','dual-bearing'):
            for e in sim.Simulation(self.cfg(),case).generate():
                o=decoded(e)
                self.assertEqual((o.target_dis_m,o.target_lng_deg,o.target_lat_deg),(0,0,0))
                self.assertEqual((o.track_status,o.track_count),(1,1))
                self.assertGreater(o.target_azimuth_deg,0)

    def test_dual_distinct_sites_ids_and_asynchronous_rates(self):
        rows=sim.Simulation(self.cfg(),'dual-bearing').generate()
        devices={e['device_id'] for e in rows};self.assertEqual(len(devices),2)
        stations={(decoded(e).ptz_lng_deg,decoded(e).ptz_lat_deg) for e in rows}
        self.assertEqual(len(stations),2)
        counts=[sum(e['device_id']==did for e in rows) for did in devices]
        self.assertNotEqual(*counts)

    def test_stale_positions_but_changing_angles(self):
        rows=[decoded(e) for e in sim.Simulation(self.cfg(),'stale-values').generate()
              if e['mode']=='stale_position_range']
        self.assertGreater(len(rows),10)
        self.assertEqual(len({(o.target_lng_deg,o.target_lat_deg,o.target_dis_m) for o in rows}),1)
        self.assertGreater(len({o.target_azimuth_deg for o in rows}),10)

    def test_low_speed_loss_and_eo_continuity(self):
        c=self.cfg(radar_pd=1,low_speed_pd=0,radar_coast_s=0)
        rows=sim.Simulation(c,'low-speed').generate()
        middle=[e for e in rows if .28*15<e['source_time_s']<.52*15]
        self.assertTrue(middle)
        self.assertTrue(all(e['topic']=='EOTarget' for e in middle))

    def test_timing_faults_are_real_duplicates_and_late_arrivals(self):
        rows=[e for e in sim.Simulation(self.cfg(),'timing-faults').generate() if e['topic']=='EOTarget']
        self.assertGreater(len(rows)-len({e['packet'] for e in rows}),0)
        self.assertTrue(any(decoded(a).timestamp_ms>decoded(b).timestamp_ms for a,b in zip(rows,rows[1:])))
        e=rows[0]
        self.assertEqual(decoded(e).timestamp_ms,self.cfg()['epoch_ms']+round(e['source_time_s']*1000)+1500)

    def test_rf_targets_remain_distinct_without_target_gps(self):
        rows=[decoded(e) for e in sim.Simulation(self.cfg(),'rf-distractors').generate() if e['topic']=='ESMTarget']
        self.assertEqual({r.sn for r in rows},{'SIM-UAV-1','SIM-UAV-2','SIM-UAV-3'})
        self.assertTrue(all(r.lng_deg==r.lat_deg==0 for r in rows))

    def test_config_validation(self):
        for kw in (dict(duration_s=0),dict(radar_pd=2),dict(eo_rates_hz=[0,8]),
                   dict(seed=True),dict(origin=[120,90,0]),dict(eo_device_ids=[1,1])):
            with self.subTest(kw=kw),self.assertRaises(ValueError): sim.load_settings(**kw)

    def test_generator_does_not_import_fusion(self):
        tree=ast.parse((ROOT/'tools/field_simulator.py').read_text())
        imported=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
        self.assertNotIn('company_input_node',imported)
        self.assertNotIn('fusion',imported)
        self.assertNotIn('company_dds',imported)

    def test_capture_compatible_and_exclusive(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'run';meta=sim.generate_run(dest,self.cfg(),'baseline')
            with tempfile.TemporaryFile('w+b') as spool:
                result=prepare(dest/'capture.jsonl',spool,None,'rebase',False)
            self.assertEqual(result['selected_records'],meta['packets'])
            self.assertEqual(meta['dds_packets_sent'],0)
            scenario=json.loads((dest/'scenario.yaml').read_text())
            self.assertEqual(scenario['targets'],[])
            before=hashlib.sha256((dest/'capture.jsonl').read_bytes()).hexdigest()
            with self.assertRaises(FileExistsError):sim.generate_run(dest,self.cfg(),'baseline')
            self.assertEqual(before,hashlib.sha256((dest/'capture.jsonl').read_bytes()).hexdigest())

    def test_send_guards_and_dryrun_never_publish(self):
        with tempfile.TemporaryDirectory() as td,patch.object(sim.subprocess,'run') as run:
            dest=Path(td)/'run';sim.generate_run(dest,self.cfg(),'baseline')
            for isolated,stopped in ((False,False),(True,False),(False,True)):
                with self.assertRaises(ValueError):sim.send_run(dest,True,isolated,stopped)
            run.assert_not_called()
            sim.send_run(dest)
            self.assertNotIn('--publish',run.call_args.args[0])
            sim.send_run(dest,True,True,True)
            self.assertIn('--publish',run.call_args.args[0])
            self.assertIn('--inputs-stopped',run.call_args.args[0])

    def test_gps_profiles_and_tamper_guard(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'run';sim.generate_run(dest,self.cfg(),'gps-fallback')
            profile=json.loads((dest/'eo_devices.yaml').read_text())
            self.assertEqual(profile['devices']['71112']['station_enu'],[31.,21.,3.])
            with (dest/'capture.jsonl').open('a') as f:f.write('\n')
            with self.assertRaises(ValueError):sim.send_run(dest)
            with self.assertRaises(ValueError):evaluate(dest)


class OfflinePipelineTests(unittest.TestCase):
    def test_memory_pipeline_modes_and_no_truth_leak(self):
        with tempfile.TemporaryDirectory() as td:
            for case in ('baseline','single-bearing','dual-bearing','weak-geometry',
                         'colocated-bearing','recovery','ranged-only','radar-extrapolation',
                         'stale-values','gps-fallback','timing-faults'):
                with self.subTest(case=case):
                    dest=Path(td)/case
                    sim.generate_run(dest,sim.load_settings(duration_s=15),case)
                    result=evaluate(dest)
                    self.assertTrue(result['all_checks_pass'],result['checks'])
                    self.assertEqual(result['dds_packets_sent'],0)
                    self.assertEqual(result['diagnostics']['truth_leaks'],0)


if __name__=='__main__': unittest.main()
