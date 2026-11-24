"""Analytic fixtures for metrics, plus real offline multi-target pipeline tests."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'dds_swarm_fusion'))
from field_sim_metrics import match_positions, summarize_multi
import field_simulator as sim
from company_protocol import parse_company_packet
from evaluate_field_sim import evaluate


class MetricTests(unittest.TestCase):
    def report(self,positions,frames,*,duration=2,gate=20,velocity=0,period=3):
        with tempfile.TemporaryDirectory() as td:
            directory=Path(td)
            with (directory/'truth.jsonl').open('w') as f:
                for i,p in enumerate(positions,1):
                    for t in (0,duration):
                        f.write(json.dumps(dict(truth_id=i,timestamp_ms=t*1000,
                            position=[p[0]+velocity*t,p[1],p[2]]))+'\n')
            meta=dict(case='custom',evaluation_truth_ids=list(range(1,len(positions)+1)),
                settings=dict(epoch_ms=0,duration_s=duration,radar_period_s=period,evaluation_gate_m=gate))
            return summarize_multi(directory,meta,frames)

    def row(self,id,x,y=0,**kw):return dict(id=id,x=x,y=y,vx=0,vy=0,**kw)
    def frame(self,t,*rows):return dict(timestamp_ms=t*1000,fused=list(rows))

    def test_perfect_two_targets_and_altitude(self):
        r=self.report([[0,0,10],[100,0,20]],[self.frame(0,self.row(91,0,z=10),self.row(14,100,z=20))])
        self.assertEqual((r['precision'],r['recall'],r['f1']),(1,1,1))
        self.assertEqual(r['true_positive_samples'],8)
        self.assertEqual(r['position_3d_error']['rmse_m'],0)
        self.assertEqual(r['id_switches'],0)
        self.assertEqual(r['per_target'][1]['first_match_s'],0)

    def test_no_output_is_missed_not_perfect_or_zero_error(self):
        r=self.report([[0,0,10],[100,0,20]],[])
        self.assertIsNone(r['precision']);self.assertEqual(r['f1'],0)
        self.assertEqual(r['false_negative_samples'],8)
        self.assertIsNone(r['horizontal_error']['rmse_m'])
        self.assertEqual(r['per_target'][0]['longest_miss_s'],2)

    def test_one_output_cannot_cover_two_truths(self):
        r=self.report([[0,0,10],[10,0,20]],[self.frame(0,self.row(1,3))])
        self.assertEqual(r['recall'],.5);self.assertEqual(r['precision'],1)
        self.assertEqual(r['true_positive_samples'],4)
        self.assertEqual(r['all_targets_covered_fraction'],0)

    def test_duplicate_and_outside_gate_penalized(self):
        r=self.report([[0,0,10]],[self.frame(0,self.row(1,1),self.row(2,2),self.row(3,200))])
        self.assertAlmostEqual(r['precision'],1/3)
        self.assertEqual(r['false_positive_samples'],8)
        self.assertEqual(r['duplicate_candidate_samples'],4)
        self.assertEqual(r['horizontal_error']['rmse_m'],1)

    def test_all_bad_outputs_count_as_both_missed_and_extra(self):
        r=self.report([[0,0,10]],[self.frame(0,self.row(1,200))])
        self.assertEqual((r['precision'],r['recall'],r['f1']),(0,0,0))
        self.assertEqual(r['false_positive_samples'],4)
        self.assertEqual(r['false_negative_samples'],4)

    def test_assignment_maximizes_matches_before_minimizing_error(self):
        pairs=match_positions([[0,0,0],[8,0,0]],[self.row(1,7),self.row(2,11)],8)
        self.assertEqual({(i,j) for i,j,_ in pairs},{(0,0),(1,1)})
        self.assertEqual(len(match_positions([[0,0,0]],[self.row(1,20)],20)),1)

    def test_id_switches_and_gap_reacquisition_are_separate(self):
        frames=[self.frame(0,self.row(1,0)),self.frame(.5,self.row(2,0)),
                self.frame(1),self.frame(1.5,self.row(3,0))]
        r=self.report([[0,0,10]],frames)
        self.assertEqual(r['id_switches'],1)
        self.assertEqual(r['reacquisition_id_changes'],1)
        self.assertEqual(r['fragments'],1)
        self.assertEqual(r['per_target'][0]['longest_miss_s'],.5)

    def test_no_future_snapshot_and_velocity_time_alignment_and_expiry(self):
        r=self.report([[0,0,10]],[self.frame(.5,dict(id=1,x=1,y=0,vx=2,vy=0))],velocity=2)
        self.assertEqual(r['recall'],.75)
        self.assertEqual(r['horizontal_error']['rmse_m'],0)
        r=self.report([[0,0,10]],[self.frame(0,self.row(1,0))],duration=5,period=.5)
        self.assertEqual(r['recall'],.3)  # 0, .5, 1.0 only

    def test_missing_altitude_and_invalid_output_not_silently_zero(self):
        r=self.report([[0,0,10]],[self.frame(0,self.row(1,0),self.row(2,float('nan')))])
        self.assertIsNone(r['position_3d_error']['rmse_m'])
        self.assertEqual(r['invalid_output_samples'],4)
        self.assertEqual(r['false_positive_samples'],4)
        json.dumps(r,allow_nan=False)


class MultiTargetTests(unittest.TestCase):
    def test_validation_count_eo_assignment_and_gate(self):
        for kw in [dict(custom_target_count=x) for x in (0,11,1.5,True)]+[
            dict(eo_target_ids=[1,2]),dict(eo_target_ids=[True,1]),dict(eo_target_ids=[1]),
            dict(evaluation_gate_m=0),dict(evaluation_gate_m=float('nan')),dict(evaluation_gate_m=True)]:
            with self.subTest(kw=kw),self.assertRaises(ValueError):sim.load_settings(**kw)

    def test_ten_independent_radar_tracks_and_one_target_per_eo(self):
        cfg=sim.load_settings(duration_s=15,custom_target_count=10,custom_eo_count=2,
            eo_target_ids=[3,10],radar_pd=1,custom_low_speed_loss=False)
        events=sim.Simulation(cfg,'custom').generate()
        radar={}
        for e in events:
            o=parse_company_packet(e['packet'])[0][2]
            if e['topic']=='RadarTarget':radar.setdefault(o.track_id,[]).append(e)
            elif e['topic']=='EOTarget':self.assertEqual(e['truth_id'],3 if e['device_id']==71112 else 10)
        self.assertEqual(len(radar),10)
        for rows in radar.values():
            self.assertEqual(len({r['truth_id'] for r in rows}),1)
            self.assertTrue(all(2.98<=b['source_time_s']-a['source_time_s']<=3.02 for a,b in zip(rows,rows[1:])))
        self.assertEqual(len({tuple(sim.truth_state(0,'custom',15,t)[0]) for t in range(1,11)}),10)

    def test_custom_target_switch_changes_eo_wire_position_and_source_id(self):
        def first(t):
            cfg=sim.load_settings(duration_s=15,custom_radar_enabled=False,custom_target_count=3,eo_target_ids=[t,1])
            return parse_company_packet(sim.Simulation(cfg,'custom').generate()[0]['packet'])[0][2]
        a,b=first(1),first(3)
        self.assertNotEqual(a.target_lng_deg,b.target_lng_deg)
        self.assertNotEqual(a.target_id,b.target_id)

    def test_offline_three_targets_counts_unobserved_and_never_dds(self):
        with tempfile.TemporaryDirectory() as td:
            cfg=sim.load_settings(duration_s=15,custom_target_count=3,custom_eo_count=2,
                eo_target_ids=[2,2],eo_modes=['bearing','bearing'],custom_radar_enabled=False)
            dest=Path(td)/'run';meta=sim.generate_run(dest,cfg,'custom');report=evaluate(dest)
            self.assertEqual(meta['evaluation_truth_ids'],[1,2,3])
            metrics=report['multi_target_metrics'];self.assertEqual(metrics['target_count'],3)
            self.assertEqual(metrics['per_target'][0]['recall'],0)
            self.assertEqual(metrics['per_target'][2]['recall'],0)
            self.assertGreater(metrics['per_target'][1]['recall'],0)
            self.assertEqual(report['diagnostics']['truth_leaks'],0)
            self.assertEqual(report['dds_packets_sent'],0)

    def test_modified_truth_rejected_before_evaluation(self):
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'run';sim.generate_run(dest,sim.load_settings(duration_s=15),'custom')
            with (dest/'truth.jsonl').open('a') as stream:stream.write('\n')
            with self.assertRaisesRegex(ValueError,'truth.jsonl'):evaluate(dest)
            self.assertFalse((dest/'evaluation.json').exists())


if __name__=='__main__':unittest.main()
