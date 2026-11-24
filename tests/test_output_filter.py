"""Output-policy tests: memory buses only, never creates/publishes native DDS."""
import asyncio
import copy
import itertools
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import uuid

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
from output_filter import DEFAULTS, ConfigStore, OutputFilter, validate
from fusion import Fusion, FusedTrack
from dds_fusion_node import FusionDdsNode
from company_dds import Topics


SENSORS = {"radar_3152": dict(kind="radar", pos=[0, 0, 100], station_known=True,
                              station_source="configured_lla", station_lla=[120, 30, 122])}


class MemoryBus:
    def __init__(self):
        self.messages, self.commands, self.official = {}, [], []
        self.on_write = None

    def write(self, topic, payload, **kw):
        self.messages[topic] = payload
        if topic == Topics.CONTROL_COMMAND.name:
            self.commands.append(payload)
        if self.on_write:
            self.on_write(topic, payload)

    def read(self, topic):
        if topic == Topics.CONTROL_COMMAND.name:
            rows, self.commands = self.commands, []
            return rows
        return []

    def publish_official_fusion(self, rows, **kw):
        self.official.append((copy.deepcopy(rows), kw))


class PolicyFixture(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"output_filter.yaml"
        self.path.write_text(yaml.safe_dump(DEFAULTS))
        self.policy = OutputFilter(self.path)

    def row(self, **changes):
        return dict(dict(id=1, position_update_count=2, vx=3., vy=4., z=130.,
                         x=0., y=0., cls="unknown", track_age_s=0.), **changes)

    def decide(self, row, sensors=None, eligible=True):
        self.policy.select([row], [row] if eligible else [], SENSORS if sensors is None else sensors, 1000)
        return self.policy.last_decisions[0]

    def command(self, **extra):
        return dict(dict(cmd="output_filter_apply", request_id=uuid.uuid4().hex,
                    expires_at=time.time()+5, instance_id=self.policy.instance_id,
                    expected_revision=self.policy.revision, expected_saved_hash=self.policy.saved_hash,
                    settings=dict(self.policy.settings), save=False), **extra)


class PolicyTests(PolicyFixture):
    def test_all_independent_switch_combinations(self):
        flags = ['points_enabled', 'min_speed_enabled', 'max_speed_enabled', 'height_enabled']
        for values in itertools.product((False, True), repeat=4):
            settings = {**DEFAULTS, **dict(zip(flags, values))}
            self.policy.command(self.command(settings=settings))
            self.assertTrue(self.policy.last_command['ok'])
            for speed, flag, reason in ((.1, 'min_speed_enabled', 'speed_below_min'),
                                         (20., 'max_speed_enabled', 'speed_above_max')):
                with self.subTest(values=values, speed=speed):
                    row = self.row(position_update_count=1, vx=speed, vy=0, z=110)
                    expected = []
                    if settings['points_enabled']: expected.append('too_few_points')
                    if settings[flag]: expected.append(reason)
                    if settings['height_enabled']: expected.append('height_below_min')
                    self.assertEqual(self.decide(row)['reasons'], expected)
                    row['track_age_s'] = 9.
                    self.assertTrue(self.decide(row)['passed'])
                    self.assertEqual(self.decide(row, eligible=False)['reasons'], ['existing_output_gate'])

    def test_switches_and_thresholds_saved_together_and_loaded_at_startup(self):
        flags = ['points_enabled', 'min_speed_enabled', 'max_speed_enabled', 'height_enabled']
        for values in itertools.product((False, True), repeat=4):
            with self.subTest(values=values):
                before = self.path.read_bytes()
                settings = {**DEFAULTS, **dict(zip(flags, values)), 'min_points': 4,
                            'min_speed_mps': .9, 'max_speed_mps': 20., 'min_height_m': 40.}
                self.policy.command(self.command(settings=settings, save=True))
                self.assertTrue(self.policy.last_command['ok'])
                self.assertEqual(yaml.safe_load(self.path.read_text()), settings)
                self.assertEqual(OutputFilter(self.path).settings, settings)
                self.assertEqual(Path(self.policy.last_command['backup_path']).read_bytes(), before)
                self.assertFalse(self.policy.snapshot()['unsaved'])

    def test_temporary_switch_changes_do_not_write_config(self):
        before = self.path.read_bytes()
        settings = {**DEFAULTS, 'points_enabled': False, 'min_speed_enabled': False,
                    'max_speed_enabled': False, 'height_enabled': False}
        self.policy.command(self.command(settings=settings))
        self.assertTrue(self.policy.last_command['ok'])
        self.assertEqual(self.policy.settings, settings)
        self.assertEqual(self.path.read_bytes(), before)
        self.assertEqual(OutputFilter(self.path).settings, DEFAULTS)

    def test_one_sided_speed_bound_and_reenable_validation(self):
        for disabled in ('min_speed_enabled', 'max_speed_enabled'):
            settings = {**DEFAULTS, disabled: False, 'min_speed_mps': 20., 'max_speed_mps': 15.}
            self.policy.command(self.command(settings=settings, save=True))
            self.assertTrue(self.policy.last_command['ok'])
            self.assertEqual(OutputFilter(self.path).settings, settings)
            self.policy.command(self.command(settings={**settings, disabled: True}, save=True))
            self.assertFalse(self.policy.last_command['ok'])
            self.assertEqual(OutputFilter(self.path).settings, settings)
            self.assertEqual(self.policy.settings, settings)

    def test_failed_save_does_not_apply_switch_changes(self):
        before = self.path.read_bytes()
        for flag in ('enabled', 'points_enabled', 'min_speed_enabled', 'max_speed_enabled', 'height_enabled'):
            with patch.object(self.policy.store, 'save', side_effect=OSError('permission denied')):
                self.policy.command(self.command(settings={**DEFAULTS, flag: False}, save=True))
            self.assertFalse(self.policy.last_command['ok'])
            self.assertEqual(self.policy.settings, DEFAULTS)
            self.assertEqual(self.path.read_bytes(), before)

    def test_all_four_rules_end_at_nine_seconds(self):
        for age in (0.,3.,8.999):
            row=self.row(track_age_s=age,position_update_count=1,vx=.1,vy=0,z=110)
            result=self.decide(row)
            self.assertFalse(result['passed'])
            self.assertEqual(result['filter_phase'],'initial')
            self.assertEqual(set(result['reasons']),{'too_few_points','speed_below_min','height_below_min'})
            self.assertIn('speed_above_max',self.decide(self.row(track_age_s=age,vx=20,vy=0))['reasons'])
        for age in (9.,9.001,100.):
            for speed in (.1,20.):
                result=self.decide(self.row(track_age_s=age,position_update_count=1,vx=speed,vy=0,z=110))
                self.assertTrue(result['passed'])
                self.assertEqual(result['filter_phase'],'complete')
                self.assertFalse(result['initial_filter_active'])

    def test_missing_values_stop_blocking_after_window_but_old_gate_remains(self):
        row=self.row(track_age_s=9.,position_update_count=None,vx=None,vy=None,z=None)
        self.assertTrue(self.decide(row,{})['passed'])
        self.assertEqual(self.decide(row,{},eligible=False)['reasons'],['existing_output_gate'])
        for age in (None,float('nan'),-1):
            result=self.decide(self.row(track_age_s=age,vx=0,vy=0))
            self.assertEqual(result['filter_phase'],'age_unknown')
            self.assertFalse(result['passed'])

    def test_applying_parameters_does_not_restart_old_track_window(self):
        self.policy.command(self.command(settings={**DEFAULTS,'min_points':999,'min_height_m':900}))
        row=self.row(track_age_s=12.,position_update_count=1,vx=0,vy=0,z=0)
        self.assertTrue(self.decide(row)['passed'])
        self.assertEqual(self.policy.snapshot()['filter_window_s'],9.)

    def test_inclusive_boundaries_and_relative_height(self):
        for speed in [1.,15.]:
            d = self.decide(self.row(vx=speed,vy=0.))
            self.assertTrue(d["passed"])
            self.assertEqual(d["height_above_radar_m"],30.)
        self.assertFalse(self.decide(self.row(z=129.99))["passed"])
        self.assertFalse(self.decide(self.row(z=30))["passed"])

    def test_each_rule_or_logic(self):
        for update, expected in [({"position_update_count":1},"too_few_points"),
                ({"vx":.99,"vy":0},"speed_below_min"),({"vx":15.01,"vy":0},"speed_above_max"),
                ({"z":129},"height_below_min")]:
            with self.subTest(update=update):
                self.assertIn(expected,self.decide(self.row(**update))["reasons"])

    def test_missing_and_non_finite_fail_closed(self):
        for update, reason in [({"vx":None},"speed_unavailable"),({"vx":float("nan")},"speed_unavailable"),
                ({"vy":float("inf")},"speed_unavailable"),({"z":None},"height_unavailable"),
                ({"position_update_count":None},"points_unavailable")]:
            self.assertIn(reason,self.decide(self.row(**update))["reasons"])
        self.assertIn("radar_station_unavailable",self.decide(self.row(),{})["reasons"])
        unknown=copy.deepcopy(SENSORS);unknown["radar_3152"]["station_known"]=False
        self.assertFalse(self.decide(self.row(),unknown)["passed"])

    def test_disabled_rules_do_not_block_missing_values(self):
        self.policy.settings.update(height_enabled=False,points_enabled=False,min_speed_enabled=False,max_speed_enabled=False)
        self.assertTrue(self.decide(self.row(vx=None,z=None,position_update_count=None),{})["passed"])

    def test_master_off_preserves_original_gate(self):
        self.policy.settings["enabled"]=False
        self.assertTrue(self.decide(self.row(vx=0,vy=0,z=None),{})["passed"])
        self.assertFalse(self.decide(self.row(),eligible=False)["passed"])

    def test_does_not_mutate_or_reclassify_input(self):
        row=self.row(z=0);before=copy.deepcopy(row)
        self.decide(row)
        self.assertEqual(row,before)
        self.assertEqual(row["cls"],"unknown")

    def test_capacity_and_empty_batches(self):
        rows=[self.row(id=i) for i in range(260)]
        self.assertEqual(len(self.policy.select(rows,rows,SENSORS,1)),255)
        self.assertEqual(self.policy.last_decisions[-1]["reasons"],["dds_capacity"])
        self.assertEqual(self.policy.select([],[],SENSORS,2),[])
        self.policy.published()
        self.assertTrue(self.policy.snapshot()["batch"]["publish_ok"])

    def test_validation(self):
        for changes in [dict(min_points=2.1),dict(min_points=True),dict(min_speed_mps=-1),
                        dict(min_height_m=float("nan")),dict(enabled="true"),dict(min_speed_mps=16),
                        dict(reference_radar_id="../../bad"),dict(unexpected=1)]:
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                validate({**DEFAULTS,**changes})

    def test_temporary_apply_and_restart(self):
        before=self.path.read_bytes()
        self.policy.command(self.command(settings={**DEFAULTS,"min_points":5}))
        self.assertTrue(self.policy.last_command["ok"])
        self.assertTrue(self.policy.snapshot()["unsaved"])
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(OutputFilter(self.path).settings["min_points"],2)

    def test_saved_apply_backup_and_restart(self):
        before=self.path.read_bytes()
        self.policy.command(self.command(settings={**DEFAULTS,"min_points":4},save=True))
        self.assertTrue(self.policy.last_command["ok"])
        self.assertEqual(Path(self.policy.last_command["backup_path"]).read_bytes(),before)
        self.assertEqual(OutputFilter(self.path).settings["min_points"],4)
        self.assertFalse(self.policy.snapshot()["unsaved"])

    def test_stale_revision_restart_and_expiration(self):
        for extra in [dict(expected_revision=-1),dict(instance_id="old"),dict(expires_at=time.time()-1)]:
            self.policy.command(self.command(**extra))
            self.assertFalse(self.policy.last_command["ok"])
            self.assertEqual(self.policy.revision,0)

    def test_file_conflict_or_failed_save_leaves_live_state(self):
        old=dict(self.policy.settings)
        self.path.write_text(yaml.safe_dump({**DEFAULTS,"min_points":9}))
        self.policy.command(self.command(settings={**DEFAULTS,"min_points":5},save=True))
        self.assertFalse(self.policy.last_command["ok"])
        self.assertEqual(self.policy.settings,old)
        self.policy.refresh_saved()
        self.assertEqual(self.policy.saved_settings["min_points"],9)
        with patch.object(self.policy.store,"save",side_effect=OSError("permission denied")):
            self.policy.command(self.command(settings={**DEFAULTS,"min_points":6},save=True))
        self.assertEqual(self.policy.settings,old)

    def test_duplicate_command_not_applied_twice(self):
        cmd=self.command(save=True)
        self.policy.command(cmd);self.policy.command(cmd)
        self.assertEqual(self.policy.revision,1)
        self.assertEqual(len(list(Path(self.temp.name).glob("*.bak-*"))),1)

    def test_invalid_file_fails_startup(self):
        self.path.write_text("enabled: nope")
        with self.assertRaises(ValueError):OutputFilter(self.path)


def radar(stamp, *, fresh=True, x=500., z=150.):
    return dict(id=8,source_track_id=89,sensor_id="radar_3152",x=x,y=100.,z=z,
                vx=4.,vy=0.,cov=[[100.,0],[0,100.]],fresh=fresh,new_report=True,
                source_fresh=fresh,input_mode="device_tracks",report_timestamp_ms=stamp,
                last_measurement_ms=stamp if fresh else None,cls="unknown",cls_conf=.8)


class PipelineTests(PolicyFixture):
    def setUp(self):
        super().setUp()
        self.bus=MemoryBus();self.node=FusionDdsNode(bus=self.bus,output_filter_path=self.path)

    def send(self,tick,rt=None,stream="test"):
        msg=dict(tick=tick,dt=3.,sensors=SENSORS,stream_id=stream,live_timing=True,
                 source_timestamp_ms=100000+tick*3000,t=tick*3.,radar_tracks=[] if rt is None else [rt],
                 bearing_tracks={},spectrum={},measurements={},coordinates={"origin":[120,30,22]})
        self.node.process_tracks(msg)
        return self.bus.messages[Topics.FUSED_TRACKS.name]

    def test_only_official_output_is_filtered_and_id_recovers(self):
        first=self.send(1,radar(103000))
        self.assertEqual(self.bus.official[-1][0],[])
        self.assertEqual(len(first["fused"]),1)
        self.assertEqual(len(first["external_tracks"]),1)
        ident=first["fused"][0]["id"]
        self.send(2,radar(106000,x=512))
        self.assertEqual(self.bus.official[-1][0][0]["id"],ident)
        self.assertEqual(self.bus.official[-1][0][0]["position_update_count"],2)
        self.send(3,radar(109000,x=524,z=120))
        self.assertEqual(self.bus.official[-1][0],[])
        self.send(4,radar(112000,x=536,z=150))
        self.assertEqual(self.bus.official[-1][0][0]["id"],ident)

    def test_pipeline_window_uses_birth_not_latest_report(self):
        self.node.output_filter.settings['min_points']=999
        for tick in range(1,5):
            row=radar(100000+tick*3000,z=120);row['vx']=0
            result=self.send(tick,row)
            age=result['fused'][0]['track_age_s']
            self.assertEqual(age,(tick-1)*3.)
            self.assertEqual(bool(self.bus.official[-1][0]),tick>=4)
        age_before=self.node.fusion.export()[0]['track_age_s']
        self.send(4,radar(112000))  # repeated tick: ignored, not a new birth
        self.assertEqual(self.node.fusion.export()[0]['track_age_s'],age_before)
        self.send(5,radar(115000,z=120),stream='new-stream')
        self.assertEqual(self.node.fusion.export()[0]['track_age_s'],0.)
        self.assertEqual(self.bus.official[-1][0],[])

    def test_deleted_track_rebirth_gets_new_window(self):
        first=self.send(1,radar(103000))['fused'][0]['id']
        self.send(10)  # report gap is well past existing deletion horizon
        self.assertEqual(self.node.fusion.tracks,[])
        born=self.send(11,radar(133000))['fused'][0]
        self.assertNotEqual(born['id'],first)
        self.assertEqual(born['track_age_s'],0.)

    def test_demo_age_accumulates_variable_dt_and_is_not_frame_count(self):
        f=Fusion({},3.)
        rt=radar(1000);f.step([rt],{}, {})
        self.assertEqual(f.export(include_coasting=True)[0]['track_age_s'],0.)
        for step,dt in enumerate((2.98,3.02,3.),start=1):
            f.dt=dt;f.step([radar(1000+step*3000)],{}, {})
        self.assertAlmostEqual(f.export(include_coasting=True)[0]['track_age_s'],9.)

    def test_demo_decimal_dt_reaches_exact_nine_second_boundary(self):
        f=Fusion({},.1);f.step([radar(1000)],{}, {})
        for i in range(90):f.step([radar(1100+i*100)],{}, {})
        row=f.export(include_coasting=True)[0]
        self.assertEqual(row['track_age_s'],9.)
        self.assertFalse(self.decide(row)['initial_filter_active'])

    def test_new_track_born_late_still_gets_its_own_window(self):
        self.send(20,radar(160000))
        self.assertEqual(self.node.fusion.export()[0]['track_age_s'],0.)
        second=radar(163000,x=5000);second['source_track_id']=90;second['id']=9
        result=self.send(21,second)
        rows={r['source_track_id']:r for r in result['external_tracks']}
        self.assertEqual(rows[89]['track_age_s'],3.)
        self.assertEqual(rows[90]['track_age_s'],0.)

    def test_merge_keeps_oldest_birth_even_if_newer_id_survives(self):
        f=Fusion({},.1);f.time_s=12.
        a=FusedTrack([500,100,0,0],np.eye(4),.1,'radar',alt=100)
        b=FusedTrack([501,100,0,0],np.eye(4),.1,'radar',alt=100)
        a.created_at_s=1.;b.created_at_s=11.
        a.touch('radar',24,position_capable=True);b.touch('radar',24,position_capable=True)
        b.touch('rf_df',24)
        f.tracks=[a,b];f.step([],{}, {})
        self.assertEqual(len(f.tracks),1)
        self.assertEqual(f.tracks[0].id,b.id)
        self.assertEqual(f.tracks[0].created_at_s,1.)
        self.assertEqual(f.export(include_coasting=True)[0]['track_age_s'],11.)

    def test_prediction_extrapolation_duplicate_and_restart_points(self):
        self.send(1,radar(103000))
        self.send(2,radar(106000,fresh=False))
        self.assertEqual(self.node.fusion.tracks[0].position_update_count,1)
        self.send(3,radar(103000))
        self.assertEqual(self.node.fusion.tracks[0].position_update_count,1)
        self.send(4)
        self.assertEqual(self.node.fusion.tracks[0].position_update_count,1)
        self.send(5,radar(115000),stream="restart")
        self.assertEqual(self.node.fusion.tracks[0].position_update_count,1)

    def test_counter_distinct_time_not_state_change(self):
        t=FusedTrack([1,1,0,0],np.eye(4),.1,"test")
        for stamp in [1000,1000,999,None,float("nan"),2000,2000]:t.record_position_update(stamp)
        self.assertEqual(t.position_update_count,2)
        t.predict();t.update_bearing([0,0,0],.78,.01)
        self.assertEqual(t.position_update_count,2)

    def test_apply_without_input_data_acknowledged(self):
        p=self.node.output_filter
        cmd=dict(cmd="output_filter_apply",request_id=uuid.uuid4().hex,expires_at=time.time()+5,
                 instance_id=p.instance_id,expected_revision=0,save=False,settings={**DEFAULTS,"min_points":7})
        self.bus.commands.append(cmd);self.node.process_controls()
        status=self.bus.messages[Topics.DIAGNOSTICS.name]["output_filter"]
        self.assertTrue(status["last_command"]["ok"])
        self.assertEqual(status["settings"]["min_points"],7)

    def test_publish_error_not_reported_as_success(self):
        with patch.object(self.bus,"publish_official_fusion",side_effect=RuntimeError("DDS failed")):
            with self.assertRaises(RuntimeError):self.send(1,radar(103000))
        state=self.bus.messages[Topics.DIAGNOSTICS.name]["output_filter"]
        self.assertFalse(state["batch"]["publish_ok"])
        self.assertIn("DDS failed",state["batch"]["publish_error"])

    def test_eo_position_counts_but_single_bearing_does_not(self):
        from test_eo_fusion import STATIONS, position, bearing
        from dds_frame import SensorStub
        f=Fusion({s:SensorStub(s,'eo',p) for s,p in STATIONS.items()},.1)
        def step(stamp, observations):
            f.time_s=stamp;f.step([],observations,{})
            return f.export(include_coasting=True)
        rows=step(1,{'eo_201':[position(1)],'eo_202':[position(1,(501,501),99)]})
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['position_update_count'],1)
        rows=step(1.1,{'eo_201':[bearing('eo_201',1.1)]})
        self.assertEqual(rows[0]['position_update_count'],1)
        rows=step(1.2,{'eo_201':[position(1.2,(501,500))]})
        self.assertEqual(rows[0]['position_update_count'],2)


class Rk3588IntegrationTests(PolicyFixture):
    def test_both_filter_and_existing_dds_monitor_apis_remain_available(self):
        from dds_ws_gateway import create_app
        from test_alignment_debug import AsgiClient
        app = create_app(bus=MemoryBus(), alignment_profile_path=Path(self.temp.name)/'alignment.json')
        client = AsgiClient(app)
        self.assertEqual(client.get('/api/output-filter').status_code, 200)
        self.assertEqual(client.get('/api/official-fusion').status_code, 200)
        self.assertIn('official_fusion', client.get('/api/diagnostics').json())

    def test_filter_does_not_replace_rk3588_confidence_policy(self):
        for minimum in (.05, .2):
            node = FusionDdsNode(bus=MemoryBus(), min_confidence=minimum, output_filter_path=self.path)
            node.sync_fusion(SENSORS)
            self.assertEqual(node.fusion.min_confidence, minimum)
            self.assertEqual(node.fusion_eo.min_confidence, minimum)
            self.assertEqual(node.output_filter.settings, DEFAULTS)


class APITests(PolicyFixture):
    def test_api_ack_save_conflict_and_cross_origin(self):
        from test_alignment_debug import AsgiClient
        from fastapi import FastAPI
        from types import SimpleNamespace
        from output_filter_api import install_output_filter_api
        bus=MemoryBus()
        gateway=SimpleNamespace(bus=bus,nodes={},node_received_at={},latest_frame=None)
        def update():
            gateway.nodes["fusion"]={"output_filter":self.policy.snapshot()}
            gateway.node_received_at["fusion"]=time.time()
        update()
        def dispatch(topic,payload):
            if topic==Topics.CONTROL_COMMAND.name:self.policy.command(payload);update()
        bus.on_write=dispatch
        app=FastAPI();install_output_filter_api(app,gateway)
        client=AsgiClient(app)
        state=client.get('/api/output-filter').json()
        payload=dict(instance_id=self.policy.instance_id,expected_revision=0,
            expected_saved_hash=self.policy.saved_hash,settings={**DEFAULTS,"min_points":6,
                "points_enabled":False,"max_speed_enabled":False,"min_speed_mps":20.,"height_enabled":False},save=True)
        headers={"X-Output-Filter-Token":state["token"]}
        bad=client.post('/api/output-filter/apply',json=payload)
        self.assertEqual(bad.status_code,403)
        cross=client.post('/api/output-filter/apply',json=payload,headers={**headers,"Origin":"https://bad.example"})
        self.assertEqual(cross.status_code,403)
        ok=client.post('/api/output-filter/apply',json=payload,headers=headers)
        self.assertEqual(ok.status_code,200)
        self.assertTrue(ok.json()["result"]["saved"])
        self.assertEqual(OutputFilter(self.path).settings['min_points'],6)
        self.assertEqual(OutputFilter(self.path).settings,payload['settings'])
        self.assertEqual(ok.json()['state']['settings'],payload['settings'])
        conflict=client.post('/api/output-filter/apply',json=payload,headers=headers)
        self.assertEqual(conflict.status_code,409)
        # A delivery with no matching fusion ACK is pending, never success.
        payload.update(expected_revision=self.policy.revision,expected_saved_hash=self.policy.saved_hash,save=False)
        bus.on_write=None
        pending=client.post('/api/output-filter/apply',json=payload,headers=headers)
        self.assertEqual(pending.status_code,202)
        self.assertTrue(pending.json()['pending'])
        gateway.node_received_at['fusion']=0
        unavailable=client.post('/api/output-filter/apply',json=payload,headers=headers)
        self.assertEqual(unavailable.status_code,503)


if __name__ == '__main__':
    unittest.main()
