"""Control-page regressions. Every publishing test replaces the sender with a harmless child."""
import http.client
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
sys.path.insert(0,str(ROOT/'dds_swarm_fusion'))
import field_sim_web as web
import field_simulator as sim
from company_protocol import parse_company_packet


def wait(ctl):
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        state=ctl.state();job=state['job']
        if job and job['state'] not in web.common.ACTIVE:
            if job['action']!='check' or job['state']!='completed' or (state['preflight'] and state['preflight'].get('bundle_sha256')):
                return state
        time.sleep(.015)
    raise AssertionError('Task did not finish')


class CustomSimulatorTests(unittest.TestCase):
    def test_independent_ids_sites_rates_and_modes(self):
        cfg=sim.load_settings(duration_s=15,custom_radar_enabled=False,custom_eo_count=2,
            eo_device_ids=[8001,8002],eo_stations_enu=[[10,20,4],[450,30,5]],
            eo_rates_hz=[5,8],eo_modes=['position','bearing'])
        rows=sim.Simulation(cfg,'custom').generate()
        devices={}
        for event in rows:
            obj=parse_company_packet(event['packet'])[0][2]
            devices.setdefault(obj.device_id,[]).append(obj)
        self.assertEqual(set(devices),{8001,8002})
        self.assertLess(len(devices[8001]),len(devices[8002]))
        self.assertTrue(all(x.target_lng_deg!=0 for x in devices[8001]))
        self.assertTrue(all(x.target_lng_deg==0 and x.target_dis_m==0 for x in devices[8002]))
        self.assertNotEqual(devices[8001][0].ptz_lng_deg,devices[8002][0].ptz_lng_deg)

    def test_one_eo_does_not_send_second(self):
        rows=sim.Simulation(sim.load_settings(duration_s=15,custom_radar_enabled=False),'custom').generate()
        self.assertEqual({e['device_id'] for e in rows},{71112})

    def test_modes_and_profiles_are_independent(self):
        cfg=sim.load_settings(duration_s=15,custom_eo_count=2,eo_modes=['ranged','range-failure'])
        with tempfile.TemporaryDirectory() as td:
            dest=Path(td)/'run';sim.generate_run(dest,cfg,'custom')
            devices=json.loads((dest/'eo_devices.yaml').read_text())['devices']
            self.assertEqual(devices['71112']['range_policy'],'reported_positive')
            self.assertEqual(devices['71113']['range_policy'],'disabled')
            events=[json.loads(line) for line in (dest/'events.jsonl').read_text().splitlines()]
            self.assertTrue(any(e['device_id']==71113 and e['mode']=='bearing' for e in events))

    def test_invalid_custom_settings_rejected(self):
        for kw in (dict(custom_eo_count=3),dict(custom_eo_count=True),dict(eo_modes=['unknown','bearing']),
                   dict(eo_modes=[{},'bearing']),dict(custom_radar_enabled='yes'),
                   dict(eo_stations_enu=[[0,0,-1],[300,0,3]]),dict(radar_station_enu=[0,float('nan'),3]),
                   dict(unknown=1),dict(eo_device_ids=[2,2])):
            with self.subTest(kw=kw),self.assertRaises(ValueError):sim.load_settings(**kw)


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.ctl=web.Controller(self.root);self.addCleanup(self.ctl.close)

    def generate(self):
        reply=self.ctl.start({'action':'generate','settings':dict(duration_s=15,custom_eo_count=2,
            custom_radar_enabled=False,eo_modes=['bearing','bearing'])})
        state=wait(self.ctl);self.assertEqual(state['job']['state'],'completed',state['job'])
        return reply['run_id']

    def checked(self):
        run=self.generate();self.ctl.start({'action':'check','run_id':run});state=wait(self.ctl)
        self.assertEqual(state['job']['state'],'completed')
        return run,state['preflight']

    def test_generate_details_evaluate_and_check_do_not_publish(self):
        run=self.generate();meta=self.ctl.metadata(run,verify=True)
        self.assertEqual(len(meta['device_counts']),2)
        details=self.ctl.details(run);self.assertIn('SB_EO_CONFIG=',details['launch_command'])
        self.assertIn(run,details['launch_command'])
        self.assertEqual(meta['dds_packets_sent'],0)
        self.ctl.start({'action':'evaluate','run_id':run});state=wait(self.ctl)
        self.assertEqual(state['job']['state'],'completed',state['job'])
        report=self.ctl.details(run)['evaluation']
        self.assertTrue(report['all_checks_pass']);self.assertGreater(report['formal_state_coverage'],0)
        with self.assertRaises(web.Error):self.ctl.start({'action':'evaluate','run_id':run})
        self.ctl.start({'action':'check','run_id':run});state=wait(self.ctl)
        self.assertTrue(state['preflight']['bundle_sha256'])
        self.assertTrue(any(e.get('native_library_loaded') is False for e in state['job']['events']))

    def test_no_publish_without_matching_ticket_and_all_confirmations(self):
        run,ticket=self.checked();self.ctl.native_available=True
        body={'action':'replay','run_id':run,'preflight_id':ticket['id'],'isolated':True,
              'fusion_ready':True,'confirm_send':True,'inputs_stopped':True}
        for changes in ({'isolated':False},{'fusion_ready':False},{'confirm_send':False},
                        {'inputs_stopped':False},{'preflight_id':'wrong'}):
            with self.subTest(changes=changes),patch.object(self.ctl,'popen') as spawn,self.assertRaises(web.Error):
                self.ctl.start({**body,**changes});spawn.assert_not_called()

    def test_changed_bundle_refuses_send(self):
        run,ticket=self.checked();self.ctl.native_available=True
        with (self.root/run/'eo_devices.yaml').open('a') as f:f.write('\n')
        with patch.object(self.ctl,'popen') as spawn,self.assertRaises(web.Error):
            self.ctl.start({'action':'replay','run_id':run,'preflight_id':ticket['id'],
                'isolated':True,'fusion_ready':True,'confirm_send':True,'inputs_stopped':True})
            spawn.assert_not_called()

    def test_publishing_command_is_fixed_and_per_device_counts_are_not_acks(self):
        run,ticket=self.checked();self.ctl.native_available=True;seen=[]
        def harmless(args,**kwargs):
            seen.append((args,kwargs))
            return subprocess.Popen([sys.executable,'-c',
                'print(\'{"event":"replay_progress","published_calls":3,"total":270}\')'],**kwargs)
        self.ctl.popen=harmless
        self.ctl.start({'action':'replay','run_id':run,'preflight_id':ticket['id'],
            'isolated':True,'fusion_ready':True,'confirm_send':True,'inputs_stopped':True})
        state=wait(self.ctl);args,kwargs=seen[0]
        self.assertEqual(args[2],str(ROOT/'tools/dds_data_tool.py'))
        self.assertIn('--expected-data-sha256',args);self.assertIn('--publish',args)
        self.assertNotIn('shell',kwargs);self.assertIsNone(state['preflight'])
        self.assertEqual(sum(state['job']['sent_by_device'].values()),3)
        with self.assertRaises(web.Error):self.ctl.start({'action':'replay','run_id':run,'isolated':True,'fusion_ready':True})

    def test_path_and_symlink_refused(self):
        for run in ('../run-20261022-000000-12345678','/etc','bad'):
            with self.assertRaises(web.Error):self.ctl.run_path(run)
        run='run-20261022-000000-12345678';(self.root/run).symlink_to(self.root,target_is_directory=True)
        with self.assertRaises(web.Error):self.ctl.run_path(run)
        with self.assertRaises(web.Error):self.ctl.file_path(run,'../../secret')

    def test_busy_stop_only_owned_child_and_no_web_token_in_child(self):
        seen=[]
        def sleeper(args,**kw):
            seen.append(kw)
            return subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'],**kw)
        ctl=web.Controller(self.root/'separate',popen=sleeper);self.addCleanup(ctl.close)
        with patch.dict('os.environ',{'FIELD_SIM_WEB_TOKEN':'secret-key-that-must-not-be-in-child'}):
            result=ctl.start({'action':'generate','settings':{'duration_s':15}})
        self.assertNotIn('FIELD_SIM_WEB_TOKEN',seen[0]['env'])
        with self.assertRaises(web.Error):ctl.start({'action':'generate','settings':{}})
        with self.assertRaises(web.Error):ctl.stop('wrong-id')
        ctl.stop(result['job_id']);self.assertIn(wait(ctl)['job']['state'],('stopped','failed'))


class HttpTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.ctl=web.Controller(self.tmp.name);self.addCleanup(self.ctl.close)
        self.key='test-only-key-for-local-http-test'
        self.server=web.Server(('127.0.0.1',0),self.ctl,self.key)
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.addCleanup(self.server.server_close);self.addCleanup(self.server.shutdown)
        self.port=self.server.server_address[1];self.cookie=''

    def req(self,method,path,body=None,origin=None,cookie=None):
        con=http.client.HTTPConnection('127.0.0.1',self.port,timeout=10)
        headers={'Cookie':self.cookie if cookie is None else cookie}
        if method=='POST':headers.update(Origin=origin or f'http://127.0.0.1:{self.port}',**{'Content-Type':'application/json'})
        con.request(method,path,json.dumps(body) if body is not None else None,headers)
        response=con.getresponse();result=response.status,dict(response.getheaders()),response.read();con.close();return result

    def login(self):
        code,headers,_=self.req('POST','/api/login',{'key':self.key});self.assertEqual(code,200)
        self.cookie=headers['Set-Cookie'].split(';')[0]

    def test_page_assets_and_separate_cookie_auth(self):
        for path in ('/','/app.js','/style.css'):
            code,headers,data=self.req('GET',path);self.assertEqual(code,200)
            self.assertIn("frame-ancestors 'none'",headers['Content-Security-Policy'])
        self.assertEqual(self.req('GET','/api/state')[0],401)
        self.login();self.assertTrue(self.cookie.startswith('field_sim_session='))
        self.assertEqual(self.req('GET','/api/state')[0],200)
        self.assertEqual(self.req('GET','/api/state',cookie='dds_ui_session='+self.server.session_key)[0],401)

    def test_csrf_bad_login_and_no_arbitrary_endpoints(self):
        self.assertEqual(self.req('POST','/api/login',{'key':self.key},origin='http://attacker.invalid')[0],403)
        self.assertEqual(self.req('POST','/api/login',{'key':'wrong'})[0],401)
        self.login()
        self.assertEqual(self.req('POST','/api/jobs',{'action':'record'})[0],400)
        self.assertEqual(self.req('GET','/api/file?id=../etc&name=passwd')[0],403)
        self.assertEqual(self.req('POST','/api/jobs',{'action':'generate','settings':{'custom_eo_count':99}})[0],400)
        self.assertIsNone(self.ctl.job)

    def test_http_generation_and_download(self):
        self.login();code,_,raw=self.req('POST','/api/jobs',{'action':'generate','settings':{'duration_s':15}})
        self.assertEqual(code,202);run=json.loads(raw)['run_id'];wait(self.ctl)
        code,_,raw=self.req('GET','/api/run?id='+run);self.assertEqual(code,200)
        self.assertIn('launch_command',json.loads(raw))
        code,headers,data=self.req('GET',f'/api/file?id={run}&name=capture.jsonl')
        self.assertEqual(code,200);self.assertIn('attachment',headers['Content-Disposition'])
        self.assertEqual(json.loads(data.splitlines()[0])['format'],'sb-dds-capture-v1')
        self.assertEqual(self.req('POST','/api/logout',{})[0],200)


if __name__=='__main__':unittest.main()
