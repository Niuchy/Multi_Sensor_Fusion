"""Independent UI tests. No native DDS loads, no production server or publisher."""
import contextlib
import http.client
import io
import json
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.parse import urlencode

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
import dds_data_web as web
import dds_data_tool as data


def capture(path, complete=True):
    writer = data.CaptureWriter(path, data.TOPICS, data.MAX_FILE)
    writer.add("RadarTarget", b"TEST ONLY / NOT A DEVICE PACKET", 0, 1)
    if complete:
        writer.finish("test")
    else:
        writer.stream.close()


def wait_job(controller):
    deadline = time.monotonic()+5
    while time.monotonic()<deadline:
        state=controller.state()
        if state["job"]["state"] not in web.ACTIVE:
            return state
        time.sleep(.01)
    raise AssertionError("test child did not finish")


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name)
        self.controller=web.Controller(self.root)
        self.addCleanup(self.controller.close)
        capture(self.root/"test.jsonl")

    def check(self, **options):
        self.controller.start({"action":"check","file":"test.jsonl",**options})
        return wait_job(self.controller)

    def test_preflight_runs_without_dds_and_requires_positive_count(self):
        state=self.check()
        self.assertEqual(state["job"]["state"],"completed")
        self.assertEqual(state["preflight"]["summary"]["selected_records"],1)
        self.assertTrue(any(e.get("native_library_loaded") is False for e in state["job"]["events"]))
        self.assertIsNone(self.check(topics=["EOTarget"])["preflight"])

    def test_malformed_or_changed_file_refused_before_native(self):
        state=self.check()
        capture(self.root/'replacement.jsonl')
        (self.root/'replacement.jsonl').replace(self.root/'test.jsonl')
        args=['replay',str(self.root/"test.jsonl"),'--expected-data-sha256',state['preflight']['sha256'],'--publish','--inputs-stopped']
        with patch.object(data,'NativeBus') as native,patch.object(data,'ensure_runtime') as env,contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(data.main(args),1)
            native.assert_not_called();env.assert_not_called()
        self.assertEqual(len(state['preflight']['sha256']),64)

    def test_confirm_ticket_and_options_required(self):
        state=self.check();self.controller.native_available=True
        request={"action":"replay","file":"test.jsonl","preflight_id":state["preflight"]["id"],"confirm_send":True,"inputs_stopped":True}
        for changes in ({"confirm_send":False},{"inputs_stopped":False},{"preflight_id":"bad"},{"speed":2},{"time_mode":"rebase"},{"topics":["RadarTarget"]}):
            with self.subTest(changes=changes), self.assertRaises(web.RequestError), patch.object(self.controller,'popen') as child:
                self.controller.start({**request,**changes})
                child.assert_not_called()

    def test_replay_uses_fixed_argv_and_one_use_checksum_ticket(self):
        state=self.check();self.controller.native_available=True
        seen=[]
        def harmless(args,**kwargs):
            seen.append((args,kwargs))
            return subprocess.Popen([sys.executable,'-c','print("test worker, no DDS")'],**kwargs)
        self.controller.popen=harmless
        request={"action":"replay","file":"test.jsonl","preflight_id":state['preflight']['id'],"confirm_send":True,"inputs_stopped":True}
        self.controller.start(request);wait_job(self.controller)
        args,kwargs=seen[0]
        self.assertIn('--publish',args);self.assertIn('--inputs-stopped',args)
        self.assertEqual(args[-1],state['preflight']['sha256'])
        self.assertNotIn('shell',kwargs)
        self.assertNotIn('DDS_WEB_TOKEN',kwargs['env'])
        self.assertIsNone(self.controller.ticket)
        with self.assertRaises(web.RequestError): self.controller.start(request)

    def test_rebase_invalid_protocol_fails_without_ticket(self):
        state=self.check(time_mode='rebase')
        self.assertEqual(state['job']['state'],'failed')
        self.assertIsNone(state['preflight'])

    def test_incomplete_requires_explicit_choice(self):
        capture(self.root/'incomplete.jsonl',complete=False)
        for allowed in (False,True):
            self.controller.start({'action':'check','file':'incomplete.jsonl','allow_incomplete':allowed})
            state=wait_job(self.controller)
            self.assertEqual(bool(state['preflight']),allowed)

    def test_busy_and_stop_only_own_process(self):
        ready=threading.Event()
        def harmless(args,**kwargs):
            child=subprocess.Popen([sys.executable,'-u','-c',
                'import signal,time; signal.signal(signal.SIGINT,lambda *_:exit(130)); print("ready",flush=True); time.sleep(30)'],**kwargs)
            ready.set();return child
        self.controller.popen=harmless
        job=self.controller.start({'action':'check','file':'test.jsonl'})
        ready.wait(1)
        with self.assertRaises(web.RequestError): self.controller.start({'action':'check','file':'test.jsonl'})
        with self.assertRaises(web.RequestError): self.controller.stop('another-job')
        self.controller.stop(job['job_id'])
        state=wait_job(self.controller)
        self.assertEqual(state['job']['state'],'stopped')
        self.assertIsNotNone(self.controller.process.poll())

    def test_paths_symlinks_numeric_and_control_topics_rejected(self):
        for name in ('../test.jsonl','/tmp/evil.jsonl','x;echo.jsonl','.hidden.jsonl','bad\\name.jsonl','a\n.jsonl'):
            with self.subTest(name=name),self.assertRaises(web.RequestError): self.controller.path(name,False)
        (self.root/'link.jsonl').symlink_to(self.root/'test.jsonl')
        with self.assertRaises(web.RequestError): self.controller.path('link.jsonl')
        for options in ({'speed':float('nan')},{'speed':True},{'allow_incomplete':'yes'},{'topics':['FusionMultiTarget']},{'topics':[]}):
            with self.subTest(options=options),self.assertRaises(web.RequestError): web.playback_options(options)
        self.assertEqual(web.filename('雷达 01.jsonl'),'雷达 01.jsonl')

    def test_record_no_overwrite(self):
        self.controller.native_available=True
        with self.assertRaises(web.RequestError),patch.object(self.controller,'popen') as child:
            self.controller.start({'action':'record','file':'test.jsonl'})
            child.assert_not_called()

    def test_shutdown_stops_owned_child_and_refuses_new_work(self):
        self.controller.close()
        with self.assertRaises(web.RequestError): self.controller.start({'action':'check','file':'test.jsonl'})

    def test_browse_create_and_restrict_directories(self):
        root=self.controller.directory
        (root/'existing').mkdir();(root/'.hidden').mkdir()
        result=self.controller.browse()
        self.assertEqual([f['name'] for f in result['folders']],['existing'])
        self.assertIsNone(result['parent'])
        created=self.controller.mkdir(str(root),'雷达 01')
        self.assertTrue(Path(created['directory']).is_dir())
        self.assertEqual(self.controller.browse(created['directory'])['parent'],str(root))
        for bad in (str(root.parent),str(root/'existing/../existing'),str(root/'.hidden'),'relative/path'):
            with self.subTest(bad=bad),self.assertRaises(web.RequestError): self.controller.resolve_directory(bad)
        (root/'alias').symlink_to(root/'existing',target_is_directory=True)
        with self.assertRaises(web.RequestError): self.controller.browse(str(root/'alias'))
        for bad in ('../escape','.hidden','a/b','a\n'):
            with self.assertRaises(web.RequestError): self.controller.mkdir(str(root),bad)
        with self.assertRaises(FileExistsError): self.controller.mkdir(str(root),'existing')

    def test_extra_root_can_be_browsed_without_opening_entire_filesystem(self):
        with tempfile.TemporaryDirectory() as extra:
            controller=web.Controller(self.controller.directory,storage_roots=[extra])
            self.addCleanup(controller.close)
            self.assertEqual(controller.browse(str(Path(extra).resolve()))['directory'],str(Path(extra).resolve()))
        with self.assertRaises(ValueError): web.Controller(self.controller.directory,storage_roots=['/'])

    def test_preflight_binds_directory_even_when_filename_and_bytes_match(self):
        state=self.check();self.controller.native_available=True
        other=self.controller.directory/'other';other.mkdir()
        (other/'test.jsonl').write_bytes((self.root/'test.jsonl').read_bytes())
        with patch.object(self.controller,'popen') as child,self.assertRaises(web.RequestError):
            self.controller.start({'action':'replay','file':'test.jsonl','directory':str(other),
                'preflight_id':state['preflight']['id'],'confirm_send':True,'inputs_stopped':True})
            child.assert_not_called()
        self.controller.start({'action':'check','file':'test.jsonl','directory':str(other)})
        state=wait_job(self.controller)
        self.assertEqual(state['preflight']['path'],str(other/'test.jsonl'))

    def test_record_selected_directory_and_remember_last_recording(self):
        other=self.controller.directory/'save here';other.mkdir()
        self.controller.native_available=True
        def harmless(args,**kwargs):
            self.assertEqual(args[3:5],['record',str(other/'record.jsonl')])
            capture(Path(args[4]))
            return subprocess.Popen([sys.executable,'-c','print(\'{"event":"record_finished","records":1,"reason":"duration"}\')'],**kwargs)
        self.controller.popen=harmless
        self.controller.start({'action':'record','file':'record.jsonl','directory':str(other),'duration':1})
        state=wait_job(self.controller)
        self.assertEqual(state['job']['directory'],str(other))
        self.assertEqual(state['last_recording']['path'],str(other/'record.jsonl'))
        self.assertTrue(state['last_recording']['download_available'])
        self.controller.popen=subprocess.Popen
        self.check()
        self.assertEqual(self.controller.state()['last_recording']['file'],'record.jsonl')

    def test_record_write_permission_and_disk_space_checked_before_spawn(self):
        self.controller.native_available=True
        with patch.object(web.tempfile,'TemporaryFile',side_effect=PermissionError('read only')),patch.object(self.controller,'popen') as child:
            with self.assertRaises(PermissionError): self.controller.start({'action':'record','file':'new.jsonl'})
            child.assert_not_called()
        with patch.object(web.shutil,'disk_usage') as disk,patch.object(self.controller,'popen') as child:
            disk.return_value.free=0
            with self.assertRaises(web.RequestError): self.controller.start({'action':'record','file':'new.jsonl'})
            child.assert_not_called()

    def test_missing_listing_directory_does_not_hide_task_state(self):
        self.check()
        state=self.controller.state(str(self.controller.directory/'missing'))
        self.assertIsNotNone(state['job'])
        self.assertIsNotNone(state['file_error'])
        self.assertEqual(state['files'],[])


class HTTPTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.controller=web.Controller(self.tmp.name)
        self.server=web.Server(('127.0.0.1',0),self.controller,'test-access-key-at-least-24-characters')
        self.thread=threading.Thread(target=self.server.serve_forever,kwargs={'poll_interval':.01},daemon=True);self.thread.start()
        self.port=self.server.server_port;self.cookie=''
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.server.shutdown();self.server.server_close();self.controller.close();self.thread.join(2)

    def request(self,path,body=None,origin=None,raw=None):
        conn=http.client.HTTPConnection('127.0.0.1',self.port,timeout=5)
        headers={'Cookie':self.cookie}
        method='POST' if body is not None or raw is not None else 'GET'
        if method=='POST':
            headers.update({'Origin':origin or f'http://127.0.0.1:{self.port}','Content-Type':'application/json' if raw is None else 'application/octet-stream'})
        conn.request(method,path,body=json.dumps(body) if body is not None else raw,headers=headers)
        response=conn.getresponse();content=response.read();status=response.status
        cookie=response.getheader('Set-Cookie')
        if cookie:self.cookie=cookie.split(';')[0]
        conn.close()
        try:content=json.loads(content)
        except ValueError:pass
        return status,content

    def login(self):
        self.assertEqual(self.request('/api/login',{'key':self.server.access_key})[0],200)

    def test_static_and_auth(self):
        self.assertEqual(self.request('/')[0],200)
        self.assertEqual(self.request('/api/state')[0],401)
        self.assertEqual(self.request('/api/login',{'key':'wrong'})[0],401)
        self.login();self.assertEqual(self.request('/api/state')[0],200)
        self.assertEqual(self.request('/../DDS_DATA_TOOL.md')[0],404)
        self.assertEqual(self.request('/api/logout',{})[0],200)
        self.assertEqual(self.request('/api/state')[0],401)

    def test_cross_origin_rejected_even_with_cookie(self):
        self.login()
        self.assertEqual(self.request('/api/jobs',{'action':'record'},origin='http://evil.example')[0],403)

    def test_upload_download_no_overwrite_and_bad_files(self):
        self.login()
        source=Path(self.tmp.name)/'source.jsonl';capture(source)
        original=source.read_bytes()
        self.assertEqual(self.request('/api/upload?name=uploaded.jsonl',raw=original)[0],201)
        self.assertEqual(self.request('/api/download?name=uploaded.jsonl'),(200,original))
        self.assertEqual(self.request('/api/upload?name=uploaded.jsonl',raw=original)[0],409)
        self.assertEqual(self.request('/api/upload?name=bad.jsonl',raw=b'wrong format')[0],400)
        self.assertFalse((Path(self.tmp.name)/'bad.jsonl').exists())
        self.assertEqual(self.request('/api/upload?name=../escape.jsonl',raw=original)[0],400)
        self.assertFalse(list(Path(self.tmp.name).glob('.upload-*')))

    def test_http_precheck_and_state(self):
        self.login();capture(Path(self.tmp.name)/'test.jsonl')
        status,result=self.request('/api/jobs',{'action':'check','file':'test.jsonl'})
        self.assertEqual(status,202)
        wait_job(self.controller)
        status,result=self.request('/api/state')
        self.assertEqual(result['preflight']['summary']['selected_records'],1)
        self.assertEqual(result['files'][0]['name'],'test.jsonl')

    def test_directory_routes_are_authenticated_and_same_origin(self):
        self.assertEqual(self.request('/api/browse')[0],401)
        self.login()
        root=str(self.controller.directory)
        self.assertEqual(self.request('/api/mkdir',{'directory':root,'name':'abc'},origin='http://other.invalid')[0],403)
        status,result=self.request('/api/mkdir',{'directory':root,'name':'abc'})
        self.assertEqual(status,201)
        status,listing=self.request('/api/browse?'+urlencode({'directory':root}))
        self.assertEqual(status,200);self.assertEqual(listing['folders'][0]['name'],'abc')
        self.assertEqual(self.request('/api/browse?'+urlencode({'directory':'/etc'}))[0],403)

    def test_upload_and_download_selected_directory_not_default(self):
        self.login();root=self.controller.directory
        (root/'target').mkdir();capture(root/'source.jsonl');original=(root/'source.jsonl').read_bytes()
        query=urlencode({'name':'uploaded.jsonl','directory':str(root/'target')})
        status,result=self.request('/api/upload?'+query,raw=original)
        self.assertEqual(status,201);self.assertEqual(result['directory'],str(root/'target'))
        self.assertFalse((root/'uploaded.jsonl').exists())
        self.assertEqual(self.request('/api/download?'+query),(200,original))
        self.assertTrue((root/'target/uploaded.jsonl').exists())
        self.assertEqual(self.request('/api/upload?'+query,raw=original)[0],409)
        state=self.request('/api/state?'+urlencode({'directory':str(root/'target')}))[1]
        self.assertEqual(state['files'][0]['name'],'uploaded.jsonl')

    def test_download_busy_checks_full_path_not_basename(self):
        self.login();root=self.controller.directory
        (root/'other').mkdir();capture(root/'test.jsonl');capture(root/'other/test.jsonl')
        self.controller.job={'state':'running','path':str(root/'test.jsonl'),'file':'test.jsonl'}
        try:
            self.assertEqual(self.request('/api/download?name=test.jsonl')[0],409)
            self.assertEqual(self.request('/api/download?'+urlencode({'name':'test.jsonl','directory':str(root/'other')}))[0],200)
        finally:self.controller.job=None


if __name__=='__main__':
    unittest.main()
