#!/usr/bin/env python3
"""Independent simulator UI. No fusion service control, no DDS in HTTP process."""
from __future__ import annotations
import argparse
import bisect
from collections import deque
import fcntl
import hashlib
from http.cookies import SimpleCookie
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit

import dds_data_web as common
import field_simulator as simulator

ROOT=Path(__file__).resolve().parents[1]
STATIC=Path(__file__).resolve().parent/'field_sim_ui'
BUNDLE=('capture.jsonl','events.jsonl','truth.jsonl','scenario.yaml','eo_devices.yaml')
DOWNLOADS=(*BUNDLE,'metadata.json','evaluation.json')
Error=common.RequestError


class Controller(common.Controller):
    def __init__(self,directory,popen=subprocess.Popen):
        def spawn(args,**kwargs):
            if 'env' in kwargs:kwargs['env'].pop('FIELD_SIM_WEB_TOKEN',None)
            return popen(args,**kwargs)
        super().__init__(directory,popen=spawn)
        self.sent_sequences={}

    def run_path(self,run_id):
        if not isinstance(run_id,str) or not re.fullmatch(r'run-[0-9]{8}-[0-9]{6}-[a-f0-9]{8}',run_id):
            raise Error('无效测试编号')
        path=self.directory/run_id
        if path.is_symlink() or not path.is_dir():raise Error('测试目录不存在',404)
        return path

    def file_path(self,run_id,name):
        if name not in DOWNLOADS:raise Error('不允许访问此文件',403)
        path=self.run_path(run_id)/name
        if path.is_symlink() or not path.is_file():raise Error('文件尚未生成',404)
        if path.stat().st_size>128*1024*1024:raise Error('文件超过 128 MiB',413)
        return path

    def metadata(self,run_id,verify=False):
        path=self.file_path(run_id,'metadata.json')
        if path.stat().st_size>65536:raise Error('测试元数据过大')
        meta=json.loads(path.read_text())
        if meta.get('format')!='sb-field-sim-v1':raise Error('不是本工具生成的数据')
        if verify:
            for name in BUNDLE:
                actual=hashlib.sha256(self.file_path(run_id,name).read_bytes()).hexdigest()
                if actual!=meta.get('files_sha256',{}).get(name):
                    raise Error('文件已变化或不完整，请重新生成：'+name,409)
        return meta

    def runs(self):
        result=[]
        # Only fixed, generated child folders are exposed; no arbitrary file browser.
        names=sorted((p.name for p in self.directory.iterdir() if p.name.startswith('run-')),reverse=True)
        for name in names[:100]:
            try:
                meta=self.metadata(name)
                result.append({'id':name,'case':meta['case'],'packets':meta['packets'],
                               'settings':meta['settings'],'device_counts':meta.get('device_counts',[])})
            except (Error,ValueError,OSError,KeyError):continue
        return result

    def state(self,directory=None):
        with self.lock:
            state=super().state()
            state.update(runs=self.runs(),defaults=simulator.load_settings(),modes=list(simulator.EO_MODES),api_version=2)
            job=state['job']
            if job and job['action']=='replay':
                count=job['progress'].get('published_calls',0)
                job['sent_by_device']={did:bisect.bisect_right(seqs,count) for did,seqs in self.sent_sequences.items()}
            return state

    def details(self,run_id):
        meta=self.metadata(run_id)
        path=self.run_path(run_id)
        command=('cd '+shlex.quote(str(ROOT))+'\nSB_FUSION_MIN_CONFIDENCE=0.2 SB_EO_CONFIG='+shlex.quote(str(path/'eo_devices.yaml'))+
                 ' ./start_equipment_fusion.sh --scenario '+shlex.quote(str(path/'scenario.yaml')))
        result={'id':run_id,'metadata':meta,'launch_command':command,'evaluation':None}
        report=path/'evaluation.json'
        if report.is_file() and not report.is_symlink() and report.stat().st_size<1024*1024:
            result['evaluation']=json.loads(report.read_text())
        return result

    def _spawn(self,action,run_id,args):
        child=self.popen(args,cwd=str(ROOT),env=self._child_env(),stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',start_new_session=True)
        path=self.directory/run_id/'capture.jsonl'
        job={'id':secrets.token_hex(12),'run_id':run_id,'action':action,'file':'capture.jsonl',
             'directory':str(path.parent),'path':str(path),'state':'running','phase':'starting',
             'started_at':time.time(),'_started':time.monotonic(),'events':deque(maxlen=160),
             'progress':{},'summary':None,'error':None,'forced_stop':False,'options':None}
        self.job,self.process,self.ticket=job,child,None
        threading.Thread(target=self._watch,args=(job,child),daemon=True).start()
        return {'job_id':job['id'],'run_id':run_id}

    @staticmethod
    def _child_env():
        env=dict(os.environ)
        for name in ('FIELD_SIM_WEB_TOKEN','DDS_WEB_TOKEN','SB_DDS_DATA_TOOL_ENV'):env.pop(name,None)
        return env

    def start(self,body):
        with self.lock:
            if self.closed:raise Error('服务正在退出',503)
            if self.job and self.job['state'] in common.ACTIVE:raise Error('请先停止或等待当前任务完成',409)
            action=body.get('action')
            if action=='generate':
                settings=body.get('settings')
                if not isinstance(settings,dict):raise Error('缺少设备参数')
                cfg=simulator.load_settings(**settings)
                if shutil.disk_usage(self.directory).free<128*1024*1024:raise Error('剩余空间不足 128 MiB')
                run_id=time.strftime('run-%Y%m%d-%H%M%S-')+secrets.token_hex(4)
                request=self.directory/(run_id+'.json')
                simulator.write_json(request,cfg)
                return self._spawn(action,run_id,[sys.executable,'-u',str(ROOT/'tools/field_simulator.py'),
                    'generate','--case','custom','--config',str(request),'--output',str(self.directory/run_id)])
            if action not in ('evaluate','check','replay'):raise Error('未知操作')
            run_id=body.get('run_id');self.metadata(run_id,verify=True)
            path=self.run_path(run_id)
            bundle=hashlib.sha256((path/'metadata.json').read_bytes()).hexdigest()
            if action=='evaluate':
                if (path/'evaluation.json').exists() or (path/'evaluation.frames.jsonl').exists():
                    raise Error('已有离线结果；请查看结果或生成新的测试',409)
                return self._spawn(action,run_id,[sys.executable,'-u',str(ROOT/'tools/evaluate_field_sim.py'),str(path)])
            if action=='replay':
                if body.get('isolated') is not True or body.get('fusion_ready') is not True:
                    raise Error('请确认隔离现场业务，并已用本测试配置手动启动融合')
                if not self.ticket or self.ticket.get('bundle_sha256')!=bundle:
                    raise Error('请先完成本测试的 DDS 预检查',409)
                self.sent_sequences={}
                with self.file_path(run_id,'events.jsonl').open() as stream:
                    for line in stream:
                        row=json.loads(line)
                        self.sent_sequences.setdefault(str(row['device_id']),[]).append(row['seq'])
            request={'action':action,'file':'capture.jsonl','directory':str(path),'time_mode':'rebase','speed':1,
                     'preflight_id':body.get('preflight_id'),'confirm_send':body.get('confirm_send'),
                     'inputs_stopped':body.get('inputs_stopped')}
            result=super().start(request)
            self.job.update(run_id=run_id,_bundle=bundle)
            return dict(result,run_id=run_id)

    def _watch(self,job,child):
        super()._watch(job,child)
        with self.lock:
            if self.ticket and self.ticket['id']==job['id']:
                self.ticket['bundle_sha256']=job.get('_bundle')
                self.ticket['run_id']=job.get('run_id')


class Handler(common.Handler):
    server_version='Field-Simulation-Workbench/1'

    def auth(self):
        cookie=SimpleCookie()
        try:
            cookie.load(self.headers.get('Cookie',''))
            value=cookie['field_sim_session'].value if 'field_sim_session' in cookie else ''
        except Exception:value=''
        if not secrets.compare_digest(value,self.server.session_key):raise Error('请使用启动终端的口令登录',401)

    def route(self,method):
        parsed=urlsplit(self.path);path=parsed.path;ctl=self.server.controller
        assets={'/':('index.html','text/html; charset=utf-8'),'/app.js':('app.js','text/javascript; charset=utf-8'),
                '/style.css':('style.css','text/css; charset=utf-8')}
        if method=='GET' and path in assets:
            name,mime=assets[path];return self.send(200,(STATIC/name).read_bytes(),mime)
        if method=='POST':self.origin()
        if method=='POST' and path=='/api/login':
            body=self.read_json();ip=self.client_address[0];now=time.monotonic()
            if sum(peer==ip and now-when<60 for peer,when in self.server.login_failures)>=5:raise Error('口令错误过多，请 60 秒后重试',429)
            key=body.get('key','')
            if not isinstance(key,str) or not secrets.compare_digest(key.encode(),self.server.access_key.encode()):
                self.server.login_failures.append((ip,now));raise Error('口令不正确，请查看启动终端',401)
            return self.json({'ok':True},extra=[('Set-Cookie','field_sim_session='+self.server.session_key+'; HttpOnly; SameSite=Strict; Path=/; Max-Age=28800')])
        self.auth()
        if method=='GET' and path=='/api/state':return self.json(ctl.state())
        if method=='GET' and path=='/api/run':return self.json(ctl.details(parse_qs(parsed.query).get('id',[''])[0]))
        if method=='POST' and path=='/api/jobs':return self.json(ctl.start(self.read_json()),202)
        if method=='POST' and path=='/api/stop':return self.json(ctl.stop(self.read_json().get('job_id')))
        if method=='POST' and path=='/api/logout':
            self.read_json();return self.json({'ok':True},extra=[('Set-Cookie','field_sim_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0')])
        if method=='GET' and path=='/api/file':
            params=parse_qs(parsed.query);name=params.get('name',[''])[0];run_id=params.get('id',[''])[0]
            with ctl.lock:
                if ctl.job and ctl.job['state'] in common.ACTIVE and ctl.job.get('run_id')==run_id:raise Error('当前测试任务完成后可下载',409)
                file=ctl.file_path(run_id,name)
                fd=os.open(file,os.O_RDONLY|os.O_NOFOLLOW)
            with os.fdopen(fd,'rb') as stream:
                size=os.fstat(stream.fileno()).st_size
                self.send_response(200);self.send_header('Content-Type','application/octet-stream')
                self.send_header('Content-Length',str(size));self.send_header('Cache-Control','no-store')
                self.send_header('X-Content-Type-Options','nosniff')
                self.send_header('Content-Disposition','attachment; filename="'+run_id+'-'+name+'"')
                self.end_headers();shutil.copyfileobj(stream,self.wfile,256*1024)
            return
        raise Error('接口不存在',404)


class Server(common.Server):
    def __init__(self,address,controller,key):
        super().__init__(address,controller,key)
        self.RequestHandlerClass=Handler


def main(argv=None):
    p=argparse.ArgumentParser(description='独立实装仿真控制页，不启停融合，不自动发送 DDS')
    p.add_argument('--host',default='0.0.0.0');p.add_argument('--port',type=int,default=8011)
    p.add_argument('--data-dir',type=Path,default=ROOT/'artifacts/field_sim_runs')
    args=p.parse_args(argv)
    if not 1<=args.port<=65535:p.error('端口范围 1–65535')
    key=os.environ.get('FIELD_SIM_WEB_TOKEN') or secrets.token_urlsafe(24)
    if len(key)<24:p.error('FIELD_SIM_WEB_TOKEN 至少 24 个字符')
    ctl=Controller(args.data_dir)
    with os.fdopen(os.open(ctl.directory/'.field-sim-web.lock',os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600),'a+') as lock:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print('已有控制页使用此数据目录，请访问已启动的界面。',file=sys.stderr);return 1
        try:server=Server((args.host,args.port),ctl,key)
        except OSError as exc:
            print(f'控制页启动失败：{exc}。可用 --port 换端口，不会停止其他服务。',file=sys.stderr);return 1
        def stop(*_):raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM,stop)
        print(f'\n独立实装仿真控制页\n地址：http://<设备IP>:{args.port}/\n访问口令：{key}\n'
              f'数据目录：{ctl.directory}\n默认不发 DDS，不启动融合；只在可信调试局域网使用，HTTP 不加密。\n'
              '网页关闭不停止发送；请点停止。Ctrl+C 退出服务会停止本工具的任务。\n',flush=True)
        try:server.serve_forever(poll_interval=.2)
        except KeyboardInterrupt:pass
        finally:ctl.close();server.server_close()
    return 0


if __name__=='__main__':raise SystemExit(main())
