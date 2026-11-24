#!/usr/bin/env python3
"""Independent field-like device simulator; generation is offline by default.

Only stdlib + the standalone capture codec. NEVER imports fusion algorithms.
This is an L2 device/report behaviour model, not an electromagnetic/optical model.
"""
from __future__ import annotations
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import random
import struct
import subprocess
import sys

from dds_data_tool import CaptureWriter, PXSB, PROTOCOL, timestamp_fields

ROOT = Path(__file__).resolve().parents[1]
CASES = {
    'custom': '自定义组合：可选雷达、一台或两台光电及各自观测模式',
    'baseline': '雷达约 3 秒一次，单光电有效坐标，量测含噪声',
    'range-failure': '中段光电测距/坐标失效，仍有角度，雷达保持报告',
    'radar-outage': '中段雷达静默且单光电仅测向，验证定位支撑超时',
    'low-speed': '悬停阶段雷达低探测率，先外推后停止，光电测距正常',
    'dual-bearing': '无雷达、两台独立光电仅测向，异步交汇',
    'single-bearing': '无雷达，单光电仅测向，不应凭空输出定位航迹',
    'weak-geometry': '两台光电视线近平行，交汇应拒绝',
    'colocated-bearing': '两台光电基线仅 1 米，交汇应拒绝',
    'recovery': '单光电坐标→短时纯角度→坐标，检查 ID 接续',
    'stale-values': '中段距离/目标 GPS 冻结，角度继续变化',
    'ranged-only': '无目标 GPS，距离+角度有效；中段距离失效',
    'gps-fallback': '报文站址有效→零值→恢复；config 故意偏置作负对照',
    'timing-faults': '光电时钟固定偏移，插入重复包及延迟到达包',
    'optical-loss': '正常跟踪→记忆跟踪→丢失→恢复',
    'radar-extrapolation': '仅雷达外推报告，没有真实点证据',
    'rf-distractors': '一个主目标及两个额外射频目标，射频仅测向辅助',
}
DEFAULTS = dict(duration_s=72., seed=20261021, epoch_ms=1792540800000,
    origin=[120.,30.,0.], radar_period_s=3., radar_jitter_s=.02,
    radar_pd=.97, low_speed_threshold_mps=.8, low_speed_pd=.05,
    radar_coast_s=6., radar_range_sigma_m=10., radar_angle_sigma_deg=.3,
    eo_rates_hz=[10.,8.], eo_range_sigma_m=5., eo_angle_sigma_deg=.3,
    latency_ms=[5.,40.], clock_offset_ms=1500, azimuth_bias_deg=0.,
    radar_device_id=73152, eo_device_ids=[71112,71113], rf_device_id=74001,
    custom_radar_enabled=True, custom_low_speed_loss=True, custom_eo_count=1,
    radar_station_enu=[0.,0.,3.], eo_stations_enu=[[1.,1.,3.],[300.,0.,3.]],
    eo_modes=['position','position'], custom_target_count=1, eo_target_ids=[1,1],
    evaluation_gate_m=50.)
EO_MODES={'position':'baseline','bearing':'single-bearing','range-failure':'range-failure',
          'ranged':'ranged-only','stale-values':'stale-values','optical-loss':'optical-loss'}


def load_settings(path=None, **overrides):
    data = json.loads(Path(path).read_text()) if path else {}
    unknown=(set(data)|set(overrides))-set(DEFAULTS)
    if unknown:
        raise ValueError('Unknown settings: '+str(sorted(unknown)))
    cfg = {**DEFAULTS, **data, **{k:v for k,v in overrides.items() if v is not None}}
    for name in ('duration_s','radar_period_s','radar_jitter_s','radar_pd',
                 'low_speed_threshold_mps','low_speed_pd','radar_coast_s',
                 'radar_range_sigma_m','radar_angle_sigma_deg','eo_range_sigma_m',
                 'eo_angle_sigma_deg','clock_offset_ms','azimuth_bias_deg'):
        if type(cfg[name]) not in (int,float) or not math.isfinite(cfg[name]):
            raise ValueError(f'Invalid {name}')
    if not 15 <= cfg['duration_s'] <= 600 or not .5 <= cfg['radar_period_s'] <= 30:
        raise ValueError('duration_s: 15–600; radar_period_s: .5–30')
    for key in ('radar_pd','low_speed_pd'):
        if not 0 <= cfg[key] <= 1: raise ValueError(key+' must be 0–1')
    for key in ('radar_range_sigma_m','radar_angle_sigma_deg','eo_range_sigma_m','eo_angle_sigma_deg'):
        if not 0 < cfg[key] <= 100: raise ValueError('Invalid noise '+key)
    if not 0 <= cfg['radar_jitter_s'] < cfg['radar_period_s']/4:
        raise ValueError('Radar jitter must be less than one quarter period')
    if not 0 <= cfg['radar_coast_s'] <= 30 or not 0 <= cfg['low_speed_threshold_mps'] <= 30:
        raise ValueError('Invalid radar dropout settings')
    if abs(cfg['clock_offset_ms']) > 10000 or abs(cfg['azimuth_bias_deg']) > 180:
        raise ValueError('Clock/angle bias too large for this test model')
    for key, length in [('origin',3),('latency_ms',2),('eo_rates_hz',2),('eo_device_ids',2)]:
        if not isinstance(cfg[key],list) or len(cfg[key]) != length or any(
                type(x) not in (int,float) or not math.isfinite(x) for x in cfg[key]):
            raise ValueError('Invalid '+key)
    if not (-179 < cfg['origin'][0] < 179 and -80 < cfg['origin'][1] < 80 and 0 <= cfg['origin'][2] < 10000):
        raise ValueError('Origin outside supported small-area/unsigned-altitude test region')
    if not 0 <= cfg['latency_ms'][0] <= cfg['latency_ms'][1] <= 1000:
        raise ValueError('latency_ms must be increasing within 0–1000')
    if any(not 1 <= x <= 30 for x in cfg['eo_rates_hz']): raise ValueError('EO rates: 1–30 Hz')
    ids = [cfg['radar_device_id'],*cfg['eo_device_ids'],cfg['rf_device_id']]
    if len(set(ids)) != 4 or any(type(x) is not int or not 1 <= x < 2**32 for x in ids):
        raise ValueError('Four unique integer device IDs required')
    if type(cfg['seed']) is not int or type(cfg['epoch_ms']) is not int or not 10**12 <= cfg['epoch_ms'] < 2*10**12:
        raise ValueError('Invalid seed/epoch_ms')
    if any(type(cfg[k]) is not bool for k in ('custom_radar_enabled','custom_low_speed_loss')):
        raise ValueError('Radar enable/low-speed switch must be boolean')
    if type(cfg['custom_eo_count']) is not int or cfg['custom_eo_count'] not in (1,2):
        raise ValueError('custom_eo_count must be 1 or 2')
    if type(cfg['custom_target_count']) is not int or not 1 <= cfg['custom_target_count'] <= 10:
        raise ValueError('custom_target_count must be an integer from 1 to 10')
    if (not isinstance(cfg['eo_target_ids'],list) or len(cfg['eo_target_ids'])!=2 or
        any(type(x) is not int or not 1 <= x <= cfg['custom_target_count'] for x in cfg['eo_target_ids'])):
        raise ValueError('Each EO target must exist in the selected target count')
    if (type(cfg['evaluation_gate_m']) not in (int,float) or not math.isfinite(cfg['evaluation_gate_m'])
        or not 1 <= cfg['evaluation_gate_m'] <= 1000):
        raise ValueError('evaluation_gate_m must be within 1–1000 m')
    if not isinstance(cfg['eo_modes'],list) or len(cfg['eo_modes'])!=2 or any(not isinstance(x,str) or x not in EO_MODES for x in cfg['eo_modes']):
        raise ValueError('Invalid EO modes')
    if not isinstance(cfg['eo_stations_enu'],list) or len(cfg['eo_stations_enu'])!=2:
        raise ValueError('Two EO station configurations required')
    for site in [cfg['radar_station_enu'],*cfg['eo_stations_enu']]:
        if (not isinstance(site,list) or len(site)!=3 or any(type(x) not in (int,float) or not math.isfinite(x) for x in site)
            or any(abs(x)>100000 for x in site) or cfg['origin'][2]+site[2]<0):
            raise ValueError('Invalid station ENU / unsigned protocol altitude')
    return cfg


def truth_state(t, case, duration, target=1):
    """Known test truth. The evaluator reads a separate sampled truth file."""
    if case=='custom' and target!=1:
        # Separated, reproducible trajectories with independent headings/speeds
        # and staggered hover intervals. Not a crossing/formation stress test.
        k=target-1; a=duration*(.18+.025*k); b=a+duration*.18
        first=min(max(t,0),a); second=max(0,t-b)
        angle=math.radians(30*k); u=[2.5*math.cos(angle),2.5*math.sin(angle),.08]
        v=[2*math.cos(angle+.6),2*math.sin(angle+.6),-.03]
        start=[500.+(k%3)*400,400.+(k//3)*400,100.+15*k]
        return ([x+du*first+dv*second for x,du,dv in zip(start,u,v)],
                u if t<a else [0.,0.,0.] if t<b else v)
    if target != 1:
        return ([250+target*230+1.5*t, -350-target*120+t, 100+target*20], [1.5,1,0])
    start = [15000.,100.,100.] if case == 'weak-geometry' else [500.,400.,100.]
    a,b = duration*.28,duration*.52
    leg1,leg2 = min(max(t,0),a),max(0,t-b)
    pos = [start[0]+4*leg1+3*leg2, start[1]+leg1-.8*leg2, start[2]+.1*leg1]
    vel = [4.,1.,.1] if t<a else [0.,0.,0.] if t<b else [3.,-.8,0.]
    return pos,vel


def distance(a,b): return math.sqrt(sum((x-y)**2 for x,y in zip(a,b)))
def lla(cfg, xyz):
    lon,lat,alt = cfg['origin']
    return [lon+xyz[0]/(111320*math.cos(math.radians(lat))),lat+xyz[1]/111320,alt+xyz[2]]
def wire_lla(cfg, xyz):
    lon,lat,alt = lla(cfg,xyz)
    if alt < 0: raise ValueError('Protocol unsigned altitude cannot encode this position')
    return [round(lon*1e7),round(lat*1e7),round(alt*10)]
def polar(pos,station):
    x,y,z = [a-b for a,b in zip(pos,station)]
    return math.sqrt(x*x+y*y+z*z), math.degrees(math.atan2(x,y))%360, math.degrees(math.atan2(z,math.hypot(x,y)))
def cartesian(station,r,az,el):
    a,e = math.radians(az),math.radians(el)
    return [station[0]+r*math.cos(e)*math.sin(a),station[1]+r*math.cos(e)*math.cos(a),station[2]+r*math.sin(e)]
def measured(rng,pos,station,rs,angs,bias=0):
    r,az,el = polar(pos,station)
    r=max(1,r+rng.gauss(0,rs));az=(az+rng.gauss(0,angs)+bias)%360
    el=max(-89,min(89,el+rng.gauss(0,angs)))
    return r,az,el,cartesian(station,r,az,el)


def packet(topic,device,stamp,body,seq):
    tag,magic,size = PROTOCOL[topic]
    frame = struct.pack('<IBHHHIQ',magic,1,tag,1,size,device,stamp)+body+struct.pack('<I',0)
    if len(frame)!=size: raise ValueError('Wrong device body size')
    unit = struct.pack('<IHQ',size,tag,stamp)+frame
    result = struct.pack(PXSB,0x7e,1,0,0,len(unit),0,seq%65536,0,0,stamp)+unit
    timestamp_fields(topic,result)  # codec/length/time guard, no DDS opened
    return result


class Simulation:
    def __init__(self,cfg,case):
        if case not in CASES: raise ValueError('Unknown case')
        self.cfg,self.case,self.rng = cfg,case,random.Random(cfg['seed'])
        self.events=[]; self.serial=0
        self.stations={'radar':[0.,0.,3.],'eo1':[1.,1.,3.],
                       'eo2':[2.,1.,3.] if case=='colocated-bearing' else [300.,0.,3.],
                       'rf':[-1.,1.,3.]}
        if case=='custom':
            self.stations.update(radar=cfg['radar_station_enu'][:],
                                 eo1=cfg['eo_stations_enu'][0][:],eo2=cfg['eo_stations_enu'][1][:])

    def emit(self,topic,device,source_time,body,*,truth_id=1,mode='',extra_delay=0):
        c=self.cfg; self.serial+=1
        bias=c['clock_offset_ms'] if self.case=='timing-faults' and topic=='EOTarget' else 0
        stamp=c['epoch_ms']+round(source_time*1000)+round(bias)
        arrival=source_time+self.rng.uniform(*c['latency_ms'])/1000+extra_delay
        self.events.append(dict(arrival_s=arrival,source_time_s=source_time,topic=topic,
            device_id=device,truth_id=truth_id,mode=mode,packet=packet(topic,device,stamp,body,self.serial)))

    def radar(self,target=1):
        c=self.cfg; case=self.case; d=c['duration_s']; station=self.stations['radar']
        if case=='custom' and not c['custom_radar_enabled']:return
        if case in ('dual-bearing','single-bearing','weak-geometry','colocated-bearing','recovery','ranged-only'): return
        t=polar(truth_state(0,case,d,target)[0],station)[1]/360*c['radar_period_s']
        last=None
        if case=='radar-extrapolation':
            # A noisy prior from before the test window, not new measurement evidence.
            pos=measured(self.rng,truth_state(0,case,d)[0],station,c['radar_range_sigma_m'],c['radar_angle_sigma_deg'])[3]
            last=dict(t=0.,pos=pos,vel=[0.,0.,0.])
        while t<d:
            pos,vel=truth_state(t,case,d,target)
            silence=case=='radar-outage' and .25*d<=t<.75*d
            low_speed=case=='low-speed' or (case=='custom' and c['custom_low_speed_loss'])
            pd=c['low_speed_pd'] if low_speed and math.sqrt(sum(v*v for v in vel))<c['low_speed_threshold_mps'] else c['radar_pd']
            real=not silence and case!='radar-extrapolation' and self.rng.random()<pd
            if real:
                r,az,el,measurement=measured(self.rng,pos,station,c['radar_range_sigma_m'],c['radar_angle_sigma_deg'])
                estimate=measurement[:]; estimate_v=[0.,0.,0.]
                if last:
                    dt=t-last['t']; prediction=[x+v*dt for x,v in zip(last['pos'],last['vel'])]
                    residual=[a-b for a,b in zip(measurement,prediction)]
                    estimate=[p+.7*e for p,e in zip(prediction,residual)]
                    estimate_v=[v+.2*e/dt for v,e in zip(last['vel'],residual)]
                last=dict(t=t,pos=estimate,vel=estimate_v)
            elif not silence and last and t-last['t']<=c['radar_coast_s']:
                estimate=[x+v*(t-last['t']) for x,v in zip(last['pos'],last['vel'])]
                r,az,el=polar(estimate,station)
            else:
                t+=c['radar_period_s']+self.rng.uniform(-c['radar_jitter_s'],c['radar_jitter_s']); continue
            signed_az=round(((az+180)%360-180)*100)
            body=struct.pack('<IIhhIhiiIIHBBB',88+target,round(r*10),signed_az,round(el*100),
                round(max(0,estimate[2]-station[2])*10),0,*wire_lla(c,estimate),
                round(math.hypot(*last['vel'][:2])*10),0,int(real),2,80)
            self.emit('RadarTarget',c['radar_device_id'],t,body,truth_id=target,mode='real' if real else 'extrapolated')
            t+=c['radar_period_s']+self.rng.uniform(-c['radar_jitter_s'],c['radar_jitter_s'])

    def eo(self,index):
        c=self.cfg; case=self.case; d=c['duration_s']; station=self.stations[f'eo{index+1}']
        behavior=EO_MODES[c['eo_modes'][index]] if case=='custom' else case
        target=c['eo_target_ids'][index] if case=='custom' else 1
        t=.07*index; saved=None; seq=0
        while t<d:
            seq+=1; pos,_=truth_state(t,case,d,target)
            r,az,el,xyz=measured(self.rng,pos,station,c['eo_range_sigma_m'],c['eo_angle_sigma_deg'],c['azimuth_bias_deg'])
            # Unsigned protocol altitude cannot express a noisy position below
            # the reference ellipsoid. Mark L2 unavailable, never clip it into
            # an apparently valid position (especially at long slant ranges).
            target_gps=wire_lla(c,xyz) if lla(c,xyz)[2]>=0 else [0,0,0]
            station_gps=wire_lla(c,station)
            mid=.25*d<=t<.75*d; status=1; mode='position'
            if behavior in ('dual-bearing','single-bearing','weak-geometry','colocated-bearing') or (
                behavior in ('range-failure','radar-outage') and mid) or (
                behavior=='recovery' and .3*d<=t<.4*d):
                target_gps=[0,0,0];r=0.;mode='bearing'
            if behavior=='ranged-only':
                target_gps=[0,0,0];mode='ranged'
                if .3*d<=t<.4*d: r=0.;mode='bearing'
            if behavior=='stale-values' and mid and saved:
                r,target_gps=saved[0],saved[1][:];mode='stale_position_range'
            else:
                saved=(r,target_gps[:])
            if case=='gps-fallback' and mid: station_gps=[0,0,0]
            if behavior=='optical-loss' and .3*d<=t<.6*d:
                status=2 if t<.4*d else 3; mode='memory' if status==2 else 'lost'
                r=0;target_gps=[0,0,0]
            azc=round(az*100)%36000; elc=round(el*100)
            body=struct.pack('<IBBBBHHhHiiIiiIIHhBB2s',7+index*20+(target-1)*100,0,status,int(status==1),1,
                0,0,elc,azc,*station_gps,*target_gps,round(r*10),azc,elc,1,80,b'\0\0')
            delay=.7 if case=='timing-faults' and seq%19==0 else 0
            self.emit('EOTarget',c['eo_device_ids'][index],t,body,truth_id=target,mode=mode,extra_delay=delay)
            if case=='timing-faults' and seq%23==0:
                previous=self.events[-1]
                self.events.append({**previous,'arrival_s':previous['arrival_s']+.015,'mode':'duplicate'})
            t+=1/c['eo_rates_hz'][index]+self.rng.uniform(-.003,.003)

    def rf(self):
        c=self.cfg; station=self.stations['rf'];t=.02
        while t<c['duration_s']:
            for target in (1,2,3):
                pos,_=truth_state(t,self.case,c['duration_s'],target)
                r,az,_,_=measured(self.rng,pos,station,20,2)
                # Existing ESM input consumes east-zero CCW; this assumption is
                # explicit in metadata, unlike EO/radar north-zero clockwise.
                az=(90-az)%360
                body=struct.pack('<I16s32siiIIIIiiIIhHQB7s',100+target,
                    f'SIM-UAV-{target}'.encode(),b'SYNTHETIC TEST',0,0,0,0,0,round(r*10),
                    0,0,2400000000,20000000,-600,round(az*100)%36000,
                    c['epoch_ms']+round(t*1000),70,b'\0'*7)
                self.emit('ESMTarget',c['rf_device_id'],t,body,truth_id=target,mode='bearing_identity')
            t+=1.

    def generate(self):
        for target in range(1,(self.cfg['custom_target_count'] if self.case=='custom' else 1)+1):
            self.radar(target)
        if self.case!='radar-extrapolation':
            self.eo(0)
            if self.case in ('dual-bearing','weak-geometry','colocated-bearing') or (self.case=='custom' and self.cfg['custom_eo_count']==2): self.eo(1)
        if self.case=='rf-distractors': self.rf()
        self.events.sort(key=lambda x:x['arrival_s'])
        return self.events


class SyntheticCaptureWriter(CaptureWriter):
    def _write(self,value,hashed=True):
        if value.get('type')=='header':
            value['timing']='synthetic scheduled arrival intervals; NOT recorded NIC/device timestamps'
        return super()._write(value,hashed)


def write_json(path,value):
    with Path(path).open('x',encoding='utf-8') as f:
        json.dump(value,f,ensure_ascii=False,indent=2,allow_nan=False); f.write('\n')


def generate_run(directory,cfg,case):
    directory=Path(directory); directory.mkdir(parents=True,exist_ok=False)
    sim=Simulation(cfg,case);events=sim.generate()
    topics=list(dict.fromkeys(e['topic'] for e in events))
    writer=SyntheticCaptureWriter(directory/'capture.jsonl',topics,128*1024*1024,
        source_info={'kind':'synthetic_field_simulator','case':case,'seed':cfg['seed'],
                     'not_field_recording':True,'version':1})
    reason='error'
    try:
        with (directory/'events.jsonl').open('x') as f:
            for seq,e in enumerate(events,1):
                offset=round(e['arrival_s']*1e9)
                if not writer.add(e['topic'],e['packet'],offset,cfg['epoch_ms']*1000000+offset):
                    raise ValueError('Simulation capture exceeds 128 MiB')
                f.write(json.dumps({k:v for k,v in e.items() if k!='packet'}|{'seq':seq})+'\n')
        reason='synthetic_complete'
    finally:
        writer.finish(reason)
    with (directory/'truth.jsonl').open('x') as f:
        for i in range(math.ceil((cfg['duration_s']+16)*10)+1):
            t=i/10
            for target in (range(1,cfg['custom_target_count']+1) if case=='custom' else
                           (1,2,3) if case=='rf-distractors' else (1,)):
                p,v=truth_state(t,case,cfg['duration_s'],target)
                f.write(json.dumps(dict(timestamp_ms=cfg['epoch_ms']+round(t*1000),
                    truth_id=target,position=p,velocity=v))+'\n')
    # JSON is a YAML subset. Keep generated test calibration separate from production.
    devices={}
    for i,did in enumerate(cfg['eo_device_ids']):
        position=sim.stations[f'eo{i+1}'][:]
        if case=='gps-fallback': position=[position[0]+30,position[1]+20,position[2]]
        devices[str(did)]=dict(station_enu=position,angles_calibrated=True,
            elevation_enabled=True,altitude_enabled=True,position_sigma_m=15,
            altitude_sigma_m=15,azimuth_sigma_deg=cfg['eo_angle_sigma_deg'],
            elevation_sigma_deg=cfg['eo_angle_sigma_deg'],azimuth_convention='north_cw',
            azimuth_offset_deg=-cfg['azimuth_bias_deg'],
            range_policy='reported_positive' if case=='ranged-only' or (case=='custom' and cfg['eo_modes'][i]=='ranged') else 'disabled',
            time_offset_ms=-round(cfg['clock_offset_ms']) if case=='timing-faults' else 0)
    write_json(directory/'eo_devices.yaml',{'reorder_window_ms':200,'devices':devices})
    # Inline all sensor entries to avoid stage-relative use_config paths.
    scenario=dict(coordinates={'frame':'ENU','origin':cfg['origin']},
                  equipment={'discover_only':True},targets=[],sensors={
        'radar':{'position':sim.stations['radar']},'eo':{'position':sim.stations['eo1']},
        'esm':{'position':sim.stations['rf']},'acoustic':{'position':[0,0,3]}})
    write_json(directory/'scenario.yaml',scenario)
    metadata=dict(format='sb-field-sim-v1',case=case,description=CASES[case],settings=cfg,
        packets=len(events),counts=dict(Counter(e['topic'] for e in events)),
        observation_model='per-scan L2 radar approximation; not a beam/EM model',
        radar_reporting='3s nominal by default; no 0.5s resending; coast from last noisy state only',
        rf_angle_convention='ENU east-zero CCW, matching current ESM input; not manufacturer-validated',
        truth_policy='truth.jsonl is separate; NEVER an input to production fusion',
        performance_claim='parameters are test assumptions, NOT identified device performance',
        dds_packets_sent=0, test_fusion_min_confidence=0.2,
        evaluation_truth_ids=list(range(1,cfg['custom_target_count']+1)) if case=='custom' else [1],
        eo_tracking_policy='each EO follows one selected target for the whole run; no simultaneous multi-target EO',
        capture_sha256=hashlib.sha256((directory/'capture.jsonl').read_bytes()).hexdigest())
    metadata['device_counts']=[dict(device_id=did,topic=topic,packets=count)
        for (did,topic),count in Counter((e['device_id'],e['topic']) for e in events).items()]
    metadata['files_sha256']={name:hashlib.sha256((directory/name).read_bytes()).hexdigest()
        for name in ('capture.jsonl','events.jsonl','truth.jsonl','scenario.yaml','eo_devices.yaml')}
    write_json(directory/'metadata.json',metadata)
    return metadata


def send_run(directory,publish=False,isolated=False,inputs_stopped=False):
    directory=Path(directory).resolve()
    metadata=json.loads((directory/'metadata.json').read_text())
    capture=directory/'capture.jsonl'
    if metadata.get('format')!='sb-field-sim-v1' or hashlib.sha256(capture.read_bytes()).hexdigest()!=metadata['capture_sha256']:
        raise ValueError('Not an intact field-simulator run')
    if publish and not (isolated and inputs_stopped):
        raise ValueError('DDS requires --publish --confirm-isolated --inputs-stopped; a port/device ID change is NOT isolation')
    args=[sys.executable,str(ROOT/'tools/dds_data_tool.py'),'replay',str(capture),
          '--time-mode','rebase','--speed','1']
    if publish: args+=['--publish','--inputs-stopped']
    return subprocess.run(args,check=False).returncode


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    subs=parser.add_subparsers(dest='command',required=True)
    subs.add_parser('list',help='列出实装测试场景，不发 DDS')
    gen=subs.add_parser('generate',help='生成独立测试目录和原协议录制文件，不发 DDS')
    gen.add_argument('--case',choices=list(CASES)+['all'],default='baseline')
    gen.add_argument('--output',required=True,type=Path)
    gen.add_argument('--config',type=Path,default=ROOT/'configs/field_simulator.json')
    gen.add_argument('--duration',type=float);gen.add_argument('--seed',type=int)
    send=subs.add_parser('send',help='默认仅校验；显式确认后由独立回放器发送 DDS')
    send.add_argument('directory',type=Path);send.add_argument('--publish',action='store_true')
    send.add_argument('--confirm-isolated',action='store_true');send.add_argument('--inputs-stopped',action='store_true')
    args=parser.parse_args(argv)
    try:
        if args.command=='list': print(json.dumps(CASES,ensure_ascii=False,indent=2));return 0
        if args.command=='send': return send_run(args.directory,args.publish,args.confirm_isolated,args.inputs_stopped)
        cfg=load_settings(args.config,duration_s=args.duration,seed=args.seed)
        cases=list(CASES) if args.case=='all' else [args.case]
        if args.case=='all': args.output.mkdir(parents=True,exist_ok=False)
        for case in cases:
            dest=args.output/case if args.case=='all' else args.output
            result=generate_run(dest,cfg,case)
            print(json.dumps({'directory':str(dest.resolve()),'case':case,'packets':result['packets'],'dds_packets_sent':0},ensure_ascii=False),flush=True)
        return 0
    except (OSError,ValueError,KeyError,struct.error) as exc:
        print(f'ERROR: {exc}',file=sys.stderr);return 1


if __name__=='__main__':
    raise SystemExit(main())
