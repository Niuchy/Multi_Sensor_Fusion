#!/usr/bin/env python3
"""Evaluate generated device packets through memory-only fusion, never DDS.

Truth is opened AFTER the fusion pass. Error is conditional on output existence;
coverage/gaps are reported separately and are not hidden by an association gate.
"""
import argparse
import base64
from collections import Counter
import copy
import hashlib
import json
import logging
import math
import os
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'dds_swarm_fusion'))
from dds_data_tool import prepare
from field_simulator import CASES, write_json
from field_sim_metrics import summarize_multi


def process_run(directory,uncorrected_time=False):
    from company_input_node import CompanyInputNode
    from company_protocol import parse_company_packet
    from company_dds import Topics
    from dds_tracking_node import TrackingDdsNode
    from dds_fusion_node import FusionDdsNode
    metadata=json.loads((directory/'metadata.json').read_text())
    if metadata.get('format')!='sb-field-sim-v1': raise ValueError('Not a generated simulator run')
    if hashlib.sha256((directory/'capture.jsonl').read_bytes()).hexdigest()!=metadata['capture_sha256']:
        raise ValueError('Capture does not match simulator metadata')
    if metadata.get('files_sha256'):
        for name in ('capture.jsonl','events.jsonl','truth.jsonl','scenario.yaml','eo_devices.yaml'):
            if hashlib.sha256((directory/name).read_bytes()).hexdigest()!=metadata['files_sha256'].get(name):
                raise ValueError('Generated evaluation bundle changed: '+name)
    cfg=metadata['settings'];epoch=cfg['epoch_ms'];clock=[0.,epoch/1000]
    frames=[];modes=Counter();quality=Counter();stations=Counter();geometry=Counter()
    truth_leaks=0

    class MemoryBus:
        def read(self,topic): return []
        def publish_official_fusion(self,*args,**kw): pass  # no network
        def write(self,topic,payload,**kw):
            nonlocal truth_leaks
            if topic==Topics.RAW_MEASUREMENTS.name:
                truth_leaks+=bool(payload.get('truth'))
                for sid,value in payload['measurements'].items():
                    for m in value.get('meas',[]):
                        modes[m.get('mode','radar' if sid.startswith('radar') else 'other')]+=1
                        quality.update(m.get('quality_reasons',[]))
                        stations[m.get('station_source','unknown')]+=1
                tracker.process_raw(payload)
            elif topic==Topics.SENSOR_TRACKS.name:
                fuser.process_tracks(payload)
                geometry.update(e['reason'] for e in fuser.fusion.geometry_events)
            elif topic==Topics.FUSED_TRACKS.name:
                frames.append(dict(timestamp_ms=payload['source_timestamp_ms'],
                    receive_time_s=clock[0],fused=copy.deepcopy(payload['fused']),
                    external=copy.deepcopy(payload['external_tracks'])))

    with tempfile.TemporaryFile('w+b') as spool:
        checked=prepare(directory/'capture.jsonl',spool,None,'original',False)
        spool.seek(0)
        packets=[json.loads(line) for line in spool]
    log=logging.getLogger('company_input');old_level=log.level;log.setLevel(logging.ERROR)
    try:
        with patch.dict(os.environ,{'SB_EO_CONFIG':str(directory/'eo_devices.yaml')}), \
             patch('time.monotonic',lambda:clock[0]),patch('time.time',lambda:clock[1]):
            bus=MemoryBus();tracker=TrackingDdsNode(bus);fuser=FusionDdsNode(bus,min_confidence=0.2)
            node=CompanyInputNode(directory/'scenario.yaml',bus=bus,
                                  radar_input_mode='device_tracks',azimuth_convention='north_cw')
            if uncorrected_time:
                for value in node.eo_observations.devices.values(): value['time_offset_ms']=0
            def advance(t):
                while clock[0]<t:
                    clock[0]=min(t,clock[0]+.05);clock[1]=epoch/1000+clock[0]
                    node.flush_due();node.emit_idle_frame()
            for row in packets:
                advance(row['offset_ns']/1e9)
                for tag,stamp,obj in parse_company_packet(base64.b64decode(row['payload_base64'])):
                    node.ingest(tag,stamp,obj)
            advance(max(cfg['duration_s'],clock[0])+16)
            node.flush_due(force=True)
            diagnostics=dict(duplicates=node.duplicates,dropped_old=node.dropped_old,
                modes=dict(modes),quality_reasons=dict(quality),station_sources=dict(stations),
                geometry_events=dict(geometry),truth_leaks=truth_leaks,
                remaining_tracks=len(fuser.fusion.tracks) if fuser.fusion else 0,
                formal_output_min_confidence=fuser.min_confidence)
    finally:
        log.setLevel(old_level)
    return metadata,checked,frames,diagnostics


def summarize(directory,metadata,frames,diagnostics):
    cfg=metadata['settings'];epoch=cfg['epoch_ms'];duration=cfg['duration_s']
    # Truth data is NOT passed to any production node or filter.
    truths=[]
    with (directory/'truth.jsonl').open() as stream:
        for line in stream:
            value=json.loads(line)
            if value['truth_id']==1: truths.append(value)
    def target_at(stamp):
        index=max(0,min(len(truths)-2,int((stamp-epoch)/100)))
        a,b=truths[index],truths[index+1]
        factor=max(0,min(1,(stamp-a['timestamp_ms'])/(b['timestamp_ms']-a['timestamp_ms'])))
        return [x+factor*(y-x) for x,y in zip(a['position'],b['position'])]
    errors=[];heights=[];ids=set();switches=0;last_id=None;multiple=0;nonfinite=0
    active=[f for f in frames if 0 <= (f['timestamp_ms']-epoch)/1000 <= duration]
    for f in active:
        rows=f['fused'];multiple+=len(rows)>1
        for r in f['external']:
            nonfinite+=not all(math.isfinite(float(r[k])) for k in ('x','y','vx','vy','conf'))
        if not rows: continue
        p=target_at(f['timestamp_ms'])
        best=min(rows,key=lambda r:math.hypot(r['x']-p[0],r['y']-p[1]))
        errors.append(math.hypot(best['x']-p[0],best['y']-p[1]))
        if best.get('z') is not None: heights.append(abs(best['z']-p[2]))
        ids.add(best['id'])
        if last_id is not None and last_id!=best['id']:switches+=1
        last_id=best['id']
    ordered=sorted(frames,key=lambda x:x['timestamp_ms']);i=0;present=0;gap=longest=0;grid_count=0
    # State-presence coverage, not a fresh-measurement rate. Last published
    # snapshot held for up to one radar period + reorder slack, explicitly.
    for tick in range(math.floor(duration*2)+1):
        stamp=epoch+tick*500
        while i+1<len(ordered) and ordered[i+1]['timestamp_ms']<=stamp:i+=1
        f=ordered[i] if ordered and ordered[i]['timestamp_ms']<=stamp else None
        ok=bool(f and f['fused'] and stamp-f['timestamp_ms']<=(cfg['radar_period_s']+.5)*1000)
        present+=ok;grid_count+=1;gap=0 if ok else gap+.5;longest=max(longest,gap)
    def stats(values):
        if not values:return dict(n=0,rmse_m=None,p95_m=None,max_m=None)
        values=sorted(values)
        return dict(n=len(values),rmse_m=math.sqrt(sum(x*x for x in values)/len(values)),
                    p95_m=values[math.ceil(.95*len(values))-1],max_m=values[-1])
    checks={'no_truth_in_fusion_input':diagnostics['truth_leaks']==0,
            'finite_outputs':nonfinite==0,'cleared_after_input_stopped':diagnostics['remaining_tracks']==0}
    case=metadata['case'];modes=diagnostics['modes']
    if case in ('single-bearing','weak-geometry','colocated-bearing','radar-extrapolation'):
        checks['no_formal_localization']=not any(f['fused'] for f in frames)
    elif case in ('baseline','dual-bearing','recovery','ranged-only','range-failure'):
        checks['produced_formal_tracks']=bool(errors)
    if case in ('range-failure','radar-outage','recovery','dual-bearing','single-bearing'):
        checks['bearing_path_exercised']=modes.get('bearing',0)>0
    if case=='ranged-only':checks['ranged_path_exercised']=modes.get('ranged',0)>0
    if case=='recovery':checks['stable_id_on_short_range_loss']=len(ids)==1
    if case=='stale-values':checks['stale_position_detected']=diagnostics['quality_reasons'].get('position_unchanged_unverified',0)>0
    if case=='gps-fallback':checks['reported_and_config_sources']=all(diagnostics['station_sources'].get(x,0)>0 for x in ('reported_lla','configured_enu'))
    if case=='timing-faults':checks['duplicates_and_late_counted']=diagnostics['duplicates']>0 and diagnostics['dropped_old']>0
    if case in ('weak-geometry','colocated-bearing'):
        checks['geometry_rejection_recorded']=bool(diagnostics['geometry_events'])
    if case=='radar-outage' and duration>=60:
        unsupported=[f for f in active if .25*duration+14 < (f['timestamp_ms']-epoch)/1000 < .75*duration]
        checks['long_bearing_only_gap_does_not_keep_localization']=bool(unsupported) and not any(f['fused'] for f in unsupported)
    return dict(case=case,description=metadata['description'],checks=checks,all_checks_pass=all(checks.values()),
        horizontal_nearest_output_error=stats(errors),vertical_nearest_output_error=stats(heights),
        formal_state_coverage=present/grid_count,longest_no_formal_state_s=longest,
        nearest_output_id_count=len(ids),nearest_output_id_changes=switches,
        frames_with_multiple_formal_tracks=multiple,active_frames=len(active),
        diagnostics=diagnostics,dds_packets_sent=0,
        formal_output_min_confidence=diagnostics['formal_output_min_confidence'],
        multi_target_metrics=summarize_multi(directory,metadata,frames),
        metric_caveats=['Errors are frame-weighted, conditional on a formal output, nearest to primary truth without a distance gate.',
            'Nearest-output ID changes include reacquisition, not proof of an identity swap.',
            'Coverage uses a 0.5s grid holding the last snapshot for at most one radar period + 0.5s.',
            'Additional RF-only truths are interference targets, not required localized targets.',
            'Synthetic behaviour regression, NOT a field accuracy or detection-probability guarantee.'])


def evaluate(directory,output=None,uncorrected_time=False):
    directory=Path(directory).resolve()
    output=Path(output) if output else directory/'evaluation.json'
    trace=output.with_suffix('.frames.jsonl')
    if output.exists() or trace.exists():raise FileExistsError('Evaluation output exists; choose another --output')
    metadata,checked,frames,diag=process_run(directory,uncorrected_time)
    report=summarize(directory,metadata,frames,diag)
    report.update(capture_records=checked['selected_records'],uncorrected_time=uncorrected_time)
    write_json(output,report)
    with trace.open('x') as f:
        for row in frames:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    return report


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('directory',type=Path);p.add_argument('--output',type=Path)
    p.add_argument('--suite',action='store_true');p.add_argument('--uncorrected-time',action='store_true')
    args=p.parse_args(argv)
    try:
        if args.suite and args.output:raise ValueError('--suite writes evaluation.json per case; omit --output')
        dirs=[args.directory/c for c in CASES] if args.suite else [args.directory]
        ok=True
        for directory in dirs:
            report=evaluate(directory,args.output,args.uncorrected_time)
            ok &= report['all_checks_pass']
            print(json.dumps({k:report[k] for k in ('case','all_checks_pass','checks','horizontal_nearest_output_error','formal_state_coverage')},ensure_ascii=False),flush=True)
        return 0 if ok else 2
    except (OSError,ValueError,KeyError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr);return 1


if __name__=='__main__':
    raise SystemExit(main())
