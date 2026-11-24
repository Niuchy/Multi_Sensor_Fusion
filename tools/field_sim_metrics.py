"""Retrospective multi-target metrics; truth is used ONLY after fusion finishes.

These are explicitly defined, gated sample metrics, not CLEAR MOTA / IDF1 and
not device classification accuracy. Matching never reads simulator truth_id
from a sensor report or an output track.
"""
from bisect import bisect_right
import json
import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def error_stats(values):
    if not values:
        return dict(n=0,rmse_m=None,p95_m=None,max_m=None,mean_m=None)
    ordered=sorted(values)
    return dict(n=len(values),rmse_m=math.sqrt(sum(x*x for x in values)/len(values)),
                p95_m=ordered[math.ceil(.95*len(values))-1],max_m=ordered[-1],
                mean_m=sum(values)/len(values))


def match_positions(truth_positions,outputs,gate_m):
    """Maximum-cardinality in-gate assignment, then minimum squared XY error.

    A separate dummy for each truth prevents out-of-gate pairs consuming a
    legitimate match. At most one output is credited to each truth and vice versa.
    """
    n,m=len(truth_positions),len(outputs)
    if not n or not m:return []
    penalty=n+1.
    cost=np.full((n,m+n),penalty)
    cost[:,:m]=2*penalty
    distances={}
    for i,p in enumerate(truth_positions):
        for j,r in enumerate(outputs):
            x,y=r.get('x'),r.get('y')
            if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in (x,y)):continue
            d=math.hypot(x-p[0],y-p[1])
            if d<=gate_m:
                distances[i,j]=d;cost[i,j]=(d/gate_m)**2
    rows,cols=linear_sum_assignment(cost)
    return [(int(i),int(j),distances[i,j]) for i,j in zip(rows,cols) if (i,j) in distances]


def summarize_multi(directory,metadata,frames):
    cfg=metadata['settings'];epoch=cfg['epoch_ms'];duration=cfg['duration_s']
    gate=cfg.get('evaluation_gate_m',50.)
    if not math.isfinite(gate) or gate<=0:raise ValueError('Invalid evaluation gate')
    ids=metadata.get('evaluation_truth_ids',list(range(1,cfg.get('custom_target_count',1)+1))
                     if metadata['case']=='custom' else [1])
    if not ids or len(set(ids))!=len(ids):raise ValueError('Invalid evaluation truth IDs')
    truths={tid:[] for tid in ids}
    with (directory/'truth.jsonl').open() as stream:
        for line in stream:
            row=json.loads(line)
            if row['truth_id'] in truths:truths[row['truth_id']].append(row)
    stamps={}
    for tid,rows in truths.items():
        rows.sort(key=lambda r:r['timestamp_ms'])
        stamps[tid]=[r['timestamp_ms'] for r in rows]
        if (len(rows)<2 or stamps[tid][0]>epoch or stamps[tid][-1]<epoch+duration*1000 or
            any(a>=b for a,b in zip(stamps[tid],stamps[tid][1:]))):
            raise ValueError('Truth does not cover evaluation window: '+str(tid))

    def position(tid,stamp):
        k=max(0,min(len(truths[tid])-2,bisect_right(stamps[tid],stamp)-1))
        a,b=truths[tid][k:k+2];q=(stamp-a['timestamp_ms'])/(b['timestamp_ms']-a['timestamp_ms'])
        return [x+(y-x)*q for x,y in zip(a['position'],b['position'])]

    step=.5;max_age=cfg['radar_period_s']+.5
    ordered=sorted(frames,key=lambda r:r['timestamp_ms']);cursor=-1
    targets={tid:dict(truth_id=tid,matched_samples=0,id_switches=0,reacquisition_id_changes=0,
                     fragments=0,first_match_s=None,longest_miss_s=0.) for tid in ids}
    histories={tid:dict(last_id=None,previous_match=False,ever=False,gap=0.,errors=[],heights=[],errors3d=[]) for tid in ids}
    tp=fp=fn=duplicates=all_covered=invalid_samples=0;all_errors=[];heights=[];errors3d=[]
    grid_count=math.ceil(duration/step)
    for tick in range(grid_count):
        t=tick*step;stamp=epoch+t*1000;span=min(step,duration-t)
        while cursor+1<len(ordered) and ordered[cursor+1]['timestamp_ms']<=stamp:cursor+=1
        rows=[]
        if cursor>=0:
            f=ordered[cursor];age=(stamp-f['timestamp_ms'])/1000
            if age<=max_age:
                # Time-align snapshots with their reported velocity. Holding a
                # state is not credited as a new sensor detection/measurement.
                for raw in f['fused']:
                    r=dict(raw)
                    for p,v in (('x','vx'),('y','vy'),('z','vz')):
                        if r.get(p) is not None and isinstance(r.get(p),(int,float)):
                            velocity=r.get(v,0)
                            if not isinstance(velocity,(int,float)) or not math.isfinite(velocity):velocity=0.
                            r[p]+=age*velocity
                    rows.append(r)
        truth_positions=[position(tid,stamp) for tid in ids]
        matches=match_positions(truth_positions,rows,gate)
        by_truth={i:(j,d) for i,j,d in matches};matched_rows={j for _,j,_ in matches}
        tp+=len(matches);fn+=len(ids)-len(matches);fp+=len(rows)-len(matches)
        all_covered+=len(matches)==len(ids)
        for j,r in enumerate(rows):
            valid=all(isinstance(r.get(k),(int,float)) and math.isfinite(r[k]) for k in ('x','y'))
            invalid_samples+=not valid
            if valid and j not in matched_rows and any(
                    math.hypot(r['x']-truth_positions[i][0],r['y']-truth_positions[i][1])<=gate for i in by_truth):
                duplicates+=1
        for i,tid in enumerate(ids):
            out=targets[tid];h=histories[tid]
            if i not in by_truth:
                h['gap']+=span;out['longest_miss_s']=max(out['longest_miss_s'],h['gap'])
                h['previous_match']=False;continue
            j,d=by_truth[i];row=rows[j];track_id=row.get('id')
            if h['ever']:
                if not h['previous_match']:out['fragments']+=1
                if track_id!=h['last_id']:
                    out['id_switches' if h['previous_match'] else 'reacquisition_id_changes']+=1
            else:out['first_match_s']=t
            h.update(last_id=track_id,previous_match=True,ever=True,gap=0.)
            out['matched_samples']+=1;h['errors'].append(d);all_errors.append(d)
            z=row.get('z')
            if isinstance(z,(int,float)) and math.isfinite(z):
                dz=abs(z-truth_positions[i][2]);d3=math.hypot(d,dz)
                heights.append(dz);errors3d.append(d3);h['heights'].append(dz);h['errors3d'].append(d3)
    for tid,out in targets.items():
        h=histories[tid];out.update(truth_samples=grid_count,missed_samples=grid_count-out['matched_samples'],
            recall=out['matched_samples']/grid_count,horizontal_error=error_stats(h['errors']),
            vertical_error=error_stats(h['heights']),position_3d_error=error_stats(h['errors3d']))
    precision=tp/(tp+fp) if tp+fp else None;recall=tp/(tp+fn)
    return dict(version=1,truth_ids=ids,target_count=len(ids),grid_step_s=step,grid_samples=grid_count,
        window_s=[0,duration],max_snapshot_age_s=max_age,association_gate_m=gate,
        true_positive_samples=tp,false_positive_samples=fp,false_negative_samples=fn,
        duplicate_candidate_samples=duplicates,invalid_output_samples=invalid_samples,
        precision=precision,recall=recall,f1=2*tp/(2*tp+fp+fn),
        all_targets_covered_fraction=all_covered/grid_count,
        horizontal_error=error_stats(all_errors),vertical_error=error_stats(heights),position_3d_error=error_stats(errors3d),
        id_switches=sum(t['id_switches'] for t in targets.values()),
        reacquisition_id_changes=sum(t['reacquisition_id_changes'] for t in targets.values()),
        fragments=sum(t['fragments'] for t in targets.values()),per_target=list(targets.values()),
        definitions={
            'scope':'Formal fused outputs only; external/coasting-only reports are excluded. All selected UAV truths count, even when no sensor is assigned to them.',
            'association':'At each 0.5 s source-time grid sample: maximum-cardinality one-to-one XY matching within the gate, then minimum squared error; no truth IDs from device reports used.',
            'precision':'TP/(TP+FP); null when there are no outputs. FP includes duplicates and outside-gate outputs, not necessarily a physical false alarm.',
            'recall':'TP/(TP+FN), target-state coverage; NOT fresh-measurement detection probability.',
            'f1':'2TP/(2TP+FP+FN); no true-negative count, so no classification accuracy is claimed.',
            'error':'RMSE/P95 are conditional on in-gate matches; report coverage alongside them. 3D/vertical use only pairs with finite output altitude; XY matching does not enforce altitude correctness.',
            'identity':'ID switches compare consecutive matched grid samples for the same truth. Reacquisition ID changes are separate; fragments count matched→missing→matched episodes. Not CLEAR MOTA/IDF1.',
            'time':'Retrospective source-time alignment, not wall-clock latency assessment. Latest snapshot extrapolated by output velocity for at most radar period + 0.5 s; no future snapshot used.',
            'limits':'Synthetic behavioural test; not real-world accuracy certification. Default paths are separated, not a crossing-target identity benchmark.'})
