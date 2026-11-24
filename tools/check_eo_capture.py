#!/usr/bin/env python3
"""Offline capture -> input -> tracker -> fusion audit. NEVER opens a DDS bus.

This checks processing/quality semantics, not positioning accuracy. No truth
positions, inferred station or learned angular corrections enter the filter.
"""
import argparse
import base64
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch


def run_capture(path, root):
    from company_input_node import CompanyInputNode
    from company_dds import Topics
    from company_protocol import parse_company_packet
    from dds_tracking_node import TrackingDdsNode
    from dds_fusion_node import FusionDdsNode

    result = dict(file=path.name, sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                  samples=0, topics=Counter(), eo_states=Counter(), eo_modes=Counter(),
                  eo_devices=Counter(), quality_reasons=Counter(), accepted_eo_updates=0,
                  raw_batches=0, frames_with_fused_output=0, max_fused_tracks=0,
                  nonfinite_outputs=0, formal_track_ids=set(),
                  note='Offline memory pipeline; no DDS; no ground truth or inferred calibration.')
    import math

    class MemoryPipeline:
        def write(self, topic, payload, **kw):
            if topic == Topics.RAW_MEASUREMENTS.name:
                result['raw_batches'] += 1
                for sid, value in payload['measurements'].items():
                    for m in value.get('meas', []):
                        if sid.startswith('eo_'):
                            result['eo_modes'][m.get('mode', 'legacy')] += 1
                            result['quality_reasons'].update(m.get('quality_reasons', []))
                            result['accepted_eo_updates'] += bool(m.get('fresh'))
                tracker.process_raw(payload)
            elif topic == Topics.SENSOR_TRACKS.name:
                fuser.process_tracks(payload)
            elif topic == Topics.FUSED_TRACKS.name:
                rows = payload['fused']
                result['frames_with_fused_output'] += bool(rows)
                result['max_fused_tracks'] = max(result['max_fused_tracks'], len(rows))
                result['formal_track_ids'].update(r['id'] for r in rows)
                for r in rows:
                    result['nonfinite_outputs'] += not all(math.isfinite(float(r[k])) for k in ('x','y','vx','vy','conf'))
        def read(self, topic):
            return []
        def publish_official_fusion(self, *a, **kw):
            pass

    clock = [0., 0.]
    with patch('time.monotonic', lambda: clock[0]), patch('time.time', lambda: clock[1]):
        bus = MemoryPipeline()
        tracker = TrackingDdsNode(bus)
        fuser = FusionDdsNode(bus)
        source = CompanyInputNode(root/'anti-drone-pkg/config/scenarios/multi_target.yaml',
                                  bus=bus, radar_input_mode='device_tracks', azimuth_convention='north_cw')
        previous_offset = -1
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row.get('type') != 'sample':
                    continue
                raw = base64.b64decode(row['payload_base64'], validate=True)
                if len(raw) != row['length'] or hashlib.sha256(raw).hexdigest() != row['sha256']:
                    raise ValueError(f'Payload integrity error in {path.name}: seq={row.get("seq")}')
                offset = int(row['offset_ns'])
                if offset < previous_offset:
                    raise ValueError('Recorder offsets must be monotonic')
                previous_offset = offset
                clock[:] = [offset/1e9, int(row['received_at_ns'])/1e9]
                # Drain before ingesting a new arrival, as a running input loop
                # does between packets. Arrival timing is not device timing.
                source.flush_due()
                source.emit_idle_frame()
                result['samples'] += 1
                result['topics'][row['topic']] += 1
                for tag, stamp, obj in parse_company_packet(raw):
                    if tag == 0x1C01:
                        result['eo_states'][str(obj.track_status)] += 1
                        result['eo_devices'][str(obj.device_id)] += 1
                    source.ingest(tag, stamp, obj)
        source.flush_due(force=True)
        result['dropped_old'] = source.dropped_old
        result['duplicates'] = source.duplicates
        result['distinct_formal_ids'] = len(result.pop('formal_track_ids'))
        result['geometry_events'] = getattr(fuser.fusion, 'geometry_events', [])
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('captures', nargs='+', type=Path)
    parser.add_argument('--project-root', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.project_root.resolve()
    sys.path.insert(0, str(root/'dds_swarm_fusion'))
    results = []
    for path in args.captures:
        result = run_capture(path.resolve(), root)
        results.append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    if args.output:
        args.output.write_text(json.dumps(results, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    if any(r['nonfinite_outputs'] for r in results):
        raise SystemExit('Nonfinite fusion output')


if __name__ == '__main__':
    main()
