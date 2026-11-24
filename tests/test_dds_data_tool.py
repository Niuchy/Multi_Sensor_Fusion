"""Standalone recorder/replayer tests. All publishers are in-memory fakes."""
import base64
from collections import defaultdict, deque
import contextlib
import ctypes
import hashlib
import io
import json
import os
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
import dds_data_tool as tool

STAMP = 1790000000000


def packet(topic='RadarTarget', stamp=STAMP, base=False, crc=0, units=1, unit_stamp=None):
    tag,magic,length=tool.PROTOCOL[topic]
    body=b''
    for i in range(units):
        frame=struct.pack('<IBHHHIQ',magic,1,tag,1,length,3152,stamp+i)
        frame+=bytes(range(length-27))+struct.pack('<I',crc)
        unit=struct.pack('<IH',length,tag)
        if not base: unit+=struct.pack('<Q',stamp+i if unit_stamp is None else unit_stamp)
        body+=unit+frame
    return struct.pack(tool.PXSB,0x7E,units,3152,0,len(body),0,784,0,0,stamp)+body


class Clock:
    def __init__(self): self.now=0
    def monotonic(self): return self.now
    def sleep(self,seconds): self.now+=seconds
    def wall_ms(self): return STAMP+1000000+round(self.now*1000)


class Bus:
    def __init__(self,clock=None,echo=True):
        self.clock=clock or Clock();self.queue=defaultdict(deque)
        self.sent=[];self.readers=[];self.writers=[];self.echo=echo
    def reader(self,t): self.readers.append(t)
    def writer(self,t): self.writers.append(t)
    def read_one(self,t): return self.queue[t].popleft() if self.queue[t] else None
    def write_bytes(self,t,p):
        self.sent.append((self.clock.now,t,p))
        if self.echo: self.queue[t].append(p)


class DataToolTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/'capture.jsonl'
        self.quiet=contextlib.redirect_stdout(io.StringIO());self.quiet.__enter__()
        self.addCleanup(self.quiet.__exit__,None,None,None)

    def capture(self,rows=None):
        rows=rows or [(0,'RadarTarget',packet()),(2_980_000_000,'RadarTarget',packet(stamp=STAMP+2980))]
        w=tool.CaptureWriter(self.path,tool.TOPICS,tool.MAX_FILE)
        for n,(offset,topic,payload) in enumerate(rows):
            self.assertTrue(w.add(topic,payload,offset,STAMP*1000000+n))
        w.finish('test')
        return rows

    def prepared(self,**kwargs):
        stream=tempfile.TemporaryFile('w+b');self.addCleanup(stream.close)
        return stream,tool.prepare(self.path,stream,**kwargs)

    def test_roundtrip_all_four_topics_and_binary_identity(self):
        rows=[(i*3000000000,t,packet(t)) for i,t in enumerate(tool.TOPICS)]
        self.capture(rows);stream,s=self.prepared()
        self.assertEqual(s['selected_records'],4)
        self.assertEqual(s['duration_s'],9)
        for row,original in zip(stream,rows):
            self.assertEqual(base64.b64decode(json.loads(row)['payload_base64']),original[2])
        self.assertTrue(s['complete'])

    def test_exclusive_create(self):
        self.capture();before=self.path.read_bytes()
        with self.assertRaises(FileExistsError): tool.CaptureWriter(self.path,tool.TOPICS,tool.MAX_FILE)
        self.assertEqual(self.path.read_bytes(),before)

    def test_bad_payload_hash_rejected_before_dds(self):
        self.capture();data=self.path.read_bytes().replace(b'"length":106',b'"length":107',1);self.path.write_bytes(data)
        with patch.object(tool,'NativeBus') as native:
            self.assertEqual(tool.main(['replay',str(self.path),'--publish','--inputs-stopped']),1)
            native.assert_not_called()

    def test_footer_hash_and_sequence(self):
        self.capture();data=self.path.read_bytes();self.path.write_bytes(data.replace(b'"seq":2',b'"seq":3'))
        with self.assertRaises(ValueError): self.prepared()
        self.path.write_bytes(data.replace(b'"reason":"test"',b'"reason":"test","counts":{}'))
        with self.assertRaises(ValueError): self.prepared()

    def test_incomplete_recovery_explicit(self):
        self.capture();lines=self.path.read_bytes().splitlines(keepends=True)
        self.path.write_bytes(b''.join(lines[:-1])+b'{"type":')
        with self.assertRaises(ValueError): self.prepared()
        _,summary=self.prepared(allow_incomplete=True)
        self.assertEqual(summary['selected_records'],2)
        self.assertFalse(summary['complete']);self.assertTrue(summary['ignored_truncated_tail'])

    def test_malformed_complete_line_not_ignored(self):
        self.capture();self.path.write_bytes(self.path.read_bytes().splitlines(keepends=True)[0]+b'bad\n')
        with self.assertRaises(ValueError): self.prepared(allow_incomplete=True)

    def test_empty_inspect_and_publish_refusal(self):
        w=tool.CaptureWriter(self.path,tool.TOPICS,tool.MAX_FILE);w.finish('duration')
        _,s=self.prepared();self.assertEqual(s['selected_records'],0)
        with patch.object(tool,'NativeBus') as native:
            self.assertEqual(tool.main(['replay',str(self.path),'--publish','--inputs-stopped']),1)
            native.assert_not_called()

    def test_dry_run_and_inspection_never_load_native(self):
        self.capture()
        with patch.object(tool,'NativeBus') as native,patch.object(tool,'ensure_runtime') as env:
            self.assertEqual(tool.main(['inspect',str(self.path)]),0)
            self.assertEqual(tool.main(['replay',str(self.path)]),0)
            self.assertEqual(tool.main(['replay',str(self.path),'--time-mode','rebase']),0)
            native.assert_not_called();env.assert_not_called()

    def test_ack_and_numeric_arguments_required(self):
        self.capture()
        with patch.object(tool,'NativeBus') as native:
            for flags in (['--publish'],['--speed','nan'],['--quiet-seconds','0'],['--speed','100']):
                self.assertEqual(tool.main(['replay',str(self.path),*flags]),1)
            native.assert_not_called()

    def test_topic_filter_maintains_relative_timing(self):
        self.capture([(0,'RadarTarget',packet()),(100,'ESMTarget',packet('ESMTarget')),
                      (300,'ESMTarget',packet('ESMTarget'))])
        _,s=self.prepared(topics='ESMTarget')
        self.assertEqual(s['file_records'],3);self.assertEqual(s['selected_records'],2)
        self.assertEqual(s['first_offset_ns'],100);self.assertEqual(s['duration_s'],.0000002)

    def test_no_fusion_or_control_topics_allowed(self):
        for value in ('FusionMultiTarget','SwarmFusion.ControlCommand','RadarTarget,RadarTarget'):
            with self.assertRaises(ValueError): tool.select_topics(value)

    def test_original_replay_exact_bytes_speed_and_order(self):
        rows=self.capture();spool,s=self.prepared();clock=Clock();bus=Bus(clock)
        sent=tool.replay(bus,spool,s,speed=2,monotonic=clock.monotonic,wall_ms=clock.wall_ms,sleep=clock.sleep)
        self.assertEqual(sent,2)
        self.assertEqual([x[2] for x in bus.sent],[x[2] for x in rows])
        self.assertAlmostEqual(bus.sent[1][0]-bus.sent[0][0],1.49)
        self.assertGreaterEqual(bus.sent[0][0],4)

    def test_source_changed_after_validation_cannot_change_replay(self):
        rows=self.capture();spool,s=self.prepared();self.path.write_bytes(b'changed')
        clock=Clock();bus=Bus(clock)
        tool.replay(bus,spool,s,monotonic=clock.monotonic,sleep=clock.sleep)
        self.assertEqual([x[2] for x in bus.sent],[x[2] for x in rows])

    def test_rebase_shifts_headers_only_all_topics_and_both_unit_forms(self):
        for topic in tool.TOPICS:
            for base in (False,True):
                before=packet(topic,base=base,units=2)
                fields=tool.timestamp_fields(topic,before)
                after=tool.relocate(topic,before,12345)
                changed={i for off,stamp,_ in fields if stamp for i in range(off,off+8)}
                for i,(a,b) in enumerate(zip(before,after)):
                    if i not in changed: self.assertEqual(a,b)
                for off,stamp,_ in fields: self.assertEqual(struct.unpack_from('<Q',after,off)[0],stamp+12345)
                self.assertEqual(len(before),len(after))

    def test_rebase_keeps_zero_unit_and_preserves_time_intervals(self):
        before=packet(unit_stamp=0);after=tool.relocate('RadarTarget',before,100)
        self.assertEqual(struct.unpack_from('<Q',after,32)[0],0)
        self.capture();spool,s=self.prepared(time_mode='rebase');clock=Clock();bus=Bus(clock)
        tool.replay(bus,spool,s,time_mode='rebase',monotonic=clock.monotonic,wall_ms=clock.wall_ms,sleep=clock.sleep)
        stamps=[struct.unpack_from('<Q',p,55)[0] for _,_,p in bus.sent]
        self.assertEqual(stamps,[STAMP+1004000,STAMP+1006980])

    def test_rebase_unknown_crc_and_bad_size_refused(self):
        for payload in (packet(crc=99),packet()+b'X',b'raw'):
            with self.assertRaises(ValueError): tool.timestamp_fields('RadarTarget',payload)

    def test_rebase_mixed_clock_basis_refused_before_any_send(self):
        self.capture([(0,'RadarTarget',packet(unit_stamp=123))])
        with self.assertRaises(ValueError): self.prepared(time_mode='rebase')
        self.prepared(time_mode='original')

    def test_quiet_guard_blocks_active_equipment(self):
        self.capture();spool,s=self.prepared();bus=Bus();bus.queue['RadarTarget'].append(packet())
        with self.assertRaises(RuntimeError): tool.replay(bus,spool,s,monotonic=bus.clock.monotonic,sleep=bus.clock.sleep)
        self.assertEqual(bus.sent,[])

    def test_live_traffic_mid_replay_aborts(self):
        self.capture();spool,s=self.prepared();clock=Clock()
        class Intruder(Bus):
            def read_one(self,t):
                if clock.now>=5 and t=='ESMTarget': return b'live'
                return super().read_one(t)
        bus=Intruder(clock)
        with self.assertRaises(RuntimeError): tool.replay(bus,spool,s,monotonic=clock.monotonic,sleep=clock.sleep)
        self.assertEqual(len(bus.sent),1)

    def test_record_only_reads_and_closes_valid_capture(self):
        clock=Clock();bus=Bus(clock)
        for topic in tool.TOPICS: bus.queue[topic].append(packet(topic))
        tool.record(bus,self.path,tool.TOPICS,.02,tool.MAX_FILE,clock.monotonic,lambda: STAMP*1000000,clock.sleep)
        _,s=self.prepared();self.assertEqual(s['file_records'],4)
        self.assertEqual(bus.sent,[]);self.assertEqual(bus.writers,[])

    def test_record_ctrl_c_finishes_and_size_cap_is_visible(self):
        class Interrupted(Bus):
            def read_one(self,t): raise KeyboardInterrupt()
        self.assertEqual(tool.record(Interrupted(),self.path,tool.TOPICS,0,tool.MAX_FILE),130)
        _,s=self.prepared();self.assertEqual(s['completion_reason'],'interrupted')
        other=Path(self.tmp.name)/'small.jsonl'
        w=tool.CaptureWriter(other,tool.TOPICS,3000)
        self.assertFalse(w.add('RadarTarget',b'x'*3000,0,0));w.finish('size_limit')
        self.assertLess(other.stat().st_size,3000)

    def test_reject_nan_and_invalid_record_parameters(self):
        with patch.object(tool,'NativeBus') as native:
            for flag in (['--duration','nan'],['--max-mib','999']):
                self.assertEqual(tool.main(['record',str(self.path),*flag]),1)
            native.assert_not_called()

    def test_native_environment_does_not_change_parent_and_uses_x86_path(self):
        old={'LD_LIBRARY_PATH':'/keep/lib','MY_MARKER':'x'}
        env=tool.runtime_environment(old,system='Linux',machine='x86_64')
        self.assertIn('linux-x86/lib',env['LD_LIBRARY_PATH'])
        self.assertTrue(env['LD_LIBRARY_PATH'].endswith('/keep/lib'))
        self.assertNotIn('SB_DDS_DATA_TOOL_ENV',old)

    def test_web_import_merges_per_target_rows_and_keeps_bytes(self):
        source=Path(self.tmp.name)/'web.json'
        r={'received_at_ms':STAMP,'captured_at_monotonic_s':50.,'packet_base64':base64.b64encode(packet(units=2)).decode()}
        second={**r,'captured_at_monotonic_s':50.00001}
        third={**r,'received_at_ms':STAMP+3000,'captured_at_monotonic_s':53.}
        source.write_text(json.dumps({'format':'sb-fusion-alignment-v1','records':[r,second,third]}))
        self.assertEqual(tool.main(['import-web',str(source),str(self.path)]),0)
        stream,s=self.prepared();self.assertEqual(s['selected_records'],2)
        self.assertEqual(s['duration_s'],3);self.assertEqual(s['source']['merged_duplicate_rows'],1)
        self.assertEqual(base64.b64decode(json.loads(next(stream))['payload_base64']),packet(units=2))

    def test_web_import_missing_bytes_rejected_without_output(self):
        source=Path(self.tmp.name)/'web.json'
        source.write_text(json.dumps({'format':'sb-fusion-alignment-v1','records':[{'received_at_ms':STAMP}]}))
        self.assertEqual(tool.main(['import-web',str(source),str(self.path)]),1)
        self.assertFalse(self.path.exists())


class Fn:
    def __init__(self,fn): self.fn=fn
    def __call__(self,*args): return self.fn(*args)


class NativeAbiTests(unittest.TestCase):
    def test_ctypes_read_write_matches_existing_bridge_abi(self):
        class Library: pass
        lib=Library();writes=[];closes=[];queue=deque([packet()])
        for name in ('init','create_reader','create_writer'):
            setattr(lib,'company_dds_'+name,Fn(lambda *args:0))
        def poll(topic,buffer,capacity,actual,timeout):
            if not queue:return 0
            data=queue.popleft();ctypes.cast(actual,ctypes.POINTER(ctypes.c_uint32))[0]=len(data)
            ctypes.memmove(buffer,data,len(data));return 1
        lib.company_dds_poll=Fn(poll)
        lib.company_dds_publish=Fn(lambda topic,b,n: (writes.append((topic,bytes(b[:n]))) or 0))
        lib.company_dds_close=Fn(lambda:closes.append(True))
        with patch.object(tool.platform,'system',return_value='Linux'),patch.object(tool.ctypes,'CDLL',return_value=lib):
            bus=tool.NativeBus('/fake/bridge.so')
            data=bus.read_one('RadarTarget');self.assertEqual(data,packet())
            bus.write_bytes('RadarTarget',data);bus.close()
        self.assertEqual(writes,[(b'RadarTarget',packet())]);self.assertEqual(closes,[True])


if __name__=='__main__': unittest.main()
