"""No native DDS participant is created by these tests."""
import asyncio
from copy import deepcopy
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dds_swarm_fusion"))
from alignment_debug import (AlignmentWorkbench, ProfileStore, DEFAULTS, FORMAT,
    compare, positions, receiver_log_records, validate_params, validate_record)
from company_protocol import PXSB_HEADER_FORMAT

CONTEXT = {"origin": [120, 30, 0], "station_enu_m": [10, 20, 30], "azimuth_convention": "north_cw"}
STAMP = 1_790_000_000_000


def record(**overrides):
    raw = dict(device_id=3152, track_id=89, timestamp_ms=STAMP, range_m=100,
        azimuth_deg=0, elevation_deg=0, lng_deg=120.001, lat_deg=30.002,
        alt_m=80, speed_ms=6, course_deg=0, extrap_flag=0)
    raw.update(overrides)
    return validate_record({"raw": raw, "context": deepcopy(CONTEXT),
        "received_at_ms": STAMP+500, "unit_timestamp_ms": STAMP+10,
        "header_timestamp_ms": STAMP+20, "sequence": 1})


def packet(base=False):
    frame = struct.pack("<IBHHHIQ", 0xA55AA55A, 1, 0x1A01, 1, 66, 3152, STAMP)
    frame += struct.pack("<IIhhIhiiIIHBBB", 89, 1000, 0, 0, 0, 0,
        1200010000, 300020000, 800, 60, 0, 0, 0, 80)
    frame += struct.pack("<I", 0)
    unit = struct.pack("<IH", len(frame), 0x1A01)
    if not base:
        unit += struct.pack("<Q", STAMP+10)
    unit += frame
    return struct.pack(PXSB_HEADER_FORMAT, 0x7E, 1, 3152, 0, len(unit), 0, 784, 0, 0, STAMP+20)+unit


class ComparisonTests(unittest.TestCase):
    def test_defaults_identity_preserves_record(self):
        r=record(); before=deepcopy(r)
        result=compare([r], {})
        self.assertEqual(r, before)
        self.assertEqual(result["rows"][0]["raw_position"], result["rows"][0]["corrected_position"])
        self.assertEqual(result["summary"]["position_change_max_m"], 0)
        self.assertFalse(result["accuracy_assessed"])

    def test_l2_ignores_range_angle_station(self):
        p={"range_bias_m":300, "azimuth_bias_deg":20, "station_enu_m":[900,100,0]}
        self.assertEqual(compare([record()],p)["summary"]["position_change_max_m"],0)
        self.assertEqual(compare([record()],{"position_bias_enu_m":[3,4,0]})["summary"]["position_change_max_m"],5)

    def test_l1_north_cw_and_station(self):
        r=record(lng_deg=0,lat_deg=0)
        p=positions(r,validate_params({}))
        self.assertEqual(p["source"],"L1")
        for a,b in zip(p["xyz"],[10,120,30]): self.assertAlmostEqual(a,b)
        p=positions(r,validate_params({"azimuth_bias_deg":90}))
        for a,b in zip(p["xyz"],[110,20,30]): self.assertAlmostEqual(a,b)

    def test_force_unavailable_l2_does_not_fall_back(self):
        r=compare([record(lng_deg=0,lat_deg=0)],{"coordinate_mode":"l2"})["rows"][0]
        self.assertIsNotNone(r["raw_position"])
        self.assertIsNone(r["corrected_position"])
        self.assertTrue(r["errors"])

    def test_no_origin_does_not_guess(self):
        r=record();r["context"]["origin"]=None
        self.assertEqual(compare([r],{})["summary"]["invalid_positions"],1)

    def test_time_unknown_not_fake_latency(self):
        r=compare([record()],{"time_offset_ms":100})["rows"][0]
        self.assertEqual(r["corrected_timestamp_ms"],STAMP+100)
        self.assertIsNone(r["receive_minus_corrected_ms"])
        self.assertEqual(r["position_change_m"],0)

    def test_time_selection_signed_offset(self):
        r=compare([record()],{"time_source":"unit","clock_basis":"unix_ms","time_offset_ms":-40})["rows"][0]
        self.assertEqual(r["corrected_timestamp_ms"],STAMP-30)
        self.assertEqual(r["receive_minus_corrected_ms"],530)

    def test_relative_time_cannot_masquerade_as_epoch(self):
        r=compare([record(timestamp_ms=48265852)],{"clock_basis":"unix_ms"})["rows"][0]
        self.assertIsNone(r["receive_minus_corrected_ms"])
        self.assertTrue(r["warnings"])

    def test_per_track_intervals_duplicate_and_late(self):
        records=[record(), record(track_id=90),record(timestamp_ms=STAMP+3000),
                 record(timestamp_ms=STAMP+3000),record(timestamp_ms=STAMP+2999)]
        s=compare(records,{})["summary"]
        self.assertEqual(s["interval_mean_s"],3)
        self.assertEqual(s["duplicate_observations"],1)
        self.assertEqual(s["out_of_order_observations"],1)

    def test_real_log_21_predicted_reports(self):
        rows=receiver_log_records((ROOT/"samples/radar_3s.log").read_text(), CONTEXT)
        self.assertEqual(len(rows),21)
        self.assertEqual({r["raw"]["extrap_flag"] for r in rows},{0})
        self.assertEqual(rows[-1]["raw"]["timestamp_ms"]-rows[0]["raw"]["timestamp_ms"],59933)
        self.assertAlmostEqual(compare(rows,{})["summary"]["interval_mean_s"],2.99665)
        self.assertIsNone(rows[0]["received_at_ms"])
        self.assertIsNone(rows[0]["header_timestamp_ms"])

    def test_reject_nonfinite_and_unknown_fields(self):
        for p in ({"time_offset_ms":float("nan")},{"position_bias_enu_m":[1,2]},
                  {"live_apply":True},{"coordinate_mode":"other"},{"range_bias_m":False}):
            with self.assertRaises((ValueError,TypeError)): validate_params(p)
        r=record();r["context"]=None
        with self.assertRaises(ValueError): validate_record(r)


class StoreAndObserverTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/"profiles.json"

    def test_read_does_not_create_config(self):
        self.assertEqual(ProfileStore(self.path).read()["revision"],0)
        self.assertFalse(self.path.exists())

    def test_persist_and_rollback_as_new_revision(self):
        a=ProfileStore(self.path)
        a.save(0,"radar_3152",{"time_offset_ms":125})
        b=ProfileStore(self.path)
        b.save(1,"radar_3152",{"time_offset_ms":250})
        r=b.save(2,rollback=1)
        self.assertEqual(r["revision"],3)
        self.assertEqual(r["devices"]["radar_3152"]["time_offset_ms"],125)
        self.assertEqual(len(r["history"]),3)
        self.assertEqual(b.save(3,rollback=0)["devices"],{})

    def test_stale_revision_rejected_without_write(self):
        a=ProfileStore(self.path);a.save(0,"radar_3152",{})
        before=self.path.read_bytes()
        with self.assertRaises(FileExistsError): a.save(0,"radar_3152",{})
        self.assertEqual(before,self.path.read_bytes())

    def test_corrupt_config_not_overwritten(self):
        self.path.write_text('{"schema": 99}')
        with self.assertRaises(ValueError): ProfileStore(self.path).save(0,"radar_3152",{})
        self.assertEqual(self.path.read_text(),'{"schema": 99}')

    def test_atomic_replace_failure_preserves_old(self):
        a=ProfileStore(self.path);a.save(0,"radar_3152",{})
        before=self.path.read_bytes()
        with patch("alignment_debug.os.replace",side_effect=OSError("test disk failure")):
            with self.assertRaises(OSError): a.save(1,"radar_3152",{})
        self.assertEqual(self.path.read_bytes(),before)
        self.assertEqual(list(self.path.parent.glob('.alignment-*')),[])

    def test_observer_raw_bytes_and_absent_unit_time(self):
        w=AlignmentWorkbench(self.path)
        init={"coordinates":{"origin":CONTEXT["origin"]},"sensors":{"radar_3152":{"pos":CONTEXT["station_enu_m"]}}}
        w.observe(packet(),init,"north_cw",STAMP+500)
        w.observe(packet(base=True),init,"north_cw",STAMP+501)
        rows=w.export()["records"]
        self.assertEqual(rows[0]["unit_timestamp_ms"],STAMP+10)
        self.assertIsNone(rows[1]["unit_timestamp_ms"])
        self.assertEqual(rows[1]["header_timestamp_ms"],STAMP+20)
        import base64
        self.assertEqual(base64.b64decode(rows[0]["packet_base64"]),packet())
        self.assertEqual(rows[0]["raw"]["extrap_flag"],0)
        self.assertFalse(self.path.exists())

    def test_observer_bounds_and_malformed_packet(self):
        w=AlignmentWorkbench(self.path)
        with patch("alignment_debug.MAX_RECORDS",2):
            for _ in range(3): w.observe(packet(),{},"north_cw")
        self.assertEqual(len(w.records),2)
        self.assertEqual(w.evicted_records,1)
        w.observe(b"bad",{},"north_cw")
        self.assertEqual(w.errors,1)
        class BrokenBus:
            def read_bytes(self,*args,**kwargs): raise RuntimeError("unavailable")
        w.poll(BrokenBus(),{},{});self.assertIn("unavailable",w.reader_error)


class ApiTests(unittest.TestCase):
    def setUp(self):
        from fastapi import FastAPI
        from alignment_api import install_alignment_api
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/"profiles.json"
        self.app=FastAPI()
        self.w=install_alignment_api(self.app,SimpleNamespace(),self.path)
        self.client=AsgiClient(self.app)
        self.headers={"X-Alignment-Token":self.w.token}
        self.w.observe(packet(),{"coordinates":{"origin":CONTEXT["origin"]},
            "sensors":{"radar_3152":{"pos":CONTEXT["station_enu_m"]}}},"north_cw",STAMP+500)

    def post(self,path,data):
        return self.client.post('/api/alignment/'+path,json=data,headers=self.headers)

    def test_api_export_reimport_identical(self):
        capture=self.client.get('/api/alignment/capture').json()
        original=self.post('preview',{"device":"radar_3152","params":{}}).json()
        replay=self.post('replay',{"format":"capture","content":json.dumps(capture),"device":"radar_3152","params":{}})
        self.assertEqual(replay.status_code,200)
        self.assertEqual(replay.json()["rows"],original["rows"])
        self.assertEqual(replay.json()["dds_packets_sent"],0)

    def test_api_save_conflict_and_rollback(self):
        data={"expected_revision":0,"device":"radar_3152","params":{}}
        self.assertEqual(self.post('save',data).status_code,200)
        self.assertEqual(self.post('save',data).status_code,409)
        r=self.post('rollback',{"expected_revision":1,"revision":0})
        self.assertEqual(r.json()["revision"],2)
        self.assertEqual(r.json()["profiles"],{})

    def test_bad_body_and_csrf(self):
        self.assertEqual(self.client.post('/api/alignment/save',json={}).status_code,403)
        self.assertEqual(self.client.post('/api/alignment/save',json={},headers={**self.headers,"Origin":"http://evil.test"}).status_code,403)
        self.assertEqual(self.post('preview',{"device":"../bad"}).status_code,400)
        self.assertEqual(self.post('save',{"expected_revision":False}).status_code,400)
        from alignment_api import MAX_UPLOAD
        self.assertEqual(self.client.post('/api/alignment/replay',content=b'x'*(MAX_UPLOAD+1),headers=self.headers).status_code,413)

    def test_log_api_missing_time_not_invented(self):
        r=self.post('replay',{"format":"receiver_log","content":(ROOT/'samples/radar_3s.log').read_text(),
            "context":CONTEXT,"device":"radar_3152","params":{"time_source":"header"}})
        self.assertEqual(r.status_code,200)
        self.assertEqual(r.json()["summary"]["records"],21)
        self.assertTrue(all(x["corrected_timestamp_ms"] is None for x in r.json()["rows"]))

    def test_gateway_integration_passive(self):
        from dds_ws_gateway import create_app
        class FakeBus:
            def read(self,*args,**kwargs): return []
            def read_bytes(self,*args,**kwargs): return [packet()] if args[0]=='RadarTarget' else []
            def write(self,*args,**kwargs): raise AssertionError('No DDS writes allowed')
        app=create_app(bus=FakeBus(),alignment_profile_path=self.path)
        asyncio.run(app.state.gateway.poll_once())
        self.assertEqual(app.state.gateway.alignment.packets,1)
        self.assertFalse(self.path.exists())


class AsgiClient:
    """Exercise actual ASGI request parsing without installing test-only HTTP packages."""
    def __init__(self, app): self.app=app

    def get(self,path): return self.request('GET',path)

    def post(self,path,**kwargs): return self.request('POST',path,**kwargs)

    def request(self,method,path,json=None,content=None,headers=None):
        import json as codec
        async def run():
            body=content if content is not None else codec.dumps(json).encode() if json is not None else b''
            response=[]
            async def receive(): return {'type':'http.request','body':body,'more_body':False}
            async def send(message): response.append(message)
            scope={'type':'http','asgi':{'version':'3.0'},'http_version':'1.1','method':method,
                'scheme':'http','path':path,'raw_path':path.encode(),'query_string':b'',
                'root_path':'','server':('testserver',80),'client':('127.0.0.1',10000),
                'headers':[(b'host',b'testserver'),(b'content-type',b'application/json')]+
                    [(k.lower().encode(),v.encode()) for k,v in (headers or {}).items()]}
            await self.app(scope,receive,send)
            status=next(m['status'] for m in response if m['type']=='http.response.start')
            data=b''.join(m.get('body',b'') for m in response if m['type']=='http.response.body')
            return SimpleNamespace(status_code=status,json=lambda:codec.loads(data))
        return asyncio.run(run())


if __name__ == '__main__':
    unittest.main()
