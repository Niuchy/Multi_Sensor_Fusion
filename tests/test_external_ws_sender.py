import asyncio
import json
import math
import unittest

from fused_tracks_ws_sender import (
    FusionSnapshotState,
    ExternalWebSocketPublisher,
    TRACK_COASTING,
    TRACK_CONFIRMED,
    TRACK_DELETED,
    format_track,
    target_type_for,
)


class ExternalTrackFormattingTests(unittest.TestCase):
    def test_formats_required_schema_and_enu_coordinates(self):
        ref_lng, ref_lat, ref_alt = 116.0, 39.0, 100.0
        track = {
            "id": 123,
            "x": 111320.0 * math.cos(math.radians(ref_lat)) * 0.001,
            "y": 111.32,
            "z": 400.0,
            "vx": 2.3,
            "vy": 1.5,
            "vz": -0.7,
            "conf": 0.85,
            "cls": "drone",
            "class_evidence": True,
            "sensor_ids": ["radar_1", "rf_df_2"],
            "age_ticks": 0,
            "rf_freq": 2_400_000_000,
            "sn_id": "SN123456",
            "model": "DJI-M300",
        }

        row = format_track(
            track,
            origin=[ref_lng, ref_lat, ref_alt],
            timestamp_ms=1719500000123,
        )

        self.assertEqual(row["trackId"], 123)
        self.assertAlmostEqual(row["lat"], 39.001, places=7)
        self.assertAlmostEqual(row["lon"], 116.001, places=7)
        self.assertEqual(row["alt"], 500.0)
        self.assertEqual(row["vNorth"], 1.5)
        self.assertEqual(row["vEast"], 2.3)
        self.assertEqual(row["vUp"], -0.7)
        self.assertAlmostEqual(row["speed"], math.hypot(2.3, 1.5))
        self.assertAlmostEqual(
            row["heading"], math.degrees(math.atan2(2.3, 1.5))
        )
        self.assertEqual(row["targetType"], 2)
        self.assertEqual(row["sensorIds"], ["esm_2", "radar_1"])
        self.assertEqual(row["status"], TRACK_CONFIRMED)
        self.assertEqual(row["updateTime"], 1719500000123)
        self.assertEqual(row["rf_freq"], 2_400_000_000.0)
        self.assertIsNone(row["rcs"])
        self.assertEqual(row["sn_id"], "SN123456")
        self.assertEqual(row["model"], "DJI-M300")

    def test_missing_vertical_velocity_is_null(self):
        row = format_track(
            {
                "id": 1,
                "x": 0.0,
                "y": 0.0,
                "z": 20.0,
                "vx": 1.0,
                "vy": 2.0,
                "conf": 0.5,
            },
            origin=[116.0, 39.0, 100.0],
            timestamp_ms=1000,
        )
        self.assertIsNone(row["vUp"])

    def test_unclassified_default_is_unknown(self):
        self.assertEqual(
            target_type_for({"cls": "drone", "class_evidence": False}),
            0,
        )
        self.assertEqual(
            target_type_for({"cls": "bird", "class_evidence": True}), 1
        )
        self.assertEqual(
            target_type_for({"cls": "large_target", "class_evidence": True}),
            5,
        )
        self.assertEqual(
            target_type_for({"cls": "other", "class_evidence": True}), 6
        )

    def test_snapshot_outputs_coasting_then_minimal_delete_once(self):
        state = FusionSnapshotState()
        state.ingest({
            "tick": 10,
            "source_timestamp_ms": 1000,
            "coordinates": {"origin": [116.0, 39.0, 0.0]},
            "external_tracks": [{
                "id": 7,
                "x": 0,
                "y": 0,
                "z": 10,
                "vx": 0,
                "vy": 0,
                "conf": 0.4,
                "cls": "unknown",
                "class_evidence": True,
                "age_ticks": 2,
            }],
        })
        first, first_deleted, _ = state.render()
        self.assertEqual(first["data"][0]["status"], TRACK_COASTING)
        self.assertEqual(first_deleted, set())

        state.ingest({
            "tick": 11,
            "source_timestamp_ms": 1500,
            "coordinates": {"origin": [116.0, 39.0, 0.0]},
            "external_tracks": [],
        })
        second, deleted_ids, _ = state.render()
        self.assertEqual(deleted_ids, {7})
        self.assertEqual(
            second["data"],
            [{"trackId": 7, "status": TRACK_DELETED, "updateTime": 1500}],
        )

        state.acknowledge_deletions(deleted_ids)
        third, _, _ = state.render()
        self.assertEqual(third["data"], [])


class ExternalWebSocketTransportTests(unittest.IsolatedAsyncioTestCase):
    async def test_sends_latest_snapshot_to_real_websocket_server(self):
        import websockets

        received = asyncio.get_running_loop().create_future()

        async def handler(websocket, *unused):
            if not received.done():
                received.set_result(json.loads(await websocket.recv()))

        server = await websockets.serve(handler, "127.0.0.1", 0)
        port = server.sockets[0].getsockname()[1]
        publisher = ExternalWebSocketPublisher(
            f"ws://127.0.0.1:{port}/ws/main", bus=object()
        )
        publisher.state.ingest({
            "tick": 1,
            "source_timestamp_ms": 1000,
            "coordinates": {"origin": [116.0, 39.0, 0.0]},
            "external_tracks": [{
                "id": 9,
                "x": 0,
                "y": 0,
                "z": 10,
                "vx": 1,
                "vy": 2,
                "conf": 0.7,
                "cls": "drone",
                "class_evidence": True,
                "age_ticks": 0,
            }],
        })

        task = asyncio.create_task(publisher.publish())
        try:
            envelope = await asyncio.wait_for(received, timeout=3)
            self.assertEqual(envelope["type"], "Ele")
            self.assertEqual(envelope["module"], "DataFusion")
            self.assertEqual(envelope["interface"], "FusedTracks")
            self.assertEqual(envelope["data"][0]["trackId"], 9)
        finally:
            task.cancel()
            with self.assertRaises(asyncio.CancelledError):
                await task
            server.close()
            await server.wait_closed()


if __name__ == "__main__":
    unittest.main()
