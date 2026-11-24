import asyncio
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import unittest
from http.server import BaseHTTPRequestHandler, HTTPServer
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
import run_sb_fusion as launcher
from replay_radar_dds import get_diagnostics, websocket_options, check_connections, inspect_official_packet
from replay_radar_log import MemoryBus, replay
from company_input_node import CompanyInputNode


class RkRuntimeTests(unittest.TestCase):
    def test_external_ws_defaults_to_nonconflicting_port(self):
        from fused_tracks_ws_server import parse_args
        self.assertEqual(launcher.parse_public_args(["--equipment"]).external_ws_port, 11409)
        self.assertEqual(parse_args([]).port, 11409)

    def test_equipment_alias_and_profile(self):
        for flag in ("--equipment", "--external-simulator"):
            args = launcher.parse_public_args([flag])
            self.assertTrue(args.external_simulator)
            self.assertEqual(Path(args.scenario), ROOT / "configs/equipment.yaml")
        self.assertFalse(launcher.parse_public_args([]).external_simulator)
        self.assertIn("multi_target.yaml", launcher.parse_public_args([]).scenario)
        self.assertEqual(launcher.parse_public_args(["--equipment", "--scenario", "site.yaml"]).scenario, "site.yaml")

    def test_live_profile_does_not_preseed_demo_devices(self):
        node = CompanyInputNode(ROOT / "configs/equipment.yaml", bus=MemoryBus())
        self.assertEqual(node.sensors_info, {})
        sid, info = node._sensor("radar", 3152)
        self.assertEqual(sid, "radar_3152")
        self.assertEqual(info["pos"], [0, 0, 0])
        self.assertEqual(info["source_topic"], "RadarTarget")
        self.assertEqual(list(node.sensors_info), ["radar_3152"])

    @patch.dict(os.environ, {"SB_FUSION_MIN_CONFIDENCE": "0.2"})
    def test_supplied_log_with_equipment_profile(self):
        result = replay(ROOT / "samples/radar_3s.log", scenario=ROOT / "configs/equipment.yaml",
                        radar_input_mode="device_tracks", azimuth_convention="north_cw")
        self.assertEqual(result["records"], 21)
        self.assertEqual(result["frames_with_external_output"], 21)
        self.assertEqual(result["frames_with_fused_output"], 0)
        self.assertEqual(len({i for f in result["frames"] for i in f["external_ids"]}), 1)

    def test_arm64_environment_keeps_user_library_paths(self):
        with patch.object(launcher.platform, "machine", return_value="aarch64"), \
             patch.object(launcher.platform, "system", return_value="Linux"):
            env = launcher.native_runtime_env({"LD_LIBRARY_PATH": "/site/lib", "LD_PRELOAD": "/site/extra.so"})
        self.assertEqual(env[launcher.NATIVE_ENV_MARKER], "1")
        self.assertIn("linux-arm64/lib/libcrypto.so.3", env["LD_PRELOAD"])
        self.assertTrue(env["LD_PRELOAD"].endswith("/site/extra.so"))
        self.assertTrue(env["LD_LIBRARY_PATH"].endswith("/site/lib"))

    def test_direct_replay_reexec_preserves_its_entrypoint(self):
        script = ROOT / "tools/replay_radar_dds.py"
        with patch.dict(os.environ, {}, clear=True), \
             patch.object(launcher, "native_runtime_env", return_value={launcher.NATIVE_ENV_MARKER: "1"}), \
             patch.object(launcher.os, "execve") as execute:
            launcher.ensure_native_runtime(["1.log", "--publish"], entrypoint=script)
        self.assertEqual(execute.call_args.args[1], [sys.executable, str(script), "1.log", "--publish"])

    def test_equipment_shell_forwards_arguments_and_mode(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            shutil.copyfile(ROOT / "start_equipment_fusion.sh", tmp / "start_equipment_fusion.sh")
            (tmp / "start_sb_fusion.sh").write_text(
                '#!/bin/bash\nprintf "MODE=%s AZ=%s WS=%s\\n" "$SB_RADAR_INPUT_MODE" "$SB_AZIMUTH_CONVENTION" "$EXTERNAL_WS_SERVER"\nprintf "ARG=%s\\n" "$@"\n')
            env = {k:v for k,v in os.environ.items() if not k.startswith(("EXTERNAL_WS", "SB_FUSION", "SB_RADAR", "SB_AZIMUTH"))}
            result = subprocess.run(["bash", str(tmp / "start_equipment_fusion.sh"), "--scenario", "a path/site.yaml"],
                                    env=env, text=True, capture_output=True, check=True)
            self.assertIn("MODE=device_tracks AZ=north_cw WS=1", result.stdout)
            self.assertIn("ARG=--equipment\nARG=--scenario\nARG=a path/site.yaml", result.stdout)
            env["EXTERNAL_WS_URL"] = "ws://127.0.0.1:9999/ws"
            result = subprocess.run(["bash", str(tmp / "start_equipment_fusion.sh")],
                                    env=env, text=True, capture_output=True, check=True)
            self.assertIn("WS=0", result.stdout)

    def test_websocket_options_support_old_and_new_libraries(self):
        def old_connect(uri, *, open_timeout=10, **kwargs): pass
        def new_connect(uri, *, proxy=True, **kwargs): pass
        self.assertNotIn("proxy", websocket_options(old_connect))
        self.assertIsNone(websocket_options(new_connect)["proxy"])

    def test_start_script_builds_equipment_before_launch_and_stops_on_build_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            for name in ("start_equipment_fusion.sh", "start_sb_fusion.sh"):
                shutil.copyfile(ROOT / name, tmp / name)
            bin_dir = tmp / "fake-bin"
            bin_dir.mkdir()
            uname = bin_dir / "uname"
            uname.write_text('#!/bin/bash\nif [[ "$1" == "-s" ]]; then echo Linux; else echo aarch64; fi\n')
            uname.chmod(0o755)
            py = tmp / ".rk3588-env/bin/python"
            py.parent.mkdir(parents=True)
            py.write_text('#!/bin/bash\nif [[ "$1" == "-c" ]]; then exit 0; fi\nprintf "LAUNCH=%s\\n" "$@"\n')
            py.chmod(0o755)
            build = tmp / "build_native.sh"
            build.write_text('#!/bin/bash\nprintf "BUILD=%s\\n" "$@"\n')
            env = {k:v for k,v in os.environ.items() if not k.startswith(("EXTERNAL_WS", "SB_FUSION", "SB_RADAR", "SB_AZIMUTH", "PYTHON"))}
            env["PATH"] = str(bin_dir) + os.pathsep + os.environ.get("PATH", "")
            command = ["bash", str(tmp / "start_equipment_fusion.sh")]
            result = subprocess.run(command, env=env, text=True, capture_output=True, check=True)
            self.assertIn("BUILD=--equipment-only", result.stdout)
            self.assertIn("LAUNCH=--equipment", result.stdout)
            self.assertIn("LAUNCH=--external-ws-server", result.stdout)
            self.assertLess(result.stdout.index("BUILD="), result.stdout.index("LAUNCH="))
            build.write_text('#!/bin/bash\nexit 7\n')
            failed = subprocess.run(command, env=env, text=True, capture_output=True)
            self.assertEqual(failed.returncode, 7)
            self.assertNotIn("LAUNCH=", failed.stdout)

    def test_http_diagnostics_ignore_dead_proxy(self):
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                self.send_response(200)
                self.end_headers()
                self.wfile.write(b'{"direct":true}')
            def log_message(self, *args): pass
        server = HTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            with patch.dict(os.environ, {"http_proxy": "http://127.0.0.1:1", "HTTP_PROXY": "http://127.0.0.1:1", "no_proxy": "", "NO_PROXY": ""}):
                self.assertEqual(get_diagnostics(f"http://127.0.0.1:{server.server_port}"), {"direct": True})
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


class ProxyWebsocketTests(unittest.IsolatedAsyncioTestCase):
    async def test_check_only_ignores_proxy_without_creating_dds_bus(self):
        import websockets
        async def handler(ws, path=None):
            await ws.wait_closed()
        async with websockets.serve(handler, "127.0.0.1", 0) as server:
            port = server.sockets[0].getsockname()[1]
            args = SimpleNamespace(gateway=f"http://127.0.0.1:{port}", external_ws=f"ws://127.0.0.1:{port}/ws/main")
            with patch.dict(os.environ, {"https_proxy": "http://127.0.0.1:1", "HTTPS_PROXY": "http://127.0.0.1:1", "all_proxy": "http://127.0.0.1:1", "no_proxy": "", "NO_PROXY": ""}), \
                 patch("replay_radar_dds.get_diagnostics", return_value={"nodes": {n: {"alive": True} for n in ("input", "tracking", "fusion")}}), \
                 patch("replay_radar_dds.company_dds.CompanyDdsBus") as bus:
                result = await check_connections(args)
                self.assertEqual(result["dds_packets_sent"], 0)
                bus.assert_not_called()


class NativeWireTests(unittest.TestCase):
    def test_cpp_encoder_with_zero_one_and_255_targets(self):
        compiler = shutil.which("c++") or shutil.which("g++")
        if not compiler:
            self.skipTest("C++ compiler unavailable")
        with tempfile.TemporaryDirectory() as tmp:
            binary = str(Path(tmp) / "wire-probe")
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra",
                "-I" + str(ROOT / "dds_swarm_fusion/company_bridge"),
                "-I" + str(ROOT / "zy-anti-drone/include/common_msgs"),
                str(ROOT / "tests/company_wire_probe.cpp"), "-o", binary], check=True, capture_output=True, text=True)
            output = subprocess.check_output([binary], text=True)
        for count, line in zip((0, 1, 255), output.splitlines()):
            packet = bytes.fromhex(line)
            audit = inspect_official_packet(packet)
            self.assertTrue(audit["valid"], audit)
            self.assertEqual(audit["target_count"], count)
            self.assertEqual(audit["unit_timestamp_ms"], 1790000000123)
            self.assertEqual(struct.unpack_from("<H", packet, 12)[0], 784)
            if count == 0:
                self.assertEqual(len(packet), 41)
        self.assertEqual(len(output.splitlines()), 3)


if __name__ == "__main__":
    unittest.main()
