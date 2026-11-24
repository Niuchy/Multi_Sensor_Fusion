"""Root-level ARM64/x86_64 launcher for the read-only fusion package."""

from __future__ import annotations

import argparse
import importlib.util
import os
import platform
import runpy
import signal
import struct
import subprocess
import sys
import time
from pathlib import Path
from urllib.parse import urlsplit


ROOT = Path(__file__).resolve().parent
DDS_DIR = ROOT / "dds_swarm_fusion"
ZY_DIR = ROOT / "zy-anti-drone"
SIM_DIR = ZY_DIR / "data_generator"
ROOT_BRIDGE = ROOT / "libcompany_dds_bridge.so"
EXTERNAL_WS_SCRIPT = ROOT / "fused_tracks_ws_sender.py"
EXTERNAL_WS_SERVER_SCRIPT = ROOT / "fused_tracks_ws_server.py"
NATIVE_ENV_MARKER = "SB_FUSION_NATIVE_ENV_READY"
PROJECT_ENV_LIB = ROOT / ".rk3588-env" / "lib"

FUSION_NODES = (
    ("company_input", "company_input_node.py"),
    ("tracking", "dds_tracking_node.py"),
    ("fusion", "dds_fusion_node.py"),
)

ELF_MACHINE = {"x86_64": 62, "aarch64": 183}


def normalize_machine(machine: str) -> str | None:
    machine = machine.lower()
    if machine in {"x86_64", "amd64"}:
        return "x86_64"
    if machine in {"aarch64", "arm64"}:
        return "aarch64"
    return None


def elf_machine(path: Path) -> str | None:
    try:
        with path.open("rb") as stream:
            header = stream.read(20)
    except OSError:
        return None
    if len(header) < 20 or header[:4] != b"\x7fELF":
        return None
    byte_order = "<" if header[5] == 1 else ">" if header[5] == 2 else None
    if byte_order is None:
        return None
    machine_id = struct.unpack(f"{byte_order}H", header[18:20])[0]
    for name, expected in ELF_MACHINE.items():
        if machine_id == expected:
            return name
    return f"elf-machine-{machine_id}"


def install_company_dds_override() -> None:
    """Load the original adapter while redirecting only its native bridge."""

    sys.path.insert(0, str(DDS_DIR))
    adapter_path = DDS_DIR / "company_dds.py"
    spec = importlib.util.spec_from_file_location("company_dds", adapter_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载融合DDS适配器: {adapter_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["company_dds"] = module
    spec.loader.exec_module(module)
    module._bridge_path = lambda: ROOT_BRIDGE


def run_fusion_node(script_name: str, script_args: list[str]) -> int:
    script = DDS_DIR / script_name
    if not script.is_file():
        raise RuntimeError(f"融合节点不存在: {script}")
    install_company_dds_override()
    sys.argv = [str(script), *script_args]
    runpy.run_path(str(script), run_name="__main__")
    return 0


def run_external_ws_sender(script_args: list[str]) -> int:
    if not EXTERNAL_WS_SCRIPT.is_file():
        raise RuntimeError(f"外部WebSocket发送脚本不存在: {EXTERNAL_WS_SCRIPT}")
    install_company_dds_override()
    sys.argv = [str(EXTERNAL_WS_SCRIPT), *script_args]
    runpy.run_path(str(EXTERNAL_WS_SCRIPT), run_name="__main__")
    return 0


def run_external_ws_server(script_args: list[str]) -> int:
    if not EXTERNAL_WS_SERVER_SCRIPT.is_file():
        raise RuntimeError(
            f"外部WebSocket服务脚本不存在: {EXTERNAL_WS_SERVER_SCRIPT}"
        )
    install_company_dds_override()
    sys.argv = [str(EXTERNAL_WS_SERVER_SCRIPT), *script_args]
    runpy.run_path(str(EXTERNAL_WS_SERVER_SCRIPT), run_name="__main__")
    return 0


def run_company_simulator(scenario: Path) -> int:
    """Run the company simulator in real time without changing its package."""

    sys.path.insert(0, str(SIM_DIR / "src"))
    from scenario.controller import SimulationController

    controller = SimulationController(str(scenario))
    if not controller.enable_output():
        raise RuntimeError(
            "公司仿真DDS输出初始化失败；请检查 zy-anti-drone/bridge/"
            "libad_proto.so 和 ARM64依赖。"
        )

    dt = controller.get_dt()
    duration = controller.get_duration()
    steps = int(duration / dt)
    print(
        f"公司仿真已启动: {scenario.name}, duration={duration}s, "
        f"dt={dt}s, steps={steps}",
        flush=True,
    )

    try:
        for index in range(steps):
            wall_started = time.monotonic()
            result = controller.step()
            if index % 40 == 39:
                print(
                    f"[company_sim] T={result['time']:.1f}s "
                    f"radar={len(result['radar_detections'])} "
                    f"eo={len(result['eo_detections'])} "
                    f"esm={len(result['esm_detections'])} "
                    f"acoustic={len(result.get('acoustic_detections', []))}",
                    flush=True,
                )
            elapsed = time.monotonic() - wall_started
            speed = max(float(getattr(controller, "speed", 1.0)), 0.01)
            time.sleep(max(0.0, dt / speed - elapsed))
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
    return 0


def validate_runtime(
    machine: str, platform_dir: str, *, require_simulator: bool
) -> None:
    required_dirs = (DDS_DIR, SIM_DIR, ZY_DIR / "bridge")
    for directory in required_dirs:
        if not directory.is_dir():
            raise RuntimeError(f"缺少目录: {directory}")

    bridges = [("融合Python桥接库", ROOT_BRIDGE)]
    if require_simulator:
        bridges.append(
            ("公司仿真桥接库", ZY_DIR / "bridge" / "libad_proto.so")
        )
    for label, bridge in bridges:
        actual = elf_machine(bridge)
        if actual != machine:
            raise RuntimeError(
                f"{label}架构错误: {bridge}，检测到 "
                f"{actual or '不存在/非ELF'}，需要 {machine}。"
                "请先运行 ./build_native.sh。"
            )

    library_dir = ZY_DIR / "thirdparty" / platform_dir / "lib"
    for name in (
        "libPubSubTools.so",
        "libcore.so",
        "libfastrtps.so",
        "libfastcdr.so",
    ):
        if not (library_dir / name).exists():
            raise RuntimeError(f"缺少公司DDS库: {library_dir / name}")


def child_command(*args: str) -> list[str]:
    return [sys.executable, str(Path(__file__).resolve()), *args]


def native_runtime_env(environ=None) -> dict[str, str]:
    """Build an environment that loads the matching company OpenSSL pair."""
    env = dict(os.environ if environ is None else environ)
    machine = normalize_machine(platform.machine())
    if platform.system() != "Linux" or machine != "aarch64":
        return env

    library_dir = ZY_DIR / "thirdparty/linux-arm64/lib"
    crypto = library_dir / "libcrypto.so.3"
    ssl = library_dir / "libssl.so.3"
    if not crypto.is_file() or not ssl.is_file():
        return env

    existing_preload = [
        entry for entry in env.get("LD_PRELOAD", "").split(":") if entry
    ]
    company_preload = [str(crypto), str(ssl)]
    env["LD_PRELOAD"] = ":".join(
        company_preload
        + [entry for entry in existing_preload if entry not in company_preload]
    )
    runtime_dirs = [str(library_dir)]
    if PROJECT_ENV_LIB.is_dir():
        runtime_dirs.append(str(PROJECT_ENV_LIB))
    existing_ld_path = [
        entry for entry in env.get("LD_LIBRARY_PATH", "").split(":") if entry
    ]
    env["LD_LIBRARY_PATH"] = ":".join(
        runtime_dirs
        + [entry for entry in existing_ld_path if entry not in runtime_dirs]
    )
    env["OPENSSL_MODULES"] = str(library_dir / "ossl-modules")
    env[NATIVE_ENV_MARKER] = "1"
    return env


def ensure_native_runtime(argv: list[str], *, entrypoint: Path | None = None) -> None:
    """Re-exec once so LD_PRELOAD takes effect before Conda loads libcrypto."""
    if os.environ.get(NATIVE_ENV_MARKER) == "1":
        return
    env = native_runtime_env()
    if env.get(NATIVE_ENV_MARKER) != "1":
        return
    os.execve(
        sys.executable,
        [sys.executable, str(entrypoint or Path(__file__).resolve()), *argv],
        env,
    )


def add_no_proxy(env: dict[str, str], hostname: str | None) -> None:
    if not hostname:
        return
    values = []
    for key in ("NO_PROXY", "no_proxy"):
        values.extend(
            value.strip()
            for value in env.get(key, "").split(",")
            if value.strip()
        )
    for value in (hostname, "127.0.0.1", "localhost", "::1"):
        if value not in values:
            values.append(value)
    joined = ",".join(values)
    env["NO_PROXY"] = joined
    env["no_proxy"] = joined


def orchestrate(args: argparse.Namespace) -> int:
    system = platform.system()
    machine = normalize_machine(platform.machine())
    if system != "Linux" or machine is None:
        raise RuntimeError(
            f"整栈仅支持 Linux/aarch64 或 Linux/x86_64，"
            f"当前为 {system}/{platform.machine()}。"
        )
    platform_dir = "linux-arm64" if machine == "aarch64" else "linux-x86"
    validate_runtime(
        machine,
        platform_dir,
        require_simulator=not args.external_simulator,
    )

    scenario = Path(args.scenario).expanduser().resolve()
    if not scenario.is_file():
        raise RuntimeError(f"场景文件不存在: {scenario}")

    library_dir = ZY_DIR / "thirdparty" / platform_dir / "lib"
    child_env = os.environ.copy()
    if args.external_simulator:
        child_env.setdefault("SB_RADAR_INPUT_MODE", "device_tracks")
        child_env.setdefault("SB_AZIMUTH_CONVENTION", "north_cw")
    previous_ld_path = child_env.get("LD_LIBRARY_PATH")
    runtime_dirs = [str(library_dir)]
    if PROJECT_ENV_LIB.is_dir():
        runtime_dirs.append(str(PROJECT_ENV_LIB))
    child_env["LD_LIBRARY_PATH"] = ":".join(
        runtime_dirs + ([previous_ld_path] if previous_ld_path else [])
    )
    child_env["PYTHONUNBUFFERED"] = "1"
    if args.external_ws_url:
        add_no_proxy(child_env, urlsplit(args.external_ws_url).hostname)

    processes: list[tuple[str, subprocess.Popen]] = []

    def stop_processes() -> None:
        for _, process in processes:
            if process.poll() is None:
                process.send_signal(signal.SIGTERM)
        for _, process in processes:
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()

    try:
        for name, script in FUSION_NODES:
            command = child_command("_node", script)
            if name == "company_input":
                command.extend(["--scenario", str(scenario)])
            print("+", " ".join(command), flush=True)
            processes.append(
                (
                    name,
                    subprocess.Popen(
                        command, cwd=DDS_DIR, env=child_env
                    ),
                )
            )

        gateway_command = child_command(
            "_node",
            "dds_ws_gateway.py",
            "--host",
            args.host,
            "--port",
            str(args.port),
        )
        print("+", " ".join(gateway_command), flush=True)
        processes.append(
            (
                "gateway",
                subprocess.Popen(
                    gateway_command, cwd=DDS_DIR, env=child_env
                ),
            )
        )

        if args.external_ws_url:
            external_ws_command = child_command(
                "_external_ws",
                "--url",
                args.external_ws_url,
            )
            print("+", " ".join(external_ws_command), flush=True)
            processes.append(
                (
                    "external_ws",
                    subprocess.Popen(
                        external_ws_command, cwd=ROOT, env=child_env
                    ),
                )
            )

        if args.external_ws_server:
            external_ws_server_command = child_command(
                "_external_ws_server",
                "--host",
                args.external_ws_host,
                "--port",
                str(args.external_ws_port),
                "--path",
                args.external_ws_path,
            )
            print("+", " ".join(external_ws_server_command), flush=True)
            processes.append(
                (
                    "external_ws_server",
                    subprocess.Popen(
                        external_ws_server_command, cwd=ROOT, env=child_env
                    ),
                )
            )

        if not args.external_simulator:
            time.sleep(1.0)
            for name, process in processes:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"{name} 启动失败，退出码 {process.returncode}"
                    )
            simulator_command = child_command(
                "_simulator", str(scenario)
            )
            print("+", " ".join(simulator_command), flush=True)
            processes.append(
                (
                    "company_sim",
                    subprocess.Popen(
                        simulator_command, cwd=SIM_DIR, env=child_env
                    ),
                )
            )

        external_ws_mode = (
            f"push:{args.external_ws_url}"
            if args.external_ws_url
            else (
                f"serve:{args.external_ws_host}:"
                f"{args.external_ws_port}{args.external_ws_path}"
                if args.external_ws_server
                else "disabled"
            )
        )
        print(
            f"SB Fusion started: http://{args.host}:{args.port} "
            f"(platform={platform_dir}, "
            f"external_ws={external_ws_mode})",
            flush=True,
        )
        while True:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    if name == "company_sim" and code == 0:
                        print(
                            "公司仿真场景已完成，正在停止 SB Fusion...",
                            flush=True,
                        )
                        return 0
                    raise RuntimeError(f"{name} 已退出，退出码 {code}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n正在停止 SB Fusion...", flush=True)
    finally:
        stop_processes()
    return 0


def parse_public_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="从 sb_fusion 根目录启动公司DDS融合整栈"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument(
        "--scenario",
        default=None,
        help="设备布局/坐标配置；实装默认 configs/equipment.yaml",
    )
    parser.add_argument("--equipment", "--external-simulator", dest="external_simulator",
                        action="store_true", help="只接收装备 DDS，不启动内置仿真器")
    parser.add_argument(
        "--external-ws-url",
        default=os.environ.get("SB_FUSION_EXTERNAL_WS_URL"),
        help="可选：将融合结果推送到该 ws:// 或 wss:// 地址",
    )
    parser.add_argument(
        "--external-ws-server",
        action="store_true",
        help="在本机提供只读融合结果WebSocket服务，供对方主动连接",
    )
    parser.add_argument("--external-ws-host", default="0.0.0.0")
    parser.add_argument("--external-ws-port", type=int, default=11409)
    parser.add_argument("--external-ws-path", default="/ws/main")
    args = parser.parse_args(argv)
    if args.scenario is None:
        args.scenario = str(ROOT / "configs/equipment.yaml" if args.external_simulator
                            else SIM_DIR / "config/scenarios/multi_target.yaml")
    if args.external_ws_server and args.external_ws_url:
        parser.error(
            "--external-ws-server 与 --external-ws-url 不能同时使用"
        )
    if not 1 <= args.external_ws_port <= 65535:
        parser.error("--external-ws-port 必须在 1~65535 之间")
    if not args.external_ws_path.startswith("/"):
        args.external_ws_path = f"/{args.external_ws_path}"
    return args


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    ensure_native_runtime(argv)
    try:
        if argv and argv[0] == "_node":
            if len(argv) < 2:
                raise RuntimeError("_node 缺少脚本名")
            return run_fusion_node(argv[1], argv[2:])
        if argv and argv[0] == "_simulator":
            if len(argv) != 2:
                raise RuntimeError("_simulator 缺少场景路径")
            return run_company_simulator(Path(argv[1]).resolve())
        if argv and argv[0] in {"_external_ws", "external-ws"}:
            return run_external_ws_sender(argv[1:])
        if argv and argv[0] in {
            "_external_ws_server",
            "external-ws-server",
        }:
            return run_external_ws_server(argv[1:])
        if argv and argv[0] == "inspect":
            return run_fusion_node("dds_inspect.py", argv[1:])
        if argv and argv[0] == "replay-radar":
            script = ROOT / "tools/replay_radar_dds.py"
            sys.path.insert(0, str(script.parent))
            sys.argv = [str(script), *argv[1:]]
            runpy.run_path(str(script), run_name="__main__")
            return 0
        return orchestrate(parse_public_args(argv))
    except RuntimeError as exc:
        print(f"错误: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
