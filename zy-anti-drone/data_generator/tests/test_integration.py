"""集成测试 — 测试完整的数据流。"""

import sys
sys.path.insert(0, 'src')

import time
import numpy as np

from scenario.config import load_scenario
from scenario.controller import SimulationController
from truth.trajectory import WaypointTrajectory, Waypoint, TrajectoryState


def test_config_loading():
    """测试配置加载。"""
    config = load_scenario('config/scenarios/single_target.yaml')

    assert config.name == "单目标基础场景"
    assert len(config.targets) == 1
    assert config.targets[0].label == "DJI-Mavic3"
    assert config.radar.update_rate == 10.0
    assert config.eo.update_rate == 30.0
    assert config.esm.update_rate == 1.0

    print("[PASS] test_config_loading")


def test_controller_init():
    """测试控制器初始化。"""
    controller = SimulationController('config/scenarios/single_target.yaml')

    assert len(controller.targets) == 1
    assert controller.get_duration() == 120.0
    assert controller.get_dt() == 0.05

    controller.shutdown()
    print("[PASS] test_controller_init")


def test_simulation_step():
    """测试仿真步进。"""
    controller = SimulationController('config/scenarios/single_target.yaml')

    # 推进几步
    for _ in range(10):
        result = controller.step()

    assert 'time' in result
    assert 'truth' in result
    assert 'radar_detections' in result
    assert 'eo_detections' in result
    assert 'esm_detections' in result

    # 真值应该包含目标
    assert len(result['truth']) == 1

    controller.shutdown()
    print("[PASS] test_simulation_step")


def test_trajectory_sampling():
    """测试轨迹采样 — 验证真值数据的合理性。"""
    from truth.target import Target, EmitterParams

    waypoints = [
        Waypoint(1000, 500, 100, 0),
        Waypoint(800, 400, 80, 30),
        Waypoint(200, 100, 40, 90),
    ]
    traj = WaypointTrajectory(waypoints)
    target = Target(
        id=1, label="Test", trajectory=traj,
        emitter=EmitterParams(),
    )

    # 采样整个轨迹
    states = []
    for t in np.arange(0, 90, 1.0):
        state = target.get_state(t)
        if state:
            states.append(state)

    assert len(states) == 90
    assert all(s.speed >= 0 for s in states)
    assert all(len(s.position) == 3 for s in states)

    print(f"[PASS] test_trajectory_sampling ({len(states)} points)")


def test_shared_memory():
    """测试共享内存读写。"""
    from bus.shm_pool import RingBuffer

    buf = RingBuffer("test_integration", capacity=10, item_size=256, create=True)

    # 写入
    for i in range(5):
        buf.write(f"message_{i}".encode())

    # 读取
    for i in range(5):
        data = buf.read(i)
        assert data is not None
        assert data.decode() == f"message_{i}"

    buf.close()
    buf.unlink()
    print("[PASS] test_shared_memory")


if __name__ == '__main__':
    test_config_loading()
    test_controller_init()
    test_simulation_step()
    test_trajectory_sampling()
    test_shared_memory()
    print("\n✅ 所有集成测试通过")
