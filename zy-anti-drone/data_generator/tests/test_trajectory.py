"""轨迹模型测试。"""

import sys
sys.path.insert(0, 'src')

import numpy as np
from truth.trajectory import Waypoint, WaypointTrajectory, LinearTrajectory


def test_waypoint_trajectory():
    """测试三次样条轨迹。"""
    waypoints = [
        Waypoint(0, 0, 0, 0),
        Waypoint(100, 0, 50, 10),
        Waypoint(200, 100, 100, 20),
        Waypoint(300, 200, 50, 30),
    ]
    traj = WaypointTrajectory(waypoints)

    # 测试时长
    assert traj.get_duration() == 30.0

    # 测试中间时刻状态
    state = traj.get_state(15.0)
    assert state.position.shape == (3,)
    assert state.velocity.shape == (3,)
    assert state.speed >= 0

    # 测试起始点
    state0 = traj.get_state(0)
    np.testing.assert_allclose(state0.position, [0, 0, 0], atol=1e-6)

    # 测试终点
    state_end = traj.get_state(30)
    np.testing.assert_allclose(state_end.position, [300, 200, 50], atol=1e-2)

    # 测试越界
    state_over = traj.get_state(35)
    np.testing.assert_allclose(state_over.position, [300, 200, 50], atol=1e-2)

    print("[PASS] test_waypoint_trajectory")


def test_linear_trajectory():
    """测试线性插值轨迹。"""
    waypoints = [
        Waypoint(0, 0, 0, 0),
        Waypoint(100, 0, 0, 10),
    ]
    traj = LinearTrajectory(waypoints)

    # 中点应在 (50, 0, 0)
    state = traj.get_state(5.0)
    np.testing.assert_allclose(state.position, [50, 0, 0], atol=0.1)

    # 速度应为 (10, 0, 0)
    np.testing.assert_allclose(state.velocity, [10, 0, 0], atol=0.1)

    print("[PASS] test_linear_trajectory")


def test_sample():
    """测试采样。"""
    waypoints = [
        Waypoint(0, 0, 0, 0),
        Waypoint(10, 0, 0, 5),
    ]
    traj = WaypointTrajectory(waypoints)
    samples = traj.sample(1.0)
    assert len(samples) == 5
    print("[PASS] test_sample")


def test_from_waypoint_lists():
    """测试从航点列表创建。"""
    wp_data = [[0, 0, 0, 0], [10, 0, 0, 5]]
    traj = WaypointTrajectory.from_waypoint_lists(wp_data)
    assert traj.get_duration() == 5.0
    print("[PASS] test_from_waypoint_lists")


if __name__ == '__main__':
    test_waypoint_trajectory()
    test_linear_trajectory()
    test_sample()
    test_from_waypoint_lists()
    print("\n✅ 所有轨迹测试通过")
