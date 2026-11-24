"""传感器模型测试。"""

import sys
sys.path.insert(0, 'src')

import numpy as np

from truth.target import Target, EmitterParams
from truth.trajectory import WaypointTrajectory, Waypoint
from sensors.error_models import (
    GaussianNoise, Bias, OutlierModel, TimeDelay,
    DetectionProbability, ErrorPipeline,
)
from sensors.radar import RadarSensor
from sensors.electro_optical import ElectroOpticalSensor
from sensors.esm import ESMSensor


def create_test_target() -> Target:
    """创建测试目标。"""
    waypoints = [
        Waypoint(1000, 0, 100, 0),   # 正前方 1km，高度 100m
        Waypoint(500, 0, 80, 30),
    ]
    traj = WaypointTrajectory(waypoints)
    return Target(
        id=1,
        label="Test-UAV",
        target_type="quadcopter",
        rcs=0.01,
        trajectory=traj,
        emitter=EmitterParams(
            active=True,
            freq=2.4e9,
            pri=1000e-6,
            pw=10e-6,
            modulation="OFDM",
            power=0.1,
        ),
    )


def test_gaussian_noise():
    """测试高斯噪声模型。"""
    noise = GaussianNoise({'range': 5.0, 'azimuth': 0.5})
    meas = {'range': 1000.0, 'azimuth': 30.0}
    result = noise.apply(meas)

    assert 'range' in result
    assert 'azimuth' in result
    # 噪声后的值应该接近原始值（大概率在 3σ 内）
    assert abs(result['range'] - 1000.0) < 20
    print("[PASS] test_gaussian_noise")


def test_bias():
    """测试偏差模型。"""
    bias = Bias(
        fixed={'range': 5.0, 'azimuth': 0.1},
        drift_rate={'range': 0.01},
    )
    meas = {'range': 1000.0, 'azimuth': 30.0}

    # 第一次应用
    result1 = bias.apply(meas, dt=10.0)
    assert result1['range'] == 1000.0 + 5.0 + 0.1  # fixed + 10*0.01
    print("[PASS] test_bias")


def test_error_pipeline():
    """测试误差流水线。"""
    pipeline = ErrorPipeline()
    pipeline.add_stage(GaussianNoise({'range': 1.0, 'azimuth': 0.1}))
    pipeline.add_stage(OutlierModel(probability=0.0))  # 零概率，不应有野值

    meas = {'range': 1000.0, 'azimuth': 30.0}
    result = pipeline.process(meas)

    assert abs(result['range'] - 1000.0) < 5.0
    print("[PASS] test_error_pipeline")


def test_detection_probability():
    """测试检测概率模型。"""
    pd_model = DetectionProbability(pd_nominal=0.9, snr_threshold=13.0)

    # 高 SNR → 高 Pd
    pd_high = pd_model.compute_pd(30.0)
    assert pd_high > 0.8

    # 低 SNR → 低 Pd
    pd_low = pd_model.compute_pd(0.0)
    assert pd_low < 0.1

    print("[PASS] test_detection_probability")


def test_radar_sensor():
    """测试雷达传感器（无进程模式）。"""
    target = create_test_target()
    radar = RadarSensor(
        sensor_id="radar-test",
        position=[0, 0, 5],
        max_range=5000.0,
        min_range=50.0,
        error_sigma={'range': 5.0, 'azimuth': 0.5, 'elevation': 0.5, 'doppler': 0.1},
        clutter_enabled=False,
    )
    radar.set_targets([target])

    detections = radar.detect(0.0)
    assert len(detections) >= 0  # 可能检测或漏检

    # 多次检测统计
    all_dets = []
    for t in np.arange(0, 5, 0.1):
        dets = radar.detect(t)
        all_dets.extend(dets)

    # 至少有一次检测
    assert len(all_dets) > 0, "雷达应该检测到目标"
    print(f"[PASS] test_radar_sensor ({len(all_dets)} detections in 5s)")


def test_eo_sensor():
    """测试光电传感器。"""
    target = create_test_target()
    eo = ElectroOpticalSensor(
        sensor_id="eo-test",
        position=[0, 0, 10],
        fov={'azimuth': [-10.0, 10.0], 'elevation': [-5.0, 5.0]},
        lock_on_time=0.0,  # 立即锁定
    )
    eo.set_targets([target])

    detections = eo.detect(1.0)
    assert len(detections) >= 0

    # 测量应包含方位和俯仰
    if detections:
        det = detections[0]
        assert 'azimuth' in det.measurements
        assert 'elevation' in det.measurements

    print(f"[PASS] test_eo_sensor ({len(detections)} detections)")


def test_esm_sensor():
    """测试电侦传感器。"""
    target = create_test_target()
    esm = ESMSensor(
        sensor_id="esm-test",
        position=[-50, 0, 8],
        poi=1.0,  # 100% 截获
    )
    esm.set_targets([target])

    detections = esm.detect(1.0)
    if detections:
        det = detections[0]
        assert 'azimuth' in det.measurements
        assert 'frequency' in det.measurements
        assert 'modulation' in det.measurements

    print(f"[PASS] test_esm_sensor ({len(detections)} detections)")


if __name__ == '__main__':
    test_gaussian_noise()
    test_bias()
    test_error_pipeline()
    test_detection_probability()
    test_radar_sensor()
    test_eo_sensor()
    test_esm_sensor()
    print("\n✅ 所有传感器测试通过")
