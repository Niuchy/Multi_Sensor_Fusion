"""配置加载与校验模块。

支持 YAML 配置文件的分层加载：场景文件 → 传感器子配置文件 → 默认值合并。
"""

import yaml
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# ──────────────────────────────────────────────
# 配置数据类
# ──────────────────────────────────────────────

@dataclass
class SimulationConfig:
    duration: float = 300.0
    dt: float = 0.05
    seed: int = 42
    start_time: float = 0.0


@dataclass
class FOVConfig:
    azimuth: list[float] = field(default_factory=lambda: [-60.0, 60.0])
    elevation: list[float] = field(default_factory=lambda: [-10.0, 30.0])


@dataclass
class RadarResolution:
    range: float = 10.0
    azimuth: float = 1.0
    elevation: float = 2.0


@dataclass
class RadarDetection:
    pd: float = 0.9
    pfa: float = 1.0e-6
    snr_threshold: float = 13.0


@dataclass
class RadarError:
    range_bias: float = 0.0
    range_sigma: float = 5.0
    azimuth_bias: float = 0.0
    azimuth_sigma: float = 0.5
    elevation_bias: float = 0.0
    elevation_sigma: float = 0.5
    doppler_sigma: float = 0.1


@dataclass
class ClutterConfig:
    enabled: bool = True
    density: float = 1.0e-4
    distribution: str = "uniform"


@dataclass
class JammerConfig:
    enabled: bool = False
    type: str = "noise"
    power: float = 0.0


@dataclass
class ScanConfig:
    type: str = "staring"            # "staring" / "mechanical"
    azimuth_rate: float = 60.0       # °/s, 机械扫描速率
    azimuth_beamwidth: float = 3.6   # °, 方位波束宽度
    scan_sector: list[float] = field(default_factory=lambda: [0.0, 360.0])  # 扫描扇区


@dataclass
class RadarConfig:
    id: str = "radar-01"
    position: list[float] = field(default_factory=lambda: [0.0, 0.0, 5.0])
    orientation: list[float] = field(default_factory=lambda: [0.0, 0.0])
    update_rate: float = 10.0
    max_range: float = 5000.0
    min_range: float = 200.0
    fov: FOVConfig = field(default_factory=FOVConfig)
    resolution: RadarResolution = field(default_factory=RadarResolution)
    detection: RadarDetection = field(default_factory=RadarDetection)
    error: RadarError = field(default_factory=RadarError)
    clutter: ClutterConfig = field(default_factory=ClutterConfig)
    jammer: JammerConfig = field(default_factory=JammerConfig)
    scan: ScanConfig = field(default_factory=ScanConfig)
    frequency: float = 16.0e9          # Hz, Ku波段
    reference_rcs: float = 0.01        # m², 参考RCS
    reference_range: float = 5000.0    # m, 参考距离
    max_velocity: float = 100.0        # m/s, 最大测速


@dataclass
class EODetection:
    pd: float = 0.95
    min_contrast: float = 0.1


@dataclass
class EOError:
    azimuth_sigma: float = 0.005
    elevation_sigma: float = 0.005


@dataclass
class EOTracking:
    lock_on_time: float = 0.5
    max_track_targets: int = 3
    handover_delay: float = 1.0


@dataclass
class WeatherConfig:
    visibility: float = 10.0
    turbulence: str = "low"


@dataclass
class EOConfig:
    id: str = "eo-01"
    position: list[float] = field(default_factory=lambda: [100.0, 0.0, 10.0])
    orientation: list[float] = field(default_factory=lambda: [0.0, 5.0])
    update_rate: float = 30.0
    fov: FOVConfig = field(default_factory=lambda: FOVConfig([-4.0, 4.0], [-3.0, 3.0]))
    max_track_range: float = 8000.0
    detection: EODetection = field(default_factory=EODetection)
    error: EOError = field(default_factory=EOError)
    tracking: EOTracking = field(default_factory=EOTracking)
    weather: WeatherConfig = field(default_factory=WeatherConfig)


@dataclass
class ESMDetection:
    poi: float = 0.8
    sensitivity: float = -80.0


@dataclass
class ESMError:
    azimuth_sigma: float = 1.0
    frequency_sigma: float = 1.0e6
    pri_quantization: float = 1.0e-9
    pw_quantization: float = 50.0e-9


@dataclass
class ESMMeasurement:
    measure_frequency: bool = True
    measure_pri: bool = True
    measure_pw: bool = True
    measure_modulation: bool = True


@dataclass
class ESMConfig:
    id: str = "esm-01"
    position: list[float] = field(default_factory=lambda: [-50.0, 0.0, 8.0])
    orientation: list[float] = field(default_factory=lambda: [0.0, 0.0])
    update_rate: float = 1.0
    fov: FOVConfig = field(default_factory=lambda: FOVConfig([-180.0, 180.0], [-10.0, 40.0]))
    frequency_range: list[float] = field(default_factory=lambda: [0.1e9, 18.0e9])
    detection: ESMDetection = field(default_factory=ESMDetection)
    error: ESMError = field(default_factory=ESMError)
    measurement: ESMMeasurement = field(default_factory=ESMMeasurement)


@dataclass
class EmitterParams:
    active: bool = True
    freq: float = 2.4e9
    pri: float = 1000e-6
    pw: float = 10e-6
    modulation: str = "OFDM"
    power: float = 0.1
    active_after: float | None = None   # 此时间后开启辐射
    active_until: float | None = None   # 此时间后关闭辐射


@dataclass
class TargetConfig:
    id: int = 0
    label: str = ""
    type: str = "quadcopter"
    rcs: float = 0.01
    emitter: EmitterParams = field(default_factory=EmitterParams)
    waypoints: list[list[float]] = field(default_factory=list)


@dataclass
class AcousticDetection:
    pd: float = 0.80
    reference_range: float = 300.0
    sound_level_ref: float = 95.0


@dataclass
class AcousticError:
    azimuth_sigma: float = 3.0
    elevation_sigma: float = 8.0
    frequency_sigma: float = 50.0


@dataclass
class AcousticEnvironment:
    ambient_noise_db: float = 30.0
    wind_speed: float = 3.0
    temperature: float = 20.0


@dataclass
class AcousticConfig:
    id: str = "acoustic-01"
    position: list[float] = field(default_factory=lambda: [-30.0, 0.0, 2.0])
    orientation: list[float] = field(default_factory=lambda: [0.0, 0.0])
    update_rate: float = 5.0
    fov: FOVConfig = field(default_factory=lambda: FOVConfig([-180.0, 180.0], [-20.0, 60.0]))
    max_range: float = 1000.0
    min_range: float = 10.0
    detection: AcousticDetection = field(default_factory=AcousticDetection)
    error: AcousticError = field(default_factory=AcousticError)
    environment: AcousticEnvironment = field(default_factory=AcousticEnvironment)
    reference_range: float = 300.0
    sound_level_ref: float = 95.0


@dataclass
class CoordinatesConfig:
    origin: list[float] = field(default_factory=lambda: [120.08363526778648, 30.13911824931559, 0.0])
    frame: str = "ENU"


@dataclass
class ScenarioConfig:
    name: str = "default"
    description: str = ""
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    coordinates: CoordinatesConfig = field(default_factory=CoordinatesConfig)
    targets: list[TargetConfig] = field(default_factory=list)
    radar: RadarConfig = field(default_factory=RadarConfig)
    eo: EOConfig = field(default_factory=EOConfig)
    esm: ESMConfig = field(default_factory=ESMConfig)
    acoustic: AcousticConfig = field(default_factory=AcousticConfig)


# ──────────────────────────────────────────────
# 加载工具
# ──────────────────────────────────────────────

def _deep_merge(base: dict, override: dict) -> dict:
    """递归合并字典，override 的值覆盖 base。"""
    result = base.copy()
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def _dict_to_dataclass(cls, data: dict):
    """递归地将字典转换为 dataclass 实例。"""
    if data is None:
        return None

    # 使用 typing.get_type_hints 获取真实类型（处理 from __future__ import annotations）
    try:
        field_types = __import__('typing').get_type_hints(cls)
    except Exception:
        field_types = {f.name: f.type for f in cls.__dataclass_fields__.values()}

    kwargs = {}
    for key, value in data.items():
        if key not in field_types:
            continue
        ftype = field_types[key]

        # 处理嵌套 dataclass
        if hasattr(ftype, '__dataclass_fields__'):
            kwargs[key] = _dict_to_dataclass(ftype, value) if isinstance(value, dict) else value
        else:
            # 处理泛型 list[...]
            origin = getattr(ftype, '__origin__', None)
            if origin is list:
                import typing
                args = typing.get_args(ftype)
                if args and hasattr(args[0], '__dataclass_fields__'):
                    kwargs[key] = [_dict_to_dataclass(args[0], v) if isinstance(v, dict) else v for v in value]
                else:
                    kwargs[key] = value
            else:
                kwargs[key] = _coerce_type(value, ftype)
    return cls(**kwargs)


def _coerce_type(value, target_type):
    """将值强制转换为目标类型。"""
    if value is None:
        return value
    origin = getattr(target_type, '__origin__', None)
    if origin is list:
        return value  # list 类型在上面处理

    if target_type in (float, 'float') and not isinstance(value, float):
        try:
            return float(value)
        except (ValueError, TypeError):
            return value
    if target_type in (int, 'int') and not isinstance(value, int):
        try:
            return int(value)
        except (ValueError, TypeError):
            return value
    if target_type in (bool, 'bool') and not isinstance(value, bool):
        if isinstance(value, str):
            return value.lower() in ('true', 'yes', '1')
        return bool(value)
    return value


def load_scenario(scenario_path: str | Path) -> ScenarioConfig:
    """加载场景配置文件。

    加载顺序：
    1. 读取场景 YAML 文件
    2. 对每个传感器，如果指定了 use_config，加载传感器子配置并合并
    3. 用默认值填充缺失字段
    4. 返回 ScenarioConfig 对象

    Args:
        scenario_path: 场景 YAML 文件路径

    Returns:
        ScenarioConfig: 完整的场景配置对象
    """
    scenario_path = Path(scenario_path)

    with open(scenario_path, 'r', encoding='utf-8') as f:
        scenario_data = yaml.safe_load(f)

    # 加载传感器子配置并合并
    sensors_section = scenario_data.get('sensors', {})

    for sensor_name in ('radar', 'eo', 'esm', 'acoustic'):
        sensor_entry = sensors_section.get(sensor_name, {})
        use_config = sensor_entry.pop('use_config', None)

        if use_config:
            # 解析相对路径
            config_path = scenario_path.parent.parent / use_config
            if not config_path.exists():
                config_path = Path(use_config)
            if config_path.exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    sensor_defaults = yaml.safe_load(f)
                # 提取传感器键（如 radar / eo / esm）
                if sensor_name in sensor_defaults:
                    sensor_defaults = sensor_defaults[sensor_name]
                # 合并：子配置文件为基础，场景中的覆盖
                sensor_data = _deep_merge(sensor_defaults, sensor_entry)
            else:
                sensor_data = sensor_entry
        else:
            sensor_data = sensor_entry

        scenario_data[sensor_name] = sensor_data

    # 移除 sensors 嵌套键
    scenario_data.pop('sensors', None)

    # 展平 scenario 元信息字段
    scenario_meta = scenario_data.pop('scenario', {})
    scenario_data['name'] = scenario_meta.get('name', scenario_data.get('name', 'default'))
    scenario_data['description'] = scenario_meta.get('description', scenario_data.get('description', ''))

    # 展平 simulation 字段
    sim_data = scenario_data.pop('simulation', {})
    scenario_data['simulation'] = SimulationConfig(**sim_data)

    # 构建配置
    return _dict_to_dataclass(ScenarioConfig, scenario_data)


def load_sensor_config(sensor_type: str, config_path: str | Path) -> dict:
    """加载单个传感器的配置文件。

    Args:
        sensor_type: 'radar', 'eo', 或 'esm'
        config_path: 传感器 YAML 文件路径

    Returns:
        dict: 传感器参数字典
    """
    config_path = Path(config_path)
    with open(config_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    return data.get(sensor_type, data)
