"""蜂群场景生成器。

根据战术模式自动生成 15 架无人机的航点轨迹。
四种战术模式：饱和突击、分层协同、游荡召唤、狼群搜索。
"""

from __future__ import annotations

import math
import random
import yaml
from pathlib import Path
from dataclasses import dataclass, field


# ═══════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════

def polar_to_xy(r, theta_deg):
    """极坐标 → 直角坐标"""
    th = math.radians(theta_deg)
    return r * math.cos(th), r * math.sin(th)


def make_waypoint(x, y, z, t):
    return [round(x, 1), round(y, 1), round(z, 1), round(t, 1)]


def linspace_waypoints(start, end, times):
    """在两个 3D 点之间均匀插值生成航点"""
    wps = []
    for i, t in enumerate(times):
        frac = (t - times[0]) / (times[-1] - times[0]) if len(times) > 1 else 0
        x = start[0] + frac * (end[0] - start[0])
        y = start[1] + frac * (end[1] - start[1])
        z = start[2] + frac * (end[2] - start[2])
        wps.append(make_waypoint(x, y, z, t))
    return wps


def orbit_waypoints(cx, cy, z, radius, start_angle, duration, n_orbits, n_waypoints=6):
    """生成绕 (cx,cy) 盘旋的航点"""
    wps = []
    for i in range(n_waypoints):
        frac = i / (n_waypoints - 1)
        t = frac * duration
        angle = math.radians(start_angle + frac * n_orbits * 360)
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        # 高度微变模拟真实盘旋
        z_var = z + 20 * math.sin(frac * math.pi * 2)
        wps.append(make_waypoint(x, y, z_var, t))
    return wps


# ═══════════════════════════════════════════════════
# 场景生成器
# ═══════════════════════════════════════════════════

@dataclass
class DroneSpec:
    id: int
    label: str
    drone_type: str      # fixed_wing / quadcopter
    rcs: float
    emitter_active: bool
    emitter_freq: float  # Hz
    emitter_power: float # W
    emitter_mod: str
    waypoints: list[list[float]]
    emitter_after: float | None = None  # 此时间后开启辐射
    emitter_until: float | None = None  # 此时间后关闭辐射


def build_yaml(name, description, duration, drones, sensors_override=None):
    """构建场景 YAML 字典"""
    targets = []
    for d in drones:
        targets.append({
            'id': d.id,
            'label': d.label,
            'type': d.drone_type,
            'rcs': d.rcs,
            'emitter': {
                'active': d.emitter_active,
                'freq': d.emitter_freq,
                'pri': random.choice([200e-6, 500e-6, 800e-6, 1000e-6]),
                'pw': random.choice([2e-6, 5e-6, 10e-6, 20e-6]),
                'modulation': d.emitter_mod,
                'power': d.emitter_power,
                **({'active_after': d.emitter_after} if d.emitter_after else {}),
                **({'active_until': d.emitter_until} if d.emitter_until else {}),
            },
            'waypoints': d.waypoints,
        })

    # 默认使用 Ku 波段机械扫描雷达（参照技术协议）
    sensors = {
        'radar': {'use_config': 'config/sensors/radar_ku.yaml'},
        'eo': {'use_config': 'config/sensors/eo.yaml'},
        'esm': {'use_config': 'config/sensors/esm.yaml'},
        'acoustic': {'use_config': 'config/sensors/acoustic.yaml'},
    }
    if sensors_override:
        for k, v in sensors_override.items():
            if isinstance(v, dict):
                sensors[k] = {**sensors.get(k, {}), **v}

    return {
        'scenario': {'name': name, 'description': description},
        'simulation': {'duration': duration, 'dt': 0.05, 'seed': random.randint(0, 99999)},
        'targets': targets,
        'sensors': sensors,
    }


# ═══════════════════════════════════════════════════
# 场景 1：饱和式突击 (Saturation Assault)
# ═══════════════════════════════════════════════════

def generate_saturation_assault():
    """
    15 架无人机，无领机，分 5 个方向 × 3 个高度层同时突入。
    方向覆盖 180° 扇面（-90°～+90°，正前方为 0°）。
    高度层：低空 50m、中空 150m、高空 300m。
    速度快（固定翼），到达时间集中在最后 20 秒。
    """
    name = "饱和式突击"
    desc = "15架固定翼无人机，5方向×3高度层同时突入，无领机，牺牲数量保证突防"
    duration = 120
    drones = []

    directions = [-80, -40, 0, 40, 80]    # 5 个进入方向（度）
    altitudes = [50, 150, 300]             # 3 个高度层 (m)
    start_range = 4000                     # 起始距离 (m)
    end_range = 300                        # 终点距离 (m)

    drone_id = 1
    for i_dir, direction in enumerate(directions):
        for i_alt, alt in enumerate(altitudes):
            label = f"突击-{drone_id}"
            # 起始位置
            sx, sy = polar_to_xy(start_range, direction)
            # 终点位置（略分散）
            ex, ey = polar_to_xy(end_range, direction + random.uniform(-5, 5))

            # 低空快、高空稍慢
            speed = random.uniform(40, 60)  # m/s for fixed wing
            travel_time = (start_range - end_range) / speed
            # 到达时间集中在 80-110 秒（最后 30 秒集中突入）
            arrival_time = random.uniform(80, 110)
            start_time = max(0, arrival_time - travel_time)

            # 简单直线轨迹
            n_wp = 3
            wps = []
            for j in range(n_wp):
                frac = j / (n_wp - 1)
                t = start_time + frac * travel_time
                x = sx + frac * (ex - sx)
                y = sy + frac * (ey - sy)
                # 高度：先保持，末端俯冲
                z = alt if frac < 0.8 else alt - frac * alt * 0.5
                wps.append(make_waypoint(x, y, z, t))

            # 是否辐射（部分静默突防）
            active = random.choice([True, True, False])
            drones.append(DroneSpec(
                id=drone_id, label=label, drone_type="fixed_wing",
                rcs=random.uniform(0.05, 0.3),
                emitter_active=active,
                emitter_freq=random.choice([2.4e9, 5.8e9, 6.0e9]),
                emitter_power=random.uniform(0.5, 2.0),
                emitter_mod=random.choice(["BPSK", "QPSK", "OFDM"]),
                waypoints=wps,
            ))
            drone_id += 1

    return build_yaml(name, desc, duration, drones)


# ═══════════════════════════════════════════════════
# 场景 2：侦察-干扰-打击分层协同
# ═══════════════════════════════════════════════════

def generate_layered_kill_chain():
    """
    三层杀伤链：
    - 第一层 (5架)：侦察机，前出 30-60s，高空，电磁静默或间歇辐射
    - 第二层 (4架)：电子战机，伴随侦察后方，主动干扰
    - 第三层 (6架)：攻击机，最后进入，依赖前方数据链
    """
    name = "侦察-干扰-打击分层协同"
    desc = "三层杀伤链：5架侦察前出→4架电战伴随→6架攻击机滞后进入，分层协同打击"
    duration = 150
    drones = []
    random.seed(42)

    # -- 第一层：侦察机（5架）--
    recon_dirs = [-60, -30, 0, 30, 60]
    for i, d in enumerate(recon_dirs):
        sx, sy = polar_to_xy(5000, d)
        ex, ey = polar_to_xy(800, d)
        t0 = 0
        t1 = 120  # 前出时间
        drones.append(DroneSpec(
            id=i + 1, label=f"侦察-{i+1}", drone_type="fixed_wing",
            rcs=0.01,                                          # 隐身
            emitter_active=(i % 2 == 0),                       # 一半间歇辐射
            emitter_freq=random.choice([8.0e9, 10.0e9]),       # 高频侦察雷达
            emitter_power=0.1,
            emitter_mod="BPSK",
            waypoints=[
                make_waypoint(sx, sy, 400, 0),
                make_waypoint((sx+ex)/2, (sy+ey)/2, 350, t1*0.5),
                make_waypoint(ex, ey, 300, t1),
            ],
        ))

    # -- 第二层：电子战机（4架）--
    jammer_dirs = [-45, -15, 15, 45]
    for i, d in enumerate(jammer_dirs):
        sx, sy = polar_to_xy(4000, d)
        ex, ey = polar_to_xy(1500, d)
        t_delay = random.uniform(10, 30)
        t1 = 100
        drones.append(DroneSpec(
            id=6 + i, label=f"电战-{i+1}", drone_type="fixed_wing",
            rcs=0.5,                                           # 较大（带干扰吊舱）
            emitter_active=True,                               # 持续干扰
            emitter_freq=random.choice([2.0e9, 4.0e9, 8.0e9]),
            emitter_power=10.0,                                # 大功率干扰
            emitter_mod="NOISE",
            waypoints=[
                make_waypoint(sx, sy, 250, t_delay),
                make_waypoint((sx+ex)/2, (sy+ey)/2, 200, t_delay + t1*0.5),
                make_waypoint(ex, ey, 150, t_delay + t1),
            ],
        ))

    # -- 第三层：攻击机（6架）--
    strike_dirs = [-50, -20, 10, 40, -35, 25]
    for i, d in enumerate(strike_dirs):
        sx, sy = polar_to_xy(6000, d)
        ex, ey = polar_to_xy(300, d + random.uniform(-5, 5))
        t_delay = random.uniform(50, 70)  # 滞后进入
        t_attack = 80
        drones.append(DroneSpec(
            id=11 + i, label=f"攻击-{i+1}", drone_type="fixed_wing",
            rcs=random.uniform(0.1, 0.3),
            emitter_active=True,
            emitter_freq=random.choice([5.8e9, 6.0e9]),
            emitter_power=random.uniform(0.5, 2.0),
            emitter_mod=random.choice(["QPSK", "OFDM"]),
            waypoints=[
                make_waypoint(sx, sy, 200, t_delay),
                make_waypoint((sx+ex)/2, (sy+ey)/2, 120, t_delay + t_attack*0.5),
                make_waypoint(ex, ey, 50, t_delay + t_attack),
            ],
        ))

    return build_yaml(name, desc, duration, drones)


# ═══════════════════════════════════════════════════
# 场景 3：区域游荡与召唤打击
# ═══════════════════════════════════════════════════

def generate_loitering_strike():
    """
    15 架巡飞弹在可疑区域分散盘旋。
    阶段 1 (0-70s)：分散盘旋，搜索目标
    阶段 2 (70-100s)：发现目标，开始收敛
    阶段 3 (100-130s)：集中冲刺攻击
    """
    name = "区域游荡与召唤打击"
    desc = "15架巡飞弹在目标区域分散盘旋，发现目标后迅速集中攻击，模拟巡飞弹群召唤打击"
    duration = 130
    drones = []
    random.seed(123)

    # 15 个盘旋中心，分散在 3km × 3km 区域
    grid_positions = []
    for row in range(3):
        for col in range(5):
            cx = 500 + col * 600 + random.uniform(-100, 100)
            cy = -600 + row * 600 + random.uniform(-100, 100)
            grid_positions.append((cx, cy))

    # 最终目标点（集中攻击点）
    target_x, target_y = 100, 0

    for i, (cx, cy) in enumerate(grid_positions):
        alt_loiter = random.uniform(80, 150)
        orbit_radius = random.uniform(200, 500)

        # 阶段 1：盘旋游荡（0-70s），绕各自的中心盘旋
        phase1 = orbit_waypoints(cx, cy, alt_loiter, orbit_radius,
                                 start_angle=random.uniform(0, 360),
                                 duration=70, n_orbits=random.uniform(1.5, 3))

        # 阶段 2+3：收敛 → 冲刺（70-130s）
        last_wp = phase1[-1]
        phase2 = linspace_waypoints(
            (last_wp[0], last_wp[1], last_wp[2]),
            (target_x + random.uniform(-50, 50), target_y + random.uniform(-50, 50), 30),
            [70, 100, 130]
        )

        all_wps = phase1[:-1] + phase2  # 去掉最后一个盘旋点，接上收敛轨迹

        # 游荡阶段静默，攻击阶段（70s后）开启末制导雷达/数据链
        emitter_on = True  # 基本辐射能力
        emitter_after = random.uniform(65, 75)  # 攻击阶段开启时间
        drones.append(DroneSpec(
            id=i + 1, label=f"巡飞弹-{i+1}", drone_type="quadcopter",
            rcs=random.uniform(0.005, 0.02),           # 小型巡飞弹
            emitter_active=emitter_on,
            emitter_after=emitter_after,                # 时变：攻击阶段才辐射
            emitter_freq=random.choice([2.4e9, 5.8e9, 35e9]),  # 含毫米波末制导
            emitter_power=random.uniform(0.05, 0.5),
            emitter_mod=random.choice(["OFDM", "FMCW", "BPSK"]),
            waypoints=all_wps,
        ))

    return build_yaml(name, desc, duration, drones)


# ═══════════════════════════════════════════════════
# 场景 4：分布式"狼群"协同搜索
# ═══════════════════════════════════════════════════

def generate_wolfpack_search():
    """
    15 架无人机形成 5×3 搜索阵列：
    - 5 列（横向展开 ~5km），3 行（纵向梯次配置）
    - 间距 800-1200m
    - 整体向 +X 方向推进
    - 不同行的高度错开（低/中/高）
    - 辐射源轮换开关
    """
    name = "分布式狼群协同搜索"
    desc = "15架无人机形成5×3搜索阵列，间距800-1200m，梯次高度配置，协同推进搜索"
    duration = 150
    drones = []
    random.seed(77)

    n_cols = 5
    n_rows = 3
    col_spacing = 1000   # 列间距 (m)
    row_spacing = 800    # 行间距 (m)
    row_altitudes = [60, 150, 300]  # 前/中/后排高度

    start_x = -3000      # 阵列起始 X
    end_x = 3000         # 阵列终止 X
    center_y = -1000     # 搜索区域中心 Y

    drone_id = 1
    for row in range(n_rows):
        for col in range(n_cols):
            sx = start_x + random.uniform(-200, 200)
            sy = center_y + (col - n_cols // 2) * col_spacing + row * row_spacing * 0.3
            alt = row_altitudes[row] + random.uniform(-20, 20)

            # 每架无人机走 S 形搜索路径
            wps = []
            n_segments = 4
            for seg in range(n_segments + 1):
                frac = seg / n_segments
                t = frac * duration
                x = sx + frac * (end_x - sx)
                # 之字形：奇数列左右摆动
                zigzag = 300 * math.sin(frac * math.pi * 3) * (1 if col % 2 == 0 else -1)
                y = sy + zigzag
                z = alt + 30 * math.sin(frac * math.pi * 2)  # 高度微变
                wps.append(make_waypoint(x, y, z, t))

            # 辐射源：每隔一个开关，定期轮换
            emitter_on = (drone_id % 3 != 0)
            drones.append(DroneSpec(
                id=drone_id, label=f"狼群-{drone_id}", drone_type="quadcopter",
                rcs=random.uniform(0.01, 0.05),
                emitter_active=emitter_on,
                emitter_freq=random.choice([2.4e9, 5.8e9]),
                emitter_power=random.uniform(0.1, 0.5),
                emitter_mod=random.choice(["OFDM", "BPSK"]),
                waypoints=wps,
            ))
            drone_id += 1

    # 狼群场景：雷达部署在阵地后方，朝向搜索方向，宽视场覆盖
    sensors_override = {
        'radar': {
            'position': [500, -1000, 10],
            'orientation': [-30.0, 0.0],  # 略微偏向阵列中心
            'update_rate': 10,
            'fov': {
                'azimuth': [-90.0, 90.0],  # ±90° 宽视场覆盖搜索扇面
                'elevation': [-15.0, 45.0],
            },
        },
    }
    return build_yaml(name, desc, duration, drones, sensors_override)


# ═══════════════════════════════════════════════════
# 主入口
# ═══════════════════════════════════════════════════

def main():
    out_dir = Path("config/scenarios")
    out_dir.mkdir(parents=True, exist_ok=True)

    generators = {
        "swarm_saturation": (generate_saturation_assault, "饱和式突击"),
        "swarm_layered": (generate_layered_kill_chain, "侦察-干扰-打击分层协同"),
        "swarm_loitering": (generate_loitering_strike, "区域游荡与召唤打击"),
        "swarm_wolfpack": (generate_wolfpack_search, "分布式狼群协同搜索"),
    }

    for filename, (gen_func, desc) in generators.items():
        data = gen_func()
        path = out_dir / f"{filename}.yaml"
        with open(path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
        n_drones = len(data['targets'])
        print(f"  ✅ {path.name} — {desc} ({n_drones}架, {data['simulation']['duration']}s)")

    print(f"\n共生成 {len(generators)} 个场景文件 → {out_dir}/")


if __name__ == '__main__':
    main()
