#!/usr/bin/env python
"""将 single_target.yaml 场景的四传感器输出打印到 txt 文件。"""
import sys, math
sys.path.insert(0, 'src')

from scenario.controller import SimulationController
from bus.proto_bridge import CoordConverter

out = []
def p(s=''):
    out.append(s)
    print(s)

ctrl = SimulationController('config/scenarios/single_target.yaml')
conv = CoordConverter(120.0, 30.0, 0.0)

p('=' * 70)
p('  single_target.yaml — 四传感器输出数据')
p('=' * 70)
p(f'  场景: {ctrl.config.name}')
p(f'  时长: {ctrl.config.simulation.duration}s')
p(f'  目标: {ctrl.targets[0].label} ({ctrl.targets[0].target_type})')
p(f'  雷达: {ctrl.config.radar.id}')
p(f'  光电: {ctrl.config.eo.id}')
p(f'  电侦: {ctrl.config.esm.id}')
p(f'  声探测: {ctrl.config.acoustic.id}')
p(f'  ENU参考点: ({conv.ref_lng}°E, {conv.ref_lat}°N, {conv.ref_alt}m)')
p('')

# 跑仿真累积数据
for i in range(400):
    ctrl.step()
tracks = ctrl._radar_sensor.get_track_states()
confirmed = [tid for tid, info in tracks.items() if info['state'] == 'confirmed']
if not confirmed:
    for i in range(800):
        ctrl.step()

# 每 5 秒采一次数据
sample_times = list(range(5, 90, 5))

for target_time in sample_times:
    while ctrl.sim_time < target_time:
        ctrl.step()

    t = ctrl.targets[0]
    state = t.get_state(ctrl.sim_time)
    if state is None:
        continue
    rx, ry, rz = state.position
    sx, sy, sz = ctrl._radar_sensor.position
    dx, dy, dz = rx - sx, ry - sy, rz - sz
    r_true = math.sqrt(dx * dx + dy * dy + dz * dz)
    az_true = math.degrees(math.atan2(dy, dx)) % 360
    el_true = math.degrees(math.atan2(dz, math.sqrt(dx * dx + dy * dy)))
    lng_e7, lat_e7, alt_dm = conv.enu_to_wgs84(rx, ry, rz)

    p(f'--- T={ctrl.sim_time:.0f}s  ENU({rx:.0f},{ry:.0f},{rz:.0f}) 距雷达{r_true:.0f}m ---')
    p(f'    真值WGS84: {lng_e7/1e7:.6f}°E  {lat_e7/1e7:.6f}°N  {alt_dm/10:.1f}m')

    # 雷达
    tracks = ctrl._radar_sensor.get_track_states()
    t1 = tracks.get(1, {})
    if t1.get('state') in ('confirmed', 'coasting') and t1.get('predicted') is not None:
        px, py, pz = t1['predicted']
        plng, plat, palt = conv.enu_to_wgs84(px, py, pz)
        p(f'  [雷达] range={r_true:.0f}m az={az_true:.1f}° el={el_true:.1f}°')
        p(f'         航迹WGS84: {plng/1e7:.6f}°E {plat/1e7:.6f}°N {palt/10:.1f}m  state={t1["state"]} hits={t1["hits"]}')
        p(f'         速度={state.speed:.1f}m/s 航向={math.degrees(state.heading):.0f}°')
    else:
        p(f'  [雷达] (搜索中，无确认航迹)')

    # 光电
    eo = ctrl._eo_sensor.get_state()
    p(f'  [光电] state={eo["state"]} n_locked={eo["n_locked"]} point=Az{eo["current_az"]:.1f}° El{eo["current_el"]:.1f}°')

    # 电侦
    esm_dets = getattr(ctrl, '_last_esm_dets', [])
    if esm_dets:
        d = esm_dets[0]
        p(f'  [电侦] az={d["azimuth"]:.1f}° freq={d.get("frequency",0)/1e9:.2f}GHz conf={d["confidence"]:.0%}')
    else:
        p(f'  [电侦] (当前帧无检测)')

    # 声探测
    ac_dets = getattr(ctrl, '_last_acoustic_dets', [])
    dist_ac = math.sqrt((rx + 30) ** 2 + ry ** 2 + (rz - 2) ** 2)
    if ac_dets:
        d = ac_dets[0]
        p(f'  [声探测] az={d["azimuth"]:.1f}° el={d["elevation"]:.1f}° class={d.get("classification","?")} conf={d["confidence"]:.0%}')
    else:
        p(f'  [声探测] 距声传感器{dist_ac:.0f}m (有效约500m，无检测)')
    p('')

ctrl.shutdown()

filepath = 'output/single_target_output.txt'
with open(filepath, 'w', encoding='utf-8') as f:
    f.write('\n'.join(out))
print(f'\n已写入 {filepath} ({len(out)} 行)')
