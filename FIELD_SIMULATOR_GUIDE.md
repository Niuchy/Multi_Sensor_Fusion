# 独立实装特性测试模式（第一版）

本工具与原演示模式并存：没有改动原演示入口、仿真器、融合算法、正式 DDS 输出封装。
生成器 `tools/field_simulator.py` 独立产生设备输入；评估器 `tools/evaluate_field_sim.py`
在内存中调用当前融合链路。两者默认都不发送 DDS，也不启动或停止服务。

这是**设备报文行为仿真**，用于检验退化、恢复及异常处理，不是设备精度标定或电磁/光学数字孪生。
噪声、探测概率和光电频率为可调测试假设，不能把本工具的误差指标当成实装性能承诺。

## 网页操作入口（新增）

在 RK3588 项目目录运行 `bash field_sim_web.sh`，然后打开 `http://192.168.8.76:8011/`。
使用启动终端显示的访问口令登录。可在页面点击“添加第二台光电”，分别设置两台的设备号、
ENU 站址、频率和观测模式；也可选“雷达 + 双光电”或“双光电纯测向”快速组合。

完整流程和安全说明见 `FIELD_SIM_WEB_GUIDE.md`。控制页默认不发 DDS，不自动启停融合。
原来的态势台仍在 8000，录制回放页仍在 8010；控制页不往原页面添加设备按钮。
只有实际发送原协议 DDS 数据后，融合接入页面才会出现新的设备号。

## 1. 先离线测试（推荐）

在 RK3588 项目目录执行；这里不用启动融合服务：

```bash
cd /home/kylin/code/sb_fusion_2
PY=./.rk3588-env/bin/python

# 查看场景
"$PY" tools/field_simulator.py list

# 生成一次“测距失败”测试。输出目录必须是新目录，已有数据不会被覆盖。
"$PY" tools/field_simulator.py generate --case range-failure --output artifacts/field-test-001

# 原始报文→实际输入节点→跟踪节点→融合节点，随后才读取真值计算误差
"$PY" tools/evaluate_field_sim.py artifacts/field-test-001

# 一次生成、评估全部场景（原 16 个预设 + custom 自定义组合）
"$PY" tools/field_simulator.py generate --case all --output artifacts/field-suite-001
"$PY" tools/evaluate_field_sim.py artifacts/field-suite-001 --suite
```

每个测试默认模拟 72 秒，离线评估不需要按实时时长等待。
可用 `--duration 120 --seed 42` 改时长和随机种子；时长支持 15–600 秒。
为了检验 12 秒定位支撑超时，建议使用默认 72 秒或更长场景。
相同参数/种子的设备负载和事件可复现；文件头会有新的会话号和生成时间，因此整文件哈希不同。

## 2. 场景与预期

| `--case` | 模拟内容 | 应观察到的行为 |
|---|---|---|
| `baseline` | 3 秒雷达 + 单光电有效目标坐标 | 有正常融合输出 |
| `range-failure` | 中间 25%–75% 光电无距离/坐标，但有角度 | 雷达提供定位支撑，光电继续测向更新 |
| `radar-outage` | 中间 25%–75% 雷达静默、光电仅测向 | 原航迹短时维持；定位支撑超时后清理；恢复可重新起批 |
| `low-speed` | 中段悬停，雷达降低探测率，光电有坐标 | 雷达可先外推后停止，光电仍提供定位 |
| `single-bearing` | 只有一台光电纯测向，无先验定位 | 不能凭一条视线编造空间定位航迹 |
| `dual-bearing` | 两台光电纯测向、约 299 米基线、异步报告 | 几何和确认条件满足后交汇起批 |
| `weak-geometry` | 两站目标约 15 公里、视线近平行 | 拒绝不可靠交汇，不应强行输出位置 |
| `colocated-bearing` | 两站间距只有 1 米 | 拒绝基线不足的交汇 |
| `recovery` | 单光电坐标→短时仅角度→坐标 | 默认 72 秒下，约 7.2 秒失距期间保持融合 ID |
| `stale-values` | 中段坐标和距离冻结，角度继续改变 | 过期旧值被识别，不持续冒充新坐标 |
| `ranged-only` | 无目标 GPS、有距离和角度；中段短时失距 | 显式启用测试距离策略后可定位，失距则降级 |
| `gps-fallback` | 光电站址 GPS 有效→零值→恢复 | 报文站址优先；缺失时明确回退配置 |
| `timing-faults` | 光电时钟 +1500 ms、重复及迟到包 | 已知时间校正生效，重复/迟到计数增加 |
| `optical-loss` | 正常→记忆跟踪→丢失→恢复 | 记忆/丢失状态不被当作新的光电量测 |
| `radar-extrapolation` | 只有雷达外推报告，随后停止 | 只能产生低置信度外推展示，不能冒充正式确认输出 |
| `rf-distractors` | 主无人机 + 两个额外射频目标 | 测试多身份/方位干扰；射频自身不提供定位真值 |

单光电纯角度、弱交汇和纯外推场景中“正式输出为 0”是预期，不是测试失败。
`rf-distractors` 是功能压力场景，不代表已证明任意多目标情况下身份关联均正确。

## 3. 生成了哪些文件

- `capture.jsonl`：兼容现有独立录制回放工具的 `sb-dds-capture-v1` 文件。
  使用原 `RadarTarget / EOTarget / ESMTarget` Topic 和原 PXSB + dev_proto 布局。
  synthetic 来源说明在录制文件头，不往 DDS 协议中添加字段。协议尾 CRC 按现有仿真约定为 0。
- `scenario.yaml`：仅本次测试的坐标原点和站址，`targets` 为空，避免真值进入融合。
- `eo_devices.yaml`：仅本次测试的设备号、站址、角度/时间校准和距离策略。
- `truth.jsonl`：独立真值，10 Hz，**不发送 DDS，也不作为融合输入**。
- `events.jsonl`：报文对应的采样时刻、模拟到达时刻、设备号、模式和真值目标号；仅用于检查。
- `metadata.json`：场景、参数、报文数、录制文件哈希和模型限制。
- `evaluation.json`：离线评估结果；`evaluation.frames.jsonl`：实际融合逐帧输出。

输出不覆盖；同一场景再次评估请指定新结果路径，例如：

```bash
"$PY" tools/evaluate_field_sim.py artifacts/field-test-001 --output artifacts/field-test-001/evaluation-second.json
```

主要评估项：正式输出存在率、最长无输出时段、最近输出的水平/高度误差、ID 变化、
重复/迟到丢弃、站址来源、退化模式和交汇拒绝原因。
误差是“有正式输出时、距主目标真值最近的一条输出”的逐帧统计，不设距离门限删大误差；
不等于已完成多目标身份匹配。输出存在率以 0.5 秒网格、保持最近状态不超过雷达周期 +0.5 秒计算，
不等于新量测率。无输出时误差为 `null`，不是 0。

## 4. 在页面里看实际 DDS 接入效果

**先确保隔离测试环境：真实设备、旧演示、其他回放发送者均已停发；正式输出不能流入现场业务。
更换网页端口或设备号不等于隔离 DDS。**工具不会替你停止设备或重启服务。

先用离线命令生成一个场景，再在终端 A 手动启动测试融合实例。
如果已有融合实例，请自行确认、停止该实例后再启动，避免端口冲突：

```bash
cd /home/kylin/code/sb_fusion_2
SB_FUSION_MIN_CONFIDENCE=0.2 SB_EO_CONFIG="$PWD/artifacts/field-test-001/eo_devices.yaml" \
  ./start_equipment_fusion.sh --scenario "$PWD/artifacts/field-test-001/scenario.yaml"
```

此命令使用“外部设备输入”入口，**不会启动旧演示仿真器**。
一定同时指定这次生成的站址与 EO 配置，否则测试设备号可能缺少角度校准/站址，无法检验预期路径。
默认测试设备号为雷达 `73152`、光电 `71112/71113`、射频 `74001`；不是现场设备配置。

终端 B 先只校验（不发包）：

```bash
PY=./.rk3588-env/bin/python
"$PY" tools/field_simulator.py send artifacts/field-test-001
```

确认隔离、停发其他输入后，才显式发布：

```bash
"$PY" tools/field_simulator.py send artifacts/field-test-001 \
  --publish --confirm-isolated --inputs-stopped
```

这会委托现有独立 DDS 回放器，按 1 倍速度发送并将时间戳整体平移到当前时间，保留跨设备偏移和原间隔。
保留原回放器的外部流量检测；检测只能辅助防混流，不是网络隔离保证。
没有新增或改变正式 `FusionMultiTarget` 输出表头。

打开 `http://192.168.8.76:8000/debug.html` 看接收量、站址来源、EO 质量与航迹；
打开 `http://192.168.8.76:8000/` 看融合态势。
也可在现有录制回放界面选择该 `capture.jsonl`（所在目录须在该服务允许访问的目录内），
选择“时间重基/当前时间”，1 倍回放；仍须执行相同隔离要求。不要把 `truth.jsonl` 当成录制文件。
测试不同场景建议手动重启测试融合实例，避免上一场景状态残留；本工具不自动重启。

## 5. 参数与模型边界

复制 `configs/field_simulator.json` 为自己的测试配置，用 `generate --config 新配置.json`。
默认雷达周期 3 秒、周期抖动 ±20 ms、正常探测概率 0.97；
`low-speed` 中速度低于 0.8 m/s 时探测概率 0.05；外推最长 6 秒。
这是经验行为开关，**不是用真实径向多普勒/MTI 推导的低速盲区模型**。
雷达按每圈 L2 报告建模，不发扫描结束标志，不实现波束扫到目标的精细时序；不会把真值每 0.5 秒当真实雷达点重发。
设备 L2 用带噪极坐标量测做简单 alpha-beta 估计，外推只使用最后估计，不偷用当前真值。

光电默认 10 Hz / 8 Hz，独立设备号和目标号，异步采样；方位为北零顺时针。
默认距离噪声标准差 5 米、角度 0.3°；雷达距离 10 米、角度 0.3°。
这些是假设，不是从实测记录拟合的数值；第一版也不模拟杂波、遮挡地图、完整搜索/锁定控制或随机假航迹。
噪声使目标高度超出协议无符号高度可表示范围时，EO 目标 GPS 置为无效，不截断成假坐标。
射频方位暂按当前输入实现采用东零逆时针，仍需真实协议/数据标定；本工具不声称已解决厂商输出问题。
坐标采用项目现有小区域 ENU 转换，不是高精度测地误差模型。

`gps-fallback` 的配置站址故意偏东 30 米、偏北 20 米，用于确认报文优先/配置回退，不能复制到实装配置。
光电 `range_policy` 默认禁用；只有 `ranged-only` 测试显式信任正距离，并仍检查冻结旧值。
“正距离”不能在实际设备中自动证明激光成功。`angles_calibrated=true` 也仅对本工具已知角度模型成立。

时间异常场景默认生成已知 -1500 ms 校正，也可离线做未校正负对照：

```bash
"$PY" tools/evaluate_field_sim.py artifacts/field-suite-001/timing-faults \
  --uncorrected-time --output artifacts/field-suite-001/timing-faults/uncorrected.json
```

`azimuth_bias_deg` 可人为加入角度偏差，测试配置提供其反向角度校正；
目标 GPS 由同一带偏角度/距离构造，角度校正不会自动修正报文中的目标 GPS。
修改噪声、时长、概率后，部分默认场景断言可能失败；这用于暴露行为差异，不应通过降低融合门槛来“刷过”。

## 6. 回归与安全

```bash
"$PY" -B -m unittest discover -s tests -p 'test_field_simulator.py' -v
```

生成、离线评估及默认 `send` 校验均不加载原生 DDS 发送端；
`send --publish` 缺少任一隔离/停发确认会拒绝，实际运行仍使用原生库和现有回放器。
运行日志中的 `dds_packets_sent=0` 仅描述离线阶段，不是“已经实测 DDS 通信成功”。
# 多目标与新版评价补充（2026-10-22）

独立控制页可选 1～10 架无人机，每台光电选择固定跟踪对象。CLI 在 `custom` 场景配置
`custom_target_count`、`eo_target_ids`（两台的目标编号）及 `evaluation_gate_m`。
其余原有场景仍保持原行为；旧配置缺失新字段时默认单目标、两台都跟目标 1、评价门限 50 m。

离线评价新增 `multi_target_metrics`：全体目标一对一门限匹配的 Precision、Recall、F1、
水平/高度/三维误差及逐目标 ID 连续性。真值只在融合运行结束后用于评分，不输入融合。
旧顶层 nearest-output 指标仅保留兼容，不应拿来代替多目标评分。
完整口径与使用方式见 [FIELD_SIM_WEB_GUIDE.md](FIELD_SIM_WEB_GUIDE.md)。
