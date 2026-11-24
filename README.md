# SB Fusion 3588

SB Fusion 在 **RK3588 + Kylin V10（ARM64/AArch64）** 平台上的适配版本。
项目由原 NVIDIA DGX Spark 版本迁移而来，支持接收装备发布的 DDS 数据，
完成数据归一化、目标跟踪和多源融合，并通过 Web 页面及 WebSocket 输出融合结果。

单台 3 秒环扫实装请看 [RK3588 实装调试说明](RK3588_EQUIPMENT_DEBUG.md)。
当前本地源码包含 DDS 接入诊断、设备航迹接续、代理绕过及正式 DDS 封装修复；
Linux 原生库必须上板重新构建，不代表已经完成 RK3588 实机验收。

## 主要功能

- RK3588/AArch64 原生 DDS 桥接库构建与架构校验
- 项目内独立 Conda 环境，不替换系统 Python、glibc 或系统编译器
- 直接接收装备 DDS 数据，不启动本地仿真器
- 多传感器目标跟踪与融合
- FastAPI Web 页面、健康检查和融合结果 WebSocket
- 保留内置仿真模式，便于离线联调
- 独立 DDS 录制/回放工具及网页界面，不启动融合、不改融合输出协议
- 独立实装仿真控制页：1～10 架无人机、单/双光电及指定跟踪对象
- 多目标离线评价：匹配精确率、覆盖率、F1、RMSE/P95、漏航迹及 ID 连续性
- 光电坐标/距离/纯测向分路融合、双站几何检查与设备自身 GPS 来源诊断

## 数据流程

```text
装备 DDS 数据
    │
    ▼
company_input（数据接收与归一化）
    │
    ▼
tracking（目标跟踪）
    │
    ▼
fusion（多源融合）
    ├── Web 页面 / HTTP API
    └── WebSocket 融合结果
```

## 运行环境

- 硬件：Rockchip RK3588
- 操作系统：Kylin V10，Linux AArch64
- Python：3.10 及以上，初始化脚本默认安装 Python 3.12
- 环境管理：ARM64 版 Miniconda/Conda
- 网络：装备与 RK3588 之间的 DDS 网络需要互通

项目在设备上的推荐目录：

```text
/home/kylin/sb_fusion_2
```

## 首次部署

将当前更新后的项目复制到板端，并安装 ARM64 版 Miniconda 后执行。
此次改动仅在本地，未自动提交/推送到远端仓库，重新克隆旧仓库不包含这些修改。

```bash
cd /home/kylin/sb_fusion_2
chmod +x setup_rk3588.sh build_native.sh start_sb_fusion.sh \
  start_equipment_fusion.sh replay_radar_dds.sh
./setup_rk3588.sh
```

`setup_rk3588.sh` 会：

1. 在项目内创建 `.rk3588-env` 隔离环境；
2. 安装 Python 运行依赖和 AArch64 C++ 编译器；
3. 在 RK3588 本机重新构建 DDS 原生桥接库；
4. 检查 Python 依赖和动态库链接状态。

## 启动装备数据模式

实际接收装备数据时，直接运行：

```bash
cd /home/kylin/sb_fusion_2
./start_equipment_fusion.sh
```

这是实装环境推荐的启动方式。它会启动 DDS 数据接收、目标跟踪、融合、
Web 服务和融合结果 WebSocket，**不会启动项目内置仿真器**。
启动前按现场填写 `configs/equipment.yaml` 的参考点和安装位置；每次启动
会先构建实装所需融合桥接库，避免仍加载旧版 `.so`。

默认服务地址：

- Web 页面：`http://<RK3588-IP>:8000/`
- 接入诊断：`http://<RK3588-IP>:8000/debug.html`
- 健康检查：`http://<RK3588-IP>:8000/health`
- 融合结果 WebSocket：`ws://<RK3588-IP>:11409/ws/main`

例如设备地址为 `192.168.8.76` 时，Web 页面为：

```text
http://192.168.8.76:8000/
```

按 `Ctrl+C` 可停止整套服务及其子进程。

## 独立数据录制与 DDS 回放界面

本地 RK3588 部署目录已包含此功能。在现有板端项目中执行：

```bash
cd /home/kylin/code/sb_fusion_2
bash ./dds_data_web.sh
```

访问 `http://192.168.8.76:8010/`，输入启动终端显示的访问口令。可开始/停止录制、
选择录制保存目录、选择不同位置的录制文件、预检查并通过 DDS 回放。
默认保存在板端项目的 `recordings/`；录制完成后点“下载到 Mac”即可复制到浏览器所在电脑，板端原件保留。
目录选择器浏览的是设备端；Mac 文件也可先上传到当前选择的设备目录。额外数据盘目录可用 `--storage-root /实际目录` 开放。
这是独立程序，不占用融合页面的 8000 端口，也不会自动启停融合。

本功能只更新 Python、Shell、网页文件；板端已有 DDS 桥可用时无需重编译。
增量同步请保留板端 `.so`、环境目录、现场配置和已有录制数据。
本次目录选择升级只需更新 `tools/dds_data_web.py` 和 `tools/dds_data_ui/`，任务结束后手动重启独立数据工具即可，不需要重启融合。
完整文件清单与操作步骤见 [DDS 数据录制与回放说明](DDS_DATA_TOOL.md)。
真正回放前必须停止或隔离其他设备输入；发送调用数不代表融合已接收。

## 启动仿真联调模式

### 独立实装特性仿真（新增）

本地 Mac 工程已同步此功能；板端尚未部署。复制到 RK3588 后运行：

```bash
cd /home/kylin/code/sb_fusion_2
bash field_sim_web.sh
```

访问 `http://192.168.8.76:8011/`，按终端口令登录。选择无人机数量和每台光电跟踪对象，
点击“生成测试数据”→“运行离线评估”，右侧显示整体与逐目标指标，可下载 JSON 报告。
生成/评估不发 DDS；实际发送需人工确认隔离，并按页面命令另行启动测试融合。
详见 [实装仿真操作与评价说明](FIELD_SIM_WEB_GUIDE.md) 和 [光电融合配置说明](EO_FUSION_GUIDE.md)。

为保留既有现场策略，3588 正式输出置信度门槛仍默认 **0.05**。
独立离线评估和页面生成的测试融合命令显式使用 **0.2**，与 DGX 仿真口径一致；
`SB_FUSION_MIN_CONFIDENCE` 只作用于指定进程，不会改写现场配置。评价报告会标明该门槛。
它与真值匹配的“50 m 门限”不同，不能混为一谈。

本次同步未修改 C++、DDS 表头/Topic/发送方式、原生 `.so`、板端环境及现有启动脚本。
已有可用 ARM64 DDS 桥时，本次 Python/网页更新不新增重编译要求；装备入口原有构建检查仍保留。
Mac 可用于离线生成/评估，不使用 Linux DDS 动态库发包。上板复制时请保留现场配置、环境及录制文件。

### 原演示模式（保留）

只有在离线测试或联调时才使用内置仿真数据：

```bash
HOST=0.0.0.0 EXTERNAL_WS_SERVER=1 ./start_sb_fusion.sh
```

默认场景为：

```text
zy-anti-drone/data_generator/config/scenarios/multi_target.yaml
```

## 启动参数

启动命令前的变量只对本次进程生效，不会永久修改系统配置。

| 变量 | 默认值（装备模式） | 说明 |
| --- | --- | --- |
| `HOST` | `0.0.0.0` | Web 服务监听全部网卡，使局域网设备能够访问 |
| `PORT` | `8000` | Web 页面和 HTTP API 端口 |
| `EXTERNAL_WS_SERVER` | `1` | 启动独立的融合结果 WebSocket 服务 |
| `EXTERNAL_WS_HOST` | `0.0.0.0` | WebSocket 监听全部网卡 |
| `EXTERNAL_WS_PORT` | `11409` | WebSocket 服务端口 |
| `EXTERNAL_WS_PATH` | `/ws/main` | WebSocket 路径 |
| `PYTHON` | 自动选择 | 手动指定 Python 解释器路径 |
| `SB_RADAR_INPUT_MODE` | `device_tracks` | 按设备 L2 航迹号接续；`measurements` 保留量测模式 |
| `SB_AZIMUTH_CONVENTION` | `north_cw` | 实装协议北零顺时针；演示输入可用 `enu` |

自定义端口示例：

```bash
PORT=8080 EXTERNAL_WS_PORT=11407 ./start_equipment_fusion.sh
```

## 手动重新构建

修改 C++ 桥接代码或更新第三方库后，在 RK3588 上执行：

```bash
./build_native.sh --equipment-only
```

也可以重新运行完整初始化：

```bash
./setup_rk3588.sh
```

实装构建生成根目录 `libcompany_dds_bridge.so`。无参数运行 `./build_native.sh`
才同时构建仿真桥接库，完整构建产物为：

```text
libcompany_dds_bridge.so
zy-anti-drone/bridge/libad_proto.so
zy-anti-drone/bridge/libad_proto_receiver.so
zy-anti-drone/bridge/libad_proto_fusion.so
```

## 运行状态检查

检查服务健康状态：

```bash
curl --noproxy '*' http://127.0.0.1:8000/health
```

检查监听端口：

```bash
ss -lntp | grep -E ':(8000|11409)'
```

检查桥接库架构：

```bash
file libcompany_dds_bridge.so
```

输出应包含 `ARM aarch64`。检查动态链接依赖：

```bash
ldd libcompany_dds_bridge.so
```

输出中不应出现 `not found` 或 `version ... not found`。

查看项目 DDS 主题：

```bash
.rk3588-env/bin/python run_sb_fusion.py inspect list
```

查看某个主题的数据或频率：

```bash
.rk3588-env/bin/python run_sb_fusion.py inspect echo RadarTarget --once
.rk3588-env/bin/python run_sb_fusion.py inspect hz RadarTarget
```

## 常见问题

### Fast DDS 编译时出现 `offset set but not used`

如果构建输出中出现类似警告：

```text
warning: variable 'offset' set but not used [-Wunused-but-set-variable]
```

该信息来自第三方 Fast DDS 头文件，是编译器的未使用变量警告。只要后续显示
`构建完成`，并且生成的 `.so` 文件为 `ARM aarch64`，就不影响运行。

### 提示动态库架构不匹配

不要直接使用 DGX/x86_64 上编译出的 `.so` 文件。应在 RK3588 上运行：

```bash
./setup_rk3588.sh
```

或：

```bash
./build_native.sh
```

### 局域网无法访问 Web 页面

装备模式已经默认设置 `HOST=0.0.0.0`。如果仍无法访问，请检查：

- RK3588 的实际 IP 地址；
- Kylin 防火墙是否允许 TCP 端口 `8000` 和 `11409`；
- 访问端与 RK3588 是否处于可互通网络。

### 服务已启动但没有装备数据

确认装备端正在发布 DDS 数据，并检查两端网络、DDS 主题名称及通信配置。
可以使用 `dds_inspect.py list/echo/hz` 辅助判断数据是否到达 RK3588。

## 项目结构

| 路径 | 说明 |
| --- | --- |
| `start_equipment_fusion.sh` | 装备 DDS 数据模式启动入口 |
| `start_sb_fusion.sh` | 完整系统启动入口，默认包含仿真器 |
| `setup_rk3588.sh` | RK3588 首次环境初始化 |
| `build_native.sh` | ARM64/x86_64 原生桥接库构建 |
| `run_sb_fusion.py` | 各融合进程的统一编排器 |
| `dds_swarm_fusion/` | DDS 输入、跟踪、融合和 Web 网关 |
| `zy-anti-drone/` | 装备协议、DDS 依赖及仿真数据模块 |
| `RK3588_RUN.md` | RK3588 部署补充说明 |

## 说明

本仓库包含装备通信和 RK3588 适配相关代码，请按照项目保密要求控制仓库权限、
账号授权和部署文件的分发范围。
