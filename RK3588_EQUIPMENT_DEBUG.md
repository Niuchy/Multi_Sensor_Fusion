# RK3588 实装雷达调试（2026-10-07）

> 后续联调要求：正式 DDS 发送桥已恢复原版 `1.0.0`，不再使用下文所述的
> `1.1.0-pxsb14` 封装修复。变更范围、生效方式和已知风险见
> [DDS 输出恢复原版说明](DDS_OUTPUT_LEGACY_RESTORE.md)。其他接入修复保留。

这次只修改 Mac 本地项目，没有连接或更新 DGX/RK3588。代码已准备，板端
ARM64 构建与实际雷达网络仍需现场验收。历史 DGX 测试结果不等于 RK3588 验收。

## 更新内容

- 保留 3 秒环扫接入修复：时间抖动不重置航迹，按设备航迹号接续，区分
  真实点与外推，断流预测/删除，输入/跟踪/融合独立诊断心跳。
- 实装入口不再使用 DGX Python 路径，复用 `.rk3588-env`、ARM64 原生环境
  和板端构建。仅构建实装所需根目录融合桥接库，不启动仿真器。
- 正式 `FusionMultiTarget` 封装修复：显式编码 26 字节 MsgHeader +
  14 字节 UnitHeader，不再因库版本差异重复写入时间戳。空目标包应为 41 字节。
- 新回放入口支持只检查连接，HTTP/WS 明确直连，绕过终端的 7890 代理；
  不修改全局代理设置。只播放一轮，默认不发送。
- 实装专用配置只展示实际收到过报文的设备，不预置演示设备。

## 1. 复制到板端与初始化

把更新后的项目复制到板端项目目录；保留板端已有的 `.rk3588-env` 及现场
配置，不要复制 Mac 的 Python 环境。更新前先停止原有服务。需要包含本次
更新的根目录脚本、`dds_swarm_fusion/`、`tools/`、`tests/`、`configs/`、
`samples/`，尤其是新文件 `dds_swarm_fusion/company_bridge/company_wire.h`。

下面路径是示例，按实际安装目录替换：

```bash
cd /home/kylin/sb_fusion_2
chmod +x setup_rk3588.sh build_native.sh start_sb_fusion.sh \
  start_equipment_fusion.sh replay_radar_dds.sh
# 仅首次安装或环境不完整时执行；需板端 ARM64 Conda 与可用软件源
./setup_rk3588.sh
```

仓库内的旧 `.so` 不是本次源码修复后的编译产物。Mac 上没有替换它们；
`start_equipment_fusion.sh` 会在板端重新构建根目录 `libcompany_dds_bridge.so`。
修复后的桥版本是 `company-dds-python-bridge-1.1.0-pxsb14`。

## 2. 检查现场坐标

编辑 `configs/equipment.yaml`：

- `coordinates.origin`：ENU 参考点 `[经度, 纬度, 海拔米]`；
- `sensors.radar.position`：雷达相对参考点 `[东, 北, 上]` 米。

默认 `[120,30,0]` / `[0,0,0]` 是占位值，不是测定的雷达站址。若参考点
就是雷达安装点，相对位置可用 `[0,0,0]`。有效 L2 经纬高优先；L1 极坐标
回退和多设备关联依赖正确安装位置。不要根据目标坐标猜测雷达站址。

也可保留独立现场文件，启动时加 `--scenario /path/to/site.yaml`。

## 3. 启动实装服务

```bash
./start_equipment_fusion.sh
```

使用项目内 `.rk3588-env/bin/python`，可用 `PYTHON=/path/to/python` 覆盖。
该入口不启动仿真，不回放日志，没有 120 秒演示退出限制。默认是
`device_tracks` 雷达 L2 接续和 `north_cw` 北零顺时针方位。

- 主界面：`http://<RK3588-IP>:8000/`
- 接入诊断：`http://<RK3588-IP>:8000/debug.html`
- 诊断 JSON：`http://<RK3588-IP>:8000/api/diagnostics`
- 对外结果：`ws://<RK3588-IP>:11409/ws/main`

`/health` 正常只说明网关工作。请看诊断页的节点心跳、收包/解析计数、错误、
最后收包时间和设备号判断实装是否接入。历史记录保留不代表设备仍在发包。

```bash
# 自定义端口
PORT=8080 EXTERNAL_WS_PORT=11407 ./start_equipment_fusion.sh
# 推送到对方 WebSocket，而不是开本地结果 WS 服务
EXTERNAL_WS_URL=ws://192.168.8.10:11406/ws/main ./start_equipment_fusion.sh
```

Ctrl+C 停止整套服务，不要重复启动同 Topic/端口的多个实例。

## 4. 第二终端检查与回放

先启动服务，再在同一项目目录打开第二个终端：

```bash
# 仅检查随附日志的编码；不连接 DDS
./replay_radar_dds.sh samples/radar_3s.log
# 仅检查 HTTP、两路 WS、节点心跳；不创建 DDS 发送器
./replay_radar_dds.sh samples/radar_3s.log --check-only
# 只有确认真实设备停发后，才执行真实 DDS 回放
./replay_radar_dds.sh samples/radar_3s.log --publish --report replay-evidence.json
```

报告路径必须是新文件，已有文件不覆盖。若改了端口，补充
`--gateway http://127.0.0.1:8080 --external-ws ws://127.0.0.1:11407/ws/main`。
完整回放检查要求本地结果 WS 服务开启；外部推送模式下默认 11409 不监听，
不能将它误认为 DDS 故障。

日志来自用户 `1.log`，21 条，设备 3152、航迹 89，全是 `tag=0`。保留
2.980–3.021 秒原始间隔与目标字段，只整体平移时间戳。发送 59.933 秒后
观察 15 秒断流清理；不会循环发包。发送前检查数据是否活跃，发现非本轮
RadarTarget 报文会停止；现场仍必须主动停真实雷达，不能只依赖短暂安静窗口。

预期：收包/解析各增加 21，错误不增加；同一 ID 的滑行航迹在页面和 WS
出现；断流约 12 秒后删除。纯外推日志不产生常规确认目标，正式 DDS 可以是
**合法的空目标包**，不能为了输出目标而把 tag=0 当真实点。

## 5. 底层检查

```bash
./build_native.sh --equipment-only
file libcompany_dds_bridge.so
ldd libcompany_dds_bridge.so
.rk3588-env/bin/python run_sb_fusion.py inspect list
.rk3588-env/bin/python run_sb_fusion.py inspect echo company_radar --once
.rk3588-env/bin/python run_sb_fusion.py inspect hz company_radar
.rk3588-env/bin/python run_sb_fusion.py inspect echo diagnostics --once
.rk3588-env/bin/python run_sb_fusion.py inspect echo official_fusion --once
curl --noproxy '*' http://127.0.0.1:8000/health
ss -lntp | grep -E ':(8000|11409)'
```

`.so` 应显示 `ARM aarch64`，`ldd` 不应出现 `not found`。诊断命令经过
根启动器加载正确的库与 OpenSSL；不要直接运行包内旧 x86-64 `dds_receiver`。
日志原接收器用 Domain 106，Topic 是 `RadarTarget`；通信配置由公司原生库
提供，脚本没有新增虚假的 Domain 覆盖选项。现场需核对设备配置、网卡、组播。

## 验证边界与仿真入口

本机可验证 Python、脚本传参、本地 HTTP/WS、独立 C++ 协议编码；无法在
macOS 加载 Linux FastDDS 来替代板端测试。必须在 RK3588 验证构建、库依赖、
DDS 回放及物理雷达网络。旧 DGX 抓包保留为历史证据，不会伪造为修复后结果。

本次 Mac 回归：41 项通过，包含 C++ 0/1/255 目标包编码、失效代理下的
本地 HTTP/WS 直连、实装启动先构建/失败即停止、原日志 21 次同 ID 滑行更新。
板端也可运行 `.rk3588-env/bin/python -m unittest discover -s tests -v`；
若 C++ 编译器不在 PATH，编码测试会注明跳过，不能将跳过当成板端编译通过。

仅离线演示时使用以下命令，它才会构建并启动仿真；不要与实装服务并行：

```bash
HOST=0.0.0.0 EXTERNAL_WS_SERVER=1 ./start_sb_fusion.sh
```
