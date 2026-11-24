# RK3588 + Kylin V10 部署与启动

实装雷达调试请优先阅读 [RK3588 实装调试说明](RK3588_EQUIPMENT_DEBUG.md)。
本地已补入跟踪/诊断、正式 DDS 封装和代理绕过修复；尚未在 RK3588 实机验收。

## 首次初始化

设备端需要预先安装 ARM64 版 Miniconda。进入项目后执行：

```bash
cd /home/kylin/sb_fusion_2
chmod +x setup_rk3588.sh build_native.sh start_sb_fusion.sh start_equipment_fusion.sh replay_radar_dds.sh
./setup_rk3588.sh
```

脚本会在项目内创建 `.rk3588-env`，从 conda-forge 安装 Python 3.12、
NumPy/SciPy、Web 服务依赖和 AArch64 C++ 编译器，然后在板端重编译实装
所需的融合 DDS 桥接库。它不会替换系统 Python、glibc 或系统编译器。

## 启动

```bash
cd /home/kylin/sb_fusion_2
./start_equipment_fusion.sh
```

启动器会自动选择 `.rk3588-env/bin/python`，并在每次启动前重建原生桥接
库。实装默认 `configs/equipment.yaml`，不会启动模拟目标。需要离线演示时才用
`HOST=0.0.0.0 EXTERNAL_WS_SERVER=1 ./start_sb_fusion.sh`；该演示场景结束后会退出。
下面 IP 只是示例，替换为板端实际 IP：

- Web 页面：`http://192.168.8.76:8000/`
- 健康检查：`http://192.168.8.76:8000/health`
- 接入诊断：`http://192.168.8.76:8000/debug.html`
- 融合 WebSocket：`ws://192.168.8.76:11409/ws/main`

如果外部设备负责发布传感器 DDS 数据，不启动本项目自带仿真器：

```bash
./start_equipment_fusion.sh
```

该脚本只启动装备 DDS 数据接收、跟踪融合、Web 页面和融合结果 WebSocket，
不会启动 `company_sim` 仿真数据进程。

## 验证

```bash
file libcompany_dds_bridge.so
curl --noproxy '*' http://127.0.0.1:8000/health
ss -lntp | grep -E ':(8000|11409)'
```

融合桥接库应显示 `ARM aarch64`，`ldd` 输出中不应包含 `not found` 或
`version ... not found`。

## 重新初始化

源代码或第三方库更新后，重新运行：

```bash
./setup_rk3588.sh
```

若需要指定其他 Python，可在启动时设置 `PYTHON=/path/to/python`；该解释器
必须为 Python 3.10 或更高版本并安装 `dds_swarm_fusion/requirements-dds.txt`。
