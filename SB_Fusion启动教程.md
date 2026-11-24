
## RK3588 + Kylin V10

首次部署：

```bash
cd /home/kylin/sb_fusion_2
./setup_rk3588.sh
```

启动实装雷达接入（不会启动仿真器）：

```bash
./start_equipment_fusion.sh
```

先按现场填写 `configs/equipment.yaml` 的参考点和雷达安装位置。
打开 `http://<RK3588-IP>:8000/debug.html` 查看真实收包情况。

详细说明见 [实装调试说明](RK3588_EQUIPMENT_DEBUG.md) 和 `RK3588_RUN.md`。
只有离线演示才使用 `HOST=0.0.0.0 EXTERNAL_WS_SERVER=1 ./start_sb_fusion.sh`。
