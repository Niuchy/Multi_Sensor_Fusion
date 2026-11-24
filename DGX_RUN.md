# NVIDIA DGX 启动说明

融合算法目录使用。

## 一键启动

```bash
cd ~/code/det_uavs/sb_fusion_2
conda activate sb_fusion
HOST=0.0.0.0 \
EXTERNAL_WS_SERVER=1 \
./start_sb_fusion.sh
```

启动后，对方使用下面的地址主动连接：

```text
ws://<DGX-IP>:11406/ws/main
```

浏览器访问：

```text
http://<DGX-IP>:8000/
```

## 分步启动

```bash
cd ~/code/det_uavs/sb_fusion_2
./build_dgx.sh
python run_sb_fusion.py \
  --host 0.0.0.0 \
  --port 8000 \
  --external-ws-server
```

如果仍需使用原来的“我方主动推送给对方”模式：

```bash
python run_sb_fusion.py \
  --host 0.0.0.0 \
  --port 8000 \
  --external-ws-url ws://192.168.1.200:11406/ws/main
```

服务端也可以单独启动（融合主链路必须已经在公司DDS中发布
`SwarmFusion.FusedTracks`）：

```bash
python run_sb_fusion.py external-ws-server \
  --host 0.0.0.0 \
  --port 11406 \
  --path /ws/main
```

验证：

```bash
file zy-anti-drone/bridge/libad_proto.so
file libcompany_dds_bridge.so
curl http://127.0.0.1:8000/health
ss -lntp | grep 11406
python run_sb_fusion.py inspect hz raw
python run_sb_fusion.py inspect echo fused_tracks --once
```

两个 `.so` 在 DGX 上都应显示 `ARM aarch64`。
