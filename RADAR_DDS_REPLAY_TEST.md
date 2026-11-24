# 1.log 真实 DDS 回放测试（2026-10-07）

本文是此前 DGX 上的历史实测记录。后续 Mac 本地 RK3588 源码已修复正式 DDS
重复时间戳及代理问题，但没有更新 DGX 或完成 RK3588 实机复测；板端步骤见
[RK3588 实装调试说明](RK3588_EQUIPMENT_DEBUG.md)。下面的旧抓包结论保持原样。

## 结论

已在 DGX 上完成一次真实 DDS 发送测试，不是内存调用，也不是页面模拟响应。
输入、内部跟踪融合、页面 WebSocket、对外 WebSocket 均通过；正式对外
`FusionMultiTarget` 二进制 DDS 封装发现独立缺陷，因此完整测试结论为**部分通过**。
本轮没有修改生产融合节点或重编译原生库。

| 检查环节 | 实测结果 |
| --- | --- |
| RadarTarget 发送 | 21 条，每条 106 字节，合计 2226 字节 |
| 融合输入接收 / 解析 | 21 / 21，错误 0，重复/迟到丢弃 0 |
| RawMeasurements | 对应原始报告的 21 帧全部收到 |
| SensorTracks | 对应原始报告的 21 帧全部收到 |
| FusedTracks / FusedFrame | 各 21 次报告更新，融合 ID 始终为 1 |
| 主界面 WebSocket | 21 次报告更新全部收到 |
| 对外 WebSocket | 21 次更新全部收到，status=2；断流后 status=3 |
| 停发清理 | 最后报告后 12.044 秒的状态时刻删除，观测窗口 15 秒 |
| 正式 FusionMultiTarget DDS | 抓到 44 条，全部存在封装长度错误 |

原始日志 21 条均为 `tag=0`；测试保持该字段，没有把外推点改成真实点。
页面和外部 WebSocket 的滑行航迹可见；常规确认输出为 0 是当前置信度策略的
预期结果，与下面正式 DDS 封装缺陷是两个不同问题。

## 发送方法与保真范围

- 使用公司 TopicAPI/FastDDS，发布到 `RadarTarget`，走现有输入节点完整链路。
- 按协议重新构造 `MsgHeader(26) + UnitHeader(14) + RadarFrame(66)`。
- 保留设备 3152、源航迹 89、消息号 784–804、L1/L2/L3 字段及日志中的 CRC=0。
- 保留 2.980–3.021 秒的原始时间间隔，总时长 59.933 秒；实际发送相对
  调度时刻最大偏差约 20.4 毫秒。
- 日志没有日期，所以整体平移到测试当前 epoch；报文三处时间戳保持一致。
- 日志未打印的信封字段使用 0。此次是语义重建，不宣称逐字节还原原始抓包。
- 发送前检查无活跃设备数据并等待 4 秒 DDS 发现；如果收到非本次预期的
  RadarTarget 报文会停止发送。只发一轮，不循环。

## 正式 DDS 新发现的问题（未修改）

位置：`dds_swarm_fusion/company_bridge/company_dds_bridge.cpp`，
`publish_protocol_body()`。

DGX 当前库的 `unit.serialize()` 已包含 8 字节时间戳，但桥接代码随后又追加
8 字节 `stamp`；同时 `unit.timestamp` 没有赋值。

本轮空目标输出应为 26 + 14 + 1 = 41 字节，实际为 49 字节。
包头 `msgLength=15`，实际载荷却为 23 字节。标准解码器会在目标数量所在位置
读到重复时间戳的首字节，不能将这些包认定为有效正式输出。

后续修复应统一 UnitHeader 序列化方式、初始化并设置源时间戳、取消重复追加，
按实际序列化长度填写 `msgLength`，重新构建对应 ARM64 桥接库后重跑本测试。
目前回放工具已把此问题作为失败断言，避免只验证页面就漏掉正式 DDS 接口。

## 重复运行

先确认无现场设备数据，现有融合服务已运行；不要启动第二套融合实例。

```bash
cd /home/dgx/code/det_uavs/sb_fusion_2
/home/dgx/miniconda3/envs/sb_fusion/bin/python tools/replay_radar_dds.py \
  artifacts/radar-replay-20261007-3L8J5a/1.log --publish
```

不加 `--publish` 只校验并预览，不连接 DDS。不提供加速/循环参数，保证本次
工具默认按原始时间间隔只播放一轮。可加 `--report 新文件.json` 保存证据；
已有报告不会覆盖。当前正式 DDS 缺陷尚在，完整检查会返回退出码 1。

页面：<http://192.168.8.69:8000/debug.html>。
测试结束后收包计数仍为 21、历史记录保留、输出航迹清空，是预期行为。
生产服务继续运行，回放发送器已退出。

## 证据与边界

DGX 与本地均保存：`artifacts/radar-replay-20261007-3L8J5a/`。

- `evidence.json`：首次采集的报文及输出；其初版断言未覆盖正式封装，不作为
  最终总判定。
- `audited-evidence.json`：对同一次抓包补充正式封装检查后的最终结果，
  `all_checks_passed=false`，17 项通过，1 项正式 DDS 封装失败。
- `debug-live.png` / `main-live.png`：真实 DDS 回放期间的浏览器截图。
- DGX 的 `1.log` 是用户原文件副本；SHA-256 为
  `395746ab6e758d994c6cc757ff373cd64df0fabe310bbbd1fdc064543e360974`。

项目附带的旧 C++ `dds_receiver` 是 x86-64，而本机 DGX 为 aarch64，未成功
启动，未把它计作验证通过。实际接收证据来自正在运行的融合输入进程、独立
DDS 订阅器和两路 WebSocket。未覆盖物理雷达到 DGX 的跨机网络路径，也未验证
厂商为何持续输出 tag=0。

主界面预置的设备列表/在线开关来自场景配置，不等于实际发现了这些设备；
实际接入判断以诊断页的 Topic 收包、最近接收时间和设备号为准。
