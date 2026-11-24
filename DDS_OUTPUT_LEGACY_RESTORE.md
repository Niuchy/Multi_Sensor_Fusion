# RK3588 正式 DDS 输出恢复原版（2026-10-07）

按用户联调要求，仅恢复正式融合 DDS 输出的原始桥接实现。不回退雷达接入、
航迹处理、诊断页面、项目环境或 WebSocket 11409 端口。

## 当前约定

- Topic 仍为 `FusionMultiTarget`，通过公司的 TopicAPI / ByteSequence 发布。
- `company_dds_bridge.cpp` 恢复原始 `PXSBProto::MsgHeader::serialize()`、
  `PXSBProto::UnitHeader::serialize()` 和后续时间戳追加逻辑。
- 不再调用新增加的 `company_wire::protocol_packet()`。
- 消息体仍由原 `fusion_proto.h` 的 `FusionMultiTargetMsg.serialize()` 编码。
- 桥版本恢复为 `company-dds-python-bridge-1.0.0`。

恢复基线为修改前保留的源码副本；整个 `company_dds_bridge.cpp` 已逐字节比对，
SHA-256 为 `bbf3dbc2e3b4afc7b95c4ea60e9a603863a8962fe71d7ad4a44d7e430fe6e2f7`。

## 生效方式

仅更新源码并在 RK3588 编译根目录 `libcompany_dds_bridge.so`，不停止或重启
当前服务。构建使用临时文件并重命名替换，已运行的进程继续使用旧的已加载库。
用户在原运行终端 Ctrl+C 后，手动重新运行：

```bash
cd /home/kylin/code/sb_fusion_2
EXTERNAL_WS_PORT=11409 ./start_equipment_fusion.sh
```

不要在原服务仍运行时启动第二套实例。本次没有发送模拟雷达或测试融合数据。

## 已知边界

- 这是按联调要求恢复旧实现，不代表已修复或重新验收旧实现的报文正确性。
  当底层 `UnitHeader.serialize()` 已包含时间戳时，原版会重复追加时间戳，
  可能再次出现空目标包 49 字节而声明长度不一致的问题；源时间戳未初始化的
  旧行为也随原版恢复。以对方接收器实际联调结果为准。
- `company_wire.h` 和相关独立编码测试保留为参考，但不再用于生产发送。
  其 41 字节空目标包测试通过，不代表当前原版发送封装通过。
  真实 DDS 回放工具保留严格封装校验，不为兼容回退而隐藏错误。
- 正式 DDS 的置信度门槛未修改：用户 `1.log` 全为 `tag=0`，当前纯外推
  航迹可以显示在页面，但不会因此变成正式 DDS 确认目标。

若其他文档仍描述 `1.1.0-pxsb14` 修复后的发送方式，以本回退说明为准。
