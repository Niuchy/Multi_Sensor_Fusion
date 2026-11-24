/**
 * @file publisher.cpp
 * @brief 融合多目标消息发布测试
 *
 * 使用 FusionMultiTargetMsg 作为消息体，
 * 组装 MsgHeader + UnitHeader + sourceTime + body 完整报文，
 * 通过 TopicAPI 发布到 DDS 主题。
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cstring>

#include "TopicAPI.h"
#include "proto.h"
#include "fusion_proto.h"

using namespace PXSBProto;
using namespace AntiDroneFusionProtocol;

int main()
{
    // 初始化 Topic 接口
    InitTopic();

    std::string topic_name = "fusion_test";
    createWriter(topic_name, false);

    int msg_id = 0;

    while (true)
    {
        // ================================================================
        // 第一步：构建 FusionMultiTargetMsg 消息体
        // ================================================================
        FusionMultiTargetMsg body;

        // --- 目标1：敌方无人机（雷达+光电融合） ---
        {
            FusionTarget target;
            target.setGlobalId(1);
            target.setTargetBasicInfo(1, static_cast<uint8_t>(SourceMask::RADAR_EO));
            target.setTargetIdentity(
                static_cast<uint8_t>(TargetType::DRONE),  // 无人机
                85,                                        // 置信度 85%
                2                                           // 敌方
            );

            AbsPos pos;
            pos.setPosition(1163000000, 399000000, 1500);   // 经度 116.3°, 纬度 39.9°, 高度 150.0m
            target.setPosition(pos);

            Attitude att;
            att.setAttitude(0, 0, 9000);                    // 水平飞行，航向 90°
            target.setTargetAttitude(att);
            target.setMotion(2000);                          // 速度 20.00 m/s

            // 添加预测轨迹点
            PredictionPoint pp;
            pp.setAbsPos(pos);
            pp.setAttitude(att);
            pp.setSpeed(2000);
            pp.setTime(5000);                                // 5 秒后位置
            target.addPredictionPoint(pp);

            body.addTarget(target);
        }

        // --- 目标2：不明飞鸟（雷达探测） ---
        {
            FusionTarget target;
            target.setGlobalId(2);
            target.setTargetBasicInfo(2, static_cast<uint8_t>(SourceMask::RADAR));
            target.setTargetIdentity(
                static_cast<uint8_t>(TargetType::BIRD),   // 鸟
                60,                                        // 置信度 60%
                0                                           // 不明
            );

            AbsPos pos;
            pos.setPosition(1163100000, 399100000, 800);    // 经度 116.31°, 纬度 39.91°, 高度 80.0m
            target.setPosition(pos);

            Attitude att;
            att.setAttitude(-500, 300, 4500);               // 小幅俯仰/横滚, 航向 45°
            target.setTargetAttitude(att);
            target.setMotion(1500);                          // 速度 15.00 m/s

            body.addTarget(target);
        }

        // ================================================================
        // 第二步：序列化 body
        // ================================================================
        uint32_t body_size = static_cast<uint32_t>(body.size());
        std::vector<uint8_t> body_buffer(body_size);
        if (!body.serialize(body_buffer.data(), body_size))
        {
            std::cerr << "[PUB] 错误: body 序列化失败" << std::endl;
            continue;
        }

        // ================================================================
        // 第三步：构建 MsgHeader (26 字节)
        // ================================================================
        uint64_t now_ms = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count()
        );

        MsgHeader header;
        header.startTag   = 0x7E;       // 起始标识
        header.unitNums   = 1;          // 信息单元个数
        header.srcAddress  = 0x0001;    // 源地址
        header.destAddress = 0xFFFF;    // 目的地址（广播）
        header.msgLength  = 6 + 8 + body_size;  // UnitHeader(6) + sourceTime(8) + body
        header.msgTag     = 0x0000;     // 报文标志
        header.msgID      = msg_id++;   // 报文序号
        header.checkID    = 0;          // 确认号
        header.bak        = 0;          // 备用
        header.timestamp  = now_ms;     // 发送时间戳

        // ================================================================
        // 第四步：构建 UnitHeader (6 字节，不含 sourceTime)
        // ================================================================
        UnitHeader unit_header;
        unit_header.unitLength = body_size;
        unit_header.unitTag    = static_cast<uint16_t>(MessageType::FUSION_MULTI_TARGET);  // 0x01

        // ================================================================
        // 第五步：序列化头部
        // ================================================================
        ByteSequence header_bytes = header.serialize();       // 26 字节
        ByteSequence unit_bytes   = unit_header.serialize();  // 6 字节

        // ================================================================
        // 第六步：构建 sourceTime (8 字节，小端序)
        //
        // 注意: UnitHeader::serialize() 不含 sourceTime 字段，
        //       需业务层手动处理
        // ================================================================
        ByteSequence source_time_bytes(8);
        for (size_t i = 0; i < 8; ++i)
        {
            source_time_bytes[i] = static_cast<uint8_t>((now_ms >> (i * 8)) & 0xFF);
        }

        // ================================================================
        // 第七步：拼接完整报文
        //
        // 报文格式: MsgHeader(26B) + UnitHeader(6B) + sourceTime(8B) + body(NB)
        // ================================================================
        ByteSequence full_msg;
        full_msg.insert(full_msg.end(), header_bytes.begin(), header_bytes.end());
        full_msg.insert(full_msg.end(), unit_bytes.begin(), unit_bytes.end());
        full_msg.insert(full_msg.end(), source_time_bytes.begin(), source_time_bytes.end());
        full_msg.insert(full_msg.end(), body_buffer.begin(), body_buffer.end());

        // ================================================================
        // 第八步：发布
        // ================================================================
        publish(full_msg, topic_name);

        std::cout << "[PUB] msgID=" << msg_id - 1
                  << " targets=" << body.getTargetCount()
                  << " body=" << body_size << "B"
                  << " total=" << full_msg.size() << "B"
                  << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
