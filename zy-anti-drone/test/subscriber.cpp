/**
 * @file subscriber.cpp
 * @brief 融合多目标消息订阅测试
 *
 * 从 DDS 主题接收完整报文（MsgHeader + UnitHeader + sourceTime + body），
 * 逐层解析并打印 FusionMultiTargetMsg 的详细内容。
 */

#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cstring>
#include <ctime>

#include "TopicAPI.h"
#include "proto.h"
#include "fusion_proto.h"

using namespace PXSBProto;
using namespace AntiDroneFusionProtocol;

// ============================================================================
// 枚举值 -> 字符串映射
// ============================================================================

static const char* targetTypeName(uint8_t type)
{
    switch (static_cast<TargetType>(type))
    {
    case TargetType::DRONE:       return "无人机";
    case TargetType::BIRD:        return "鸟";
    case TargetType::LARGE_TARGET: return "大型目标";
    case TargetType::VEHICLE:     return "车辆";
    case TargetType::PERSON:      return "人员";
    case TargetType::OTHER:       return "其它";
    case TargetType::INVALID:     return "无效";
    default:                      return "未知";
    }
}

static const char* sourceMaskName(uint8_t mask)
{
    switch (static_cast<SourceMask>(mask))
    {
    case SourceMask::RADAR:              return "雷达";
    case SourceMask::EO:                 return "光电";
    case SourceMask::ES:                 return "电侦";
    case SourceMask::ACOUSTIC:           return "声探";
    case SourceMask::RADAR_EO:           return "雷达+光电";
    case SourceMask::RADAR_ES:           return "雷达+电侦";
    case SourceMask::RADAR_ACOUSTIC:     return "雷达+声探";
    case SourceMask::EO_ES:              return "光电+电侦";
    case SourceMask::EO_ACOUSTIC:        return "光电+声探";
    case SourceMask::ES_ACOUSTIC:        return "电侦+声探";
    case SourceMask::RADAR_EO_ES:        return "雷达+光电+电侦";
    case SourceMask::RADAR_EO_ACOUSTIC:  return "雷达+光电+声探";
    case SourceMask::EO_ES_ACOUSTIC:     return "光电+电侦+声探";
    case SourceMask::ALL:                return "四源融合";
    case SourceMask::INVALID:            return "无效";
    default:                             return "未知";
    }
}

static const char* iffName(uint8_t iff)
{
    switch (iff)
    {
    case 0: return "不明";
    case 1: return "我方";
    case 2: return "敌方";
    default: return "未知";
    }
}

// ============================================================================
// 回调函数：接收并解析完整报文
// ============================================================================
void onMessage(const ByteSequence &bytes)
{
    if (bytes.size() < 26 + 6 + 8)  // MsgHeader(26) + UnitHeader(6) + sourceTime(8)
    {
        std::cerr << "[SUB] 错误: 报文太短 (" << bytes.size() << " 字节)" << std::endl;
        return;
    }

    size_t pos = 0;

    // --- 解析 MsgHeader (26 字节) ---
    MsgHeader header;
    header.deserialize(bytes, pos);

    // --- 解析 UnitHeader (6 字节) ---
    UnitHeader unit_header;
    unit_header.deserialize(bytes, pos);

    // --- 提取 sourceTime (8 字节，小端序) ---
    uint64_t source_time = 0;
    for (size_t i = 0; i < 8; ++i)
    {
        source_time |= static_cast<uint64_t>(bytes[pos++]) << (i * 8);
    }

    // --- 提取 body ---
    uint32_t body_len = static_cast<uint32_t>(bytes.size() - pos);
    if (body_len == 0)
    {
        std::cerr << "[SUB] 错误: body 为空" << std::endl;
        return;
    }

    FusionMultiTargetMsg body;
    if (!body.deserialize(&bytes[pos], body_len))
    {
        std::cerr << "[SUB] 错误: body 反序列化失败" << std::endl;
        return;
    }

    // ================================================================
    // 打印解析结果
    // ================================================================
    std::cout << "\n========================================" << std::endl;
    std::cout << "[SUB] 收到消息" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 报文头信息
    std::cout << "[MsgHeader]" << std::endl;
    std::cout << "  startTag   = 0x" << std::hex << std::uppercase
              << static_cast<int>(header.startTag) << std::dec << std::endl;
    std::cout << "  unitNums   = " << static_cast<int>(header.unitNums) << std::endl;
    std::cout << "  srcAddr    = 0x" << std::hex << header.srcAddress << std::dec << std::endl;
    std::cout << "  destAddr   = 0x" << std::hex << header.destAddress << std::dec << std::endl;
    std::cout << "  msgLength  = " << header.msgLength << " bytes" << std::endl;
    std::cout << "  msgID      = " << header.msgID << std::endl;
    std::cout << "  timestamp  = " << header.timestamp << " ms" << std::endl;

    // 信息头信息
    std::cout << "[UnitHeader]" << std::endl;
    std::cout << "  unitLength = " << unit_header.unitLength << " bytes" << std::endl;
    std::cout << "  unitTag    = 0x" << std::hex << std::uppercase
              << unit_header.unitTag << std::dec;
    if (unit_header.unitTag == static_cast<uint16_t>(MessageType::FUSION_MULTI_TARGET))
    {
        std::cout << " (FUSION_MULTI_TARGET)";
    }
    std::cout << std::endl;
    std::cout << "  sourceTime = " << source_time << " ms" << std::endl;

    // 消息体
    std::cout << "[Body] target_count = " << body.getTargetCount() << std::endl;

    for (size_t i = 0; i < body.getTargetCount(); ++i)
    {
        const FusionTarget &t = body.getTargets()[i];

        std::cout << "----------------------------------------" << std::endl;
        std::cout << "  Target #" << i + 1 << ":" << std::endl;
        std::cout << "    global_id   = " << static_cast<int>(t.getGlobalId()) << std::endl;
        std::cout << "    fusion_type = " << targetTypeName(t.getFusionType())
                  << " (" << static_cast<int>(t.getFusionType()) << ")" << std::endl;
        std::cout << "    confidence  = " << static_cast<int>(t.getConfidence()) << "%" << std::endl;
        std::cout << "    source_mask = " << sourceMaskName(t.getSourceMask())
                  << " (0x" << std::hex << static_cast<int>(t.getSourceMask()) << std::dec << ")" << std::endl;
        std::cout << "    iff         = " << iffName(t.getIff())
                  << " (" << static_cast<int>(t.getIff()) << ")" << std::endl;

        const AbsPos &ap = t.getAbsPos();
        std::cout << "    position    = (lng:" << ap.getLng() << "e-7°, "
                  << "lat:" << ap.getLat() << "e-7°, "
                  << "alt:" << ap.getAlt() << "*0.1m)" << std::endl;

        const Attitude &att = t.getAttitude();
        std::cout << "    attitude    = (pitch:" << att.getPitch() << "*0.01°, "
                  << "roll:" << att.getRoll() << "*0.01°, "
                  << "yaw:" << att.getYaw() << "*0.01°)" << std::endl;
        std::cout << "    speed       = " << t.getSpeed() << "*0.01m/s" << std::endl;

        // 预测轨迹点
        const auto &preds = t.getPredictionPoints();
        std::cout << "    pred_points = " << static_cast<int>(t.getPredPointCount()) << std::endl;
        for (size_t j = 0; j < preds.size(); ++j)
        {
            const PredictionPoint &pp = preds[j];
            std::cout << "      [" << j << "] time=" << pp.getTime() << "ms"
                      << " speed=" << pp.getSpeed() << "*0.01m/s"
                      << " pos=(" << pp.getAbsPos().getLng() << "e-7°,"
                      << pp.getAbsPos().getLat() << "e-7°,"
                      << pp.getAbsPos().getAlt() << "*0.1m)"
                      << std::endl;
        }
    }
    std::cout << "========================================\n" << std::endl;
}

int main()
{
    // 初始化 Topic 接口
    InitTopic();

    std::string topic_name = "fusion_test";
    createReader(topic_name, false);

    // 注册回调函数
    modifyProcess(topic_name, onMessage);

    std::cout << "[SUB] 开始监听主题: " << topic_name << std::endl;
    std::cout << "[SUB] 等待消息..." << std::endl;

    while (true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    return 0;
}
