/**
 * @file ad_proto_fusion_bridge.cpp
 * @brief 融合层多目标数据 DDS 发送桥接
 *
 * Python 通过 ctypes 传递 FusionTargetData 数组，
 * C++ 序列化为 FusionMultiTargetMsg → PXSB 封包 → DDS 发布。
 *
 * 消息类型: FUSION_MULTI_TARGET (unitTag=0x01)
 * 话题名:   "FusionMultiTarget"
 */

#include "fusion_proto.h"
#include "TopicAPI.h"
#include "PubSubTools.h"
#include "proto.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <vector>
#include <mutex>

using namespace AntiDroneFusionProtocol;

// ============================================================
// POD 结构体 (ctypes 可直接构建)
// ============================================================

#define FUSION_MAX_PRED_POINTS 8

#pragma pack(push, 1)

struct FusionPredPointData {
    int32_t  lng_e7;
    int32_t  lat_e7;
    uint32_t alt;
    int16_t  pitch;
    int16_t  roll;
    uint16_t yaw;
    uint32_t speed;
    uint64_t time;
};

struct FusionTargetData {
    uint8_t  global_id;
    int32_t  lng_e7;
    int32_t  lat_e7;
    uint32_t alt;
    int16_t  pitch;
    int16_t  roll;
    uint16_t yaw;
    uint32_t speed;
    uint8_t  fusion_type;
    uint8_t  confidence;
    uint8_t  source_mask;
    uint8_t  iff;
    uint8_t  pred_point_count;                           // 实际预测点数 (0~8)
    FusionPredPointData pred_points[FUSION_MAX_PRED_POINTS];
};

#pragma pack(pop)

// ============================================================
// 全局状态
// ============================================================

#define TOPIC_FUSION "FusionMultiTarget"

static bool g_fusion_inited = false;
static std::mutex g_fusion_mutex;

// ============================================================
// 发布辅助 — PXSB 封包
// ============================================================

static void publish_fusion(const uint8_t* body, uint32_t body_len,
                           uint32_t device_id, uint64_t timestamp_ms) {
    // UnitHeader (14B): unitLength + unitTag + timestamp
    PXSBProto::UnitHeader uh;
    uh.unitLength = body_len;
    uh.unitTag    = 0x01;  // FUSION_MULTI_TARGET
    uh.timestamp  = timestamp_ms;
    ByteSequence ub = uh.serialize();

    // MsgHeader (26B)
    PXSBProto::MsgHeader mh;
    mh.startTag    = 0x7E;
    mh.unitNums    = 1;
    mh.srcAddress  = static_cast<uint16_t>(device_id & 0xFFFF);
    mh.destAddress = 0;
    mh.msgLength   = 26 + static_cast<uint32_t>(ub.size()) + body_len;
    mh.msgTag      = 0;
    mh.msgID       = 0;
    mh.checkID     = 0;
    mh.bak         = 0;
    mh.timestamp   = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ByteSequence mb = mh.serialize();

    // 拼接: MsgHeader + UnitHeader + body
    ByteSequence pkt;
    pkt.reserve(mb.size() + ub.size() + body_len);
    pkt.insert(pkt.end(), mb.begin(), mb.end());
    pkt.insert(pkt.end(), ub.begin(), ub.end());
    pkt.insert(pkt.end(), body, body + body_len);

    publish(pkt, std::string(TOPIC_FUSION));

    fprintf(stderr, "[Fusion] publish topic=\"%s\" body=%uB pktSize=%zuB\n",
            TOPIC_FUSION, body_len, pkt.size());
}

// ============================================================
// C API
// ============================================================

extern "C" {

int ad_proto_fusion_init(void) {
    std::lock_guard<std::mutex> lock(g_fusion_mutex);
    if (g_fusion_inited) return 0;
    try {
        if (!InitTopic()) {
            fprintf(stderr, "[Fusion] DDS 初始化失败\n");
            return -1;
        }
        bool ok = createWriter(std::string(TOPIC_FUSION), false);
        fprintf(stderr, "[Fusion] createWriter(\"%s\") → %s\n", TOPIC_FUSION, ok ? "OK" : "FAIL");
        if (!ok) return -1;

        g_fusion_inited = true;
        return 0;
    } catch (...) { return -1; }
}

int ad_proto_send_fusion_multi_target(
    uint32_t device_id, uint64_t timestamp_ms,
    const FusionTargetData* targets, uint32_t count)
{
    if (!targets || count == 0) return 0;
    if (!g_fusion_inited) return -1;
    try {
        FusionMultiTargetMsg msg;

        for (uint32_t i = 0; i < count; ++i) {
            const auto& t = targets[i];

            FusionTarget ft;
            ft.setGlobalId(t.global_id);
            {
                AbsPos pos; pos.setPosition(t.lng_e7, t.lat_e7, t.alt);
                ft.setPosition(pos);
            }
            {
                Attitude att; att.setAttitude(t.pitch, t.roll, t.yaw);
                ft.setTargetAttitude(att);
            }
            ft.setMotion(t.speed);
            ft.setTargetIdentity(t.fusion_type, t.confidence, t.iff);
            ft.setTargetBasicInfo(t.global_id, t.source_mask);

            uint8_t n = t.pred_point_count;
            if (n > FUSION_MAX_PRED_POINTS) n = FUSION_MAX_PRED_POINTS;
            for (uint8_t j = 0; j < n; ++j) {
                const auto& pp = t.pred_points[j];
                PredictionPoint p;
                {
                    AbsPos pos; pos.setPosition(pp.lng_e7, pp.lat_e7, pp.alt);
                    p.setAbsPos(pos);
                }
                {
                    Attitude att; att.setAttitude(pp.pitch, pp.roll, pp.yaw);
                    p.setAttitude(att);
                }
                p.setSpeed(pp.speed);
                p.setTime(pp.time);
                ft.addPredictionPoint(p);
            }

            msg.addTarget(ft);
        }

        // 序列化 body
        size_t total = msg.size();
        std::vector<uint8_t> buf(total);
        if (!msg.serialize(buf.data(), total)) return -1;

        // PXSB 封包 → DDS
        publish_fusion(buf.data(), total, device_id, timestamp_ms);
        return 0;
    } catch (...) { return -1; }
}

int ad_proto_fusion_ready(void) { return g_fusion_inited ? 1 : 0; }

void ad_proto_fusion_close(void) { g_fusion_inited = false; }

const char* ad_proto_fusion_version(void) { return "fusion-bridge-1.0.0"; }

} // extern "C"
