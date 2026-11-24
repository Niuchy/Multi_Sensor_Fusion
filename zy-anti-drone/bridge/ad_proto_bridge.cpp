/**
 * 反无数据生成系统 — 标准化协议桥接层
 *
 * 按 dev_proto.h 定义的二进制帧格式序列化传感器数据，通过 DDS 发布。
 * 四种传感器独立话题，固定长度帧，Magic Number 区分。
 */

#include "dev_proto.h"
#include "TopicAPI.h"
#include "proto.h"
#include "PubSubTools.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <mutex>

using namespace AntiDroneDevProtocol;

// ============================================================
// 批量发送用 POD 结构体（ctypes 可直接构建，无需序列化）
// ============================================================

#pragma pack(push, 1)

struct RadarTargetData {
    uint32_t track_id;
    uint8_t  extrap_flag;
    uint32_t range_dm;
    int16_t  azi_cdeg;
    int16_t  ele_cdeg;
    uint32_t height_dm;
    int16_t  radial_vel_dms;
    int32_t  lng_e7;
    int32_t  lat_e7;
    uint32_t alt_dm;
    uint32_t speed_dms;
    uint16_t course_cdeg;
    uint8_t  target_type;
    uint8_t  confidence;
};

struct FreqDetectTargetData {
    uint32_t target_id;
    char     sn[16];
    char     brand_model[32];
    int32_t  lng_e7;
    int32_t  lat_e7;
    uint32_t alt_dm;
    uint32_t height_dm;
    uint32_t speed_dms;
    uint32_t distance_dm;
    int32_t  rctrl_lng_e7;
    int32_t  rctrl_lat_e7;
    uint32_t freq_hz;
    uint32_t bandw_hz;
    int16_t  power_db;
    uint16_t azimuth_cdeg;
    uint64_t discovery_time_ms;
    uint8_t  confidence;
};

struct CamTargetData {
    uint32_t target_id;
    uint8_t  fault_status;
    uint8_t  track_status;
    uint8_t  track_count;
    uint8_t  camfield;
    uint16_t ptz_zeroazi;
    uint16_t ptz_roll;
    int16_t  ptz_pitch;
    uint16_t ptz_yaw;
    int32_t  ptz_lng_e7;
    int32_t  ptz_lat_e7;
    uint32_t ptz_alt_dm;
    int32_t  target_lng_e7;
    int32_t  target_lat_e7;
    uint32_t target_alt_dm;
    uint32_t target_dis_dm;
    uint16_t target_azimuth_cdeg;
    int16_t  target_elevation_cdeg;
    uint8_t  target_type;
    uint8_t  confidence;
};

struct AcousticTargetData {
    uint32_t target_id;
    uint8_t  point_count;
    int16_t  device_roll;
    int16_t  device_pitch;
    int16_t  device_yaw;
    uint16_t target_roll;
    uint16_t target_pitch;
    uint16_t target_yaw;
    uint8_t  target_type;
    uint8_t  confidence;
    int32_t  device_lng_e7;
    int32_t  device_lat_e7;
    uint32_t device_alt_dm;
};

#pragma pack(pop)

// ============================================================
// DDS 参数打印
// ============================================================
static void print_dds_params() {
    fprintf(stderr, "============================================================\n");
    fprintf(stderr, "  DDS 初始化参数 (PubSubTools 库默认值)\n");
    fprintf(stderr, "============================================================\n");
    fprintf(stderr, "  Domain ID             : %u\n", Default_Domain_ID);
    fprintf(stderr, "  Multicast IP          : %s\n", Default_MetatrafficMulticastLocator_IP);
    fprintf(stderr, "  Multicast Port        : %u\n", Default_MetatrafficMulticastLocator_Port);
    fprintf(stderr, "  Discovery Protocol     : SIMPLE (FastDDS 默认)\n");
    fprintf(stderr, "  Topics                 : RadarTarget / ESMTarget / EOTarget / AcousticTarget\n");
    fprintf(stderr, "============================================================\n");
}

// ============================================================
// 全局状态 & 话题名称
// ============================================================

static bool g_inited = false;
static std::mutex g_mutex;
static uint16_t g_msg_seq = 0;
static FILE* g_log_file = nullptr;

// 二进制日志格式 (每条记录):
//   [4B timestamp_ms] [2B frame_type] [4B pkt_len] [NB pkt_data]
#define DDS_LOG_PATH "output/dds_send.log"

#define TOPIC_RADAR    "RadarTarget"
#define TOPIC_FREQ     "ESMTarget"
#define TOPIC_CAMERA   "EOTarget"
#define TOPIC_ACOUSTIC "AcousticTarget"

static void ensure_writer(const char* topic) {
    bool ok = createWriter(std::string(topic), false);
    fprintf(stderr, "[DDS] createWriter(\"%s\") → %s\n", topic, ok ? "OK" : "FAIL");
}

// ============================================================
// PXSB 协议封包: MsgHeader + UnitHeader + dev_proto frame
// ============================================================

/**
 * 将 dev_proto.h 帧包装为 PXSB 标准报文并发布。
 *
 * 报文格式:
 *   MsgHeader (26B) + UnitHeader (6B) + dev_proto_frame (NB)
 *
 * @param frame_data  dev_proto.h 序列化后的帧字节
 * @param frame_len   帧长度
 * @param frame_type  帧类型 (dev_proto.h FrameType, 用作 UnitHeader.unitTag)
 * @param device_id   设备ID (用作 MsgHeader.srcAddress)
 * @param timestamp_ms 时间戳(ms)
 * @param topic       DDS 话题名
 */
static void publish_with_pxsb_header(
    const uint8_t* frame_data, uint32_t frame_len,
    uint16_t frame_type, uint32_t device_id, uint64_t timestamp_ms,
    const char* topic)
{
    // 构建 UnitHeader (14B: unitLength=4B + unitTag=2B + timestamp=8B)
    // timestamp 携带上游消息的时间戳，用于将消息链串起来
    PXSBProto::UnitHeader unit_hdr;
    unit_hdr.unitLength = frame_len;
    unit_hdr.unitTag    = frame_type;
    unit_hdr.timestamp  = timestamp_ms;

    ByteSequence unit_bytes = unit_hdr.serialize();

    // 构建 MsgHeader (26B)
    PXSBProto::MsgHeader msg_hdr;
    msg_hdr.startTag    = 0x7E;
    msg_hdr.unitNums    = 1;
    msg_hdr.srcAddress  = static_cast<uint16_t>(device_id & 0xFFFF);
    msg_hdr.destAddress = 0;
    msg_hdr.msgLength   = 26 + unit_bytes.size() + frame_len;
    msg_hdr.msgTag      = 0;
    msg_hdr.msgID       = ++g_msg_seq;
    msg_hdr.checkID     = 0;
    msg_hdr.bak         = 0;
    msg_hdr.timestamp   = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    ByteSequence msg_bytes = msg_hdr.serialize();

    // 拼接: MsgHeader + UnitHeader + FrameBody
    ByteSequence pkt;
    pkt.reserve(msg_bytes.size() + unit_bytes.size() + frame_len);
    pkt.insert(pkt.end(), msg_bytes.begin(), msg_bytes.end());
    pkt.insert(pkt.end(), unit_bytes.begin(), unit_bytes.end());
    pkt.insert(pkt.end(), frame_data, frame_data + frame_len);

    publish(pkt, std::string(topic));

    // 保存到二进制日志文件，方便接收端排查
    if (g_log_file) {
        uint32_t ts32 = (uint32_t)(timestamp_ms & 0xFFFFFFFF);
        uint32_t len32 = (uint32_t)pkt.size();
        fwrite(&ts32, 4, 1, g_log_file);
        fwrite(&frame_type, 2, 1, g_log_file);
        fwrite(&len32, 4, 1, g_log_file);
        fwrite(pkt.data(), 1, pkt.size(), g_log_file);
        fflush(g_log_file);
    }

    fprintf(stderr, "[DDS] publish topic=\"%s\" frameType=0x%04X pktSize=%zuB seq=%u\n",
            topic, frame_type, pkt.size(), g_msg_seq);
}

// ============================================================
// 批量发布：将 N 个 dev_proto 帧打包为一条 PXSB 多单元报文
// ============================================================

/**
 * 报文格式:
 *   MsgHeader (26B, unitNums=N)
 *   + UnitHeader_0(14B) + frame_0
 *   + UnitHeader_1(14B) + frame_1
 *   + ...
 *
 * @param frames       帧数据指针数组
 * @param frame_lens   每个帧的长度
 * @param frame_types  每个帧的 FrameType
 * @param count        帧数量
 * @param device_id    设备 ID
 * @param timestamp_ms 时间戳 (ms)，来自上游消息
 * @param topic        DDS 话题名
 */
static void publish_batch(
    const uint8_t* const* frames,
    const uint32_t* frame_lens,
    const uint16_t* frame_types,
    uint32_t count,
    uint32_t device_id,
    uint64_t timestamp_ms,
    const char* topic)
{
    if (count == 0) return;

    // 计算总帧长
    uint32_t total_frame_bytes = 0;
    for (uint32_t i = 0; i < count; ++i)
        total_frame_bytes += frame_lens[i];

    // MsgHeader (26B), unitNums = count
    // MsgHeader.timestamp 使用当前发送时间
    PXSBProto::MsgHeader msg_hdr;
    msg_hdr.startTag    = 0x7E;
    msg_hdr.unitNums    = static_cast<uint8_t>(count & 0xFF);
    msg_hdr.srcAddress  = static_cast<uint16_t>(device_id & 0xFFFF);
    msg_hdr.destAddress = 0;
    msg_hdr.msgLength   = 26 + count * 14 + total_frame_bytes;
    msg_hdr.msgTag      = 0;
    msg_hdr.msgID       = ++g_msg_seq;
    msg_hdr.checkID     = 0;
    msg_hdr.bak         = 0;
    msg_hdr.timestamp   = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    ByteSequence msg_bytes = msg_hdr.serialize();

    // 拼接: MsgHeader + UnitHeader_0 + frame_0 + UnitHeader_1 + frame_1 + ...
    // UnitHeader.timestamp 携带上游消息的时间戳，用于将消息链串起来
    ByteSequence pkt;
    pkt.reserve(msg_bytes.size() + count * 14 + total_frame_bytes);
    pkt.insert(pkt.end(), msg_bytes.begin(),  msg_bytes.end());
    for (uint32_t i = 0; i < count; ++i) {
        PXSBProto::UnitHeader uh;
        uh.unitLength = frame_lens[i];
        uh.unitTag    = frame_types[i];
        uh.timestamp  = timestamp_ms;
        ByteSequence ub = uh.serialize();
        pkt.insert(pkt.end(), ub.begin(), ub.end());
        pkt.insert(pkt.end(), frames[i], frames[i] + frame_lens[i]);
    }

    publish(pkt, std::string(topic));

    // 二进制日志
    if (g_log_file) {
        uint32_t ts32 = static_cast<uint32_t>(timestamp_ms & 0xFFFFFFFF);
        uint32_t len32 = static_cast<uint32_t>(pkt.size());
        uint16_t tag   = frame_types[0];
        fwrite(&ts32, 4, 1, g_log_file);
        fwrite(&tag,   2, 1, g_log_file);
        fwrite(&len32, 4, 1, g_log_file);
        fwrite(pkt.data(), 1, pkt.size(), g_log_file);
        fflush(g_log_file);
    }

    fprintf(stderr, "[DDS] publish_batch topic=\"%s\" frames=%u pktSize=%zuB seq=%u\n",
            topic, count, pkt.size(), g_msg_seq);
}

// ============================================================
// extern "C" API
// ============================================================

extern "C" {

/**
 * 初始化协议桥接（全部使用 third_party 库默认值）。
 * @return 0=成功, -1=失败
 */
int ad_proto_init(void) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_inited) return 0;
    try {
        InitTopic();  // 全部参数由 PubSubTools 库内部决定
        print_dds_params();

        // 打开二进制日志文件
        g_log_file = fopen(DDS_LOG_PATH, "wb");
        if (g_log_file) {
            fprintf(stderr, "[DDS] 日志文件: %s\n", DDS_LOG_PATH);
        } else {
            fprintf(stderr, "[DDS] 无法创建日志文件: %s\n", DDS_LOG_PATH);
        }

        // 预创建四个传感器话题 Writer
        ensure_writer(TOPIC_RADAR);
        ensure_writer(TOPIC_FREQ);
        ensure_writer(TOPIC_CAMERA);
        ensure_writer(TOPIC_ACOUSTIC);

        g_inited = true;
        return 0;
    } catch (...) {
        return -1;
    }
}

// ============================================================
// 雷达目标帧 (66 字节)
// ============================================================

int ad_proto_send_radar(
    uint32_t device_id, uint64_t timestamp_ms,
    uint32_t track_id, uint8_t extrap_flag,
    // L1 量测 (雷达局部坐标)
    uint32_t range_dm, int16_t azi_cdeg, int16_t ele_cdeg,
    uint32_t height_dm, int16_t radial_vel_dms,
    // L2 航迹 (WGS84 全局坐标)
    int32_t lng_e7, int32_t lat_e7, uint32_t alt_dm,
    uint32_t speed_dms, uint16_t course_cdeg,
    // L3 识别
    uint8_t target_type, uint8_t confidence
) {
    try {
        RadarTargetFrame frame;
        frame.setDeviceId(device_id);
        frame.setTimestamp(timestamp_ms);
        frame.setTargetId(track_id);
        frame.setExtrapolationFlag(extrap_flag);
        frame.setMeasurement(range_dm, azi_cdeg, ele_cdeg);
        frame.setHeightAndRadialVel(height_dm, radial_vel_dms);
        frame.setPosition(lng_e7, lat_e7, alt_dm);
        frame.setMotion(speed_dms, course_cdeg);
        frame.setTargetInfo(target_type, confidence);

        uint8_t buf[RadarTargetFrame::size()];
        if (!frame.serialize(buf, sizeof(buf))) return -1;
        publish_with_pxsb_header(buf, sizeof(buf), 0x1A01, device_id, timestamp_ms, TOPIC_RADAR);
        return 0;
    } catch (...) { return -1; }
}

// ============================================================
// 频侦(电侦)目标帧 (139 字节)
// ============================================================

int ad_proto_send_freqdetect(
    uint32_t device_id, uint64_t timestamp_ms,
    uint32_t target_id, const char* sn, const char* brand_model,
    int32_t lng_e7, int32_t lat_e7, uint32_t alt_dm, uint32_t height_dm,
    uint32_t speed_dms, uint32_t distance_dm,
    int32_t rctrl_lng_e7, int32_t rctrl_lat_e7,
    uint32_t freq_hz, uint32_t bandw_hz, int16_t power_db,
    uint16_t azimuth_cdeg, uint64_t discovery_time_ms, uint8_t confidence
) {
    try {
        FrequencyDetectFrame frame;
        frame.setDeviceId(device_id);
        frame.setTimestamp(timestamp_ms);
        frame.setTargetIdAndConfidence(target_id, confidence);
        frame.setStringInfo(sn ? sn : "", brand_model ? brand_model : "");
        frame.setUavPosition(lng_e7, lat_e7, alt_dm);
        frame.setHeightInfo(alt_dm, height_dm);
        frame.setMotionInfo(speed_dms, distance_dm);
        frame.setRctrlPosition(rctrl_lng_e7, rctrl_lat_e7);
        frame.setSignalParams(freq_hz, bandw_hz, power_db);
        // 手动设置方位角 (没有包装的 set 函数)
        frame.getBody().azimuth_cdeg = azimuth_cdeg;
        frame.getBody().discovery_time = discovery_time_ms;

        uint8_t buf[FrequencyDetectFrame::size()];
        if (!frame.serialize(buf, sizeof(buf))) return -1;
        publish_with_pxsb_header(buf, sizeof(buf), 0x1B01, device_id, timestamp_ms, TOPIC_FREQ);
        return 0;
    } catch (...) { return -1; }
}

// ============================================================
// 光电目标帧 (79 字节)
// ============================================================

int ad_proto_send_camera(
    uint32_t device_id, uint64_t timestamp_ms,
    uint32_t target_id,
    uint8_t fault_status, uint8_t track_status, uint8_t track_count, uint8_t camfield,
    uint16_t ptz_zeroazi, uint16_t ptz_roll, int16_t ptz_pitch, uint16_t ptz_yaw,
    int32_t ptz_lng_e7, int32_t ptz_lat_e7, uint32_t ptz_alt_dm,
    int32_t target_lng_e7, int32_t target_lat_e7, uint32_t target_alt_dm,
    uint32_t target_dis_dm, uint16_t target_azimuth_cdeg, int16_t target_elevation_cdeg,
    uint8_t target_type, uint8_t confidence
) {
    try {
        CamTargetFrame frame;
        frame.setDeviceId(device_id);
        frame.setTimestamp(timestamp_ms);
        frame.setTargetId(target_id);
        frame.setDeviceStatus(fault_status, track_status, track_count);
        frame.setCamfield(camfield);
        frame.setPtzPosition(ptz_lng_e7, ptz_lat_e7, ptz_alt_dm);
        frame.setPtzAttitude(ptz_roll, ptz_pitch, ptz_yaw);
        frame.getBody().ptz_zeroazi = ptz_zeroazi;
        frame.setTargetPosition(target_lng_e7, target_lat_e7, target_alt_dm);
        frame.setTargetDistanceInfo(target_dis_dm, target_alt_dm);
        frame.setTargetAngles(target_azimuth_cdeg, target_elevation_cdeg);
        frame.setTargetInfo(target_type, confidence);

        uint8_t buf[CamTargetFrame::size()];
        if (!frame.serialize(buf, sizeof(buf))) return -1;
        publish_with_pxsb_header(buf, sizeof(buf), 0x1C01, device_id, timestamp_ms, TOPIC_CAMERA);
        return 0;
    } catch (...) { return -1; }
}

// ============================================================
// 声探测目标帧 (55 字节)
// ============================================================

int ad_proto_send_acoustic(
    uint32_t device_id, uint64_t timestamp_ms,
    uint32_t target_id, uint8_t point_count,
    int16_t device_roll, int16_t device_pitch, int16_t device_yaw,
    uint16_t target_roll, uint16_t target_pitch, uint16_t target_yaw,
    uint8_t target_type, uint8_t confidence,
    int32_t device_lng_e7, int32_t device_lat_e7, uint32_t device_alt_dm
) {
    try {
        AcousticTargetFrame frame;
        frame.setDeviceId(device_id);
        frame.setTimestamp(timestamp_ms);
        frame.setTargetBasicInfo(target_id, point_count);
        frame.setDevicePosition(device_lng_e7, device_lat_e7, device_alt_dm);
        frame.setDeviceAttitude(device_roll, device_pitch, device_yaw);
        frame.setTargetRelativeAngles(target_roll, target_pitch, target_yaw);
        frame.setTargetInfo(target_type, confidence);

        uint8_t buf[AcousticTargetFrame::size()];
        if (!frame.serialize(buf, sizeof(buf))) return -1;
        publish_with_pxsb_header(buf, sizeof(buf), 0x1D01, device_id, timestamp_ms, TOPIC_ACOUSTIC);
        return 0;
    } catch (...) { return -1; }
}

// ============================================================
// 批量发送接口: 一次 FFI 调用 → 一条 PXSB 多单元报文 → 一次 DDS publish
// ============================================================

int ad_proto_send_radar_batch(
    uint32_t device_id, uint64_t timestamp_ms,
    const RadarTargetData* targets, uint32_t count)
{
    if (!targets || count == 0) return 0;
    try {
        std::vector<std::vector<uint8_t>> bufs(count);
        std::vector<const uint8_t*> ptrs(count);
        std::vector<uint32_t> lens(count);
        std::vector<uint16_t> types(count);

        for (uint32_t i = 0; i < count; ++i) {
            const auto& t = targets[i];
            RadarTargetFrame frame;
            frame.setDeviceId(device_id);
            frame.setTimestamp(timestamp_ms);
            frame.setTargetId(t.track_id);
            frame.setExtrapolationFlag(t.extrap_flag);
            frame.setMeasurement(t.range_dm, t.azi_cdeg, t.ele_cdeg);
            frame.setHeightAndRadialVel(t.height_dm, t.radial_vel_dms);
            frame.setPosition(t.lng_e7, t.lat_e7, t.alt_dm);
            frame.setMotion(t.speed_dms, t.course_cdeg);
            frame.setTargetInfo(t.target_type, t.confidence);

            bufs[i].resize(RadarTargetFrame::size());
            if (!frame.serialize(bufs[i].data(), bufs[i].size())) return -1;
            ptrs[i]  = bufs[i].data();
            lens[i]  = static_cast<uint32_t>(bufs[i].size());
            types[i] = 0x1A01;
        }
        publish_batch(ptrs.data(), lens.data(), types.data(), count,
                      device_id, timestamp_ms, TOPIC_RADAR);
        return 0;
    } catch (...) { return -1; }
}

int ad_proto_send_freqdetect_batch(
    uint32_t device_id, uint64_t timestamp_ms,
    const FreqDetectTargetData* targets, uint32_t count)
{
    if (!targets || count == 0) return 0;
    try {
        std::vector<std::vector<uint8_t>> bufs(count);
        std::vector<const uint8_t*> ptrs(count);
        std::vector<uint32_t> lens(count);
        std::vector<uint16_t> types(count);

        for (uint32_t i = 0; i < count; ++i) {
            const auto& t = targets[i];
            FrequencyDetectFrame frame;
            frame.setDeviceId(device_id);
            frame.setTimestamp(timestamp_ms);
            frame.setTargetIdAndConfidence(t.target_id, t.confidence);
            frame.setStringInfo(t.sn, t.brand_model);
            frame.setUavPosition(t.lng_e7, t.lat_e7, t.alt_dm);
            frame.setHeightInfo(t.alt_dm, t.height_dm);
            frame.setMotionInfo(t.speed_dms, t.distance_dm);
            frame.setRctrlPosition(t.rctrl_lng_e7, t.rctrl_lat_e7);
            frame.setSignalParams(t.freq_hz, t.bandw_hz, t.power_db);
            frame.getBody().azimuth_cdeg   = t.azimuth_cdeg;
            frame.getBody().discovery_time = t.discovery_time_ms;

            bufs[i].resize(FrequencyDetectFrame::size());
            if (!frame.serialize(bufs[i].data(), bufs[i].size())) return -1;
            ptrs[i]  = bufs[i].data();
            lens[i]  = static_cast<uint32_t>(bufs[i].size());
            types[i] = 0x1B01;
        }
        publish_batch(ptrs.data(), lens.data(), types.data(), count,
                      device_id, timestamp_ms, TOPIC_FREQ);
        return 0;
    } catch (...) { return -1; }
}

int ad_proto_send_camera_batch(
    uint32_t device_id, uint64_t timestamp_ms,
    const CamTargetData* targets, uint32_t count)
{
    if (!targets || count == 0) return 0;
    try {
        std::vector<std::vector<uint8_t>> bufs(count);
        std::vector<const uint8_t*> ptrs(count);
        std::vector<uint32_t> lens(count);
        std::vector<uint16_t> types(count);

        for (uint32_t i = 0; i < count; ++i) {
            const auto& t = targets[i];
            CamTargetFrame frame;
            frame.setDeviceId(device_id);
            frame.setTimestamp(timestamp_ms);
            frame.setTargetId(t.target_id);
            frame.setDeviceStatus(t.fault_status, t.track_status, t.track_count);
            frame.setCamfield(t.camfield);
            frame.setPtzPosition(t.ptz_lng_e7, t.ptz_lat_e7, t.ptz_alt_dm);
            frame.setPtzAttitude(t.ptz_roll, t.ptz_pitch, t.ptz_yaw);
            frame.getBody().ptz_zeroazi = t.ptz_zeroazi;
            frame.setTargetPosition(t.target_lng_e7, t.target_lat_e7, t.target_alt_dm);
            frame.setTargetDistanceInfo(t.target_dis_dm, t.target_alt_dm);
            frame.setTargetAngles(t.target_azimuth_cdeg, t.target_elevation_cdeg);
            frame.setTargetInfo(t.target_type, t.confidence);

            bufs[i].resize(CamTargetFrame::size());
            if (!frame.serialize(bufs[i].data(), bufs[i].size())) return -1;
            ptrs[i]  = bufs[i].data();
            lens[i]  = static_cast<uint32_t>(bufs[i].size());
            types[i] = 0x1C01;
        }
        publish_batch(ptrs.data(), lens.data(), types.data(), count,
                      device_id, timestamp_ms, TOPIC_CAMERA);
        return 0;
    } catch (...) { return -1; }
}

int ad_proto_send_acoustic_batch(
    uint32_t device_id, uint64_t timestamp_ms,
    const AcousticTargetData* targets, uint32_t count)
{
    if (!targets || count == 0) return 0;
    try {
        std::vector<std::vector<uint8_t>> bufs(count);
        std::vector<const uint8_t*> ptrs(count);
        std::vector<uint32_t> lens(count);
        std::vector<uint16_t> types(count);

        for (uint32_t i = 0; i < count; ++i) {
            const auto& t = targets[i];
            AcousticTargetFrame frame;
            frame.setDeviceId(device_id);
            frame.setTimestamp(timestamp_ms);
            frame.setTargetBasicInfo(t.target_id, t.point_count);
            frame.setDevicePosition(t.device_lng_e7, t.device_lat_e7, t.device_alt_dm);
            frame.setDeviceAttitude(t.device_roll, t.device_pitch, t.device_yaw);
            frame.setTargetRelativeAngles(t.target_roll, t.target_pitch, t.target_yaw);
            frame.setTargetInfo(t.target_type, t.confidence);

            bufs[i].resize(AcousticTargetFrame::size());
            if (!frame.serialize(bufs[i].data(), bufs[i].size())) return -1;
            ptrs[i]  = bufs[i].data();
            lens[i]  = static_cast<uint32_t>(bufs[i].size());
            types[i] = 0x1D01;
        }
        publish_batch(ptrs.data(), lens.data(), types.data(), count,
                      device_id, timestamp_ms, TOPIC_ACOUSTIC);
        return 0;
    } catch (...) { return -1; }
}

/** 获取就绪状态 */
int ad_proto_ready(void) { return g_inited ? 1 : 0; }

/** 关闭 (不析构 DDS，避免 FastDDS 退出 crash) */
void ad_proto_close(void) {
    if (g_log_file) { fclose(g_log_file); g_log_file = nullptr; }
    g_inited = false;
}

const char* ad_proto_version(void) { return "ad-proto-2.0.0"; }

} // extern "C"
