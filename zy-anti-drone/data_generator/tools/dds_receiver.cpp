/**
 * @file dds_receiver.cpp
 * @brief 反无数据生成系统 — DDS 接收端（本地测试用）
 *
 * 接收 ad_proto_bridge.cpp 通过 DDS 发出的四类传感器数据：
 *   - RadarTarget    0x1A01  雷达目标  66B
 *   - ESMTarget      0x1B01  频侦目标 139B
 *   - EOTarget       0x1C01  光电目标  79B
 *   - AcousticTarget 0x1D01  声探测目标 67B
 *
 * 支持单帧报文 (unitNums=1) 和批量多帧报文 (unitNums>1)。
 * 数据格式 (PXSB 协议):
 *   MsgHeader(26B) + UnitHeader_0(14B) + frame_0 + UnitHeader_1(14B) + frame_1 + ...
 *
 * 用法: ./dds_receiver
 * DDS 参数全部使用 third_party 库默认值
 */

#include "dev_proto.h"
#include "TopicAPI.h"
#include "PubSubTools.h"
#include "proto.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <atomic>
#include <thread>
#include <chrono>
#include <signal.h>

using namespace AntiDroneDevProtocol;

// ============================================================
// 统计计数器
// ============================================================
static std::atomic<uint64_t> g_radar_count{0};
static std::atomic<uint64_t> g_esm_count{0};
static std::atomic<uint64_t> g_eo_count{0};
static std::atomic<uint64_t> g_acoustic_count{0};
static std::atomic<bool> g_running{true};

// ============================================================
// 辅助函数
// ============================================================

/** 将 Unix 毫秒时间戳转为可读字符串 */
static std::string ts_to_str(uint64_t ts_ms) {
    time_t sec = static_cast<time_t>(ts_ms / 1000);
    int ms = static_cast<int>(ts_ms % 1000);
    char buf[32];
    struct tm tm_info;
    localtime_r(&sec, &tm_info);
    strftime(buf, sizeof(buf), "%H:%M:%S", &tm_info);
    char out[40];
    snprintf(out, sizeof(out), "%s.%03d", buf, ms);
    return std::string(out);
}

/** 通过 Magic Number 获取传感器名称 */
static const char* sensor_name_by_magic(uint32_t magic) {
    switch (magic) {
        case static_cast<uint32_t>(MagicNumber::RADAR_MAGIC):       return "Radar";
        case static_cast<uint32_t>(MagicNumber::FREQ_DETECT_MAGIC): return "ESM";
        case static_cast<uint32_t>(MagicNumber::CAMERA_MAGIC):      return "EO";
        case static_cast<uint32_t>(MagicNumber::ACOUSTIC_MAGIC):    return "Acoustic";
        default: return "Unknown";
    }
}

/** 目标类型 → 字符串 */
static const char* target_type_str(uint8_t t) {
    switch (t) {
        case 0: return "未知";
        case 1: return "鸟";
        case 2: return "无人机";
        case 3: return "车辆";
        case 4: return "人员";
        case 5: return "大型目标";
        case 6: return "其他";
        default: return "?";
    }
}

// ============================================================
// 回调前向声明（dispatch_pxsb_message 调它们，定义在后）
// ============================================================
static void on_radar(const PXSBProto::MsgHeader& msg_hdr,
                      const PXSBProto::UnitHeader& unit_hdr,
                      const uint8_t* frame_data, uint32_t frame_len);
static void on_esm(const PXSBProto::MsgHeader& msg_hdr,
                    const PXSBProto::UnitHeader& unit_hdr,
                    const uint8_t* frame_data, uint32_t frame_len);
static void on_eo(const PXSBProto::MsgHeader& msg_hdr,
                   const PXSBProto::UnitHeader& unit_hdr,
                   const uint8_t* frame_data, uint32_t frame_len);
static void on_acoustic(const PXSBProto::MsgHeader& msg_hdr,
                         const PXSBProto::UnitHeader& unit_hdr,
                         const uint8_t* frame_data, uint32_t frame_len);

// ============================================================
// PXSB 封套解析 — 支持单帧及多帧批量报文
// ============================================================

/**
 * 报文格式:
 *   MsgHeader(26B) + UnitHeader_0(14B) + frame_0 + UnitHeader_1(14B) + frame_1 + ...
 *
 * @return 成功解析的帧数量
 */
static int dispatch_pxsb_message(const ByteSequence& data) {
    constexpr uint32_t MSG_HDR_SIZE = 26;
    constexpr uint32_t UNIT_HDR_SIZE = 14;

    if (data.size() < MSG_HDR_SIZE + UNIT_HDR_SIZE) {
        fprintf(stderr, "[PXSB] 数据太短: %zu bytes (最小 %u)\n",
                data.size(), MSG_HDR_SIZE + UNIT_HDR_SIZE);
        return 0;
    }

    // 1. 解析 MsgHeader
    size_t pos = 0;
    PXSBProto::MsgHeader msg_hdr;
    msg_hdr.deserialize(data, pos);

    if (msg_hdr.startTag != 0x7E) {
        fprintf(stderr, "[PXSB] 起始标识错误: 0x%02X (期望 0x7E)\n", msg_hdr.startTag);
        return 0;
    }

    uint8_t unit_count = msg_hdr.unitNums;
    if (unit_count == 0) return 0;

    // 2. 逐个解析 UnitHeader + frame
    int parsed = 0;
    for (uint8_t u = 0; u < unit_count; ++u) {
        if (data.size() < pos + UNIT_HDR_SIZE) {
            fprintf(stderr, "[PXSB] 第 %u 个 UnitHeader 越界\n", u);
            break;
        }

        PXSBProto::UnitHeader unit_hdr;
        unit_hdr.deserialize(data, pos);

        uint32_t frame_len = unit_hdr.unitLength;
        if (data.size() < pos + frame_len) {
            fprintf(stderr, "[PXSB] 第 %u 帧越界: 需要 %u 剩余 %zu\n",
                    u, frame_len, data.size() - pos);
            break;
        }

        const uint8_t* frame_start = data.data() + pos;
        pos += frame_len;

        // 3. 按 unitTag 分发到对应处理器
        switch (unit_hdr.unitTag) {
            case 0x1A01: on_radar(msg_hdr, unit_hdr, frame_start, frame_len); break;
            case 0x1B01: on_esm(msg_hdr, unit_hdr, frame_start, frame_len);   break;
            case 0x1C01: on_eo(msg_hdr, unit_hdr, frame_start, frame_len);    break;
            case 0x1D01: on_acoustic(msg_hdr, unit_hdr, frame_start, frame_len); break;
            default:
                fprintf(stderr, "[PXSB] 未知 unitTag: 0x%04X (跳过)\n", unit_hdr.unitTag);
                break;
        }
        ++parsed;
    }

    return parsed;
}

// ============================================================
// 打印帧头
// ============================================================
static void print_msg_header(const PXSBProto::MsgHeader& mh, const PXSBProto::UnitHeader& uh) {
    printf("  [PXSB] src=0x%04X dst=0x%04X msgID=%u units=%u unitTag=0x%04X unitLen=%u\n",
           mh.srcAddress, mh.destAddress, mh.msgID,
           mh.unitNums, uh.unitTag, uh.unitLength);
}

static void print_frame_header(const DevFrameHeader& h) {
    printf("  [Frame] magic=0x%08X(%s) ver=%u ftype=0x%04X dtype=%u len=%u dev=%u ts=%s\n",
           h.getMagic(), sensor_name_by_magic(h.getMagic()),
           h.getProtoVer(), h.getFrameType(), h.getDataType(),
           h.getLength(), h.getDeviceId(), ts_to_str(h.getTimestamp()).c_str());
}

// ============================================================
// 数据回调 — 每个 dev_proto 帧 (由 dispatch_pxsb_message 调用)
// ============================================================
static void on_radar(const PXSBProto::MsgHeader& msg_hdr,
                      const PXSBProto::UnitHeader& unit_hdr,
                      const uint8_t* frame_data, uint32_t frame_len) {

    RadarTargetFrame frame;
    if (!frame.deserialize(frame_data, frame_len)) {
        fprintf(stderr, "[Radar] 反序列化失败 (magic=0x%08X)\n",
                *reinterpret_cast<const uint32_t*>(frame_data));
        return;
    }

    auto& b = frame.getBody();
    uint64_t n = g_radar_count.fetch_add(1) + 1;

    printf("\n╔══ Radar #%lu ═══════════════════════════════════\n", n);
    print_msg_header(msg_hdr, unit_hdr);
    print_frame_header(frame.getHeader());
    printf("  [Body]  track_id=%u\n", b.getTargetId());
    printf("          L1 量测: dis=%.1fm azi=%.2f° ele=%.2f° height=%.1fm radial_vel=%.1fm/s\n",
           b.dis * 0.1f, b.azi * 0.01f, b.ele * 0.01f,
           b.height * 0.1f, b.radial_vel * 0.1f);
    printf("          L2 航迹: lng=%.7f° lat=%.7f° alt=%.1fm speed=%.1fm/s course=%.2f° tag=%s\n",
           b.lng * 1e-7f, b.lat * 1e-7f, b.alt * 0.1f,
           b.speed * 0.1f, b.course * 0.01f,
           b.getExtrapolationFlag() ? "真实点" : "外推点");
    printf("          L3 识别: type=%s conf=%u%%\n",
           target_type_str(b.getTargetType()), b.getTargetConfidence());
    printf("  [Tail]  crc32=0x%08X\n", frame.getCrc32());
    fflush(stdout);
}

// ============================================================
// 数据回调 — 频侦 / ESM
// ============================================================
static void on_esm(const PXSBProto::MsgHeader& msg_hdr,
                    const PXSBProto::UnitHeader& unit_hdr,
                    const uint8_t* frame_data, uint32_t frame_len) {

    FrequencyDetectFrame frame;
    if (!frame.deserialize(frame_data, frame_len)) {
        fprintf(stderr, "[ESM] 反序列化失败 (magic=0x%08X)\n",
                *reinterpret_cast<const uint32_t*>(frame_data));
        return;
    }

    auto& b = frame.getBody();
    uint64_t n = g_esm_count.fetch_add(1) + 1;

    printf("\n╔══ ESM #%lu ═══════════════════════════════════\n", n);
    print_msg_header(msg_hdr, unit_hdr);
    print_frame_header(frame.getHeader());
    printf("  [Body]  target_id=%u\n", b.getTargetId());

    char sn[17] = {};
    std::memcpy(sn, b.sn_id.data(), 16);
    printf("          SN: %s\n", sn);

    char brand[33] = {};
    std::memcpy(brand, b.brand_model.data(), 32);
    printf("          Brand/Model: %s\n", brand);

    printf("          UAV位置: lng=%.7f° lat=%.7f° alt=%.1fm height=%.1fm\n",
           b.lng_e7 * 1e-7f, b.lat_e7 * 1e-7f,
           b.alt * 0.1f, b.height * 0.1f);
    printf("          运动: speed=%.1fm/s distance=%.1fm\n",
           b.speed * 0.1f, b.distance * 0.1f);
    printf("          遥控器: lng=%.7f° lat=%.7f°\n",
           b.rctrl_lng_e7 * 1e-7f, b.rctrl_lat_e7 * 1e-7f);
    printf("          信号: freq=%.3fMHz bw=%.3fkHz power=%.1fdBm azimuth=%.2f°\n",
           b.freq_hz * 1e-6f, b.bandw_hz * 1e-3f,
           b.power_db * 0.1f, b.azimuth_cdeg * 0.01f);
    printf("          发现时间: %s\n", ts_to_str(b.discovery_time).c_str());
    printf("          置信度: %u%%\n", b.confidence);
    printf("  [Tail]  crc32=0x%08X\n", frame.getCrc32());
    fflush(stdout);
}

// ============================================================
// 数据回调 — 光电 / EO
// ============================================================
static void on_eo(const PXSBProto::MsgHeader& msg_hdr,
                   const PXSBProto::UnitHeader& unit_hdr,
                   const uint8_t* frame_data, uint32_t frame_len) {

    CamTargetFrame frame;
    if (!frame.deserialize(frame_data, frame_len)) {
        fprintf(stderr, "[EO] 反序列化失败 (magic=0x%08X)\n",
                *reinterpret_cast<const uint32_t*>(frame_data));
        return;
    }

    auto& b = frame.getBody();
    uint64_t n = g_eo_count.fetch_add(1) + 1;

    const char* track_status_str = "?";
    switch (b.getTrackStatus()) {
        case 0: track_status_str = "未跟踪"; break;
        case 1: track_status_str = "跟踪正常"; break;
        case 2: track_status_str = "记忆跟踪"; break;
        case 3: track_status_str = "目标丢失"; break;
    }
    const char* camfield_str = "?";
    switch (b.getCamfield()) {
        case 0: camfield_str = "红外"; break;
        case 1: camfield_str = "可见光"; break;
        case 2: camfield_str = "微光"; break;
    }

    printf("\n╔══ EO #%lu ═══════════════════════════════════\n", n);
    print_msg_header(msg_hdr, unit_hdr);
    print_frame_header(frame.getHeader());
    printf("  [Body]  target_id=%u\n", b.getTargetId());
    printf("          设备状态: fault=%u track=%s(%u) count=%u cam=%s\n",
           b.getFaultStatus(), track_status_str, b.getTrackStatus(),
           b.getTrackCount(), camfield_str);
    printf("          转台: pos(lng=%.7f° lat=%.7f° alt=%.1fm) zero_azi=%.2f°\n",
           b.ptz_lng_e7 * 1e-7f, b.ptz_lat_e7 * 1e-7f,
           b.ptz_alt * 0.1f, b.ptz_zeroazi * 0.01f);
    printf("          转台姿态: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           b.ptz_roll * 0.01f, b.ptz_pitch * 0.01f, b.ptz_yaw * 0.01f);
    printf("          目标位置: lng=%.7f° lat=%.7f° alt=%.1fm\n",
           b.target_lng_e7 * 1e-7f, b.target_lat_e7 * 1e-7f,
           b.target_alt * 0.1f);
    printf("          目标角度/距离: azimuth=%.2f° elevation=%.2f° dis=%.1fm\n",
           b.target_azimuth * 0.01f, b.target_elevation * 0.01f,
           b.target_dis * 0.1f);
    printf("          识别: type=%s conf=%u%%\n",
           target_type_str(b.getTargetType()), b.getTargetConfidence());
    printf("  [Tail]  crc32=0x%08X\n", frame.getCrc32());
    fflush(stdout);
}

// ============================================================
// 数据回调 — 声探测 / Acoustic
// ============================================================
static void on_acoustic(const PXSBProto::MsgHeader& msg_hdr,
                         const PXSBProto::UnitHeader& unit_hdr,
                         const uint8_t* frame_data, uint32_t frame_len) {

    AcousticTargetFrame frame;
    if (!frame.deserialize(frame_data, frame_len)) {
        fprintf(stderr, "[Acoustic] 反序列化失败 (magic=0x%08X)\n",
                *reinterpret_cast<const uint32_t*>(frame_data));
        return;
    }

    auto& b = frame.getBody();
    uint64_t n = g_acoustic_count.fetch_add(1) + 1;

    printf("\n╔══ Acoustic #%lu ═══════════════════════════════════\n", n);
    print_msg_header(msg_hdr, unit_hdr);
    print_frame_header(frame.getHeader());
    printf("  [Body]  target_id=%u point_count=%u\n",
           b.getTargetId(), b.getPointCount());
    printf("          设备位置: lng=%.7f° lat=%.7f° alt=%.1fm\n",
           b.device_lng_e7 * 1e-7f, b.device_lat_e7 * 1e-7f,
           b.device_alt_dm * 0.1f);
    printf("          设备姿态(安装角): roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           b.device_roll * 0.01f, b.device_pitch * 0.01f,
           b.device_yaw * 0.01f);
    printf("          目标相对角度: roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           b.target_roll * 0.01f, b.target_pitch * 0.01f,
           b.target_yaw * 0.01f);
    printf("          识别: type=%s conf=%u%%\n",
           target_type_str(b.getTargetType()), b.getTargetConfidence());
    printf("  [Tail]  crc32=0x%08X\n", frame.getCrc32());
    fflush(stdout);
}

// ============================================================
// 信号处理
// ============================================================
static void sig_handler(int) {
    g_running = false;
}

// ============================================================
// 主函数
// ============================================================
int main(int argc, char* argv[]) {
    (void)argc; (void)argv;

    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    printf("=====================================================\n");
    printf("  Anti-Drone DDS Receiver\n");
    printf("  DDS 配置: third_party 库默认值\n");
    printf("  监听话题:\n");
    printf("    - RadarTarget    (0x1A01, 66B 帧)\n");
    printf("    - ESMTarget      (0x1B01, 139B 帧)\n");
    printf("    - EOTarget       (0x1C01, 79B 帧)\n");
    printf("    - AcousticTarget (0x1D01, 67B 帧)\n");
    printf("=====================================================\n");

    // 初始化 DDS (全部参数由 PubSubTools 库内部决定)
    printf("[Init] 初始化 DDS...\n");
    if (!InitTopic()) {
        fprintf(stderr, "[Init] DDS 初始化失败!\n");
        return 1;
    }
    printf("[Init] DDS 初始化成功\n");

    // 打印 DDS 参数
    printf("\n============================================================\n");
    printf("  DDS 初始化参数 (PubSubTools 库默认值)\n");
    printf("============================================================\n");
    printf("  Domain ID             : %u\n", Default_Domain_ID);
    printf("  Multicast IP          : %s\n", Default_MetatrafficMulticastLocator_IP);
    printf("  Multicast Port        : %u\n", Default_MetatrafficMulticastLocator_Port);
    printf("  Discovery Protocol     : SIMPLE (FastDDS 默认)\n");
    printf("  Topics                 : RadarTarget / ESMTarget / EOTarget / AcousticTarget\n");
    printf("============================================================\n\n");

    // 四个话题统一用 dispatch_pxsb_message 处理（自动按 unitTag 分发）
    processPtr dds_callback = [](const ByteSequence& data) {
        dispatch_pxsb_message(data);
    };

    struct {
        const char* topic;
    } readers[] = {
        {"RadarTarget"},
        {"ESMTarget"},
        {"EOTarget"},
        {"AcousticTarget"},
    };

    for (auto& r : readers) {
        printf("[Init] 创建 Reader: %s ... ", r.topic);
        if (!createReader(r.topic, false)) {
            fprintf(stderr, "失败!\n");
            return 1;
        }
        modifyProcess(r.topic, dds_callback);
        printf("OK\n");
    }

    printf("\n>>> 等待数据... 按 Ctrl+C 退出 <<<\n\n");

    // 主循环 — 等待数据
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 统计
    printf("\n=====================================================\n");
    printf("  统计:\n");
    printf("    Radar:    %lu 帧\n", g_radar_count.load());
    printf("    ESM:      %lu 帧\n", g_esm_count.load());
    printf("    EO:       %lu 帧\n", g_eo_count.load());
    printf("    Acoustic: %lu 帧\n", g_acoustic_count.load());
    printf("    总计:     %lu 帧\n",
           g_radar_count + g_esm_count + g_eo_count + g_acoustic_count);
    printf("=====================================================\n");

    // 不析构 DDS（避免 FastDDS 退出 crash）
    return 0;
}
