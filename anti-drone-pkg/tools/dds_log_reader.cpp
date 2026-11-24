/**
 * @file dds_log_reader.cpp
 * @brief 读取 output/dds_send.log，逐帧解析并打印，方便接收端对照排查。
 *
 * 日志格式 (每条记录):
 *   [4B timestamp_ms] [2B frame_type] [4B pkt_len] [NB pkt_data]
 *   pkt_data = MsgHeader(26B) + UnitHeader_0(14B) + frame_0 [+ UnitHeader_1(14B) + frame_1 ...]
 *
 * 用法: ./dds_log_reader [output/dds_send.log]
 */
#include "dev_proto.h"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>

using namespace AntiDroneDevProtocol;

static std::string ts_to_str(uint32_t ts) {
    time_t sec = ts / 1000;
    int ms = ts % 1000;
    char buf[40];
    struct tm tm_info;
    localtime_r(&sec, &tm_info);
    strftime(buf, sizeof(buf), "%H:%M:%S", &tm_info);
    snprintf(buf + 8, sizeof(buf) - 8, ".%03d", ms);
    return buf;
}

static void print_frame_header(const DevFrameHeader& h) {
    printf("  [Frame] magic=0x%08X ver=%u ftype=0x%04X dtype=%u len=%u dev=%u ts=%s\n",
           h.getMagic(), h.getProtoVer(), h.getFrameType(), h.getDataType(),
           h.getLength(), h.getDeviceId(), ts_to_str((uint32_t)h.getTimestamp()).c_str());
}

static void print_radar(const uint8_t* data, uint32_t len) {
    RadarTargetFrame f;
    if (!f.deserialize(data, len)) { printf("  [Radar] 解析失败\n"); return; }
    auto& b = f.getBody();
    printf("  [Radar] track_id=%u dis=%.1fm azi=%.2f° ele=%.2f° height=%.1fm v=%.1fm/s\n",
           b.id, b.dis*0.1f, b.azi*0.01f, b.ele*0.01f, b.height*0.1f, b.radial_vel*0.1f);
    printf("          lng=%.7f° lat=%.7f° alt=%.1fm speed=%.1fm/s course=%.2f° tag=%u type=%u conf=%u%%\n",
           b.lng*1e-7f, b.lat*1e-7f, b.alt*0.1f, b.speed*0.1f, b.course*0.01f,
           b.tag, b.target_type, b.target_confidence);
}

static void print_esm(const uint8_t* data, uint32_t len) {
    FrequencyDetectFrame f;
    if (!f.deserialize(data, len)) { printf("  [ESM] 解析失败\n"); return; }
    auto& b = f.getBody();
    printf("  [ESM]  target_id=%u freq=%.3fMHz bw=%.1fkHz power=%.1fdBm az=%.2f° conf=%u%%\n",
           b.target_id, b.freq_hz*1e-6f, b.bandw_hz*1e-3f, b.power_db*0.1f,
           b.azimuth_cdeg*0.01f, b.confidence);
    printf("         lng=%.7f° lat=%.7f° alt=%.1fm height=%.1fm speed=%.1fm/s dis=%.1fm\n",
           b.lng_e7*1e-7f, b.lat_e7*1e-7f, b.alt*0.1f, b.height*0.1f, b.speed*0.1f, b.distance*0.1f);
}

static void print_eo(const uint8_t* data, uint32_t len) {
    CamTargetFrame f;
    if (!f.deserialize(data, len)) { printf("  [EO] 解析失败\n"); return; }
    auto& b = f.getBody();
    printf("  [EO]   target_id=%u track_status=%u camfield=%u fault=%u\n",
           b.target_id, b.track_status, b.camfield, b.fault_status);
    printf("         ptz_pos lng=%.7f° lat=%.7f° alt=%.1fm roll=%.2f° pitch=%.2f° yaw=%.2f°\n",
           b.ptz_lng_e7*1e-7f, b.ptz_lat_e7*1e-7f, b.ptz_alt*0.1f,
           b.ptz_roll*0.01f, b.ptz_pitch*0.01f, b.ptz_yaw*0.01f);
    printf("         target lng=%.7f° lat=%.7f° alt=%.1fm az=%.2f° el=%.2f° dis=%.1fm type=%u conf=%u%%\n",
           b.target_lng_e7*1e-7f, b.target_lat_e7*1e-7f, b.target_alt*0.1f,
           b.target_azimuth*0.01f, b.target_elevation*0.01f, b.target_dis*0.1f,
           b.target_type, b.target_confidence);
}

static void print_acoustic(const uint8_t* data, uint32_t len) {
    AcousticTargetFrame f;
    if (!f.deserialize(data, len)) { printf("  [Acoustic] 解析失败\n"); return; }
    auto& b = f.getBody();
    printf("  [Acoustic] target_id=%u dev_pos lng=%.7f° lat=%.7f° alt=%.1fm\n",
           b.target_id, b.device_lng_e7*1e-7f, b.device_lat_e7*1e-7f, b.device_alt_dm*0.1f);
    printf("            dev_att roll=%.2f° pitch=%.2f° yaw=%.2f° target_angles roll=%.2f° pitch=%.2f° yaw=%.2f° type=%u conf=%u%%\n",
           b.device_roll*0.01f, b.device_pitch*0.01f, b.device_yaw*0.01f,
           b.target_roll*0.01f, b.target_pitch*0.01f, b.target_yaw*0.01f,
           b.target_type, b.target_confidence);
}

int main(int argc, char* argv[]) {
    const char* path = argc > 1 ? argv[1] : "output/dds_send.log";
    FILE* f = fopen(path, "rb");
    if (!f) { fprintf(stderr, "无法打开 %s\n", path); return 1; }

    printf("=== DDS 发送日志: %s ===\n\n", path);

    uint32_t ts, pkt_len, seq = 0;
    uint16_t ftype;
    uint8_t buf[2048];

    while (fread(&ts, 4, 1, f) == 1) {
        if (fread(&ftype, 2, 1, f) != 1) break;
        if (fread(&pkt_len, 4, 1, f) != 1) break;
        if (pkt_len > sizeof(buf)) { printf("[err] pkt too big: %u\n", pkt_len); break; }
        if (fread(buf, 1, pkt_len, f) != pkt_len) break;

        seq++;
        printf("[%04u] ts=%s pktSize=%uB\n", seq, ts_to_str(ts).c_str(), pkt_len);

        // ── 手解 PXSB 封套 (MsgHeader 26B + N x [UnitHeader 14B + frame]) ──
        constexpr uint32_t MSG_HDR_SIZE = 26;
        constexpr uint32_t UNIT_HDR_SIZE = 14;

        if (pkt_len < MSG_HDR_SIZE) {
            printf("  [PXSB] 数据太短: %uB\n", pkt_len);
            continue;
        }

        // MsgHeader (26B): startTag(1) unitNums(1) src(2) dst(2) len(4)
        //                 tag(2) msgID(2) checkID(2) bak(2) timestamp(8)
        uint8_t  start_tag = buf[0];
        uint8_t  unit_nums = buf[1];
        uint16_t msg_id    = (uint16_t)buf[12] | ((uint16_t)buf[13] << 8);

        if (start_tag != 0x7E) {
            printf("  [PXSB] startTag 错误: 0x%02X\n", start_tag);
            continue;
        }
        if (unit_nums == 0) { printf("\n"); continue; }

        printf("  [PXSB] msgID=%u units=%u\n", msg_id, unit_nums);

        // 逐个 UnitHeader (14B): unitLength(4) unitTag(2) timestamp(8)
        size_t pos = MSG_HDR_SIZE;
        for (uint8_t u = 0; u < unit_nums; ++u) {
            if (pkt_len < pos + UNIT_HDR_SIZE) {
                printf("  [%u/%u] UnitHeader 越界\n", u+1, unit_nums);
                break;
            }

            uint32_t unit_len;
            uint16_t unit_tag;
            std::memcpy(&unit_len, buf + pos, 4);
            std::memcpy(&unit_tag, buf + pos + 4, 2);
            pos += UNIT_HDR_SIZE;

            if (pkt_len < pos + unit_len) {
                printf("  [%u/%u] frame 越界 (需要%u 剩余%zu)\n",
                       u+1, unit_nums, unit_len, pkt_len - pos);
                break;
            }

            const uint8_t* frame_data = buf + pos;
            pos += unit_len;

            printf("  [%u/%u] unitTag=0x%04X unitLen=%u ",
                   u+1, unit_nums, unit_tag, unit_len);
            switch (unit_tag) {
                case 0x1A01: printf("RadarTarget\n");    print_radar(frame_data, unit_len); break;
                case 0x1B01: printf("ESMTarget\n");      print_esm(frame_data, unit_len); break;
                case 0x1C01: printf("EOTarget\n");       print_eo(frame_data, unit_len); break;
                case 0x1D01: printf("AcousticTarget\n"); print_acoustic(frame_data, unit_len); break;
                default:
                    printf("未知帧类型 (跳过)\n");
                    break;
            }
        }
        printf("\n");
    }

    fclose(f);
    printf("=== 共 %u 条记录 ===\n", seq);
    return 0;
}
