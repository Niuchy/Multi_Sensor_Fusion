/**
 * @file ad_proto_receiver.cpp
 * @brief 反无人机协议 DDS 接收端 — poll 模式 C API
 *
 * 注册 DDS 回调 → 线程安全队列 → Python ctypes poll 取帧。
 */

#include "ad_proto_receiver.h"
#include "dev_proto.h"
#include "TopicAPI.h"
#include "PubSubTools.h"
#include "proto.h"

#include <cstdio>
#include <cstring>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

using namespace AntiDroneDevProtocol;

// ============================================================
// 线程安全帧队列
// ============================================================

struct QueuedFrame {
    uint16_t frame_type;
    uint32_t device_id;
    uint64_t timestamp_ms;
    uint32_t data_len;
    uint8_t  data[AD_RX_MAX_FRAME];
};

static std::mutex              g_queue_mutex;
static std::condition_variable g_queue_cv;
static std::queue<QueuedFrame> g_queue;
static std::atomic<bool>       g_rx_running{false};
static constexpr size_t        MAX_QUEUE_SIZE = 1024;

static void push_frame(const QueuedFrame& qf) {
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    if (g_queue.size() < MAX_QUEUE_SIZE) {
        g_queue.push(qf);
    }
    g_queue_cv.notify_one();
}

// ============================================================
// DDS 回调 → 解析 PXSB → 解出所有帧 → 入队
// ============================================================

// DDS/CDR 封装了 PXSB MsgHeader，回调收到的是 UnitHeader + frame 数据。
// 搜索 dev_proto 帧的 Magic Number 定位帧起始，从 DevFrameHeader 自解析类型和长度。
static void on_dds_message(const ByteSequence& data) {
    const size_t dlen = data.size();
    const uint8_t* p = data.data();
    size_t pos = 0;

    while (pos + 23 <= dlen) {
        // 搜索下一个 dev_proto Magic Number
        uint32_t magic;
        std::memcpy(&magic, p + pos, 4);

        bool valid_magic = (magic == 0xA55AA55A || magic == 0xB44BB44B ||
                            magic == 0xC33CC33C || magic == 0xD22DD22D);
        if (!valid_magic) {
            ++pos;
            continue;
        }

        // DevFrameHeader: magic(4) ver(1) ftype(2) dtype(2) length(2) dev(4) ts(8)
        uint16_t ftype;
        uint16_t total_len;
        uint32_t device_id;
        uint64_t ts;
        std::memcpy(&ftype,     p + pos + 5, 2);
        std::memcpy(&total_len, p + pos + 9, 2);
        std::memcpy(&device_id, p + pos + 11, 4);
        std::memcpy(&ts,        p + pos + 15, 8);

        if (total_len == 0 || total_len > AD_RX_MAX_FRAME) { ++pos; continue; }
        if (pos + total_len > dlen) break;

        QueuedFrame qf;
        qf.frame_type   = ftype;
        qf.device_id    = device_id;
        qf.timestamp_ms = ts;
        qf.data_len     = total_len;
        std::memcpy(qf.data, p + pos, total_len);
        pos += total_len;

        push_frame(qf);
    }
}

// ============================================================
// C API
// ============================================================

extern "C" {

int ad_proto_receive_init(void) {
    if (g_rx_running.load()) return 0;  // already initialized

    try {
        if (!InitTopic()) {
            fprintf(stderr, "[Rx] DDS 初始化失败\n");
            return -1;
        }

        fprintf(stderr, "[Rx] DDS 初始化成功 (Domain=%u, MCast=%s:%u)\n",
                Default_Domain_ID,
                Default_MetatrafficMulticastLocator_IP,
                Default_MetatrafficMulticastLocator_Port);

        // 创建四个话题的 Reader，统一用 on_dds_message 回调
        const char* topics[] = {
            "RadarTarget", "ESMTarget", "EOTarget", "AcousticTarget"
        };

        for (const char* topic : topics) {
            if (!createReader(topic, false)) {
                fprintf(stderr, "[Rx] createReader(\"%s\") 失败\n", topic);
                return -1;
            }
            modifyProcess(topic, on_dds_message);
            fprintf(stderr, "[Rx] Reader \"%s\" 已就绪\n", topic);
        }

        g_rx_running = true;
        return 0;
    } catch (...) {
        fprintf(stderr, "[Rx] 初始化异常\n");
        return -1;
    }
}

int ad_proto_receive_poll(AdRxFrame* frame, int timeout_ms) {
    if (!frame) return -1;
    if (!g_rx_running.load()) return -1;

    std::unique_lock<std::mutex> lock(g_queue_mutex);

    if (timeout_ms == 0) {
        // 非阻塞
        if (g_queue.empty()) return 0;
    } else if (timeout_ms < 0) {
        // 永久阻塞
        g_queue_cv.wait(lock, []{ return !g_queue.empty() || !g_rx_running.load(); });
        if (!g_rx_running.load() && g_queue.empty()) return 0;
    } else {
        // 带超时
        bool got = g_queue_cv.wait_for(lock,
            std::chrono::milliseconds(timeout_ms),
            []{ return !g_queue.empty(); });
        if (!got) return 0;  // timeout
    }

    if (g_queue.empty()) return 0;

    const QueuedFrame& qf = g_queue.front();
    frame->frame_type   = qf.frame_type;
    frame->device_id    = qf.device_id;
    frame->timestamp_ms = qf.timestamp_ms;
    frame->data_len     = qf.data_len;
    std::memcpy(frame->data, qf.data, qf.data_len);

    g_queue.pop();
    return 1;
}

int ad_proto_receive_available(void) {
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    return static_cast<int>(g_queue.size());
}

void ad_proto_receive_close(void) {
    g_rx_running = false;
    g_queue_cv.notify_all();
    // 清空队列
    std::lock_guard<std::mutex> lock(g_queue_mutex);
    while (!g_queue.empty()) g_queue.pop();
    fprintf(stderr, "[Rx] 接收端已关闭\n");
}

} // extern "C"
