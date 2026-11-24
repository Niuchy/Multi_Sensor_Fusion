/**
 * Company DDS bridge for the Python fusion nodes.
 *
 * The bridge deliberately uses zy-anti-drone's TopicAPI/ByteSequence API.  It
 * provides a small C ABI for ctypes, plus a publisher for the official
 * fusion_proto.h FusionMultiTarget message.
 */

#include "TopicAPI.h"
#include "proto.h"
#include "fusion_proto.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using AntiDroneFusionProtocol::AbsPos;
using AntiDroneFusionProtocol::Attitude;
using AntiDroneFusionProtocol::FusionMultiTargetMsg;
using AntiDroneFusionProtocol::FusionTarget;
using AntiDroneFusionProtocol::MessageType;

namespace {

constexpr size_t kMaxReaders = 16;
constexpr size_t kMaxQueuedPerReader = 512;

struct ReaderSlot {
    std::string topic;
    std::deque<ByteSequence> queue;
    std::condition_variable cv;
};

std::mutex g_mutex;
bool g_initialized = false;
uint16_t g_message_id = 0;
std::array<ReaderSlot, kMaxReaders> g_slots;
std::unordered_map<std::string, size_t> g_topic_slots;
std::unordered_set<std::string> g_writers;

uint64_t now_ms() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count());
}

void enqueue(size_t slot, const ByteSequence& bytes) {
    std::lock_guard<std::mutex> lock(g_mutex);
    auto& reader = g_slots[slot];
    if (reader.queue.size() >= kMaxQueuedPerReader) {
        reader.queue.pop_front();
    }
    reader.queue.push_back(bytes);
    reader.cv.notify_one();
}

template <size_t Slot>
void reader_callback(const ByteSequence& bytes) {
    enqueue(Slot, bytes);
}

constexpr std::array<processPtr, kMaxReaders> kCallbacks = {
    &reader_callback<0>, &reader_callback<1>, &reader_callback<2>,
    &reader_callback<3>, &reader_callback<4>, &reader_callback<5>,
    &reader_callback<6>, &reader_callback<7>, &reader_callback<8>,
    &reader_callback<9>, &reader_callback<10>, &reader_callback<11>,
    &reader_callback<12>, &reader_callback<13>, &reader_callback<14>,
    &reader_callback<15>,
};

bool ensure_writer(const std::string& topic, bool reliable = false) {
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_writers.count(topic) != 0) {
            return true;
        }
    }
    if (!createWriter(topic, reliable)) {
        return false;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    g_writers.insert(topic);
    return true;
}

int publish_protocol_body(
    const char* topic,
    uint16_t unit_tag,
    uint16_t source_address,
    uint64_t timestamp_ms,
    const ByteSequence& body) {
    if (!topic || body.empty() || !ensure_writer(topic, false)) {
        return -1;
    }

    const uint64_t stamp = timestamp_ms == 0 ? now_ms() : timestamp_ms;

    PXSBProto::MsgHeader header;
    header.startTag = 0x7E;
    header.unitNums = 1;
    header.srcAddress = source_address;
    header.destAddress = 0xFFFF;
    header.msgLength = static_cast<uint32_t>(6 + 8 + body.size());
    header.msgTag = 0;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        header.msgID = ++g_message_id;
    }
    header.checkID = 0;
    header.bak = 0;
    header.timestamp = stamp;

    PXSBProto::UnitHeader unit;
    unit.unitLength = static_cast<uint32_t>(body.size());
    unit.unitTag = unit_tag;

    ByteSequence packet = header.serialize();
    ByteSequence unit_bytes = unit.serialize();
    packet.insert(packet.end(), unit_bytes.begin(), unit_bytes.end());
    for (size_t i = 0; i < sizeof(stamp); ++i) {
        packet.push_back(static_cast<uint8_t>((stamp >> (8 * i)) & 0xFF));
    }
    packet.insert(packet.end(), body.begin(), body.end());
    publish(packet, topic);
    return 0;
}

}  // namespace

#pragma pack(push, 1)
struct CompanyFusionTargetData {
    uint8_t global_id;
    int32_t lng_e7;
    int32_t lat_e7;
    uint32_t alt_dm;
    int16_t pitch_cdeg;
    int16_t roll_cdeg;
    uint16_t yaw_cdeg;
    uint32_t speed_cms;
    uint8_t fusion_type;
    uint8_t confidence;
    uint8_t source_mask;
    uint8_t iff;
};

#pragma pack(pop)

extern "C" {

int company_dds_init() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_initialized) {
        return 0;
    }
    g_initialized = InitTopic(false);
    return g_initialized ? 0 : -1;
}

int company_dds_ready() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_initialized ? 1 : 0;
}

int company_dds_create_writer(const char* topic, int reliable) {
    if (!topic || !*topic) {
        return -1;
    }
    return ensure_writer(topic, reliable != 0) ? 0 : -1;
}

int company_dds_create_reader(const char* topic, int reliable) {
    if (!topic || !*topic) {
        return -1;
    }

    size_t slot = kMaxReaders;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        const auto existing = g_topic_slots.find(topic);
        if (existing != g_topic_slots.end()) {
            return 0;
        }
        for (size_t i = 0; i < kMaxReaders; ++i) {
            if (g_slots[i].topic.empty()) {
                slot = i;
                g_slots[i].topic = topic;
                g_topic_slots.emplace(topic, i);
                break;
            }
        }
    }
    if (slot == kMaxReaders) {
        return -2;
    }
    if (!createReader(topic, reliable != 0)) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_topic_slots.erase(topic);
        g_slots[slot].topic.clear();
        return -1;
    }
    modifyProcess(topic, kCallbacks[slot]);
    return 0;
}

int company_dds_publish(
    const char* topic,
    const uint8_t* data,
    uint32_t data_len) {
    if (!topic || !data || data_len == 0 || !ensure_writer(topic, false)) {
        return -1;
    }
    ByteSequence bytes(data, data + data_len);
    publish(bytes, topic);
    return 0;
}

int company_dds_poll(
    const char* topic,
    uint8_t* output,
    uint32_t output_capacity,
    uint32_t* actual_len,
    int timeout_ms) {
    if (!topic || !actual_len) {
        return -1;
    }

    std::unique_lock<std::mutex> lock(g_mutex);
    const auto found = g_topic_slots.find(topic);
    if (found == g_topic_slots.end()) {
        return -1;
    }
    auto& slot = g_slots[found->second];
    if (slot.queue.empty()) {
        if (timeout_ms < 0) {
            slot.cv.wait(lock, [&slot] { return !slot.queue.empty(); });
        } else if (timeout_ms > 0) {
            slot.cv.wait_for(
                lock, std::chrono::milliseconds(timeout_ms),
                [&slot] { return !slot.queue.empty(); });
        }
    }
    if (slot.queue.empty()) {
        *actual_len = 0;
        return 0;
    }

    const ByteSequence& bytes = slot.queue.front();
    *actual_len = static_cast<uint32_t>(bytes.size());
    if (!output || output_capacity < bytes.size()) {
        return -2;
    }
    std::memcpy(output, bytes.data(), bytes.size());
    slot.queue.pop_front();
    return 1;
}

int company_dds_available(const char* topic) {
    if (!topic) {
        return -1;
    }
    std::lock_guard<std::mutex> lock(g_mutex);
    const auto found = g_topic_slots.find(topic);
    if (found == g_topic_slots.end()) {
        return -1;
    }
    return static_cast<int>(g_slots[found->second].queue.size());
}

int company_dds_publish_fusion(
    const char* topic,
    uint16_t source_address,
    uint64_t timestamp_ms,
    const CompanyFusionTargetData* targets,
    uint32_t target_count) {
    if ((!targets && target_count != 0) || target_count > 255) {
        return -1;
    }

    FusionMultiTargetMsg message;
    for (uint32_t i = 0; i < target_count; ++i) {
        const auto& input = targets[i];
        FusionTarget target;
        target.setTargetBasicInfo(input.global_id, input.source_mask);
        target.setTargetIdentity(input.fusion_type, input.confidence, input.iff);

        AbsPos position;
        position.setPosition(input.lng_e7, input.lat_e7, input.alt_dm);
        target.setPosition(position);

        Attitude attitude;
        attitude.setAttitude(
            input.pitch_cdeg, input.roll_cdeg, input.yaw_cdeg);
        target.setTargetAttitude(attitude);
        target.setMotion(input.speed_cms);
        message.addTarget(target);
    }

    ByteSequence body(message.size());
    if (!message.serialize(body.data(), static_cast<uint32_t>(body.size()))) {
        return -1;
    }
    return publish_protocol_body(
        topic,
        static_cast<uint16_t>(MessageType::FUSION_MULTI_TARGET),
        source_address,
        timestamp_ms,
        body);
}

void company_dds_close() {
    // TopicAPI/FastDDS teardown is intentionally skipped.  The company bridge
    // follows the same process-lifetime policy as libad_proto*.so because
    // deleting the global participant during interpreter shutdown can crash.
}

const char* company_dds_version() {
    return "company-dds-python-bridge-1.0.0";
}

}  // extern "C"
