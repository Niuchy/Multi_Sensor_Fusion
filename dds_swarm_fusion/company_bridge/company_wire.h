#pragma once

// PXSB wire encoding is explicit: some delivered PubSubTools versions serialize
// UnitHeader as 6 bytes, others as 14. Never combine that ABI with an extra stamp.
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace company_wire {

inline void append_le(std::vector<uint8_t>& bytes, uint64_t value, unsigned width) {
    for (unsigned i = 0; i < width; ++i) {
        bytes.push_back(static_cast<uint8_t>((value >> (8 * i)) & 0xFF));
    }
}

inline std::vector<uint8_t> protocol_packet(
    uint16_t unit_tag, uint16_t source_address, uint16_t message_id,
    uint64_t stamp, const std::vector<uint8_t>& body) {
    constexpr uint32_t message_header_size = 26;
    constexpr uint32_t unit_header_size = 14;
    if (body.size() > std::numeric_limits<uint32_t>::max() - unit_header_size) {
        throw std::length_error("PXSB body exceeds uint32 length");
    }
    const auto payload_size = unit_header_size + static_cast<uint32_t>(body.size());
    std::vector<uint8_t> packet;
    packet.reserve(static_cast<size_t>(message_header_size) + payload_size);
    append_le(packet, 0x7E, 1);
    append_le(packet, 1, 1); // one unit
    append_le(packet, source_address, 2);
    append_le(packet, 0xFFFF, 2);
    append_le(packet, payload_size, 4);
    append_le(packet, 0, 2); // msgTag
    append_le(packet, message_id, 2);
    append_le(packet, 0, 2); // checkID
    append_le(packet, 0, 2); // reserved
    append_le(packet, stamp, 8);
    append_le(packet, body.size(), 4);
    append_le(packet, unit_tag, 2);
    append_le(packet, stamp, 8); // exactly one source timestamp
    packet.insert(packet.end(), body.begin(), body.end());
    return packet;
}

} // namespace company_wire
