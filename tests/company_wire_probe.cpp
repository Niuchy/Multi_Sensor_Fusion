#include "company_wire.h"
#include "fusion_proto.h"
#include <iomanip>
#include <iostream>

int main() {
    using namespace AntiDroneFusionProtocol;
    constexpr uint64_t stamp = 1790000000123ULL;
    for (unsigned count : {0U, 1U, 255U}) {
        FusionMultiTargetMsg message;
        for (unsigned i = 0; i < count; ++i) {
            FusionTarget target;
            target.setTargetBasicInfo(static_cast<uint8_t>(i+1), 0);
            target.setTargetIdentity(0xFF, 15, 2);
            AbsPos pos;
            pos.setPosition(1201467056, 301104393, 870);
            target.setPosition(pos);
            message.addTarget(target);
        }
        std::vector<uint8_t> body(message.size());
        if (!message.serialize(body.data(), static_cast<uint32_t>(body.size()))) return 1;
        const auto packet = company_wire::protocol_packet(1, 201, 784, stamp, body);
        FusionMultiTargetMsg decoded;
        if (!decoded.deserialize(packet.data()+40, static_cast<uint32_t>(packet.size()-40))
            || decoded.getTargetCount() != count) return 2;
        for (uint8_t byte : packet) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << static_cast<unsigned>(byte);
        }
        std::cout << '\n';
    }
}
