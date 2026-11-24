#ifndef PROTO_H
#define PROTO_H

#include <ctime>
#include <cstdint>
#include <vector>
#include <functional>
#include <unordered_map>
#include <memory>
#include <string>

#include "pxsb_common.h"
#include "proto_helper.hpp"

using ByteSequence = std::vector<uint8_t>; // 字节序列

namespace PXSBProto
{
    struct UnitBase
    {
        virtual ~UnitBase() = default;

        virtual ByteSequence serialize() const = 0; // 序列化

        virtual void deserialize(const ByteSequence &bytes, size_t &pos) = 0; // 反序列化
    };

    // 报文头
    struct MsgHeader
    {
        uint8_t startTag;     // 起始标识
        uint8_t unitNums;     // 信息个数
        uint16_t srcAddress;  // 源端地址
        uint16_t destAddress; // 目的地址
        uint32_t msgLength;   // 报文长度
        uint16_t msgTag;      // 报文标志
        uint16_t msgID;       // 报文序号
        uint16_t checkID;     // 确认号
        uint16_t bak;         // 备用
        uint64_t timestamp;   // 发送时间戳

        // 序列化
        ByteSequence serialize() const;
        // 反序列化
        void deserialize(const ByteSequence &bytes, size_t &pos);
    };

    // 信息头
    struct UnitHeader
    {
        uint32_t unitLength; // 信息长度
        uint16_t unitTag;    // 信息类型
        uint64_t timestamp;  // 消息源时间戳

        // 序列化
        ByteSequence serialize() const;
        // 反序列化
        void deserialize(const ByteSequence &bytes, size_t &pos);
    };

    // 报文
    struct Msg
    {
        MsgHeader header;              // 报文头
        std::vector<UnitBase *> units; // 信息单元

        // 序列化
        ByteSequence serialize() const;
    };

    // 报文处理函数映射表
    extern std::unordered_map<uint16_t, std::function<void(const MsgHeader &header, const UnitBase &unit)>> handlers;
    extern std::unordered_map<uint16_t, std::function<std::unique_ptr<UnitBase>()>> createors;

    inline std::unique_ptr<UnitBase> createStruct(const uint16_t key)
    {
        auto it = createors.find(key);

        if (it != createors.end())
        {
            return it->second();
        }

        return nullptr;
    }

    // 注册报文处理函数
    template <typename T>
    void registerHandlers(uint16_t key, std::function<void(const MsgHeader &, const T &)> func)
    {
        createors[key] = []()
        {
            return std::make_unique<T>();
        };

        PXSBProto::handlers[key] = [func, key](const MsgHeader &header, const UnitBase &unit)
        {
            if (const T *derived = static_cast<const T *>(&unit))
            {
                func(header, *derived);
            }
            else
            {
                pxsb_log(LOG_LEVEL::ERROR, "0x%04X Error: Type not matched !!!", key);
            }
        };
    }

    // 报文解析
    void unpackMsg(const ByteSequence &receive_bytes);
}

#endif // PROTO_H
