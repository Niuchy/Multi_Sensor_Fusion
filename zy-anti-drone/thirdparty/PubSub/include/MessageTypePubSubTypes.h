#ifndef _FAST_DDS_GENERATED_MESSAGETYPE_PUBSUBTYPES_H_
#define _FAST_DDS_GENERATED_MESSAGETYPE_PUBSUBTYPES_H_

#include <fastdds/dds/core/policy/QosPolicies.hpp>
#include <fastdds/dds/topic/TopicDataType.hpp>
#include <fastdds/rtps/common/InstanceHandle.h>
#include <fastdds/rtps/common/SerializedPayload.h>
#include <fastrtps/utils/md5.h>

#include "MessageType.h"

#if !defined(GEN_API_VER) || (GEN_API_VER != 2)
#error Generated MessageType is not compatible with current installed Fast DDS. Please, regenerate it with fastddsgen.
#endif // GEN_API_VER

class MessageTypePubSubType : public eprosima::fastdds::dds::TopicDataType
{
public:
    typedef MessageType type;

    eProsima_user_DllExport MessageTypePubSubType();

    eProsima_user_DllExport ~MessageTypePubSubType() override;

    eProsima_user_DllExport bool serialize(
        void *data,
        eprosima::fastrtps::rtps::SerializedPayload_t *payload) override
    {
        return serialize(data, payload, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    eProsima_user_DllExport bool serialize(
        void *data,
        eprosima::fastrtps::rtps::SerializedPayload_t *payload,
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    eProsima_user_DllExport bool deserialize(
        eprosima::fastrtps::rtps::SerializedPayload_t *payload,
        void *data) override;

    eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
        void *data) override
    {
        return getSerializedSizeProvider(data, eprosima::fastdds::dds::DEFAULT_DATA_REPRESENTATION);
    }

    eProsima_user_DllExport std::function<uint32_t()> getSerializedSizeProvider(
        void *data,
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

    eProsima_user_DllExport bool getKey(
        void *data,
        eprosima::fastrtps::rtps::InstanceHandle_t *ihandle,
        bool force_md5 = false) override;

    eProsima_user_DllExport void *createData() override;

    eProsima_user_DllExport void deleteData(
        void *data) override;

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED
    eProsima_user_DllExport inline bool is_bounded() const override
    {
        return false;
    }

#endif // TOPIC_DATA_TYPE_API_HAS_IS_BOUNDED

#ifdef TOPIC_DATA_TYPE_API_HAS_IS_PLAIN
    eProsima_user_DllExport inline bool is_plain() const override
    {
        return false;
    }

    eProsima_user_DllExport inline bool is_plain(
        eprosima::fastdds::dds::DataRepresentationId_t data_representation) const override
    {
        static_cast<void>(data_representation);
        return false;
    }

#endif // TOPIC_DATA_TYPE_API_HAS_IS_PLAIN

#ifdef TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE
    eProsima_user_DllExport inline bool construct_sample(
        void *memory) const override
    {
        static_cast<void>(memory);
        return false;
    }

#endif // TOPIC_DATA_TYPE_API_HAS_CONSTRUCT_SAMPLE

    MD5 m_md5;
    unsigned char *m_keyBuffer;
};

#endif // _FAST_DDS_GENERATED_MESSAGETYPE_PUBSUBTYPES_H_
