#ifndef _DISCOVERYSERVER_H_
#define _DISCOVERYSERVER_H_

#include <string>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include "pxsb_common.h"

#define Default_Domain_ID 106                                              // 默认域ID
#define Default_DiscoveryServer_IP "192.168.0.3"                           // 默认发现服务器IP
#define Default_DiscoveryServer_Port 11206                                 // 默认发现服务器端口号
#define Default_DiscoveryServer_GUID "44.69.00.5f.45.50.52.4f.53.49.02.06" // 默认发现服务器标识

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

extern std::string DiscoveryServer_IP; // 发现服务器IP地址

class DiscoveryServerListener : public DomainParticipantListener
{
public:
    void on_participant_discovery(DomainParticipant *participant, ParticipantDiscoveryInfo &&info) override;
};

class DiscoveryServer
{
public:
    DiscoveryServer();

    bool start();

private:
    DomainParticipant *participant_;

    DiscoveryServerListener listener_;

    bool start_flag_;
};

#endif //_DISCOVERYSERVER_H_
