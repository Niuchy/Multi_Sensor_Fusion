#ifndef _PUBSUBTOOLS_H_
#define _PUBSUBTOOLS_H_

#include <vector>
#include <string>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <chrono>
#include <utility>
#include <condition_variable>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/rtps/common/Locator.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>

#include "DiscoveryServer.h"

using namespace eprosima::fastdds::dds;
using namespace eprosima::fastdds::rtps;
using namespace eprosima::fastrtps::rtps;

#define Default_Initial_Announcement_Count 3                   // 默认初始公告次数
#define Default_Announcement_Period 5                          // 默认公告周期
#define Default_Lease_Duration 60                              // 默认活跃持续时间
#define Default_ReliableWriter_HeartBeat_Perios 5              // 默认可靠传输心跳周期
#define Default_Reliable_History_Depth 100                     // 默认可靠传输历史记录上限
#define Default_MetatrafficMulticastLocator_IP "239.255.0.106" // 默认元流量多播定位器地址
#define Default_MetatrafficMulticastLocator_Port 7500          // 默认元流量多播定位器端口号

using ByteSequence = std::vector<uint8_t>; // 字节序列

using processPtr = void (*)(const ByteSequence &); // 数据读取监听回调函数

using serverProcessPtr = ByteSequence (*)(const ByteSequence &); // 服务端数据读取监听回调函数

// 数据读取监听处理器
class SubListener : public DataReaderListener
{
public:
    SubListener(const std::string &topic_name);

    void on_data_available(DataReader *reader) override;

private:
    std::string topic_name_;
};

/**
 * @brief 发布主题设置
 *
 * @param writer 数据写入器
 */
struct PubConfig
{
    DataWriter *writer = nullptr;
};

/**
 * @brief 订阅主题设置
 *
 * @param listener 数据读取监听处理器
 * @param reader 数据读取器
 */
struct SubConfig
{
    SubListener *listener = nullptr;
    DataReader *reader = nullptr;
};

class ClientListener : public DataReaderListener
{
public:
    void on_data_available(DataReader *reader) override;

    WriteParams write_params_;

    std::mutex reception_mutex_;

    std::condition_variable reception_cv_;

    bool received_reply_ = false;

    ByteSequence result_;
};

struct ClientConfig
{
    std::string topic_name;
    std::string request_topic_name;
    std::string reply_topic_name;
    ClientListener *listener = nullptr;
    DataWriter *resquest_writer = nullptr;
    DataReader *reply_reader = nullptr;
};

class ServerListener : public DataReaderListener
{
public:
    ServerListener(DataWriter *writer);

    void on_data_available(DataReader *reader) override;

    serverProcessPtr process_ = nullptr;

    std::mutex process_mtx_;

    DataWriter *writer_ = nullptr;
};

struct ServerConfig
{
    std::string topic_name;
    std::string request_topic_name;
    std::string reply_topic_name;
    ServerListener *listener = nullptr;
    DataWriter *reply_writer = nullptr;
    DataReader *request_reader = nullptr;
};

// 消息订阅发布接口
class DDSer
{
public:
    DDSer();

    virtual ~DDSer();

    /**
     * @brief 消息订阅发布接口初始化
     *
     * @param discovery_server 是否启用中心式发现, 默认不启用
     */
    bool init(const bool discovery_server = false);

    /**
     * @brief 创建主题对应的数据写入器
     *
     * @param topic_name 主题名
     * @param reliable 是否采用可靠传输，默认为非可靠传输
     */
    bool createWriter(const std::string &topic_name, const bool reliable = false);

    /**
     * @brief 删除主题对应的数据写入器
     *
     * @param topic_name 主题名
     */
    void deleteWriter(const std::string &topic_name);

    /**
     * @brief 发布数据
     *
     * @param bytes 字节序列
     * @param topic_name 主题名
     */
    void publish(const ByteSequence &bytes, const std::string &topic_name);

    /**
     * @brief 创建主题对应的数据读取器
     *
     * @param topic_name 主题名
     * @param reliable 是否采用可靠传输，默认为非可靠传输
     */
    bool createReader(const std::string &topic_name, const bool reliable = false);

    /**
     * @brief 删除主题对应的数据读取器
     *
     * @param topic_name 主题名
     */
    void deleteReader(const std::string &topic_name);

    /**
     * @brief 设置主题对应的数据读取监听回调函数
     *
     * @param topic_name 主题名
     * @param method 数据读取监听回调函数
     */
    void modifyProcess(const std::string &topic_name, const processPtr method);

    /**
     * @brief 创建 请求-响应 客户端
     *
     * @param topic_name 主题名
     */
    bool createClient(const std::string &topic_name);

    /**
     * @brief 删除 请求-响应 客户端
     *
     * @param topic_name 主题名
     */
    void deleteClient(const std::string &topic_name);

    /**
     * @brief 请求
     *
     * @param bytes 请求数据
     * @param topic_name 主题名
     * @param result 响应结果
     * @param time_wait 等待时长(秒)
     *
     * @return 1表示成功收到响应结果, 0表示发送数据为空, -1表示客户端未创建, -2表示请求发送失败, -3表示超时
     */
    int request(const ByteSequence &bytes, const std::string &topic_name, ByteSequence &result, int time_wait = 3);

    /**
     * @brief 创建 请求-响应 服务端
     *
     * @param topic_name 主题名
     */
    bool createServer(const std::string &topic_name);

    /**
     * @brief 删除 请求-响应 服务端
     *
     * @param topic_name 主题名
     */
    void deleteServer(const std::string &topic_name);

    /**
     * @brief 设置 请求-响应 服务端处理函数
     *
     * @param topic_name 主题名
     * @param method 处理函数
     */
    void modifyServerProcess(const std::string &topic_name, const serverProcessPtr method);

private:
    DomainParticipant *participant_;

    TypeSupport type_;

    Publisher *publisher_;

    Subscriber *subscriber_;

    std::mutex topic_mtx_;

    std::unordered_map<std::string, Topic *> topic_list_;

    std::unordered_map<std::string, PubConfig> writer_list_;

    std::unordered_map<std::string, SubConfig> reader_list_;

    std::unordered_map<std::string, ClientConfig> client_list_;

    std::unordered_map<std::string, ServerConfig> server_list_;

    DiscoveryServerListener participant_listener_;

    bool init_flag_;
};

#endif // _PUBSUBTOOLS_H_
