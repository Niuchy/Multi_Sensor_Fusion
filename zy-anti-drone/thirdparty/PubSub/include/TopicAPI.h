#ifndef _TOPIC_API_H_
#define _TOPIC_API_H_

#include <string>
#include <unordered_map>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <queue>
#include <condition_variable>

#include "PubSubTools.h"

extern uint16_t UserID; // 用户ID

extern std::string DBPath; // 主题配置数据库路径

extern DDSer GlobalDDSer; // 全局消息订阅发布接口

extern DiscoveryServer GlobalDDSServer; // 全局发现服务器接口

/**
 * @brief 主题设置
 *
 * @param reliable 是否采用可靠传输
 */
struct TopicConfig
{
    bool reliable = false;
};

/**
 * @brief 查询主题设置
 *
 * @param topic_name 主题名
 * @param topic_info 主题介绍
 * @param issubing 订阅状态
 * @param ispubing 发布状态
 */
struct checkConfig
{
    std::string topic_name;
    std::string topic_info;
    bool issubing = false;
    bool ispubing = false;
};

/**
 * @brief 修改主题设置
 *
 * @param topic_name 主题名
 * @param issubing 订阅状态
 * @param ispubing 发布状态
 */
struct changeConfig
{
    std::string topic_name;
    bool issubing = false;
    bool ispubing = false;
};

/**
 * @brief 主题接口初始化
 * 
 * @param discovery_server 是否启用中心式发现, 默认不启用
 */
bool InitTopic(const bool discovery_server = false);

/**
 * @brief 订阅用户ID主题，并进行默认主题初始化
 *
 * @param userID 用户ID
 * @param type 用户类型：1表示管控端，2表示用户端
 */
bool createReaderUserID(const uint16_t userID, const int type);

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
 * @brief 从数据库中查询当前主题设置
 *
 * @return 查询结果
 */
std::vector<checkConfig> checkTopicConfig();

/**
 * @brief 修改主题设置
 *
 * @param changes 修改
 */
void changeTopicConfig(const std::vector<changeConfig> &changes);

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

/**
 * @brief 启动发现服务器
 */
bool startDiscoveryServer();

/**
 * @brief 获取发现服务器IP地址
 */
std::string getDiscoveryServerIP();

#endif // _TOPIC_API_H_
