/**
 * @file dds_config.h
 * @brief 读取 config/dds.conf，为 InitTopic() 提供 DDS 初始化参数。
 *
 * 配置文件格式 (key=value, # 开头为注释):
 *   domain_id=106
 *   multicast_ip=239.255.0.106
 *
 * 用法:
 *   uint32_t domain_id;
 *   std::string multicast_ip;
 *   if (!dds_config_load(domain_id, multicast_ip)) {
 *       // 加载失败，使用代码内置默认值
 *   }
 *   InitTopic(100, domain_id, multicast_ip);
 */

#ifndef DDS_CONFIG_H
#define DDS_CONFIG_H

#include <string>
#include <cstdio>
#include <cstring>
#include <fstream>

// 默认配置文件路径（相对于工作目录）
#define DDS_CONFIG_PATH "config/dds.conf"

// 代码内置默认值（配置文件缺失时使用）
#define DDS_CONFIG_DEFAULT_DOMAIN_ID    0
#define DDS_CONFIG_DEFAULT_MULTICAST_IP "239.255.0.1"

/**
 * 从配置文件读取 DDS 参数。
 * @param domain_id    [out] 域 ID
 * @param multicast_ip [out] 多播 IP 地址
 * @return true=成功加载配置文件, false=文件不存在或格式错误（调用方可回退到默认值）
 */
inline bool dds_config_load(uint32_t& domain_id, std::string& multicast_ip) {
    std::ifstream f(DDS_CONFIG_PATH);
    if (!f.is_open()) {
        fprintf(stderr, "[Config] 无法打开 %s，使用内置默认值\n", DDS_CONFIG_PATH);
        return false;
    }

    domain_id = DDS_CONFIG_DEFAULT_DOMAIN_ID;
    multicast_ip = DDS_CONFIG_DEFAULT_MULTICAST_IP;
    bool got_domain = false, got_ip = false;

    char line[256];
    while (f.getline(line, sizeof(line))) {
        // 去除首尾空白
        char* p = line;
        while (*p == ' ' || *p == '\t') ++p;
        // 跳过空行和注释
        if (*p == '\0' || *p == '#') continue;

        // 去除尾部换行/空白
        char* end = p + strlen(p) - 1;
        while (end > p && (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')) --end;
        *(end + 1) = '\0';

        // 解析 key=value
        char* eq = strchr(p, '=');
        if (!eq) continue;
        *eq = '\0';
        const char* key = p;
        const char* val = eq + 1;

        if (strcmp(key, "domain_id") == 0) {
            // 手写解析，避免 glibc 2.38+ 的 __isoc23_strtol 符号依赖
            {
                uint32_t v = 0;
                for (const char* p = val; *p >= '0' && *p <= '9'; ++p)
                    v = v * 10 + (uint32_t)(*p - '0');
                domain_id = v;
            }
            got_domain = true;
        } else if (strcmp(key, "multicast_ip") == 0) {
            multicast_ip = val;
            got_ip = true;
        }
    }

    if (!got_domain || !got_ip) {
        fprintf(stderr, "[Config] %s 缺少必要字段 (需要 domain_id 和 multicast_ip)\n", DDS_CONFIG_PATH);
        domain_id = DDS_CONFIG_DEFAULT_DOMAIN_ID;
        multicast_ip = DDS_CONFIG_DEFAULT_MULTICAST_IP;
        return false;
    }

    fprintf(stderr, "[Config] 已加载 %s: domain_id=%u multicast_ip=%s\n",
            DDS_CONFIG_PATH, domain_id, multicast_ip.c_str());
    return true;
}

#endif // DDS_CONFIG_H
