#ifndef CONFIG_READER_H
#define CONFIG_READER_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdbool.h>

    // 配置读取器结构体（不透明指针）
    typedef struct ConfigReader ConfigReader;

    /**
     * 创建配置读取器实例
     * @return 配置读取器指针，失败返回NULL
     */
    ConfigReader *config_create(void);

    /**
     * 销毁配置读取器实例
     * @param reader 配置读取器指针
     */
    void config_destroy(ConfigReader *reader);

    /**
     * 加载配置文件
     * @param reader 配置读取器指针
     * @param filename 配置文件路径
     * @return 成功返回true，失败返回false
     */
    bool config_load_file(ConfigReader *reader, const char *filename);

    /**
     * 保存配置到文件
     * @param reader 配置读取器指针
     * @param filename 文件路径（如果为NULL，则使用之前加载的文件路径）
     * @param keep_comments 是否保留注释
     * @return 成功返回true，失败返回false
     */
    bool config_save_file(ConfigReader *reader, const char *filename, bool keep_comments);

    // ========== 读取方法 ==========

    /**
     * 获取字符串类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param default_value 默认值（当key不存在时返回）
     * @return 配置值或默认值（返回的字符串需要调用者释放，使用config_free_string）
     */
    char *config_get_string(ConfigReader *reader, const char *key, const char *default_value);

    /**
     * 获取整数类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param default_value 默认值
     * @return 配置值或默认值
     */
    int config_get_int(ConfigReader *reader, const char *key, int default_value);

    /**
     * 获取浮点数类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param default_value 默认值
     * @return 配置值或默认值
     */
    double config_get_double(ConfigReader *reader, const char *key, double default_value);

    /**
     * 获取布尔类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param default_value 默认值
     * @return 配置值或默认值
     */
    bool config_get_bool(ConfigReader *reader, const char *key, bool default_value);

    // ========== 写入方法 ==========

    /**
     * 设置字符串类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param value 配置值
     */
    void config_set_string(ConfigReader *reader, const char *key, const char *value);

    /**
     * 设置整数类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param value 配置值
     */
    void config_set_int(ConfigReader *reader, const char *key, int value);

    /**
     * 设置浮点数类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param value 配置值
     */
    void config_set_double(ConfigReader *reader, const char *key, double value);

    /**
     * 设置布尔类型的配置值
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @param value 配置值
     */
    void config_set_bool(ConfigReader *reader, const char *key, bool value);

    // ========== 辅助方法 ==========

    /**
     * 检查配置项是否存在
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @return 存在返回true，否则返回false
     */
    bool config_has_key(ConfigReader *reader, const char *key);

    /**
     * 删除配置项
     * @param reader 配置读取器指针
     * @param key 配置项名称
     * @return 成功返回true，失败返回false
     */
    bool config_remove_key(ConfigReader *reader, const char *key);

    /**
     * 获取配置项的数量
     * @param reader 配置读取器指针
     * @return 配置项数量
     */
    size_t config_get_count(ConfigReader *reader);

    /**
     * 清空所有配置数据
     * @param reader 配置读取器指针
     */
    void config_clear(ConfigReader *reader);

    /**
     * 打印所有配置项（用于调试）
     * @param reader 配置读取器指针
     */
    void config_print_all(ConfigReader *reader);

    /**
     * 释放由config_get_string返回的字符串
     * @param str 字符串指针
     */
    void config_free_string(char *str);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_READER_H