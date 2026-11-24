#ifndef __PROTOHELPER_H__
#define __PROTOHELPER_H__

#include <cstdint>
#include <vector>
#include <type_traits>
#include <algorithm>
#include <functional>
#include <string>
#include <array>

#include <nlohmann/json.hpp>

using json = nlohmann::ordered_json;

using ByteSequence = std::vector<uint8_t>; // 字节序列

// 整型数据序列化
template <typename T>
typename std::enable_if<std::is_integral<T>::value>::type
packData(ByteSequence &bytes, const T value)
{
    for (size_t i = 0; i < sizeof(T); ++i)
    {
        bytes.push_back(static_cast<uint8_t>((value >> (i * 8)) & 0xFF));
    }
}

// 整型数据反序列化
template <typename T>
typename std::enable_if<std::is_integral<T>::value>::type
unpackData(const ByteSequence &bytes, T &value, size_t &pos)
{
    value = 0;

    if (pos + sizeof(T) > bytes.size())
    {
        return;
    }

    for (size_t i = 0; i < sizeof(T); ++i)
    {
        value |= static_cast<T>(bytes[pos++]) << (i * 8);
    }
}

// 浮点型数据序列化
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
packData(ByteSequence &bytes, const T value)
{
    uint8_t buffer[sizeof(T)];

    std::memcpy(buffer, &value, sizeof(T));

    bytes.insert(bytes.end(), buffer, buffer + sizeof(T));
}

// 浮点型数据反序列化
template <typename T>
typename std::enable_if<std::is_floating_point<T>::value>::type
unpackData(const ByteSequence &bytes, T &value, size_t &pos)
{
    value = 0;

    if (pos + sizeof(T) > bytes.size())
    {
        return;
    }

    uint8_t buffer[sizeof(T)];

    std::memcpy(buffer, &bytes[pos], sizeof(T));

    pos += sizeof(T);

    std::memcpy(&value, buffer, sizeof(T));
}

// string数据序列化
inline void packData(ByteSequence &bytes, const std::string &value)
{
    uint32_t length = value.size();

    packData(bytes, length);

    bytes.insert(bytes.end(), value.begin(), value.end());
}

// string数据反序列化
inline void unpackData(const ByteSequence &bytes, std::string &value, size_t &pos)
{
    uint32_t length = 0;

    unpackData(bytes, length, pos);

    value = std::string(bytes.begin() + pos, bytes.begin() + pos + length);

    pos += length;
}

// IP数据序列化 — 使用下方的定长uint8_t数组模板
// IP数据反序列化 — 使用下方的定长uint8_t数组模板

// 定长char数组序列化（如 sn_id[16], brand_model[32]）
template <size_t N>
void packData(ByteSequence &bytes, const char (&value)[N])
{
    bytes.insert(bytes.end(), value, value + N);
}

// 定长char数组反序列化（如 sn_id[16], brand_model[32]）
template <size_t N>
void unpackData(const ByteSequence &bytes, char (&value)[N], size_t &pos)
{
    if (pos + N > bytes.size())
    {
        return;
    }
    std::memcpy(value, &bytes[pos], N);
    value[N - 1] = '\0';  // 确保null终止
    pos += N;
}

// 定长uint8_t数组序列化（如 reserved[7]）
template <size_t N>
void packData(ByteSequence &bytes, const uint8_t (&value)[N])
{
    bytes.insert(bytes.end(), value, value + N);
}

// 定长uint8_t数组反序列化（如 reserved[7]）
template <size_t N>
void unpackData(const ByteSequence &bytes, uint8_t (&value)[N], size_t &pos)
{
    if (pos + N > bytes.size())
    {
        return;
    }
    std::memcpy(value, &bytes[pos], N);
    pos += N;
}

// 结构体数据序列化
template <typename T>
typename std::enable_if<std::is_class<T>::value>::type
packData(ByteSequence &bytes, const T &value)
{
    ByteSequence temp = value.serialize();

    bytes.insert(bytes.end(), temp.begin(), temp.end());
}

// 结构体数据反序列化
template <typename T>
typename std::enable_if<std::is_class<T>::value>::type
unpackData(const ByteSequence &bytes, T &value, size_t &pos)
{
    value.deserialize(bytes, pos);
}

// 数组数据序列化
template <typename T, size_t N>
void packData(ByteSequence &bytes, const std::array<T, N> &arr)
{
    for (const auto &v : arr)
    {
        packData(bytes, v);
    }
}

// 数组数据反序列化
template <typename T, size_t N>
void unpackData(const ByteSequence &bytes, std::array<T, N> &arr, size_t &pos)
{
    for (auto &v : arr)
    {
        unpackData(bytes, v, pos);
    }
}

// vector数据序列化
template <typename T>
void packData(ByteSequence &bytes, const std::vector<T> &values)
{
    uint32_t length = values.size();

    packData(bytes, length);

    for (uint32_t i = 0; i < length; ++i)
    {
        packData(bytes, values[i]);
    }
}

// vector数据反序列化
template <typename T>
void unpackData(const ByteSequence &bytes, std::vector<T> &values, size_t &pos)
{
    uint32_t length = 0;

    unpackData(bytes, length, pos);

    values.clear();

    for (uint32_t i = 0; i < length; ++i)
    {
        T value;

        unpackData(bytes, value, pos);

        values.push_back(value);
    }
}

// Json字段解析
template <typename T>
bool transfer_json(const json &j, const char *key, T &value)
{
    if (!j.contains(key))
    {
        return false;
    }

    try
    {
        value = j.at(key).get<T>();
    }
    catch (const std::exception &e)
    {
        std::string error_msg = std::string("json [") + key + "] error: " + e.what();

        throw std::runtime_error(error_msg);
    }

    return true;
}

// Json数组字段解析
template <typename T, size_t N>
bool transfer_json(const json &j, const char *key, std::array<T, N> &arr)
{
    auto it = j.find(key);

    if (it == j.end())
    {
        return false;
    }

    try
    {
        size_t count = std::min(it->size(), arr.size());

        for (size_t i = 0; i < count; ++i)
        {
            arr[i] = (*it)[i];
        }
    }
    catch (const std::exception &e)
    {
        std::string error_msg = std::string("json [") + key + "] error: " + e.what();

        throw std::runtime_error(error_msg);
    }

    return true;
}

#endif //__PROTOHELPER_H__
