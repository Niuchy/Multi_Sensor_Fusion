/**
 * @file proto_new.h
 * @brief 反蜂群装备标准化通信协议C++头文件（新设计）
 * @version 1.0.0
 * @date 2026-06-19
 * 
 * 本文件定义了反无人机装备标准化通信协议的C++类结构。
 * 1. 在帧头、帧体、帧尾结构体内部实现get/set函数
 * 2. 上层类直接调用结构体的接口，减少重复代码
 * 3. 减少帧体get/set接口的数量，只保留必要的接口
 * 
 * 协议特点：
 * 1. 默认按主机字节序直接传递（DDS同端场景无需额外字节序处理）
 * 2. 固定长度帧结构
 * 3. 预留CRC32字段（可空，暂不计算/校验）
 * 4. 支持多目标（单目标单帧，多目标多帧）
 * 5. 统一的量化单位和无效值约定
 */

#ifndef ANTI_DRONE_DEVPROTOCOL_NEW_H
#define ANTI_DRONE_DEVPROTOCOL_NEW_H

#include <cstdint>
#include <cstring>
#include <array>
#include <string>
#include <algorithm>

namespace AntiDroneDevProtocol {

// ============================================================================
// 通用常量定义
// ============================================================================

/**
 * @brief 帧类型定义
 */
enum class FrameType : uint16_t {
    RADAR_TARGET = 0x1A01,      ///< 雷达目标数据帧
    RADAR_DEVINFO = 0x1A02,     ///< 雷达设备信息帧（预留）
    RADAR_HEARTBEAT = 0x1A03,  ///< 雷达心跳帧（预留）

    FREQ_DETECT_TARGET = 0x1B01, ///< 频侦目标数据帧
    FREQ_DETECT_DEVINFO = 0x1B02,     ///< 频侦设备信息帧（预留）
    FREQ_DETECT_HEARTBEAT = 0x1B03,  ///< 频侦心跳帧（预留）

    CAM_TARGET = 0x1C01,     ///< 光电目标数据帧
    CAM_DEVINFO = 0x1C02,     ///< 光电设备信息帧（预留）
    CAM_HEARTBEAT = 0x1C03,  ///< 光电心跳帧（预留）

    ACOUSTIC_TARGET = 0x1D01,    ///< 声探测目标数据帧
    ACOUSTIC_DEVINFO = 0x1D02,     ///< 声探测设备信息帧（预留）
    ACOUSTIC_HEARTBEAT = 0x1D03,  ///< 声探测心跳帧（预留）
    
};

/**
 * @brief 帧内数据类型定义，暂时未使用
 */
enum class DataType : uint16_t {
    SINGLE_TARGET = 0x0001,      ///< 单目标数据
    MULTI_TARGET = 0x0002,       ///< 多目标数据（预留）
    TRACK_DATA = 0x0003,         ///< 航迹数据
    MEASUREMENT_DATA = 0x0004    ///< 量测数据
};

/**
 * @brief 目标类型定义
 */
enum class TargetType : uint8_t {
    UNKNOWN = 0,     ///< 未知类型
    BIRD = 1,        ///< 鸟类
    DRONE = 2,       ///< 无人机类
    VEHICLE = 3,     ///< 车辆
    PERSON = 4,      ///< 人员
    LARGE_TARGET = 5,///< 大型目标
    OTHER = 6        ///< 其它类型
};

/**
 * @brief 无效值定义
 */
constexpr uint32_t INVALID_UINT32 = 0xFFFFFFFF;
constexpr uint16_t INVALID_UINT16 = 0xFFFF;
constexpr int16_t INVALID_INT16 = 0x7FFF;
constexpr uint8_t INVALID_UINT8 = 0xFF;

inline void pack(uint8_t* buffer, size_t& offset, const void* src, size_t len) {
    std::memcpy(buffer + offset, src, len);
    offset += len;
}

inline void unpack(const uint8_t* buffer, size_t& offset, void* dst, size_t len) {
    std::memcpy(dst, buffer + offset, len);
    offset += len;
}

/**
 * @brief 帧同步字（Magic Number）枚举定义
 * 
 * 封装所有设备的帧同步字，提高代码可读性和可维护性
 */
enum class MagicNumber : uint32_t {
    RADAR_MAGIC = 0xA55AA55A,      ///< 雷达设备固定同步字
    FREQ_DETECT_MAGIC = 0xB44BB44B, ///< 频侦设备固定同步字
    CAMERA_MAGIC = 0xC33CC33C,      ///< 光电设备固定同步字
    ACOUSTIC_MAGIC = 0xD22DD22D,    ///< 声探测设备固定同步字
};

// ============================================================================
// 第一步：定义帧头结构体（所有设备共用）
// ============================================================================

/**
 * @brief 通用帧头结构体
 * 
 * 所有设备协议帧共享的帧头结构，包含帧同步字、协议版本、帧类型等信息。
 * 总长度：23字节
 */
struct DevFrameHeader {
    uint32_t magic;          ///< 帧同步字，使用MagicNumber枚举值
    uint8_t proto_ver;       ///< 协议版本号
    uint16_t ftype;          ///< 帧类型（FrameType枚举）
    uint16_t dtype;          ///< 帧内数据类型（Radar：目标信息帧{0：无效1：搜索航迹2：跟踪航迹}；Freq：目标信息帧{0：无效1：协议解析2：无线电解析}）
    uint16_t length;         ///< 报文总长度（含CRC32）
    uint32_t device_id;      ///< 设备唯一编号
    uint64_t timestamp;      ///< Unix epoch 毫秒，时间戳
    
    /**
     * @brief 默认构造函数
     */
    DevFrameHeader() : magic(0), proto_ver(1), ftype(0), dtype(0), 
                     length(0), device_id(0), timestamp(0) {}

    static constexpr size_t wireSize() {
        return sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t) +
               sizeof(uint32_t) + sizeof(uint64_t);
    }
    
    // ========================================================================
    // 帧头get/set接口
    // ========================================================================
    
    uint32_t getMagic() const { return magic; }
    void setMagic(uint32_t magic) { this->magic = magic; }
    
    uint8_t getProtoVer() const { return proto_ver; }
    void setProtoVer(uint8_t proto_ver) { this->proto_ver = proto_ver; }
    
    uint16_t getFrameType() const { return ftype; }
    void setFrameType(uint16_t ftype) { this->ftype = ftype; }
    
    uint16_t getDataType() const { return dtype; }
    void setDataType(uint16_t dtype) { this->dtype = dtype; }
    
    uint16_t getLength() const { return length; }
    void setLength(uint16_t length) { this->length = length; }
    
    uint32_t getDeviceId() const { return device_id; }
    void setDeviceId(uint32_t device_id) { this->device_id = device_id; }
    
    uint64_t getTimestamp() const { return timestamp; }
    void setTimestamp(uint64_t timestamp) { this->timestamp = timestamp; }
};

// ============================================================================
// 第二步：定义雷达设备帧体结构体
// ============================================================================

/**
 * @brief 雷达设备帧体结构体
 * 
 * 包含L1量测、L2航迹、L3识别三层信息。
 * 总长度：39字节
 */
struct RadarTargetInfoFrameBody {
    // L2航迹层
    uint32_t id;             ///< 目标ID（track_id）
    
    // L1量测层
    uint32_t dis;            ///< 目标距离量测（相对雷达），单位：0.1m
    int16_t azi;             ///< 目标方位量测（正北为0，顺时针为正，+-180°*100），单位：0.01°
    int16_t ele;             ///< 目标俯仰量测（上为正，+-180°*100），单位：0.01°
    uint32_t height;         ///< 目标相对雷达高度，单位：0.1m
    int16_t radial_vel;      ///< 目标径向速度量测（远离雷达为正），单位：0.1m/s
    
    // L2航迹层（续）
    int32_t lng;             ///< 目标经度，单位：1e-7°
    int32_t lat;             ///< 目标纬度，单位：1e-7°
    uint32_t alt;            ///< 目标海拔，单位：0.1m
    uint32_t speed;          ///< 目标速度，单位：0.1m/s
    uint16_t course;         ///< 目标航向，单位：0.01°
    uint8_t tag;             ///< 外推标志：0=外推点；1=真实点
    
    // L3识别层
    uint8_t target_type;     ///< 目标类别（参见TargetType枚举）
    uint8_t target_confidence; ///< 目标类别置信度（0~100）
    
    /**
     * @brief 默认构造函数
     */
    RadarTargetInfoFrameBody() : id(0), dis(0), azi(0), ele(0), height(0), radial_vel(0),
                       lng(0), lat(0), alt(0), speed(0), course(0), tag(1),
                       target_type(static_cast<uint8_t>(TargetType::UNKNOWN)),
                       target_confidence(0) {}

    static constexpr size_t wireSize() {
        return sizeof(uint32_t) + sizeof(uint32_t) + sizeof(int16_t) + sizeof(int16_t) + sizeof(uint32_t) +
               sizeof(int16_t) + sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t) + sizeof(uint32_t) +
               sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);
    }
    
    // ========================================================================
    // 帧体get/set接口（只保留必要的接口）
    // ========================================================================
    
    // 基本字段访问接口
    uint32_t getTargetId() const { return id; }
    void setTargetId(uint32_t target_id) { this->id = target_id; }
    
    uint8_t getExtrapolationFlag() const { return tag; }
    void setExtrapolationFlag(uint8_t tag) { this->tag = tag; }
    
    uint8_t getTargetType() const { return target_type; }
    void setTargetType(uint8_t target_type) { this->target_type = target_type; }
    
    uint8_t getTargetConfidence() const { return target_confidence; }
    void setTargetConfidence(uint8_t target_confidence) { this->target_confidence = target_confidence; }
    
    // ========================================================================
    // 逻辑相关的set接口（一起设置相关字段）
    // ========================================================================
    
    /**
     * @brief 设置目标位置（经纬度海拔一起设置）
     * @param lng 经度，单位：1e-7°
     * @param lat 纬度，单位：1e-7°
     * @param alt 海拔，单位：0.1m
     */
    void setPosition(int32_t lng, int32_t lat, uint32_t alt) {
        this->lng = lng;
        this->lat = lat;
        this->alt = alt;
    }
    
    /**
     * @brief 设置目标运动状态（速度航向一起设置）
     * @param speed 速度，单位：0.1m/s
     * @param course 航向，单位：0.01°
     */
    void setMotion(uint32_t speed, uint16_t course) {
        this->speed = speed;
        this->course = course;
    }
    
    /**
     * @brief 设置目标量测信息（距离方位俯仰一起设置）
     * @param dis 距离，单位：0.1m
     * @param azi 方位，单位：0.01°
     * @param ele 俯仰，单位：0.01°
     */
    void setMeasurement(uint32_t dis, int16_t azi, int16_t ele) {
        this->dis = dis;
        this->azi = azi;
        this->ele = ele;
    }
    
    /**
     * @brief 设置目标识别信息（目标类型和置信度一起设置）
     * @param target_type 目标类型（参见TargetType枚举）
     * @param target_confidence 目标类别置信度（0~100）
     */
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) {
        this->target_type = target_type;
        this->target_confidence = target_confidence;
    }
    
    /**
     * @brief 设置目标ID和外推标志（航迹相关属性一起设置）
     * @param target_id 目标ID（track_id）
     * @param tag 外推标志：0=外推点；1=真实点
     */
    void setTrackInfo(uint32_t target_id, uint8_t tag) {
        this->id = target_id;
        this->tag = tag;
    }
    
    /**
     * @brief 设置目标高度和径向速度（高度相关量测一起设置）
     * @param height 目标相对雷达高度，单位：0.1m
     * @param radial_vel 目标径向速度量测，单位：0.1m/s
     */
    void setHeightAndRadialVel(uint32_t height, int16_t radial_vel) {
        this->height = height;
        this->radial_vel = radial_vel;
    }
};

/**
 * @brief 通用帧尾结构体
 * 
 * 包含CRC32校验码，所有设备共用。
 * 总长度：4字节
 */
struct DevFrameTail {
    uint32_t crc32;          ///< CRC32校验码
    
    /**
     * @brief 默认构造函数
     */
    DevFrameTail() : crc32(0) {}

    static constexpr size_t wireSize() { return sizeof(uint32_t); }
    
    // ========================================================================
    // 帧尾get/set接口
    // ========================================================================
    
    uint32_t getCrc32() const { return crc32; }
    void setCrc32(uint32_t crc32) { this->crc32 = crc32; }
};

// ============================================================================
// 第三步：定义频侦设备帧体结构体
// ============================================================================

/**
 * @brief 频侦设备帧体结构体
 * 
 * 包含无人机和遥控器位置信息。
 * 总长度：112字节
 */
struct FrequencyDetectTargetInfoFrameBody {
    uint32_t target_id;      ///< 目标编号（没有则0）
    std::array<char, 16> sn_id; ///< 无人机Sn码（UTF-8/ASCII，不足补0）
    std::array<char, 32> brand_model; ///< 无人机品牌和无人机型号（UTF-8/ASCII，不足补0）
    int32_t lng_e7;          ///< 目标经度，单位：1e-7°
    int32_t lat_e7;          ///< 目标纬度，单位：1e-7°
    uint32_t alt;            ///< 目标海拔高度，单位：0.1m
    uint32_t height;         ///< 目标距离地面高度，单位：0.1m
    uint32_t speed;          ///< 目标速度，单位：0.1m/s
    uint32_t distance;     ///< 目标距离，单位：0.1m
    int32_t rctrl_lng_e7;    ///< 遥控器经度，单位：1e-7°
    int32_t rctrl_lat_e7;    ///< 遥控器纬度，单位：1e-7°
    uint32_t freq_hz;        ///< 目标信号频率，单位：Hz
    uint32_t bandw_hz;       ///< 目标信号带宽，单位：Hz
    int16_t power_db;        ///< 目标信号功率/强度，单位：0.1dBm
    uint16_t azimuth_cdeg;   ///< 目标方位角（测向0-360°*100），单位：0.01°
    uint64_t discovery_time; ///< 发现目标时间，单位：ms
    uint8_t confidence;      ///< 目标置信度（0~100）
    std::array<uint8_t, 7> reserved; ///< 保留，固定0
    
    /**
     * @brief 默认构造函数
     */
    FrequencyDetectTargetInfoFrameBody() : target_id(0), lng_e7(0), lat_e7(0), alt(0), 
                                 height(0), speed(0), distance(0), 
                                 rctrl_lng_e7(0), rctrl_lat_e7(0), freq_hz(0),
                                 bandw_hz(0), power_db(0), azimuth_cdeg(0),
                                 discovery_time(0), confidence(0) {
        sn_id.fill(0);
        brand_model.fill(0);
        reserved.fill(0);
    }

    static constexpr size_t wireSize() {
        return sizeof(uint32_t) + 16 + 32 + sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t) + sizeof(uint32_t) +
               sizeof(uint32_t) + sizeof(uint32_t) + sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t) +
               sizeof(uint32_t) + sizeof(int16_t) + sizeof(uint16_t) + sizeof(uint64_t) + sizeof(uint8_t) + 7;
    }
    
    // ========================================================================
    // 帧体get/set接口（只保留必要的接口）
    // ========================================================================
    
    // 基本字段访问接口
    uint32_t getTargetId() const { return target_id; }
    void setTargetId(uint32_t target_id) { this->target_id = target_id; }
    
    const std::array<char, 16>& getSnId() const { return sn_id; }
    void setSnId(const std::array<char, 16>& sn_id) { this->sn_id = sn_id; }
    
    const std::array<char, 32>& getBrandModel() const { return brand_model; }
    void setBrandModel(const std::array<char, 32>& brand_model) { this->brand_model = brand_model; }
    
    uint8_t getTargetConfidence() const { return confidence; }
    void setTargetConfidence(uint8_t target_confidence) { this->confidence = target_confidence; }
    
    // ========================================================================
    // 逻辑相关的set接口（一起设置相关字段）
    // ========================================================================
    
    /**
     * @brief 设置无人机位置（经纬度海拔一起设置）
     * @param lng_e7 经度，单位：1e-7°
     * @param lat_e7 纬度，单位：1e-7°
     * @param alt 海拔，单位：0.1m
     */
    void setUavPosition(int32_t lng_e7, int32_t lat_e7, uint32_t alt) {
        this->lng_e7 = lng_e7;
        this->lat_e7 = lat_e7;
        this->alt = alt;
    }
    
    /**
     * @brief 设置遥控器位置（经纬度一起设置）
     * @param rctrl_lng_e7 遥控器经度，单位：1e-7°
     * @param rctrl_lat_e7 遥控器纬度，单位：1e-7°
     */
    void setRctrlPosition(int32_t rctrl_lng_e7, int32_t rctrl_lat_e7) {
        this->rctrl_lng_e7 = rctrl_lng_e7;
        this->rctrl_lat_e7 = rctrl_lat_e7;
    }
    
    /**
     * @brief 设置目标高度信息（海拔和离地高度一起设置）
     * @param alt 海拔，单位：0.1m
     * @param height 离地高度，单位：0.1m
     */
    void setHeightInfo(uint32_t alt, uint32_t height) {
        this->alt = alt;
        this->height = height;
    }
    
    /**
     * @brief 设置目标运动信息（速度和距离一起设置）
     * @param speed 速度，单位：0.1m/s
     * @param distance 距离，单位：0.1m
     */
    void setMotionInfo(uint32_t speed, uint32_t distance) {
        this->speed = speed;
        this->distance = distance;
    }
    
    /**
     * @brief 设置信号参数（频率、带宽、功率一起设置）
     * @param freq_hz 频率，单位：Hz
     * @param bandw_hz 带宽，单位：Hz
     * @param power_db 功率，单位：0.1dBm
     */
    void setSignalParams(uint32_t freq_hz, uint32_t bandw_hz, int16_t power_db) {
        this->freq_hz = freq_hz;
        this->bandw_hz = bandw_hz;
        this->power_db = power_db;
    }
    
    /**
     * @brief 设置目标识别信息（目标ID、置信度一起设置）
     * @param target_id 目标编号
     * @param target_confidence 目标置信度（0~100）
     */
    void setTargetIdAndConfidence(uint32_t target_id, uint8_t target_confidence) {
        this->target_id = target_id;
        this->confidence = target_confidence;
    }
    
    /**
     * @brief 设置字符串信息（SN码和品牌型号一起设置）
     * @param sn_id SN码字符串
     * @param brand_model 品牌型号字符串
     */
    void setStringInfo(const std::string& sn_id, const std::string& brand_model) {
        // 复制SN码，不足部分补0
        size_t sn_len = std::min(sn_id.length(), static_cast<size_t>(16));
        std::memcpy(this->sn_id.data(), sn_id.c_str(), sn_len);
        if (sn_len < 16) {
            std::memset(this->sn_id.data() + sn_len, 0, 16 - sn_len);
        }
        
        // 复制品牌型号，不足部分补0
        size_t brand_len = std::min(brand_model.length(), static_cast<size_t>(32));
        std::memcpy(this->brand_model.data(), brand_model.c_str(), brand_len);
        if (brand_len < 32) {
            std::memset(this->brand_model.data() + brand_len, 0, 32 - brand_len);
        }
    }
};



// ============================================================================
// 第四步：定义光电设备帧体结构体
// ============================================================================

/**
 * @brief 光电设备帧体结构体
 * 
 * 包含云台姿态和目标位置信息。
 * 总长度：52字节
 */
struct CamTargetInfoFrameBody {
    uint32_t target_id;      ///< 目标编号（没有则0）
    uint8_t fault_status;    ///< 设备故障状态：0=无故障；1=故障
    uint8_t track_status;    ///< 跟踪状态：0=未跟踪；1=跟踪正常；2=记忆跟踪；3=目标丢失
    uint8_t track_count;     ///< 跟踪目标数：0未跟踪；>0为实际跟踪目标数
    uint8_t camfield;        ///< 光学镜头选择状态：0=红外；1=可见光；2=微光
    uint16_t ptz_zeroazi;    ///< 转台零位对应的方位角，单位：0.01°
    uint16_t ptz_roll;       ///< 转台滚转角（云台roll），单位：0.01°
    int16_t ptz_pitch;       ///< 转台俯仰角（云台pitch），单位：0.01°
    uint16_t ptz_yaw;        ///< 转台航向角（云台yaw），单位：0.01°
    int32_t ptz_lng_e7;      ///< 转台所在的经度，单位：1e-7°
    int32_t ptz_lat_e7;      ///< 转台所在的纬度，单位：1e-7°
    uint32_t ptz_alt;        ///< 转台所在的高度，单位：0.1m
    int32_t target_lng_e7;   ///< 目标经度，单位：1e-7°
    int32_t target_lat_e7;   ///< 目标纬度，单位：1e-7°
    uint32_t target_alt;     ///< 目标高度，单位：0.1m
    uint32_t target_dis;     ///< 目标距离，单位：0.1m
    uint16_t target_azimuth; ///< 目标方位角，单位：0.01°
    int16_t target_elevation;///< 目标俯仰角，单位：0.01°
    uint8_t target_type;     ///< 目标类别：0=鸟；1=无人机；2=大型目标；3=不明；4=车辆；5=人员；6=其他
    uint8_t target_confidence; ///< 目标置信度（0~100）
    std::array<uint8_t, 2> reserved; ///< 保留，固定0
    
    /**
     * @brief 默认构造函数
     */
    CamTargetInfoFrameBody() : target_id(0), fault_status(0), track_status(0),
                         track_count(0), camfield(0), ptz_zeroazi(0),
                         ptz_roll(0), ptz_pitch(0), ptz_yaw(0),
                         ptz_lng_e7(0), ptz_lat_e7(0), ptz_alt(0),
                         target_lng_e7(0), target_lat_e7(0), target_alt(0),
                         target_dis(0), target_azimuth(0), target_elevation(0),
                         target_type(static_cast<uint8_t>(TargetType::UNKNOWN)),
                         target_confidence(0) {
        reserved.fill(0);
    }

    static constexpr size_t wireSize() {
        return sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t) +
               sizeof(uint16_t) + sizeof(uint16_t) + sizeof(int16_t) + sizeof(uint16_t) + sizeof(int32_t) +
               sizeof(int32_t) + sizeof(uint32_t) + sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t) +
               sizeof(uint32_t) + sizeof(uint16_t) + sizeof(int16_t) + sizeof(uint8_t) + sizeof(uint8_t) + 2;
    }
    
    // ========================================================================
    // 帧体get/set接口（只保留必要的接口）
    // ========================================================================
    
    // 基本字段访问接口
    uint32_t getTargetId() const { return target_id; }
    void setTargetId(uint32_t target_id) { this->target_id = target_id; }
    
    uint8_t getFaultStatus() const { return fault_status; }
    void setFaultStatus(uint8_t fault_status) { this->fault_status = fault_status; }
    
    uint8_t getTrackStatus() const { return track_status; }
    void setTrackStatus(uint8_t track_status) { this->track_status = track_status; }
    
    uint8_t getTrackCount() const { return track_count; }
    void setTrackCount(uint8_t track_count) { this->track_count = track_count; }
    
    uint8_t getCamfield() const { return camfield; }
    void setCamfield(uint8_t camfield) { this->camfield = camfield; }
    
    uint8_t getTargetType() const { return target_type; }
    void setTargetType(uint8_t target_type) { this->target_type = target_type; }
    
    uint8_t getTargetConfidence() const { return target_confidence; }
    void setTargetConfidence(uint8_t target_confidence) { this->target_confidence = target_confidence; }
    
    // ========================================================================
    // 逻辑相关的set接口（一起设置相关字段）
    // ========================================================================
    
    /**
     * @brief 设置转台位置（经纬度海拔一起设置）
     * @param ptz_lng_e7 转台经度，单位：1e-7°
     * @param ptz_lat_e7 转台纬度，单位：1e-7°
     * @param ptz_alt 转台海拔，单位：0.1m
     */
    void setPtzPosition(int32_t ptz_lng_e7, int32_t ptz_lat_e7, uint32_t ptz_alt) {
        this->ptz_lng_e7 = ptz_lng_e7;
        this->ptz_lat_e7 = ptz_lat_e7;
        this->ptz_alt = ptz_alt;
    }
    
    /**
     * @brief 设置目标位置（经纬度海拔一起设置）
     * @param target_lng_e7 目标经度，单位：1e-7°
     * @param target_lat_e7 目标纬度，单位：1e-7°
     * @param target_alt 目标海拔，单位：0.1m
     */
    void setTargetPosition(int32_t target_lng_e7, int32_t target_lat_e7, uint32_t target_alt) {
        this->target_lng_e7 = target_lng_e7;
        this->target_lat_e7 = target_lat_e7;
        this->target_alt = target_alt;
    }
    
    /**
     * @brief 设置转台姿态角（roll、pitch、yaw一起设置）
     * @param ptz_roll 转台滚转角，单位：0.01°
     * @param ptz_pitch 转台俯仰角，单位：0.01°
     * @param ptz_yaw 转台航向角，单位：0.01°
     */
    void setPtzAttitude(uint16_t ptz_roll, int16_t ptz_pitch, uint16_t ptz_yaw) {
        this->ptz_roll = ptz_roll;
        this->ptz_pitch = ptz_pitch;
        this->ptz_yaw = ptz_yaw;
    }
    
    /**
     * @brief 设置目标角度信息（方位角和俯仰角一起设置）
     * @param target_azimuth 目标方位角，单位：0.01°
     * @param target_elevation 目标俯仰角，单位：0.01°
     */
    void setTargetAngles(uint16_t target_azimuth, int16_t target_elevation) {
        this->target_azimuth = target_azimuth;
        this->target_elevation = target_elevation;
    }
    
    /**
     * @brief 设置目标距离信息（距离和高度一起设置）
     * @param target_dis 目标距离，单位：0.1m
     * @param target_alt 目标高度，单位：0.1m
     */
    void setTargetDistanceInfo(uint32_t target_dis, uint32_t target_alt) {
        this->target_dis = target_dis;
        this->target_alt = target_alt;
    }
    
    /**
     * @brief 设置设备状态（故障状态、跟踪状态、跟踪目标数一起设置）
     * @param fault_status 设备故障状态：0=无故障；1=故障
     * @param track_status 跟踪状态：0=未跟踪；1=跟踪正常；2=记忆跟踪；3=目标丢失
     * @param track_count 跟踪目标数：0未跟踪；>0为实际跟踪目标数
     */
    void setDeviceStatus(uint8_t fault_status, uint8_t track_status, uint8_t track_count) {
        this->fault_status = fault_status;
        this->track_status = track_status;
        this->track_count = track_count;
    }
    
    /**
     * @brief 设置目标识别信息（目标类型和置信度一起设置）
     * @param target_type 目标类别
     * @param target_confidence 目标置信度（0~100）
     */
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) {
        this->target_type = target_type;
        this->target_confidence = target_confidence;
    }
};



// ============================================================================
// 第五步：定义声探测设备帧体结构体
// ============================================================================

/**
 * @brief 声探测设备帧体结构体
 * 
 * 包含安装姿态角和目标相对角度信息。
 * 总长度：40字节（含设备经纬高）
 */
struct AcousticTargetInfoFrameBody {
    uint32_t target_id;      ///< 目标编号（没有则0）
    uint8_t point_count;     ///< 探测点数量（单目标报文建议固定1）
    std::array<uint8_t, 3> reserved0; ///< 保留，固定0
    int16_t device_roll;     ///< 安装横滚角（-18000~18000），单位：0.01°
    int16_t device_pitch;    ///< 安装俯仰角（-18000~18000），单位：0.01°
    int16_t device_yaw;      ///< 安装偏航角（-18000~18000），单位：0.01°
    std::array<uint8_t, 2> reserved1; ///< 保留，固定0
    uint16_t target_roll;    ///< 无人机相对设备横滚方向夹角（0~18000），单位：0.01°
    uint16_t target_pitch;   ///< 无人机相对设备俯仰方向夹角（0~18000），单位：0.01°
    uint16_t target_yaw;     ///< 无人机相对设备偏航方向夹角，0=正北，顺时针为正（0~36000），单位：0.01°
    uint8_t target_type;     ///< 目标类别：0=鸟；1=无人机；2=大型目标；3=不明；4=车辆；5=人员；6=其他
    uint8_t target_confidence; ///< 目标置信度（0~100）
    std::array<uint8_t, 4> reserved; ///< 保留，固定0
    // 声探测设备自身经纬高 (与雷达L2航迹格式一致)
    int32_t device_lng_e7;   ///< 设备经度，单位：1e-7°
    int32_t device_lat_e7;   ///< 设备纬度，单位：1e-7°
    uint32_t device_alt_dm;  ///< 设备海拔，单位：0.1m

    /**
     * @brief 默认构造函数
     */
    AcousticTargetInfoFrameBody() : target_id(0), point_count(1), device_roll(0),
                          device_pitch(0), device_yaw(0), target_roll(0),
                          target_pitch(0), target_yaw(0),
                          target_type(static_cast<uint8_t>(TargetType::UNKNOWN)),
                          target_confidence(0),
                          device_lng_e7(0), device_lat_e7(0), device_alt_dm(0) {
        reserved0.fill(0);
        reserved1.fill(0);
        reserved.fill(0);
    }

    static constexpr size_t wireSize() {
        return sizeof(uint32_t) + sizeof(uint8_t) + 3 + sizeof(int16_t) + sizeof(int16_t) + sizeof(int16_t) + 2 +
               sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t) + 4 +
               sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t);
    }
    
    // ========================================================================
    // 帧体get/set接口（只保留必要的接口）
    // ========================================================================
    
    // 基本字段访问接口
    uint32_t getTargetId() const { return target_id; }
    void setTargetId(uint32_t target_id) { this->target_id = target_id; }
    
    uint8_t getPointCount() const { return point_count; }
    void setPointCount(uint8_t point_count) { this->point_count = point_count; }
    
    uint8_t getTargetType() const { return target_type; }
    void setTargetType(uint8_t target_type) { this->target_type = target_type; }
    
    uint8_t getTargetConfidence() const { return target_confidence; }
    void setTargetConfidence(uint8_t target_confidence) { this->target_confidence = target_confidence; }
    
    // ========================================================================
    // 逻辑相关的set接口（一起设置相关字段）
    // ========================================================================
    
    /**
     * @brief 设置设备安装姿态角（roll、pitch、yaw一起设置）
     * @param device_roll 安装横滚角，单位：0.01°
     * @param device_pitch 安装俯仰角，单位：0.01°
     * @param device_yaw 安装偏航角，单位：0.01°
     */
    void setDeviceAttitude(int16_t device_roll, int16_t device_pitch, int16_t device_yaw) {
        this->device_roll = device_roll;
        this->device_pitch = device_pitch;
        this->device_yaw = device_yaw;
    }
    
    /**
     * @brief 设置目标相对角度（roll、pitch、yaw一起设置）
     * @param target_roll 无人机相对设备横滚方向夹角，单位：0.01°
     * @param target_pitch 无人机相对设备俯仰方向夹角，单位：0.01°
     * @param target_yaw 无人机相对设备偏航方向夹角，单位：0.01°
     */
    void setTargetRelativeAngles(uint16_t target_roll, uint16_t target_pitch, uint16_t target_yaw) {
        this->target_roll = target_roll;
        this->target_pitch = target_pitch;
        this->target_yaw = target_yaw;
    }
    
    /**
     * @brief 设置目标识别信息（目标类型和置信度一起设置）
     * @param target_type 目标类别
     * @param target_confidence 目标置信度（0~100）
     */
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) {
        this->target_type = target_type;
        this->target_confidence = target_confidence;
    }
    
    /**
     * @brief 设置目标基本信息（目标ID和探测点数量一起设置）
     * @param target_id 目标编号
     * @param point_count 探测点数量
     */
    void setTargetBasicInfo(uint32_t target_id, uint8_t point_count) {
        this->target_id = target_id;
        this->point_count = point_count;
    }

    // 设备经纬高 (与雷达L2航迹格式一致)
    void setDevicePosition(int32_t lng_e7, int32_t lat_e7, uint32_t alt_dm) {
        this->device_lng_e7 = lng_e7;
        this->device_lat_e7 = lat_e7;
        this->device_alt_dm = alt_dm;
    }
};



// ============================================================================
// 第六步：定义上层类，直接调用结构体的接口
// ============================================================================

/**
 * @brief 雷达设备标准化协议帧类
 * 
 * 包含帧头、帧体、帧尾三部分，提供序列化和反序列化接口。
 * 总长度：66字节
 */
class RadarTargetFrame {
private:
    DevFrameHeader header;     ///< 帧头部分
    RadarTargetInfoFrameBody body;     ///< 帧体部分
    DevFrameTail tail;     ///< 帧尾部分
    
public:
    /**
     * @brief 默认构造函数
     */
    RadarTargetFrame() {
        reset();
    }
    
    /**
     * @brief 重置帧数据为默认值
     */
    void reset() {
        // 设置帧头默认值
        header.setMagic(static_cast<uint32_t>(MagicNumber::RADAR_MAGIC));  // 雷达设备固定同步字
        header.setProtoVer(1);
        header.setFrameType(static_cast<uint16_t>(FrameType::RADAR_TARGET));
        header.setDataType(static_cast<uint16_t>(DataType::SINGLE_TARGET));
        header.setLength(static_cast<uint16_t>(size()));
        header.setDeviceId(0);
        header.setTimestamp(0);
        
        // 帧体已由构造函数初始化
        
        // 帧尾
        tail.setCrc32(0);
    }
    
    /**
     * @brief 序列化接口：将帧数据转换为字节数组
     * @param buffer 输出缓冲区，必须至少有size()字节的空间
     * @return true - 序列化成功，false - 失败
     */
    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        pack(buffer, offset, &header.magic, sizeof(header.magic));
        pack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        pack(buffer, offset, &header.ftype, sizeof(header.ftype));
        pack(buffer, offset, &header.dtype, sizeof(header.dtype));
        pack(buffer, offset, &header.length, sizeof(header.length));
        pack(buffer, offset, &header.device_id, sizeof(header.device_id));
        pack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        pack(buffer, offset, &body.id, sizeof(body.id));
        pack(buffer, offset, &body.dis, sizeof(body.dis));
        pack(buffer, offset, &body.azi, sizeof(body.azi));
        pack(buffer, offset, &body.ele, sizeof(body.ele));
        pack(buffer, offset, &body.height, sizeof(body.height));
        pack(buffer, offset, &body.radial_vel, sizeof(body.radial_vel));
        pack(buffer, offset, &body.lng, sizeof(body.lng));
        pack(buffer, offset, &body.lat, sizeof(body.lat));
        pack(buffer, offset, &body.alt, sizeof(body.alt));
        pack(buffer, offset, &body.speed, sizeof(body.speed));
        pack(buffer, offset, &body.course, sizeof(body.course));
        pack(buffer, offset, &body.tag, sizeof(body.tag));
        pack(buffer, offset, &body.target_type, sizeof(body.target_type));
        pack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));

        pack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return true;
    }
    
    /**
     * @brief 反序列化接口：从字节数组解析帧数据
     * @param buffer 输入缓冲区，必须至少有size()字节的数据
     * @return true - 反序列化成功，false - 失败
     */
    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        unpack(buffer, offset, &header.magic, sizeof(header.magic));
        unpack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        unpack(buffer, offset, &header.ftype, sizeof(header.ftype));
        unpack(buffer, offset, &header.dtype, sizeof(header.dtype));
        unpack(buffer, offset, &header.length, sizeof(header.length));
        unpack(buffer, offset, &header.device_id, sizeof(header.device_id));
        unpack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        unpack(buffer, offset, &body.id, sizeof(body.id));
        unpack(buffer, offset, &body.dis, sizeof(body.dis));
        unpack(buffer, offset, &body.azi, sizeof(body.azi));
        unpack(buffer, offset, &body.ele, sizeof(body.ele));
        unpack(buffer, offset, &body.height, sizeof(body.height));
        unpack(buffer, offset, &body.radial_vel, sizeof(body.radial_vel));
        unpack(buffer, offset, &body.lng, sizeof(body.lng));
        unpack(buffer, offset, &body.lat, sizeof(body.lat));
        unpack(buffer, offset, &body.alt, sizeof(body.alt));
        unpack(buffer, offset, &body.speed, sizeof(body.speed));
        unpack(buffer, offset, &body.course, sizeof(body.course));
        unpack(buffer, offset, &body.tag, sizeof(body.tag));
        unpack(buffer, offset, &body.target_type, sizeof(body.target_type));
        unpack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));

        unpack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return checkMagic() && checkLength();
    }
    
    /**
     * @brief 检查帧同步字是否正确
     * @return true - 同步字正确，false - 同步字错误
     */
    bool checkMagic() const {
        return header.getMagic() == static_cast<uint32_t>(MagicNumber::RADAR_MAGIC);
    }
    
    /**
     * @brief 检查帧长度是否正确
     * @return true - 长度正确，false - 长度错误
     */
    bool checkLength() const {
        return header.getLength() == static_cast<uint16_t>(size());
    }
    
    /**
     * @brief 获取帧数据指针
     * @return 指向帧数据的指针
     */
    const uint8_t* data() const {
        return reinterpret_cast<const uint8_t*>(this);
    }
    
    /**
     * @brief 获取帧数据大小
     * @return 帧数据大小（字节）
     */
    static constexpr size_t size() {
        return DevFrameHeader::wireSize() + RadarTargetInfoFrameBody::wireSize() + DevFrameTail::wireSize();
    }
    
    // ========================================================================
    // 上层类接口，直接调用结构体的接口
    // ========================================================================
    
    // 帧头访问接口
    DevFrameHeader& getHeader() { return header; }
    void setHeader(const DevFrameHeader& newHeader) { header = newHeader; }
    
    // 帧体访问接口
    RadarTargetInfoFrameBody& getBody() { return body; }
    void setBody(const RadarTargetInfoFrameBody& newBody) { body = newBody; }
    
    // 帧尾访问接口
    DevFrameTail& getTail() { return tail; }
    void setTail(const DevFrameTail& newTail) { tail = newTail; }
    
    // 帧头字段访问接口（直接调用结构体的接口）
    uint32_t getMagic() const { return header.getMagic(); }
    void setMagic(uint32_t magic) { header.setMagic(magic); }
    
    uint8_t getProtoVer() const { return header.getProtoVer(); }
    void setProtoVer(uint8_t proto_ver) { header.setProtoVer(proto_ver); }
    
    uint16_t getFrameType() const { return header.getFrameType(); }
    void setFrameType(uint16_t ftype) { header.setFrameType(ftype); }
    
    uint16_t getDataType() const { return header.getDataType(); }
    void setDataType(uint16_t dtype) { header.setDataType(dtype); }
    
    uint16_t getLength() const { return header.getLength(); }
    void setLength(uint16_t length) { header.setLength(length); }
    
    uint32_t getDeviceId() const { return header.getDeviceId(); }
    void setDeviceId(uint32_t device_id) { header.setDeviceId(device_id); }
    
    uint64_t getTimestamp() const { return header.getTimestamp(); }
    void setTimestamp(uint64_t timestamp) { header.setTimestamp(timestamp); }
    
    // 帧体字段访问接口（直接调用结构体的接口）
    uint32_t getTargetId() const { return body.getTargetId(); }
    void setTargetId(uint32_t target_id) { body.setTargetId(target_id); }
    
    uint8_t getExtrapolationFlag() const { return body.getExtrapolationFlag(); }
    void setExtrapolationFlag(uint8_t tag) { body.setExtrapolationFlag(tag); }
    
    uint8_t getTargetType() const { return body.getTargetType(); }
    void setTargetType(uint8_t target_type) { body.setTargetType(target_type); }
    
    uint8_t getTargetConfidence() const { return body.getTargetConfidence(); }
    void setTargetConfidence(uint8_t target_confidence) { body.setTargetConfidence(target_confidence); }
    
    // 逻辑相关的set接口（直接调用结构体的接口）
    void setPosition(int32_t lng, int32_t lat, uint32_t alt) { body.setPosition(lng, lat, alt); }
    void setMotion(uint32_t speed, uint16_t course) { body.setMotion(speed, course); }
    void setMeasurement(uint32_t dis, int16_t azi, int16_t ele) { body.setMeasurement(dis, azi, ele); }
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) { body.setTargetInfo(target_type, target_confidence); }
    void setTrackInfo(uint32_t target_id, uint8_t tag) { body.setTrackInfo(target_id, tag); }
    void setHeightAndRadialVel(uint32_t height, int16_t radial_vel) { body.setHeightAndRadialVel(height, radial_vel); }
    
    // 帧尾字段访问接口（直接调用结构体的接口）
    uint32_t getCrc32() const { return tail.getCrc32(); }
    void setCrc32(uint32_t crc32) { tail.setCrc32(crc32); }
};

/**
 * @brief 频侦设备标准化协议帧类
 * 
 * 包含帧头、帧体、帧尾三部分，提供序列化和反序列化接口。
 * 总长度：139字节
 */
class FrequencyDetectFrame {
private:
    DevFrameHeader header;     ///< 帧头部分
    FrequencyDetectTargetInfoFrameBody body; ///< 帧体部分
    DevFrameTail tail; ///< 帧尾部分
    
public:
    /**
     * @brief 默认构造函数
     */
    FrequencyDetectFrame() {
        reset();
    }
    
    /**
     * @brief 重置帧数据为默认值
     */
    void reset() {
        // 设置帧头默认值
        header.setMagic(static_cast<uint32_t>(MagicNumber::FREQ_DETECT_MAGIC));  // 频侦设备固定同步字
        header.setProtoVer(1);
        header.setFrameType(static_cast<uint16_t>(FrameType::FREQ_DETECT_TARGET));
        header.setDataType(static_cast<uint16_t>(DataType::SINGLE_TARGET));
        header.setLength(static_cast<uint16_t>(size()));
        header.setDeviceId(0);
        header.setTimestamp(0);
        
        // 帧体已由构造函数初始化
        
        // 帧尾
        tail.setCrc32(0);
    }
    
    /**
     * @brief 序列化接口：将帧数据转换为字节数组
     * @param buffer 输出缓冲区，必须至少有size()字节的空间
     * @return true - 序列化成功，false - 失败
     */
    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        pack(buffer, offset, &header.magic, sizeof(header.magic));
        pack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        pack(buffer, offset, &header.ftype, sizeof(header.ftype));
        pack(buffer, offset, &header.dtype, sizeof(header.dtype));
        pack(buffer, offset, &header.length, sizeof(header.length));
        pack(buffer, offset, &header.device_id, sizeof(header.device_id));
        pack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        pack(buffer, offset, &body.target_id, sizeof(body.target_id));
        pack(buffer, offset, body.sn_id.data(), body.sn_id.size());
        pack(buffer, offset, body.brand_model.data(), body.brand_model.size());
        pack(buffer, offset, &body.lng_e7, sizeof(body.lng_e7));
        pack(buffer, offset, &body.lat_e7, sizeof(body.lat_e7));
        pack(buffer, offset, &body.alt, sizeof(body.alt));
        pack(buffer, offset, &body.height, sizeof(body.height));
        pack(buffer, offset, &body.speed, sizeof(body.speed));
        pack(buffer, offset, &body.distance, sizeof(body.distance));
        pack(buffer, offset, &body.rctrl_lng_e7, sizeof(body.rctrl_lng_e7));
        pack(buffer, offset, &body.rctrl_lat_e7, sizeof(body.rctrl_lat_e7));
        pack(buffer, offset, &body.freq_hz, sizeof(body.freq_hz));
        pack(buffer, offset, &body.bandw_hz, sizeof(body.bandw_hz));
        pack(buffer, offset, &body.power_db, sizeof(body.power_db));
        pack(buffer, offset, &body.azimuth_cdeg, sizeof(body.azimuth_cdeg));
        pack(buffer, offset, &body.discovery_time, sizeof(body.discovery_time));
        pack(buffer, offset, &body.confidence, sizeof(body.confidence));
        pack(buffer, offset, body.reserved.data(), body.reserved.size());

        pack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return true;
    }
    
    /**
     * @brief 反序列化接口：从字节数组解析帧数据
     * @param buffer 输入缓冲区，必须至少有size()字节的数据
     * @return true - 反序列化成功，false - 失败
     */
    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        unpack(buffer, offset, &header.magic, sizeof(header.magic));
        unpack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        unpack(buffer, offset, &header.ftype, sizeof(header.ftype));
        unpack(buffer, offset, &header.dtype, sizeof(header.dtype));
        unpack(buffer, offset, &header.length, sizeof(header.length));
        unpack(buffer, offset, &header.device_id, sizeof(header.device_id));
        unpack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        unpack(buffer, offset, &body.target_id, sizeof(body.target_id));
        unpack(buffer, offset, body.sn_id.data(), body.sn_id.size());
        unpack(buffer, offset, body.brand_model.data(), body.brand_model.size());
        unpack(buffer, offset, &body.lng_e7, sizeof(body.lng_e7));
        unpack(buffer, offset, &body.lat_e7, sizeof(body.lat_e7));
        unpack(buffer, offset, &body.alt, sizeof(body.alt));
        unpack(buffer, offset, &body.height, sizeof(body.height));
        unpack(buffer, offset, &body.speed, sizeof(body.speed));
        unpack(buffer, offset, &body.distance, sizeof(body.distance));
        unpack(buffer, offset, &body.rctrl_lng_e7, sizeof(body.rctrl_lng_e7));
        unpack(buffer, offset, &body.rctrl_lat_e7, sizeof(body.rctrl_lat_e7));
        unpack(buffer, offset, &body.freq_hz, sizeof(body.freq_hz));
        unpack(buffer, offset, &body.bandw_hz, sizeof(body.bandw_hz));
        unpack(buffer, offset, &body.power_db, sizeof(body.power_db));
        unpack(buffer, offset, &body.azimuth_cdeg, sizeof(body.azimuth_cdeg));
        unpack(buffer, offset, &body.discovery_time, sizeof(body.discovery_time));
        unpack(buffer, offset, &body.confidence, sizeof(body.confidence));
        unpack(buffer, offset, body.reserved.data(), body.reserved.size());

        unpack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return checkMagic() && checkLength();
    }
    
    /**
     * @brief 检查帧同步字是否正确
     * @return true - 同步字正确，false - 同步字错误
     */
    bool checkMagic() const {
        return header.getMagic() == static_cast<uint32_t>(MagicNumber::FREQ_DETECT_MAGIC);
    }
    
    /**
     * @brief 检查帧长度是否正确
     * @return true - 长度正确，false - 长度错误
     */
    bool checkLength() const {
        return header.getLength() == static_cast<uint16_t>(size());
    }
    
    /**
     * @brief 获取帧数据指针
     * @return 指向帧数据的指针
     */
    const uint8_t* data() const {
        return reinterpret_cast<const uint8_t*>(this);
    }
    
    /**
     * @brief 获取帧数据大小
     * @return 帧数据大小（字节）
     */
    static constexpr size_t size() {
        return DevFrameHeader::wireSize() + FrequencyDetectTargetInfoFrameBody::wireSize() + DevFrameTail::wireSize();
    }
    
    // ========================================================================
    // 上层类接口，直接调用结构体的接口
    // ========================================================================
    
    // 帧头访问接口
    DevFrameHeader& getHeader() { return header; }
    void setHeader(const DevFrameHeader& newHeader) { header = newHeader; }
    
    // 帧体访问接口
    FrequencyDetectTargetInfoFrameBody& getBody() { return body; }
    void setBody(const FrequencyDetectTargetInfoFrameBody& newBody) { body = newBody; }
    
    // 帧尾访问接口
    DevFrameTail& getTail() { return tail; }
    void setTail(const DevFrameTail& newTail) { tail = newTail; }
    
    // 帧头字段访问接口（直接调用结构体的接口）
    uint32_t getMagic() const { return header.getMagic(); }
    void setMagic(uint32_t magic) { header.setMagic(magic); }
    
    uint8_t getProtoVer() const { return header.getProtoVer(); }
    void setProtoVer(uint8_t proto_ver) { header.setProtoVer(proto_ver); }
    
    uint16_t getFrameType() const { return header.getFrameType(); }
    void setFrameType(uint16_t ftype) { header.setFrameType(ftype); }
    
    uint16_t getDataType() const { return header.getDataType(); }
    void setDataType(uint16_t dtype) { header.setDataType(dtype); }
    
    uint16_t getLength() const { return header.getLength(); }
    void setLength(uint16_t length) { header.setLength(length); }
    
    uint32_t getDeviceId() const { return header.getDeviceId(); }
    void setDeviceId(uint32_t device_id) { header.setDeviceId(device_id); }
    
    uint64_t getTimestamp() const { return header.getTimestamp(); }
    void setTimestamp(uint64_t timestamp) { header.setTimestamp(timestamp); }
    
    // 帧体字段访问接口（直接调用结构体的接口）
    uint32_t getTargetId() const { return body.getTargetId(); }
    void setTargetId(uint32_t target_id) { body.setTargetId(target_id); }
    
    const std::array<char, 16>& getSnId() const { return body.getSnId(); }
    void setSnId(const std::array<char, 16>& sn_id) { body.setSnId(sn_id); }
    
    const std::array<char, 32>& getBrandModel() const { return body.getBrandModel(); }
    void setBrandModel(const std::array<char, 32>& brand_model) { body.setBrandModel(brand_model); }
    
    uint8_t getTargetConfidence() const { return body.getTargetConfidence(); }
    void setTargetConfidence(uint8_t target_confidence) { body.setTargetConfidence(target_confidence); }
    
    // 逻辑相关的set接口（直接调用结构体的接口）
    void setUavPosition(int32_t lng_e7, int32_t lat_e7, uint32_t alt) { body.setUavPosition(lng_e7, lat_e7, alt); }
    void setRctrlPosition(int32_t rctrl_lng_e7, int32_t rctrl_lat_e7) { body.setRctrlPosition(rctrl_lng_e7, rctrl_lat_e7); }
    void setHeightInfo(uint32_t alt, uint32_t height) { body.setHeightInfo(alt, height); }
    void setMotionInfo(uint32_t speed, uint32_t distance) { body.setMotionInfo(speed, distance); }
    void setSignalParams(uint32_t freq_hz, uint32_t bandw_hz, int16_t power_db) { body.setSignalParams(freq_hz, bandw_hz, power_db); }
    void setTargetIdAndConfidence(uint32_t target_id, uint8_t target_confidence) { body.setTargetIdAndConfidence(target_id, target_confidence); }
    void setStringInfo(const std::string& sn_id, const std::string& brand_model) { body.setStringInfo(sn_id, brand_model); }
    
    // 帧尾字段访问接口（直接调用结构体的接口）
    uint32_t getCrc32() const { return tail.getCrc32(); }
    void setCrc32(uint32_t crc32) { tail.setCrc32(crc32); }
};

/**
 * @brief 光电设备标准化协议帧类
 * 
 * 包含帧头、帧体、帧尾三部分，提供序列化和反序列化接口。
 * 总长度：79字节
 */
class CamTargetFrame {
private:
    DevFrameHeader header;     ///< 帧头部分
    CamTargetInfoFrameBody body;   ///< 帧体部分
    DevFrameTail tail;   ///< 帧尾部分
    
public:
    /**
     * @brief 默认构造函数
     */
    CamTargetFrame() {
        reset();
    }
    
    /**
     * @brief 重置帧数据为默认值
     */
    void reset() {
        // 设置帧头默认值
        header.setMagic(static_cast<uint32_t>(MagicNumber::CAMERA_MAGIC));  // 光电设备固定同步字
        header.setProtoVer(1);
        header.setFrameType(static_cast<uint16_t>(FrameType::CAM_TARGET));
        header.setDataType(static_cast<uint16_t>(DataType::SINGLE_TARGET));
        header.setLength(static_cast<uint16_t>(size()));
        header.setDeviceId(0);
        header.setTimestamp(0);
        
        // 帧体已由构造函数初始化
        
        // 帧尾
        tail.setCrc32(0);
    }
    
    /**
     * @brief 序列化接口：将帧数据转换为字节数组
     * @param buffer 输出缓冲区，必须至少有size()字节的空间
     * @return true - 序列化成功，false - 失败
     */
    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        pack(buffer, offset, &header.magic, sizeof(header.magic));
        pack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        pack(buffer, offset, &header.ftype, sizeof(header.ftype));
        pack(buffer, offset, &header.dtype, sizeof(header.dtype));
        pack(buffer, offset, &header.length, sizeof(header.length));
        pack(buffer, offset, &header.device_id, sizeof(header.device_id));
        pack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        pack(buffer, offset, &body.target_id, sizeof(body.target_id));
        pack(buffer, offset, &body.fault_status, sizeof(body.fault_status));
        pack(buffer, offset, &body.track_status, sizeof(body.track_status));
        pack(buffer, offset, &body.track_count, sizeof(body.track_count));
        pack(buffer, offset, &body.camfield, sizeof(body.camfield));
        pack(buffer, offset, &body.ptz_zeroazi, sizeof(body.ptz_zeroazi));
        pack(buffer, offset, &body.ptz_roll, sizeof(body.ptz_roll));
        pack(buffer, offset, &body.ptz_pitch, sizeof(body.ptz_pitch));
        pack(buffer, offset, &body.ptz_yaw, sizeof(body.ptz_yaw));
        pack(buffer, offset, &body.ptz_lng_e7, sizeof(body.ptz_lng_e7));
        pack(buffer, offset, &body.ptz_lat_e7, sizeof(body.ptz_lat_e7));
        pack(buffer, offset, &body.ptz_alt, sizeof(body.ptz_alt));
        pack(buffer, offset, &body.target_lng_e7, sizeof(body.target_lng_e7));
        pack(buffer, offset, &body.target_lat_e7, sizeof(body.target_lat_e7));
        pack(buffer, offset, &body.target_alt, sizeof(body.target_alt));
        pack(buffer, offset, &body.target_dis, sizeof(body.target_dis));
        pack(buffer, offset, &body.target_azimuth, sizeof(body.target_azimuth));
        pack(buffer, offset, &body.target_elevation, sizeof(body.target_elevation));
        pack(buffer, offset, &body.target_type, sizeof(body.target_type));
        pack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));
        pack(buffer, offset, body.reserved.data(), body.reserved.size());

        pack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return true;
    }
    
    /**
     * @brief 反序列化接口：从字节数组解析帧数据
     * @param buffer 输入缓冲区，必须至少有size()字节的数据
     * @return true - 反序列化成功，false - 失败
     */
    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        unpack(buffer, offset, &header.magic, sizeof(header.magic));
        unpack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        unpack(buffer, offset, &header.ftype, sizeof(header.ftype));
        unpack(buffer, offset, &header.dtype, sizeof(header.dtype));
        unpack(buffer, offset, &header.length, sizeof(header.length));
        unpack(buffer, offset, &header.device_id, sizeof(header.device_id));
        unpack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        unpack(buffer, offset, &body.target_id, sizeof(body.target_id));
        unpack(buffer, offset, &body.fault_status, sizeof(body.fault_status));
        unpack(buffer, offset, &body.track_status, sizeof(body.track_status));
        unpack(buffer, offset, &body.track_count, sizeof(body.track_count));
        unpack(buffer, offset, &body.camfield, sizeof(body.camfield));
        unpack(buffer, offset, &body.ptz_zeroazi, sizeof(body.ptz_zeroazi));
        unpack(buffer, offset, &body.ptz_roll, sizeof(body.ptz_roll));
        unpack(buffer, offset, &body.ptz_pitch, sizeof(body.ptz_pitch));
        unpack(buffer, offset, &body.ptz_yaw, sizeof(body.ptz_yaw));
        unpack(buffer, offset, &body.ptz_lng_e7, sizeof(body.ptz_lng_e7));
        unpack(buffer, offset, &body.ptz_lat_e7, sizeof(body.ptz_lat_e7));
        unpack(buffer, offset, &body.ptz_alt, sizeof(body.ptz_alt));
        unpack(buffer, offset, &body.target_lng_e7, sizeof(body.target_lng_e7));
        unpack(buffer, offset, &body.target_lat_e7, sizeof(body.target_lat_e7));
        unpack(buffer, offset, &body.target_alt, sizeof(body.target_alt));
        unpack(buffer, offset, &body.target_dis, sizeof(body.target_dis));
        unpack(buffer, offset, &body.target_azimuth, sizeof(body.target_azimuth));
        unpack(buffer, offset, &body.target_elevation, sizeof(body.target_elevation));
        unpack(buffer, offset, &body.target_type, sizeof(body.target_type));
        unpack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));
        unpack(buffer, offset, body.reserved.data(), body.reserved.size());

        unpack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return checkMagic() && checkLength();
    }
    
    /**
     * @brief 检查帧同步字是否正确
     * @return true - 同步字正确，false - 同步字错误
     */
    bool checkMagic() const {
        return header.getMagic() == static_cast<uint32_t>(MagicNumber::CAMERA_MAGIC);
    }
    
    /**
     * @brief 检查帧长度是否正确
     * @return true - 长度正确，false - 长度错误
     */
    bool checkLength() const {
        return header.getLength() == static_cast<uint16_t>(size());
    }
    
    /**
     * @brief 获取帧数据指针
     * @return 指向帧数据的指针
     */
    const uint8_t* data() const {
        return reinterpret_cast<const uint8_t*>(this);
    }
    
    /**
     * @brief 获取帧数据大小
     * @return 帧数据大小（字节）
     */
    static constexpr size_t size() {
        return DevFrameHeader::wireSize() + CamTargetInfoFrameBody::wireSize() + DevFrameTail::wireSize();
    }
    
    // ========================================================================
    // 上层类接口，直接调用结构体的接口
    // ========================================================================
    
    // 帧头访问接口
    DevFrameHeader& getHeader() { return header; }
    void setHeader(const DevFrameHeader& newHeader) { header = newHeader; }
    
    // 帧体访问接口
    CamTargetInfoFrameBody& getBody() { return body; }
    void setBody(const CamTargetInfoFrameBody& newBody) { body = newBody; }
    
    // 帧尾访问接口
    DevFrameTail& getTail() { return tail; }
    void setTail(const DevFrameTail& newTail) { tail = newTail; }
    
    // 帧头字段访问接口（直接调用结构体的接口）
    uint32_t getMagic() const { return header.getMagic(); }
    void setMagic(uint32_t magic) { header.setMagic(magic); }
    
    uint8_t getProtoVer() const { return header.getProtoVer(); }
    void setProtoVer(uint8_t proto_ver) { header.setProtoVer(proto_ver); }
    
    uint16_t getFrameType() const { return header.getFrameType(); }
    void setFrameType(uint16_t ftype) { header.setFrameType(ftype); }
    
    uint16_t getDataType() const { return header.getDataType(); }
    void setDataType(uint16_t dtype) { header.setDataType(dtype); }
    
    uint16_t getLength() const { return header.getLength(); }
    void setLength(uint16_t length) { header.setLength(length); }
    
    uint32_t getDeviceId() const { return header.getDeviceId(); }
    void setDeviceId(uint32_t device_id) { header.setDeviceId(device_id); }
    
    uint64_t getTimestamp() const { return header.getTimestamp(); }
    void setTimestamp(uint64_t timestamp) { header.setTimestamp(timestamp); }
    
    // 帧体字段访问接口（直接调用结构体的接口）
    uint32_t getTargetId() const { return body.getTargetId(); }
    void setTargetId(uint32_t target_id) { body.setTargetId(target_id); }
    
    uint8_t getFaultStatus() const { return body.getFaultStatus(); }
    void setFaultStatus(uint8_t fault_status) { body.setFaultStatus(fault_status); }
    
    uint8_t getTrackStatus() const { return body.getTrackStatus(); }
    void setTrackStatus(uint8_t track_status) { body.setTrackStatus(track_status); }
    
    uint8_t getTrackCount() const { return body.getTrackCount(); }
    void setTrackCount(uint8_t track_count) { body.setTrackCount(track_count); }
    
    uint8_t getCamfield() const { return body.getCamfield(); }
    void setCamfield(uint8_t camfield) { body.setCamfield(camfield); }
    
    uint8_t getTargetType() const { return body.getTargetType(); }
    void setTargetType(uint8_t target_type) { body.setTargetType(target_type); }
    
    uint8_t getTargetConfidence() const { return body.getTargetConfidence(); }
    void setTargetConfidence(uint8_t target_confidence) { body.setTargetConfidence(target_confidence); }
    
    // 逻辑相关的set接口（直接调用结构体的接口）
    void setPtzPosition(int32_t ptz_lng_e7, int32_t ptz_lat_e7, uint32_t ptz_alt) { body.setPtzPosition(ptz_lng_e7, ptz_lat_e7, ptz_alt); }
    void setTargetPosition(int32_t target_lng_e7, int32_t target_lat_e7, uint32_t target_alt) { body.setTargetPosition(target_lng_e7, target_lat_e7, target_alt); }
    void setPtzAttitude(uint16_t ptz_roll, int16_t ptz_pitch, uint16_t ptz_yaw) { body.setPtzAttitude(ptz_roll, ptz_pitch, ptz_yaw); }
    void setTargetAngles(uint16_t target_azimuth, int16_t target_elevation) { body.setTargetAngles(target_azimuth, target_elevation); }
    void setTargetDistanceInfo(uint32_t target_dis, uint32_t target_alt) { body.setTargetDistanceInfo(target_dis, target_alt); }
    void setDeviceStatus(uint8_t fault_status, uint8_t track_status, uint8_t track_count) { body.setDeviceStatus(fault_status, track_status, track_count); }
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) { body.setTargetInfo(target_type, target_confidence); }
    
    // 帧尾字段访问接口（直接调用结构体的接口）
    uint32_t getCrc32() const { return tail.getCrc32(); }
    void setCrc32(uint32_t crc32) { tail.setCrc32(crc32); }
};

/**
 * @brief 声探测设备标准化协议帧类
 * 
 * 包含帧头、帧体、帧尾三部分，提供序列化和反序列化接口。
 * 总长度：67字节
 */
class AcousticTargetFrame {
private:
    DevFrameHeader header;     ///< 帧头部分
    AcousticTargetInfoFrameBody body;  ///< 帧体部分
    DevFrameTail tail;  ///< 帧尾部分
    
public:
    /**
     * @brief 默认构造函数
     */
    AcousticTargetFrame() {
        reset();
    }
    
    /**
     * @brief 重置帧数据为默认值
     */
    void reset() {
        // 设置帧头默认值
        header.setMagic(static_cast<uint32_t>(MagicNumber::ACOUSTIC_MAGIC));  // 声探测设备固定同步字
        header.setProtoVer(1);
        header.setFrameType(static_cast<uint16_t>(FrameType::ACOUSTIC_TARGET));
        header.setDataType(static_cast<uint16_t>(DataType::SINGLE_TARGET));
        header.setLength(static_cast<uint16_t>(size()));
        header.setDeviceId(0);
        header.setTimestamp(0);
        
        // 帧体已由构造函数初始化
        
        // 帧尾
        tail.setCrc32(0);
    }
    
    /**
     * @brief 序列化接口：将帧数据转换为字节数组
     * @param buffer 输出缓冲区，必须至少有size()字节的空间
     * @return true - 序列化成功，false - 失败
     */
    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        pack(buffer, offset, &header.magic, sizeof(header.magic));
        pack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        pack(buffer, offset, &header.ftype, sizeof(header.ftype));
        pack(buffer, offset, &header.dtype, sizeof(header.dtype));
        pack(buffer, offset, &header.length, sizeof(header.length));
        pack(buffer, offset, &header.device_id, sizeof(header.device_id));
        pack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        pack(buffer, offset, &body.target_id, sizeof(body.target_id));
        pack(buffer, offset, &body.point_count, sizeof(body.point_count));
        pack(buffer, offset, body.reserved0.data(), body.reserved0.size());
        pack(buffer, offset, &body.device_roll, sizeof(body.device_roll));
        pack(buffer, offset, &body.device_pitch, sizeof(body.device_pitch));
        pack(buffer, offset, &body.device_yaw, sizeof(body.device_yaw));
        pack(buffer, offset, body.reserved1.data(), body.reserved1.size());
        pack(buffer, offset, &body.target_roll, sizeof(body.target_roll));
        pack(buffer, offset, &body.target_pitch, sizeof(body.target_pitch));
        pack(buffer, offset, &body.target_yaw, sizeof(body.target_yaw));
        pack(buffer, offset, &body.target_type, sizeof(body.target_type));
        pack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));
        pack(buffer, offset, body.reserved.data(), body.reserved.size());
        pack(buffer, offset, &body.device_lng_e7, sizeof(body.device_lng_e7));
        pack(buffer, offset, &body.device_lat_e7, sizeof(body.device_lat_e7));
        pack(buffer, offset, &body.device_alt_dm, sizeof(body.device_alt_dm));

        pack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return true;
    }
    
    /**
     * @brief 反序列化接口：从字节数组解析帧数据
     * @param buffer 输入缓冲区，必须至少有size()字节的数据
     * @return true - 反序列化成功，false - 失败
     */
    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < size()) return false;

        size_t offset = 0;
        unpack(buffer, offset, &header.magic, sizeof(header.magic));
        unpack(buffer, offset, &header.proto_ver, sizeof(header.proto_ver));
        unpack(buffer, offset, &header.ftype, sizeof(header.ftype));
        unpack(buffer, offset, &header.dtype, sizeof(header.dtype));
        unpack(buffer, offset, &header.length, sizeof(header.length));
        unpack(buffer, offset, &header.device_id, sizeof(header.device_id));
        unpack(buffer, offset, &header.timestamp, sizeof(header.timestamp));

        unpack(buffer, offset, &body.target_id, sizeof(body.target_id));
        unpack(buffer, offset, &body.point_count, sizeof(body.point_count));
        unpack(buffer, offset, body.reserved0.data(), body.reserved0.size());
        unpack(buffer, offset, &body.device_roll, sizeof(body.device_roll));
        unpack(buffer, offset, &body.device_pitch, sizeof(body.device_pitch));
        unpack(buffer, offset, &body.device_yaw, sizeof(body.device_yaw));
        unpack(buffer, offset, body.reserved1.data(), body.reserved1.size());
        unpack(buffer, offset, &body.target_roll, sizeof(body.target_roll));
        unpack(buffer, offset, &body.target_pitch, sizeof(body.target_pitch));
        unpack(buffer, offset, &body.target_yaw, sizeof(body.target_yaw));
        unpack(buffer, offset, &body.target_type, sizeof(body.target_type));
        unpack(buffer, offset, &body.target_confidence, sizeof(body.target_confidence));
        unpack(buffer, offset, body.reserved.data(), body.reserved.size());
        unpack(buffer, offset, &body.device_lng_e7, sizeof(body.device_lng_e7));
        unpack(buffer, offset, &body.device_lat_e7, sizeof(body.device_lat_e7));
        unpack(buffer, offset, &body.device_alt_dm, sizeof(body.device_alt_dm));

        unpack(buffer, offset, &tail.crc32, sizeof(tail.crc32));

        return checkMagic() && checkLength();
    }
    
    /**
     * @brief 检查帧同步字是否正确
     * @return true - 同步字正确，false - 同步字错误
     */
    bool checkMagic() const {
        return header.getMagic() == static_cast<uint32_t>(MagicNumber::ACOUSTIC_MAGIC);
    }
    
    /**
     * @brief 检查帧长度是否正确
     * @return true - 长度正确，false - 长度错误
     */
    bool checkLength() const {
        return header.getLength() == static_cast<uint16_t>(size());
    }
    
    /**
     * @brief 获取帧数据指针
     * @return 指向帧数据的指针
     */
    const uint8_t* data() const {
        return reinterpret_cast<const uint8_t*>(this);
    }
    
    /**
     * @brief 获取帧数据大小
     * @return 帧数据大小（字节）
     */
    static constexpr size_t size() {
        return DevFrameHeader::wireSize() + AcousticTargetInfoFrameBody::wireSize() + DevFrameTail::wireSize();
    }
    
    // ========================================================================
    // 上层类接口，直接调用结构体的接口
    // ========================================================================
    
    // 帧头访问接口
    DevFrameHeader& getHeader() { return header; }
    void setHeader(const DevFrameHeader& newHeader) { header = newHeader; }
    
    // 帧体访问接口
    AcousticTargetInfoFrameBody& getBody() { return body; }
    void setBody(const AcousticTargetInfoFrameBody& newBody) { body = newBody; }
    
    // 帧尾访问接口
    DevFrameTail& getTail() { return tail; }
    void setTail(const DevFrameTail& newTail) { tail = newTail; }
    
    // 帧头字段访问接口（直接调用结构体的接口）
    uint32_t getMagic() const { return header.getMagic(); }
    void setMagic(uint32_t magic) { header.setMagic(magic); }
    
    uint8_t getProtoVer() const { return header.getProtoVer(); }
    void setProtoVer(uint8_t proto_ver) { header.setProtoVer(proto_ver); }
    
    uint16_t getFrameType() const { return header.getFrameType(); }
    void setFrameType(uint16_t ftype) { header.setFrameType(ftype); }
    
    uint16_t getDataType() const { return header.getDataType(); }
    void setDataType(uint16_t dtype) { header.setDataType(dtype); }
    
    uint16_t getLength() const { return header.getLength(); }
    void setLength(uint16_t length) { header.setLength(length); }
    
    uint32_t getDeviceId() const { return header.getDeviceId(); }
    void setDeviceId(uint32_t device_id) { header.setDeviceId(device_id); }
    
    uint64_t getTimestamp() const { return header.getTimestamp(); }
    void setTimestamp(uint64_t timestamp) { header.setTimestamp(timestamp); }
    
    // 帧体字段访问接口（直接调用结构体的接口）
    uint32_t getTargetId() const { return body.getTargetId(); }
    void setTargetId(uint32_t target_id) { body.setTargetId(target_id); }
    
    uint8_t getPointCount() const { return body.getPointCount(); }
    void setPointCount(uint8_t point_count) { body.setPointCount(point_count); }
    
    uint8_t getTargetType() const { return body.getTargetType(); }
    void setTargetType(uint8_t target_type) { body.setTargetType(target_type); }
    
    uint8_t getTargetConfidence() const { return body.getTargetConfidence(); }
    void setTargetConfidence(uint8_t target_confidence) { body.setTargetConfidence(target_confidence); }
    
    // 逻辑相关的set接口（直接调用结构体的接口）
    void setDeviceAttitude(int16_t device_roll, int16_t device_pitch, int16_t device_yaw) { body.setDeviceAttitude(device_roll, device_pitch, device_yaw); }
    void setTargetRelativeAngles(uint16_t target_roll, uint16_t target_pitch, uint16_t target_yaw) { body.setTargetRelativeAngles(target_roll, target_pitch, target_yaw); }
    void setTargetInfo(uint8_t target_type, uint8_t target_confidence) { body.setTargetInfo(target_type, target_confidence); }
    void setTargetBasicInfo(uint32_t target_id, uint8_t point_count) { body.setTargetBasicInfo(target_id, point_count); }
    void setDevicePosition(int32_t lng_e7, int32_t lat_e7, uint32_t alt_dm) { body.setDevicePosition(lng_e7, lat_e7, alt_dm); }
    
    // 帧尾字段访问接口（直接调用结构体的接口）
    uint32_t getCrc32() const { return tail.getCrc32(); }
    void setCrc32(uint32_t crc32) { tail.setCrc32(crc32); }
};

} // namespace AntiDroneDevProtocol

#endif // ANTI_DRONE_DEVPROTOCOL_NEW_H
