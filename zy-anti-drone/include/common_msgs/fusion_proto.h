/**
 * @file fusion_proto.h
 * @brief 反蜂群装备融合通信协议C++头文件
 * @version 2.0.0
 * @date 2026-07-13
 *
 * 本文件定义了反无人机装备融合层通信协议的消息体结构。
 * 报文头（MsgHeader）和信息头（UnitHeader）由 PXSBProto 提供（见 proto.h），
 * 本文件只定义各消息类型的 body 部分序列化/反序列化。
 *
 * 协议特点：
 * 1. 默认按主机字节序直接传递（DDS同端场景无需额外字节序处理）
 * 2. 可变长消息结构（std::vector），支持0-255个目标和预测点
 * 3. 两层头部由 PubSub 框架管理（MsgHeader + UnitHeader）
 * 4. 统一的量化单位和无效值约定
 */

#ifndef ANTI_DRONE_FUSION_PROTOCOL_H
#define ANTI_DRONE_FUSION_PROTOCOL_H

#include <cstdint>
#include <cstring>
#include <vector>
#include <algorithm>

namespace AntiDroneFusionProtocol {

// ============================================================================
// 通用常量定义
// ============================================================================

/// 无效值定义
constexpr uint32_t INVALID_UINT32 = 0xFFFFFFFF;
constexpr uint16_t INVALID_UINT16 = 0xFFFF;
constexpr int16_t INVALID_INT16 = 0x7FFF;
constexpr uint8_t INVALID_UINT8 = 0xFF;

// ============================================================================
// 枚举定义
// ============================================================================

/**
 * @brief 目标类别定义
 */
enum class TargetType : uint8_t {
    DRONE = 0,        ///< 无人机
    BIRD = 1,         ///< 鸟
    LARGE_TARGET = 2, ///< 大型目标
    VEHICLE = 3,      ///< 车辆
    PERSON = 4,       ///< 人员
    OTHER = 5,        ///< 其它
    INVALID = 0xFF    ///< 无效值
};

/**
 * @brief 融合源掩码定义
 *
 * 标识目标数据来源的组合，支持单源和多源融合
 */
enum class SourceMask : uint8_t {
    RADAR = 0,              ///< 雷达
    EO = 1,                 ///< 光电
    ES = 2,                 ///< 电侦
    ACOUSTIC = 3,           ///< 声探
    RADAR_EO = 4,           ///< 雷达+光电
    RADAR_ES = 5,           ///< 雷达+电侦
    RADAR_ACOUSTIC = 6,     ///< 雷达+声探
    EO_ES = 7,              ///< 光电+电侦
    EO_ACOUSTIC = 8,        ///< 光电+声探
    ES_ACOUSTIC = 9,        ///< 电侦+声探
    RADAR_EO_ES = 10,       ///< 雷达+光电+电侦
    RADAR_EO_ACOUSTIC = 11, ///< 雷达+光电+声探
    EO_ES_ACOUSTIC = 12,    ///< 光电+电侦+声探
    ALL = 13,               ///< 雷达+光电+电侦+声探
    INVALID = 0xFF          ///< 无效值
};

/**
 * @brief 危险等级定义
 */
enum class DangerLevel : uint8_t {
    VERY_LOW = 0,  ///< 超低
    LOW = 1,       ///< 低
    MEDIUM = 2,    ///< 中
    HIGH = 3,      ///< 高
    VERY_HIGH = 4, ///< 超高
    INVALID = 0xFF ///< 无效值
};

/**
 * @brief 意图定义
 */
enum class Intention : uint8_t {
    SUICIDE_ATTACK = 0,     ///< 自杀突击
    JAMMING_FEINT = 1,      ///< 干扰佯动
    MISSION_WITHDRAWAL = 2, ///< 任务撤离
    UNKNOWN = 3,            ///< 未知意图
    INVALID = 0xFF          ///< 无效值
};

/**
 * @brief 反制结果定义
 */
enum class ResultId : uint8_t {
    LURED_AWAY = 0,      ///< 引离
    FORCED_LANDING = 1,  ///< 原地迫降
    HOVER = 2,           ///< 悬停
    CAPTURED = 3,        ///< 捕获
    CRASHED = 4,         ///< 坠毁
    INVALID = 0xFF       ///< 无效值
};

/**
 * @brief 武器ID定义
 */
enum class WeaponId : uint8_t {
    EW_JAMMING = 0,            ///< 电磁干扰
    NET_CAPTURE = 1,           ///< 网捕
    IMPACT = 2,                ///< 撞击
    STACKED = 3,               ///< 堆栈
    LASER_BLINDING = 4,        ///< 激光致盲
    NAVIGATION_DECEPTION = 5,  ///< 导航诱骗
    INVALID = 0xFF             ///< 无效值
};

/**
 * @brief 武器设备状态定义
 */
enum class WeaponStatus : uint8_t {
    READY = 0,         ///< 待命 — 空闲可用
    WORKING = 1,       ///< 工作中 — 正在执行反制任务
    DISCONNECTED = 2,  ///< 断联 — 通信中断或离线
    FAULT = 3,         ///< 故障
    INVALID = 0xFF     ///< 无效值
};

/**
 * @brief 消息类型定义（对应 UnitHeader.unitTag）
 */
enum class MessageType : uint16_t {
    FUSION_MULTI_TARGET = 0x01,   ///< 融合多目标数据
    SITUATION = 0x02,              ///< 态势推理数据
    WEAPON_DISTRIBUTION = 0x03,   ///< 决策反制-武器分配
    STRIKE_RESULT = 0x04,         ///< 决策反制-反制结果
    WEAPON_STATUS = 0x05,         ///< 武器设备状态上报
    INVALID = 0xFFFF              ///< 无效值
};

// ============================================================================
// 序列化/反序列化辅助函数
// ============================================================================

inline void pack(uint8_t* buffer, size_t& offset, const void* src, size_t len) {
    std::memcpy(buffer + offset, src, len);
    offset += len;
}

inline void unpack(const uint8_t* buffer, size_t& offset, void* dst, size_t len) {
    std::memcpy(dst, buffer + offset, len);
    offset += len;
}

// ============================================================================
// 第一步：定义通用数据结构体
// ============================================================================

/**
 * @brief 绝对位置信息
 *
 * WGS84坐标系下的经纬度和海拔高度。
 * 总长度：12字节
 */
struct AbsPos {
    int32_t lng_e7;    ///< 经度，单位：1e-7°，东经为正，西经为负
    int32_t lat_e7;    ///< 纬度，单位：1e-7°，北纬为正，南纬为负
    uint32_t alt;      ///< 高度，单位：0.1m

    AbsPos() : lng_e7(0), lat_e7(0), alt(0) {}

    static constexpr size_t wireSize() {
        return sizeof(int32_t) + sizeof(int32_t) + sizeof(uint32_t);
    }

    int32_t getLng() const { return lng_e7; }
    void setLng(int32_t lng) { this->lng_e7 = lng; }

    int32_t getLat() const { return lat_e7; }
    void setLat(int32_t lat) { this->lat_e7 = lat; }

    uint32_t getAlt() const { return alt; }
    void setAlt(uint32_t alt) { this->alt = alt; }

    void setPosition(int32_t lng, int32_t lat, uint32_t alt) {
        this->lng_e7 = lng;
        this->lat_e7 = lat;
        this->alt = alt;
    }
};

/**
 * @brief 相对位置信息
 *
 * 相对于观测点的球坐标位置。
 * 总长度：8字节
 */
struct RevPos {
    uint16_t azimuth;  ///< 方位角，单位：0.01°，正北为0，向东增加
    int16_t pitch;     ///< 俯仰角，单位：0.01°，水平为0，向上为正
    uint32_t distance; ///< 距离，单位：0.1m

    RevPos() : azimuth(0), pitch(0), distance(0) {}

    static constexpr size_t wireSize() {
        return sizeof(uint16_t) + sizeof(int16_t) + sizeof(uint32_t);
    }

    uint16_t getAzimuth() const { return azimuth; }
    void setAzimuth(uint16_t azimuth) { this->azimuth = azimuth; }

    int16_t getPitch() const { return pitch; }
    void setPitch(int16_t pitch) { this->pitch = pitch; }

    uint32_t getDistance() const { return distance; }
    void setDistance(uint32_t distance) { this->distance = distance; }

    void setPosition(uint16_t azimuth, int16_t pitch, uint32_t distance) {
        this->azimuth = azimuth;
        this->pitch = pitch;
        this->distance = distance;
    }
};

/**
 * @brief 姿态信息
 *
 * 目标或设备的姿态角。
 * 总长度：6字节
 */
struct Attitude {
    int16_t pitch;  ///< 俯仰角，单位：0.01°，水平为0，向上为正
    int16_t roll;   ///< 横滚角，单位：0.01°，右滚为正
    uint16_t yaw;   ///< 航向角，单位：0.01°，正北为0，顺时针增加

    Attitude() : pitch(0), roll(0), yaw(0) {}

    static constexpr size_t wireSize() {
        return sizeof(int16_t) + sizeof(int16_t) + sizeof(uint16_t);
    }

    int16_t getPitch() const { return pitch; }
    void setPitch(int16_t pitch) { this->pitch = pitch; }

    int16_t getRoll() const { return roll; }
    void setRoll(int16_t roll) { this->roll = roll; }

    uint16_t getYaw() const { return yaw; }
    void setYaw(uint16_t yaw) { this->yaw = yaw; }

    void setAttitude(int16_t pitch, int16_t roll, uint16_t yaw) {
        this->pitch = pitch;
        this->roll = roll;
        this->yaw = yaw;
    }
};

/**
 * @brief 单组武器分配（一种武器类型 + 若干设备实例）
 *
 * 表达"用电磁干扰的 #1、#3 号设备反制目标"。
 * 固定部分 2 字节 + 可变部分 instance_count × 1 字节。
 */
struct WeaponAssignment {
    uint8_t weapon_type;                 ///< 武器类别（参见 WeaponId 枚举）
    uint8_t instance_count;              ///< 分配的实例数量
    std::vector<uint8_t> instance_ids;   ///< 设备实例 ID 列表

    WeaponAssignment()
        : weapon_type(static_cast<uint8_t>(WeaponId::INVALID))
        , instance_count(0) {}

    static constexpr size_t MIN_WIRE_SIZE = 2;  // weapon_type + instance_count

    size_t wireSize() const {
        return MIN_WIRE_SIZE + instance_count * sizeof(uint8_t);
    }

    // --- get/set 接口 ---

    uint8_t getWeaponType() const { return weapon_type; }
    void setWeaponType(uint8_t type) { this->weapon_type = type; }

    uint8_t getInstanceCount() const { return instance_count; }

    const std::vector<uint8_t>& getInstanceIds() const { return instance_ids; }

    void addInstanceId(uint8_t id) {
        instance_ids.push_back(id);
        instance_count = static_cast<uint8_t>(instance_ids.size());
    }

    void setInstanceIds(const std::vector<uint8_t>& ids) {
        instance_ids = ids;
        instance_count = static_cast<uint8_t>(ids.size());
    }

    void clearInstanceIds() {
        instance_ids.clear();
        instance_count = 0;
    }

    void setAssignment(uint8_t weapon_type, const std::vector<uint8_t>& instance_ids) {
        this->weapon_type = weapon_type;
        setInstanceIds(instance_ids);
    }

    // --- 序列化辅助 ---

    static bool serializeTo(const WeaponAssignment& wa, uint8_t* buffer, size_t& offset) {
        pack(buffer, offset, &wa.weapon_type,   sizeof(wa.weapon_type));
        pack(buffer, offset, &wa.instance_count, sizeof(wa.instance_count));
        for (uint8_t id : wa.instance_ids)
            pack(buffer, offset, &id, sizeof(id));
        return true;
    }

    static bool deserializeFrom(const uint8_t* buffer, size_t buffer_len,
                                size_t& offset, WeaponAssignment& wa) {
        if (buffer_len < offset + MIN_WIRE_SIZE) return false;

        unpack(buffer, offset, &wa.weapon_type,   sizeof(wa.weapon_type));
        unpack(buffer, offset, &wa.instance_count, sizeof(wa.instance_count));

        wa.instance_ids.clear();
        wa.instance_ids.reserve(wa.instance_count);
        for (uint8_t i = 0; i < wa.instance_count; ++i) {
            if (buffer_len < offset + sizeof(uint8_t)) return false;
            uint8_t id;
            unpack(buffer, offset, &id, sizeof(id));
            wa.instance_ids.push_back(id);
        }
        return true;
    }
};

/**
 * @brief 单个武器设备实例
 *
 * 每类武器可部署多台设备，每台有独立的状态和位置。
 * 总长度：15 字节
 */
struct WeaponInstance {
    uint8_t  instance_id;   ///< 设备实例编号（同类型下唯一，1~N）
    uint8_t  weapon_type;   ///< 武器类别（参见 WeaponId 枚举）
    uint8_t  status;        ///< 设备状态（参见 WeaponStatus 枚举）
    AbsPos   position;      ///< 设备部署位置（12 字节）

    WeaponInstance()
        : instance_id(0)
        , weapon_type(static_cast<uint8_t>(WeaponId::INVALID))
        , status(static_cast<uint8_t>(WeaponStatus::INVALID))
        , position() {}

    static constexpr size_t wireSize() {
        return sizeof(uint8_t) * 3 + AbsPos::wireSize();  // 15
    }

    // --- get/set 接口 ---

    uint8_t getInstanceId() const { return instance_id; }
    void setInstanceId(uint8_t id) { this->instance_id = id; }

    uint8_t getWeaponType() const { return weapon_type; }
    void setWeaponType(uint8_t type) { this->weapon_type = type; }

    uint8_t getStatus() const { return status; }
    void setStatus(uint8_t s) { this->status = s; }

    const AbsPos& getPosition() const { return position; }
    void setPosition(const AbsPos& pos) { this->position = pos; }

    void setInstance(uint8_t id, uint8_t type, uint8_t status, const AbsPos& pos) {
        this->instance_id = id;
        this->weapon_type = type;
        this->status = status;
        this->position = pos;
    }
};

/**
 * @brief 预测轨迹点
 *
 * 单个未来轨迹点的预测信息，包含位置、姿态、速度和时间。
 * 总长度：30字节
 */
struct PredictionPoint {
    AbsPos abs_pos;      ///< 绝对位置
    Attitude attitude;   ///< 姿态
    uint32_t speed;      ///< 移动速度，单位：0.01m/s
    uint64_t time;       ///< 轨迹点时间戳，单位：ms

    PredictionPoint() : abs_pos(), attitude(), speed(0), time(0) {}

    static constexpr size_t wireSize() {
        return AbsPos::wireSize() + Attitude::wireSize() + sizeof(uint32_t) + sizeof(uint64_t);
    }

    uint32_t getSpeed() const { return speed; }
    void setSpeed(uint32_t speed) { this->speed = speed; }

    uint64_t getTime() const { return time; }
    void setTime(uint64_t time) { this->time = time; }

    const AbsPos& getAbsPos() const { return abs_pos; }
    void setAbsPos(const AbsPos& pos) { this->abs_pos = pos; }

    const Attitude& getAttitude() const { return attitude; }
    void setAttitude(const Attitude& att) { this->attitude = att; }

    void setPrediction(const AbsPos& pos, const Attitude& att, uint32_t speed, uint64_t time) {
        this->abs_pos = pos;
        this->attitude = att;
        this->speed = speed;
        this->time = time;
    }
};

// ============================================================================
// 第二步：定义消息体结构体
// ============================================================================

/**
 * @brief 单个融合目标数据
 *
 * 固定部分28字节 + 可变预测轨迹点（pred_point_count * 30字节）。
 */
struct FusionTarget {
    uint8_t global_id;                   ///< 全局唯一目标ID
    AbsPos abs_pos;                      ///< 绝对位置
    Attitude attitude;                   ///< 姿态
    uint32_t speed;                      ///< 移动速度，单位：0.01m/s
    uint8_t fusion_type;                 ///< 融合类别（参见TargetType枚举）
    uint8_t confidence;                  ///< 置信度（0~100）
    uint8_t source_mask;                 ///< 贡献源（参见SourceMask枚举）
    uint8_t iff;                         ///< 敌我属性：0=不明，1=我方，2=敌方
    uint8_t pred_point_count;            ///< 预测轨迹点个数（0-255）
    std::vector<PredictionPoint> prediction_points; ///< 预测轨迹点列表

    FusionTarget()
        : global_id(0)
        , abs_pos()
        , attitude()
        , speed(0)
        , fusion_type(static_cast<uint8_t>(TargetType::DRONE))
        , confidence(0)
        , source_mask(static_cast<uint8_t>(SourceMask::INVALID))
        , iff(0)
        , pred_point_count(0) {}

    /// 固定部分最小长度（不含预测轨迹点）
    static constexpr size_t MIN_WIRE_SIZE = 28;

    size_t wireSize() const {
        return MIN_WIRE_SIZE + pred_point_count * PredictionPoint::wireSize();
    }

    // ========================================================================
    // get/set 接口
    // ========================================================================

    uint8_t getGlobalId() const { return global_id; }
    void setGlobalId(uint8_t id) { this->global_id = id; }

    uint32_t getSpeed() const { return speed; }
    void setSpeed(uint32_t speed) { this->speed = speed; }

    uint8_t getFusionType() const { return fusion_type; }
    void setFusionType(uint8_t type) { this->fusion_type = type; }

    uint8_t getConfidence() const { return confidence; }
    void setConfidence(uint8_t confidence) { this->confidence = confidence; }

    uint8_t getSourceMask() const { return source_mask; }
    void setSourceMask(uint8_t mask) { this->source_mask = mask; }

    uint8_t getIff() const { return iff; }
    void setIff(uint8_t iff) { this->iff = iff; }

    uint8_t getPredPointCount() const { return pred_point_count; }

    const AbsPos& getAbsPos() const { return abs_pos; }
    void setAbsPos(const AbsPos& pos) { this->abs_pos = pos; }

    const Attitude& getAttitude() const { return attitude; }
    void setAttitude(const Attitude& att) { this->attitude = att; }

    const std::vector<PredictionPoint>& getPredictionPoints() const { return prediction_points; }

    // ========================================================================
    // 逻辑相关的set接口
    // ========================================================================

    void setPosition(const AbsPos& pos) { this->abs_pos = pos; }
    void setTargetAttitude(const Attitude& att) { this->attitude = att; }
    void setMotion(uint32_t speed) { this->speed = speed; }

    void setTargetIdentity(uint8_t fusion_type, uint8_t confidence, uint8_t iff) {
        this->fusion_type = fusion_type;
        this->confidence = confidence;
        this->iff = iff;
    }

    void setTargetBasicInfo(uint8_t global_id, uint8_t source_mask) {
        this->global_id = global_id;
        this->source_mask = source_mask;
    }

    void addPredictionPoint(const PredictionPoint& point) {
        prediction_points.push_back(point);
        pred_point_count = static_cast<uint8_t>(prediction_points.size());
    }

    void clearPredictionPoints() {
        prediction_points.clear();
        pred_point_count = 0;
    }

    void setPredictionPoints(const std::vector<PredictionPoint>& points) {
        prediction_points = points;
        pred_point_count = static_cast<uint8_t>(points.size());
    }

    // ========================================================================
    // 序列化/反序列化辅助（供消息类使用）
    // ========================================================================

    static bool serializeTo(const FusionTarget& target, uint8_t* buffer, size_t& offset) {
        pack(buffer, offset, &target.global_id, sizeof(target.global_id));
        pack(buffer, offset, &target.abs_pos.lng_e7, sizeof(target.abs_pos.lng_e7));
        pack(buffer, offset, &target.abs_pos.lat_e7, sizeof(target.abs_pos.lat_e7));
        pack(buffer, offset, &target.abs_pos.alt, sizeof(target.abs_pos.alt));
        pack(buffer, offset, &target.attitude.pitch, sizeof(target.attitude.pitch));
        pack(buffer, offset, &target.attitude.roll, sizeof(target.attitude.roll));
        pack(buffer, offset, &target.attitude.yaw, sizeof(target.attitude.yaw));
        pack(buffer, offset, &target.speed, sizeof(target.speed));
        pack(buffer, offset, &target.fusion_type, sizeof(target.fusion_type));
        pack(buffer, offset, &target.confidence, sizeof(target.confidence));
        pack(buffer, offset, &target.source_mask, sizeof(target.source_mask));
        pack(buffer, offset, &target.iff, sizeof(target.iff));
        pack(buffer, offset, &target.pred_point_count, sizeof(target.pred_point_count));

        for (const auto& pp : target.prediction_points) {
            pack(buffer, offset, &pp.abs_pos.lng_e7, sizeof(pp.abs_pos.lng_e7));
            pack(buffer, offset, &pp.abs_pos.lat_e7, sizeof(pp.abs_pos.lat_e7));
            pack(buffer, offset, &pp.abs_pos.alt, sizeof(pp.abs_pos.alt));
            pack(buffer, offset, &pp.attitude.pitch, sizeof(pp.attitude.pitch));
            pack(buffer, offset, &pp.attitude.roll, sizeof(pp.attitude.roll));
            pack(buffer, offset, &pp.attitude.yaw, sizeof(pp.attitude.yaw));
            pack(buffer, offset, &pp.speed, sizeof(pp.speed));
            pack(buffer, offset, &pp.time, sizeof(pp.time));
        }
        return true;
    }

    static bool deserializeFrom(const uint8_t* buffer, size_t buffer_len, size_t& offset, FusionTarget& target) {
        if (buffer_len < offset + MIN_WIRE_SIZE) return false;

        unpack(buffer, offset, &target.global_id, sizeof(target.global_id));
        unpack(buffer, offset, &target.abs_pos.lng_e7, sizeof(target.abs_pos.lng_e7));
        unpack(buffer, offset, &target.abs_pos.lat_e7, sizeof(target.abs_pos.lat_e7));
        unpack(buffer, offset, &target.abs_pos.alt, sizeof(target.abs_pos.alt));
        unpack(buffer, offset, &target.attitude.pitch, sizeof(target.attitude.pitch));
        unpack(buffer, offset, &target.attitude.roll, sizeof(target.attitude.roll));
        unpack(buffer, offset, &target.attitude.yaw, sizeof(target.attitude.yaw));
        unpack(buffer, offset, &target.speed, sizeof(target.speed));
        unpack(buffer, offset, &target.fusion_type, sizeof(target.fusion_type));
        unpack(buffer, offset, &target.confidence, sizeof(target.confidence));
        unpack(buffer, offset, &target.source_mask, sizeof(target.source_mask));
        unpack(buffer, offset, &target.iff, sizeof(target.iff));
        unpack(buffer, offset, &target.pred_point_count, sizeof(target.pred_point_count));

        target.prediction_points.clear();
        target.prediction_points.reserve(target.pred_point_count);
        for (uint8_t i = 0; i < target.pred_point_count; ++i) {
            if (buffer_len < offset + PredictionPoint::wireSize()) return false;
            PredictionPoint pp;
            unpack(buffer, offset, &pp.abs_pos.lng_e7, sizeof(pp.abs_pos.lng_e7));
            unpack(buffer, offset, &pp.abs_pos.lat_e7, sizeof(pp.abs_pos.lat_e7));
            unpack(buffer, offset, &pp.abs_pos.alt, sizeof(pp.abs_pos.alt));
            unpack(buffer, offset, &pp.attitude.pitch, sizeof(pp.attitude.pitch));
            unpack(buffer, offset, &pp.attitude.roll, sizeof(pp.attitude.roll));
            unpack(buffer, offset, &pp.attitude.yaw, sizeof(pp.attitude.yaw));
            unpack(buffer, offset, &pp.speed, sizeof(pp.speed));
            unpack(buffer, offset, &pp.time, sizeof(pp.time));
            target.prediction_points.push_back(pp);
        }
        return true;
    }
};

/**
 * @brief 单目标态势信息
 *
 * 包含目标的意图和危险等级。
 * 总长度：3字节
 */
struct SituationMes {
    uint8_t global_id;     ///< 全局唯一目标ID
    uint8_t intention;     ///< 意图（参见Intention枚举）
    uint8_t danger_level;  ///< 危险等级（参见DangerLevel枚举）

    SituationMes()
        : global_id(0)
        , intention(static_cast<uint8_t>(Intention::UNKNOWN))
        , danger_level(static_cast<uint8_t>(DangerLevel::VERY_LOW)) {}

    static constexpr size_t wireSize() {
        return sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);
    }

    uint8_t getGlobalId() const { return global_id; }
    void setGlobalId(uint8_t id) { this->global_id = id; }

    uint8_t getIntention() const { return intention; }
    void setIntention(uint8_t intention) { this->intention = intention; }

    uint8_t getDangerLevel() const { return danger_level; }
    void setDangerLevel(uint8_t level) { this->danger_level = level; }

    void setSituation(uint8_t global_id, uint8_t intention, uint8_t danger_level) {
        this->global_id = global_id;
        this->intention = intention;
        this->danger_level = danger_level;
    }
};

/**
 * @brief 武器-目标匹配信息
 *
 * 一个目标可分配多组武器（不同种类，每类可多台实例）。
 * 固定部分 14 字节 + 可变部分由 WeaponAssignment 累加。
 */
struct MatchMsg {
    uint8_t  target_id;                           ///< 目标全局 ID
    AbsPos   target_pos;                          ///< 目标位置（12 字节）
    uint8_t  assign_count;                        ///< 武器种类数量
    std::vector<WeaponAssignment> assignments;    ///< 按武器种类分组

    MatchMsg()
        : target_id(0)
        , target_pos()
        , assign_count(0) {}

    static constexpr size_t MIN_WIRE_SIZE = 14;   // target_id(1) + AbsPos(12) + assign_count(1)

    size_t wireSize() const {
        size_t sz = MIN_WIRE_SIZE;
        for (const auto& a : assignments) sz += a.wireSize();
        return sz;
    }

    // --- get/set 接口 ---

    uint8_t getTargetId() const { return target_id; }
    void setTargetId(uint8_t id) { this->target_id = id; }

    const AbsPos& getTargetPos() const { return target_pos; }
    void setTargetPos(const AbsPos& pos) { this->target_pos = pos; }

    uint8_t getAssignCount() const { return assign_count; }

    const std::vector<WeaponAssignment>& getAssignments() const { return assignments; }

    void addAssignment(const WeaponAssignment& wa) {
        assignments.push_back(wa);
        assign_count = static_cast<uint8_t>(assignments.size());
    }

    void setAssignments(const std::vector<WeaponAssignment>& vec) {
        assignments = vec;
        assign_count = static_cast<uint8_t>(vec.size());
    }

    void clearAssignments() {
        assignments.clear();
        assign_count = 0;
    }

    void setMatch(uint8_t target_id, const AbsPos& target_pos,
                  const std::vector<WeaponAssignment>& assignments) {
        this->target_id = target_id;
        this->target_pos = target_pos;
        setAssignments(assignments);
    }

    // --- 序列化辅助 ---

    static bool serializeTo(const MatchMsg& m, uint8_t* buffer, size_t& offset) {
        pack(buffer, offset, &m.target_id, sizeof(m.target_id));
        pack(buffer, offset, &m.target_pos.lng_e7, sizeof(m.target_pos.lng_e7));
        pack(buffer, offset, &m.target_pos.lat_e7, sizeof(m.target_pos.lat_e7));
        pack(buffer, offset, &m.target_pos.alt,  sizeof(m.target_pos.alt));
        pack(buffer, offset, &m.assign_count, sizeof(m.assign_count));
        for (const auto& wa : m.assignments)
            WeaponAssignment::serializeTo(wa, buffer, offset);
        return true;
    }

    static bool deserializeFrom(const uint8_t* buffer, size_t buffer_len,
                                size_t& offset, MatchMsg& m) {
        if (buffer_len < offset + MIN_WIRE_SIZE) return false;

        unpack(buffer, offset, &m.target_id, sizeof(m.target_id));
        unpack(buffer, offset, &m.target_pos.lng_e7, sizeof(m.target_pos.lng_e7));
        unpack(buffer, offset, &m.target_pos.lat_e7, sizeof(m.target_pos.lat_e7));
        unpack(buffer, offset, &m.target_pos.alt,  sizeof(m.target_pos.alt));
        unpack(buffer, offset, &m.assign_count, sizeof(m.assign_count));

        m.assignments.clear();
        m.assignments.reserve(m.assign_count);
        for (uint8_t i = 0; i < m.assign_count; ++i) {
            WeaponAssignment wa;
            if (!WeaponAssignment::deserializeFrom(buffer, buffer_len, offset, wa))
                return false;
            m.assignments.push_back(wa);
        }
        return true;
    }
};

/**
 * @brief 反制结果信息
 *
 * 描述单个目标的反制结果。
 * 总长度：2字节
 */
struct ResultMsg {
    uint8_t target_id;  ///< 目标全局ID
    uint8_t result_id;  ///< 反制结果（参见ResultId枚举）

    ResultMsg()
        : target_id(0)
        , result_id(static_cast<uint8_t>(ResultId::INVALID)) {}

    static constexpr size_t wireSize() {
        return sizeof(uint8_t) + sizeof(uint8_t);
    }

    uint8_t getTargetId() const { return target_id; }
    void setTargetId(uint8_t id) { this->target_id = id; }

    uint8_t getResultId() const { return result_id; }
    void setResultId(uint8_t id) { this->result_id = id; }

    void setResult(uint8_t target_id, uint8_t result_id) {
        this->target_id = target_id;
        this->result_id = result_id;
    }
};

// ============================================================================
// 第三步：定义消息类（仅 body 部分，不含报文头/信息头）
//
// 报文头（MsgHeader）和信息头（UnitHeader）由 PubSub 框架管理（见 proto.h）。
// 消息类只负责 body 的序列化/反序列化。
//
// 完整报文格式（由桥接层组装）:
//   MsgHeader (26B) + UnitHeader (6B + 8B sourceTime) + body (NB)
// ============================================================================

/**
 * @brief 融合多目标数据消息体
 *
 * 格式：target_count(uint8) + n * FusionTarget
 */
class FusionMultiTargetMsg {
private:
    std::vector<FusionTarget> targets_;

public:
    FusionMultiTargetMsg() { targets_.clear(); }

    void reset() { targets_.clear(); }

    /**
     * @brief 将 body 序列化到缓冲区
     */
    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        size_t total = size();
        if (!buffer || buffer_len < total) return false;

        size_t offset = 0;

        uint8_t target_count = static_cast<uint8_t>(targets_.size());
        pack(buffer, offset, &target_count, sizeof(target_count));

        for (const auto& target : targets_) {
            FusionTarget::serializeTo(target, buffer, offset);
        }
        return true;
    }

    /**
     * @brief 从缓冲区反序列化 body
     */
    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < sizeof(uint8_t)) return false;

        size_t offset = 0;

        uint8_t target_count = 0;
        unpack(buffer, offset, &target_count, sizeof(target_count));

        targets_.clear();
        targets_.reserve(target_count);
        for (uint8_t i = 0; i < target_count; ++i) {
            FusionTarget target;
            if (!FusionTarget::deserializeFrom(buffer, buffer_len, offset, target)) {
                return false;
            }
            targets_.push_back(target);
        }
        return true;
    }

    size_t size() const {
        size_t sz = sizeof(uint8_t); // target_count
        for (const auto& t : targets_) sz += t.wireSize();
        return sz;
    }

    // ========================================================================
    // 访问接口
    // ========================================================================

    const std::vector<FusionTarget>& getTargets() const { return targets_; }
    size_t getTargetCount() const { return targets_.size(); }

    void addTarget(const FusionTarget& target) { targets_.push_back(target); }
    void clearTargets() { targets_.clear(); }

    void setTargets(const std::vector<FusionTarget>& targets) { targets_ = targets; }
};

/**
 * @brief 态势推理数据消息体
 *
 * 格式：target_count(uint8) + n * SituationMes
 */
class SituationMsg {
private:
    std::vector<SituationMes> situations_;

public:
    SituationMsg() { situations_.clear(); }

    void reset() { situations_.clear(); }

    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        size_t total = size();
        if (!buffer || buffer_len < total) return false;

        size_t offset = 0;

        uint8_t target_count = static_cast<uint8_t>(situations_.size());
        pack(buffer, offset, &target_count, sizeof(target_count));

        for (const auto& s : situations_) {
            pack(buffer, offset, &s.global_id, sizeof(s.global_id));
            pack(buffer, offset, &s.intention, sizeof(s.intention));
            pack(buffer, offset, &s.danger_level, sizeof(s.danger_level));
        }
        return true;
    }

    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < sizeof(uint8_t)) return false;

        size_t offset = 0;

        uint8_t target_count = 0;
        unpack(buffer, offset, &target_count, sizeof(target_count));

        situations_.clear();
        situations_.reserve(target_count);
        for (uint8_t i = 0; i < target_count; ++i) {
            if (buffer_len < offset + SituationMes::wireSize()) return false;
            SituationMes s;
            unpack(buffer, offset, &s.global_id, sizeof(s.global_id));
            unpack(buffer, offset, &s.intention, sizeof(s.intention));
            unpack(buffer, offset, &s.danger_level, sizeof(s.danger_level));
            situations_.push_back(s);
        }
        return true;
    }

    size_t size() const {
        return sizeof(uint8_t) + situations_.size() * SituationMes::wireSize();
    }

    // ========================================================================
    // 访问接口
    // ========================================================================

    const std::vector<SituationMes>& getSituations() const { return situations_; }
    size_t getTargetCount() const { return situations_.size(); }

    void addSituation(const SituationMes& s) { situations_.push_back(s); }
    void clearSituations() { situations_.clear(); }

    void setSituations(const std::vector<SituationMes>& situations) { situations_ = situations; }
};

/**
 * @brief 武器分配消息体
 *
 * 每个目标一条 MatchMsg，内嵌可变长的多武器种类/多实例分配。
 * 格式：match_count(uint8) + n * MatchMsg（变长）
 */
class WeaponDistributionMsg {
private:
    std::vector<MatchMsg> matches_;

public:
    WeaponDistributionMsg() { matches_.clear(); }

    void reset() { matches_.clear(); }

    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        size_t total = size();
        if (!buffer || buffer_len < total) return false;

        size_t offset = 0;

        uint8_t count = static_cast<uint8_t>(matches_.size());
        pack(buffer, offset, &count, sizeof(count));

        for (const auto& m : matches_)
            MatchMsg::serializeTo(m, buffer, offset);

        return true;
    }

    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < sizeof(uint8_t)) return false;

        size_t offset = 0;

        uint8_t count = 0;
        unpack(buffer, offset, &count, sizeof(count));

        matches_.clear();
        matches_.reserve(count);
        for (uint8_t i = 0; i < count; ++i) {
            MatchMsg m;
            if (!MatchMsg::deserializeFrom(buffer, buffer_len, offset, m))
                return false;
            matches_.push_back(m);
        }
        return true;
    }

    size_t size() const {
        size_t sz = sizeof(uint8_t);  // match_count
        for (const auto& m : matches_) sz += m.wireSize();
        return sz;
    }

    // ========================================================================
    // 访问接口
    // ========================================================================

    const std::vector<MatchMsg>& getMatches() const { return matches_; }
    size_t getMatchCount() const { return matches_.size(); }

    void addMatch(const MatchMsg& match) { matches_.push_back(match); }
    void clearMatches() { matches_.clear(); }

    void setMatches(const std::vector<MatchMsg>& matches) { matches_ = matches; }
};

/**
 * @brief 反制结果消息体
 *
 * 格式：target_count(uint8) + n * ResultMsg
 */
class StrikeResultMsg {
private:
    std::vector<ResultMsg> results_;

public:
    StrikeResultMsg() { results_.clear(); }

    void reset() { results_.clear(); }

    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        size_t total = size();
        if (!buffer || buffer_len < total) return false;

        size_t offset = 0;

        uint8_t count = static_cast<uint8_t>(results_.size());
        pack(buffer, offset, &count, sizeof(count));

        for (const auto& r : results_) {
            pack(buffer, offset, &r.target_id, sizeof(r.target_id));
            pack(buffer, offset, &r.result_id, sizeof(r.result_id));
        }
        return true;
    }

    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < sizeof(uint8_t)) return false;

        size_t offset = 0;

        uint8_t count = 0;
        unpack(buffer, offset, &count, sizeof(count));

        results_.clear();
        results_.reserve(count);
        for (uint8_t i = 0; i < count; ++i) {
            if (buffer_len < offset + ResultMsg::wireSize()) return false;
            ResultMsg r;
            unpack(buffer, offset, &r.target_id, sizeof(r.target_id));
            unpack(buffer, offset, &r.result_id, sizeof(r.result_id));
            results_.push_back(r);
        }
        return true;
    }

    size_t size() const {
        return sizeof(uint8_t) + results_.size() * ResultMsg::wireSize();
    }

    // ========================================================================
    // 访问接口
    // ========================================================================

    const std::vector<ResultMsg>& getResults() const { return results_; }
    size_t getResultCount() const { return results_.size(); }

    void addResult(const ResultMsg& result) { results_.push_back(result); }
    void clearResults() { results_.clear(); }

    void setResults(const std::vector<ResultMsg>& results) { results_ = results; }
};

/**
 * @brief 武器设备状态上报消息体
 *
 * 武器管控系统周期性上报所有设备的状态和位置，融合层据此决策可用设备。
 * 格式：instance_count(uint8) + n * WeaponInstance
 */
class WeaponStatusMsg {
private:
    std::vector<WeaponInstance> instances_;

public:
    WeaponStatusMsg() { instances_.clear(); }

    void reset() { instances_.clear(); }

    bool serialize(uint8_t* buffer, uint32_t buffer_len) const {
        size_t total = size();
        if (!buffer || buffer_len < total) return false;

        size_t offset = 0;

        uint8_t count = static_cast<uint8_t>(instances_.size());
        pack(buffer, offset, &count, sizeof(count));

        for (const auto& inst : instances_) {
            pack(buffer, offset, &inst.instance_id, sizeof(inst.instance_id));
            pack(buffer, offset, &inst.weapon_type, sizeof(inst.weapon_type));
            pack(buffer, offset, &inst.status,      sizeof(inst.status));
            pack(buffer, offset, &inst.position.lng_e7, sizeof(inst.position.lng_e7));
            pack(buffer, offset, &inst.position.lat_e7, sizeof(inst.position.lat_e7));
            pack(buffer, offset, &inst.position.alt,    sizeof(inst.position.alt));
        }
        return true;
    }

    bool deserialize(const uint8_t* buffer, uint32_t buffer_len) {
        if (!buffer || buffer_len < sizeof(uint8_t)) return false;

        size_t offset = 0;

        uint8_t count = 0;
        unpack(buffer, offset, &count, sizeof(count));

        instances_.clear();
        instances_.reserve(count);
        for (uint8_t i = 0; i < count; ++i) {
            if (buffer_len < offset + WeaponInstance::wireSize()) return false;
            WeaponInstance inst;
            unpack(buffer, offset, &inst.instance_id, sizeof(inst.instance_id));
            unpack(buffer, offset, &inst.weapon_type, sizeof(inst.weapon_type));
            unpack(buffer, offset, &inst.status,      sizeof(inst.status));
            unpack(buffer, offset, &inst.position.lng_e7, sizeof(inst.position.lng_e7));
            unpack(buffer, offset, &inst.position.lat_e7, sizeof(inst.position.lat_e7));
            unpack(buffer, offset, &inst.position.alt,    sizeof(inst.position.alt));
            instances_.push_back(inst);
        }
        return true;
    }

    size_t size() const {
        return sizeof(uint8_t) + instances_.size() * WeaponInstance::wireSize();
    }

    // ========================================================================
    // 访问接口
    // ========================================================================

    const std::vector<WeaponInstance>& getInstances() const { return instances_; }
    size_t getInstanceCount() const { return instances_.size(); }

    void addInstance(const WeaponInstance& inst) { instances_.push_back(inst); }
    void clearInstances() { instances_.clear(); }

    void setInstances(const std::vector<WeaponInstance>& instances) { instances_ = instances; }
};

// ============================================================================
// 编译期大小校验
// ============================================================================

static_assert(AbsPos::wireSize() == 12, "AbsPos must be 12 bytes");
static_assert(RevPos::wireSize() == 8, "RevPos must be 8 bytes");
static_assert(Attitude::wireSize() == 6, "Attitude must be 6 bytes");
static_assert(WeaponAssignment::MIN_WIRE_SIZE == 2, "WeaponAssignment MIN_WIRE_SIZE must be 2 bytes");
static_assert(WeaponInstance::wireSize() == 15, "WeaponInstance must be 15 bytes");
static_assert(PredictionPoint::wireSize() == 30, "PredictionPoint must be 30 bytes");
static_assert(SituationMes::wireSize() == 3, "SituationMes must be 3 bytes");
static_assert(MatchMsg::MIN_WIRE_SIZE == 14, "MatchMsg MIN_WIRE_SIZE must be 14 bytes");
static_assert(ResultMsg::wireSize() == 2, "ResultMsg must be 2 bytes");
static_assert(FusionTarget::MIN_WIRE_SIZE == 28, "FusionTarget MIN_WIRE_SIZE must be 28 bytes");

} // namespace AntiDroneFusionProtocol

#endif // ANTI_DRONE_FUSION_PROTOCOL_H
