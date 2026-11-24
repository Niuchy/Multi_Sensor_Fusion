/**
 * @file ad_proto_receiver.h
 * @brief 反无人机协议 DDS 接收端 C API
 *
 * 提供 poll 模式接收接口，Python 通过 ctypes 调用。
 * 内部用线程安全队列缓冲 DDS 回调收到的帧。
 *
 * 用法:
 *   AdRxFrame frame;
 *   ad_proto_receive_init();
 *   while (ad_proto_receive_poll(&frame, 100) > 0) {
 *       // 处理 frame.data (dev_proto.h 二进制帧)
 *   }
 *   ad_proto_receive_close();
 */

#ifndef AD_PROTO_RECEIVER_H
#define AD_PROTO_RECEIVER_H

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#define AD_RX_MAX_FRAME 2048

/**
 * @brief 单帧接收结果
 *
 * data 字段存放 dev_proto.h 定义的完整二进制帧:
 *   - 0x1A01: RadarTargetFrame       66B
 *   - 0x1B01: FrequencyDetectFrame  139B
 *   - 0x1C01: CamTargetFrame         79B
 *   - 0x1D01: AcousticTargetFrame    67B
 */
typedef struct {
    uint16_t frame_type;                 ///< 帧类型 (0x1A01/0x1B01/0x1C01/0x1D01)
    uint32_t device_id;                  ///< 设备 ID
    uint64_t timestamp_ms;               ///< 时间戳 (ms)
    uint32_t data_len;                   ///< 帧数据实际长度
    uint8_t  data[AD_RX_MAX_FRAME];      ///< 原始 dev_proto 帧数据
} AdRxFrame;

/**
 * @brief 初始化接收端 (DDS 参数全部使用 third_party 库默认值)
 * @return 0=成功, -1=失败
 */
int ad_proto_receive_init(void);

/**
 * @brief 轮询获取一帧 (阻塞模式)
 * @param frame      [out] 接收到的帧数据
 * @param timeout_ms 超时 (ms), -1=永久阻塞, 0=立即返回
 * @return 1=成功获取一帧, 0=超时无数据, -1=错误
 */
int ad_proto_receive_poll(AdRxFrame* frame, int timeout_ms);

/**
 * @brief 获取当前队列中等待的帧数量
 */
int ad_proto_receive_available(void);

/**
 * @brief 关闭接收端 (不析构 DDS，避免 FastDDS 退出 crash)
 */
void ad_proto_receive_close(void);

#ifdef __cplusplus
}
#endif

#endif // AD_PROTO_RECEIVER_H
