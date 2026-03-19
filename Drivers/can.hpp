#pragma once

#include "stm32h7xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ================= CAN ID 定义 =================
#define eCAN_SEND_MOTOR_SPEED_FRAME   0x101
#define eCAN_GET_MOTOR_SPEED_FRAME    0x102

// 在线检测阈值
#define CAN_OFFLINE_COUNT 100

#ifdef __cplusplus
}
#endif


class CanDriver
{
public:
    CanDriver();

    // 初始化
    void Init(FDCAN_HandleTypeDef *hfdcan);

    // 发送电机速度
    void SendMotorSpeed(float ch2_speed, float ch3_speed);

    // 获取接收速度
    void GetMotorSpeed(float &ch2_speed, float &ch3_speed);

    // 在线检测
    bool IsOnline();

    // 中断回调处理（必须在HAL回调中调用）
    void RxCallback();

private:
    FDCAN_HandleTypeDef *hfdcan_;

    float rx_ch2_speed_;
    float rx_ch3_speed_;

    uint32_t rx_frame_count_;
    bool is_online_;
};

