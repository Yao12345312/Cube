#pragma once

#include "cmsis_os2.h"

typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

// 全局共享变量
extern Attitude_t g_attitude;

extern osMutexId_t g_att_mutex;

// MAVLink传感器数据结构体（用于CMSIS-RTOS2队列传输）
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float rollspeed;       // X轴角速度 rad/s
    float pitchspeed;      // Y轴角速度 rad/s
    float yawspeed;        // Z轴角速度 rad/s
    float voltage;         // 电池电压 V
    float current;         // 电池电流 A, -1表示未知
    int8_t battery_remaining; // 电量百分比, -1表示未知
} MavSensorData_t;

// 传感器数据队列句柄
extern osMessageQueueId_t g_mavSensorQueue;   // 控制任务 -> 通信任务
extern osMessageQueueId_t g_dispSensorQueue;  // 控制任务 -> 显示任务

