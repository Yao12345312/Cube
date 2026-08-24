#pragma once

#include "cmsis_os2.h"
#include <stdint.h>

// 控制任务回调函数
void StartControlTask(void *argument);

// 控制任务句柄（由 taskManager 创建后赋值）
extern osThreadId_t controlTaskHandle;

// =============================================================================
// 跨任务共享数据 (移植自参考工程 task_Control.cpp / attitute.hpp)
//
// 控制任务每 10 个周期 (20ms@500Hz) 打包一帧 MavSensorData 入队,
// 通信任务从队列取出后调 MAVLink::SendAttitude / SendBatteryStatus 上报。
// =============================================================================

// 共享姿态 (控制任务写, 显示菜单读), 配互斥锁
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

extern Attitude_t   g_attitude;
extern osMutexId_t  g_att_mutex;

// MAVLink 传感数据帧 (控制任务 -> 通信任务)
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float rollspeed;       // X 角速度 rad/s
    float pitchspeed;      // Y 角速度 rad/s
    float yawspeed;        // Z 角速度 rad/s
    float voltage;         // 电池电压 V (取 ESC 总线电压)
    float current;         // 电池电流 A, -1 = 未知
    int8_t battery_remaining; // 剩余百分比, 0 = 未知
} MavSensorData_t;

// 控制任务 -> 通信任务 的传感数据队列
extern osMessageQueueId_t g_mavSensorQueue;

// 控制任务 -> 显示任务 的传感数据队列 (供菜单姿态/电压显示)
extern osMessageQueueId_t g_dispSensorQueue;

// 控制模式切换信号量 (通信任务/菜单 release, 控制任务 acquire 0 即清)
extern osSemaphoreId_t g_controlModeSem;

// 遥控指令 (通信任务 ParseData 写, 控制任务读) - volatile 跨任务可见
extern volatile uint8_t g_rc_control_mode;   // 0=单边 1=单点 4=转速测试 0xFF=停机
extern volatile float   g_rc_speed_target;   // 速度环目标 (rad/s)
extern volatile float   g_rc_manual_y;       // 转向 (偏航角速度目标)

// 菜单本地选择的控制模式 (显示任务写, 控制任务读)
// 0xFF = 未选择 (停机); 与 g_rc_control_mode 取有效者
// 0=单边 1=单点 2=单边起跳 3=单点起跳 4=转速测试
extern uint8_t g_selected_control_mode;

// =============================================================================
// 校准任务间同步 (移植自参考工程 task_Control.cpp)
// 显示菜单 long-press -> release g_calibSem, 控制任务消费并执行校准
// =============================================================================
extern osSemaphoreId_t g_calibSem;
extern uint8_t         g_calib_command;   // 0=idle, 1=陀螺仪零偏
extern bool            g_gyro_calibrated;
