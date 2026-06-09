#pragma once

#include <cstdint>

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

void RunTimeStats_Init(void);
uint32_t RunTimeStats_GetCounter(void);

#ifdef __cplusplus
}

// CPU监控任务函数声明
void StartCPUUsageMonitorTask(void *argument);

// CPU监控任务句柄
extern osThreadId_t CPUUsageMonitorTaskHandle;

// CPU监控任务属性结构体
extern const osThreadAttr_t CPUUsageMonitorTask_attributes;

// 获取缓存的CPU整体占用率，0~100
// 首次调用返回0，等待监控任务首次更新后返回有效值
uint32_t GetCPUUsage(void);


#endif
