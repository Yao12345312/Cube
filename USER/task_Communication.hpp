#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

// 通信任务函数声明
void StartCommunicationTask(void *argument);

// 通信任务句柄（用于外部访问）
extern osThreadId_t communicationTaskHandle;

// 通信任务属性结构体
extern const osThreadAttr_t communicationTask_attributes;