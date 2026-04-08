#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

// 控制任务函数声明
void StartDisplayTask(void *argument);

// 控制任务句柄
extern osThreadId_t DisplayTaskHandle;

// 控制任务属性结构体
extern const osThreadAttr_t DisplayTask_attributes;
