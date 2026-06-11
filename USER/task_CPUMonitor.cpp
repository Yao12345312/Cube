#include "task_CPUMonitor.hpp"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>

#define MAX_TASKS 16

//定义任务句柄
osThreadId_t CPUUsageMonitorTaskHandle = NULL;

//定义任务属性
const osThreadAttr_t CPUUsageMonitorTask_attributes = {
    .name = "CPUMonitorTask",
    .stack_size = 2 * 1024,
    .priority = (osPriority_t)osPriorityLow, 
};


// FreeRTOS Run-Time Stats 所需的 DWT CYCCNT 接口
extern "C" {

void RunTimeStats_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t RunTimeStats_GetCounter(void)
{
    return DWT->CYCCNT;
}

} // extern "C"


static volatile uint32_t g_cpu_usage = 0;

// 执行空闲任务统计采集（会关中断，仅由监控任务调用）
static uint32_t ComputeCPUUsage(void)
{
    static uint32_t last_total = 0;
    static uint32_t last_idle  = 0;

    TaskStatus_t task_stats[MAX_TASKS];
    uint32_t     total_runtime;
    UBaseType_t  task_count = uxTaskGetSystemState(task_stats, MAX_TASKS, &total_runtime);

    uint32_t idle_runtime = 0;
    for (UBaseType_t i = 0; i < task_count; i++) {
        if (task_stats[i].pcTaskName[0] == 'I'
            && task_stats[i].pcTaskName[1] == 'D'
            && task_stats[i].pcTaskName[2] == 'L'
            && task_stats[i].pcTaskName[3] == 'E') {
            idle_runtime = task_stats[i].ulRunTimeCounter;
            break;
        }
    }

    uint32_t cpu_usage   = 0;
    uint32_t delta_total = total_runtime - last_total;
    uint32_t delta_idle  = idle_runtime  - last_idle;
	
	//CPU占用率= （总CPU运行时间增量 - 空闲任务时间增量） / 总CPU运行时间增量
    if (delta_total > 0) {
        cpu_usage = 100U - (delta_idle * 100U / delta_total);
        if (cpu_usage > 100U) cpu_usage = 100U;
    }

    last_total = total_runtime;
    last_idle  = idle_runtime;

    return cpu_usage;
}

//低优先级监控任务，每 1s 更新一次缓存值
void StartCPUUsageMonitorTask(void *argument)
{	
	uint32_t next_wake = osKernelGetTickCount();
	
    while(1) 
	{
        g_cpu_usage = ComputeCPUUsage();

        next_wake += 1000U;  // 1Hz
        osDelayUntil(next_wake);
    }
}

uint32_t GetCPUUsage(void)
{
    return g_cpu_usage;
}

