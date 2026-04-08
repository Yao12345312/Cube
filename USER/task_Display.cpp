#include "task_Display.hpp"
#include "board.hpp"

// 定义任务句柄
osThreadId_t DisplayTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t DisplayTask_attributes = {
    .name = "ControlTask",
    .stack_size = 2*1024,      // 显示任务栈大小
    .priority = (osPriority_t) osPriorityNormal,  // 显示任务默认优先级
};

void StartDisplayTask(void *argument){

	while(1)
	{
	
	osDelay(1000);
	}

}

