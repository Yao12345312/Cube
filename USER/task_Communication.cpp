#include "task_Communication.hpp"
#include "MAVLink_bridge.hpp"
#include "board.hpp"
#include <string>
#include <cstdio>

// 定义任务句柄
osThreadId_t communicationTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t communicationTask_attributes = {
    .name = "CommunicationTask",
    .stack_size = 2048,      // 通信任务栈大小，根据实际需求调整
    .priority = (osPriority_t) osPriorityNormal,
};

// 通信任务入口函数
void StartCommunicationTask(void *argument)
{

    // 等待蓝牙模块稳定
    osDelay(2500);
    
    // 打印启动信息
    printf("Communication Task Started!\r\n");
    
    // 获取硬件访问接口
    auto& bluetooth = Board::getBluetooth();
    
	if(!bluetooth.autoBaudScan())
	{
	Error_Handler();
	}
	
    while (1) {
	//发送心跳包
	MAVLink::SendHeartbeat();
		
	
	
        osDelay(100);
    }
}