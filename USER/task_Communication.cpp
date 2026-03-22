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
	MAVLink::Init();
    // 等待蓝牙模块稳定
    osDelay(2500);
    
    // 打印启动信息
    printf("Communication Task Started!\r\n");
    
    // 获取硬件访问接口
    auto& bluetooth = Board::getBluetooth();
    auto& uart = Board::getUart1();
	auto& led =Board::getLedPwm();
	if(!bluetooth.autoBaudScan())
	{
	Error_Handler();
	}
	
	MavRxFrame_t frame;
    uint32_t last_heartbeat = 0;
    uint32_t now;
	led.setRGBBlink(100,0,0,5);
	
    while (1) {
		
        
        //处理接收到的MAVLink数据
    if (osMessageQueueGet(uart.getMavQueue(), &frame, NULL, 0) == osOK) {
            MAVLink::ParseData(frame.data, frame.len);
        }
	
	if(MAVLink::get_mavlink_connect_status()){led.setRGBBlink(0,100,0,5);}
		
	//发送心跳包
	MAVLink::SendHeartbeat();
			
        osDelay(100);
    }
}