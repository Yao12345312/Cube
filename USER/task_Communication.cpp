#include "task_Communication.hpp"
#include "MAVLink_bridge.hpp"
#include "MahonyAHRS.hpp"
#include "board.hpp"
#include <string>
#include <cstdio>
extern I2C_HandleTypeDef hi2c1;
// 定义任务句柄
osThreadId_t communicationTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t communicationTask_attributes = {
    .name = "CommunicationTask",
    .stack_size = 2048,      // 通信任务栈大小
    .priority = (osPriority_t) osPriorityNormal,
};

// 通信任务入口函数
void StartCommunicationTask(void *argument)
{
	MAVLink::Init();
    
    // 打印启动信息
    printf("Communication Task Started!\r\n");
    
    // 获取硬件访问接口
    auto& bluetooth = Board::getBluetooth();
    auto& uart = Board::getUart1();
	auto& led =Board::getLedPwm();
	auto& ina226=Board::getINA226();
	auto& oled=Board::getOled();
	if(!bluetooth.autoBaudScan())
	{
	Error_Handler();
	}
	//MAVLink封装帧
	MavRxFrame_t frame;
    uint32_t last_heartbeat = 0;
    uint32_t now;
	
	float bat_vel;
	//MAVLink未连接状态，设置红灯1HZ闪烁
	led.setRGBBlink(100,0,0,1);
	

    while (1) {
	bat_vel=ina226.INA226_ReadBusVoltage();
	OLED_ShowFloat(&oled,3,0,bat_vel,4);	
        //处理接收到的MAVLink数据
    if (osMessageQueueGet(uart.getMavQueue(), &frame, NULL, 0) == osOK) {
            MAVLink::ParseData(frame.data, frame.len);
        }
	//MAVLink通信成功，设置绿灯1HZ闪烁
	if(MAVLink::get_mavlink_connect_status()){led.setRGBBlink(0,100,0,1);}
		
	//发送心跳包
	MAVLink::SendHeartbeat();
			
        osDelay(100);
    }
}