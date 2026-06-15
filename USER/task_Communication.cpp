#include "task_Communication.hpp"
#include "MAVLink_bridge.hpp"
#include "MahonyAHRS.hpp"
#include "board.hpp"
#include "attitute.hpp"
#include <string>
#include <cstdio>

//定义任务句柄
osThreadId_t communicationTaskHandle = NULL;

//定义任务属性
const osThreadAttr_t communicationTask_attributes = {
    .name = "CommunicationTask",
    .stack_size = 8 * 1024,      // 8K
    .priority = (osPriority_t) osPriorityNormal,
};

#define BLOCK_SIZE 512

uint8_t txBuf[BLOCK_SIZE];
uint8_t rxBuf[BLOCK_SIZE];

extern SD_HandleTypeDef hsd1;

extern osMessageQueueId_t g_mavSensorQueue;

//SD卡测试函数
void SD_Test(void)
{
    uint32_t blockAddr = 0;

    for (int i = 0; i < BLOCK_SIZE; i++)
    {
        txBuf[i] = i;
    }

    printf("SD Write...\r\n");

    if (HAL_SD_WriteBlocks(&hsd1, txBuf, blockAddr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Write Error\r\n");
        return;
    }

    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);

    printf("SD Read...\r\n");

    if (HAL_SD_ReadBlocks(&hsd1, rxBuf, blockAddr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        printf("Read Error\r\n");
        return;
    }

    while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER);

    //判断写入和回读的值是否相同，如果相同测试成功
    for (int i = 0; i < BLOCK_SIZE; i++)
    {
        if (txBuf[i] != rxBuf[i])
        {
            printf("Compare Failed at %d\r\n", i);
            return;
        }
    }

    printf("SD Test OK!\r\n");
}

// 定义任务函数
void StartCommunicationTask(void *argument)
{
	MAVLink::Init();
    
    printf("Communication Task Started!\r\n");
    
    //驱动对象获取
    auto& bluetooth = Board::getBluetooth();
    auto& uart = Board::getUart1();
	auto& buzzer=Board::getBuzzer();
	auto& esc_node = Board::getESCNode();
	auto& led = Board::getLedPwm();
	
	uint32_t next_wake = osKernelGetTickCount();
	//蜂鸣器提示
	//buzzer.beep(2000,100);

//  SD_Test();
		 
	if(!bluetooth.autoBaudScan())
	{
	Error_Handler();
	}
	//MAVLink数据定义
	MavRxFrame_t frame;
    uint32_t last_heartbeat = 0;
    uint32_t now;
//	
//	float bat_vel;


while (1) {

		//队列获取串口数据（透传模式下）
	    if (osMessageQueueGet(uart.getMavQueue(), &frame, NULL, 0) == osOK) {
	            MAVLink::ParseData(frame.data, frame.len);
	        }
		
		if(MAVLink::get_mavlink_connect_status()){led.setRGBBlink(0,100,0,1);}
			else{led.setRGBBlink(100,0,0,1);}

		// 获取传感器数据，打包发送
		if (g_mavSensorQueue != NULL) {
		    MavSensorData_t sensor_data;
		    if (osMessageQueueGet(g_mavSensorQueue, &sensor_data, NULL, 0) == osOK) {
				//发送姿态
		        MAVLink::SendAttitude(
		            sensor_data.roll,
		            sensor_data.pitch,
		            sensor_data.yaw,
		            sensor_data.rollspeed,
		            sensor_data.pitchspeed,
		            sensor_data.yawspeed
		        );
				//发送电池状态
		        MAVLink::SendBatteryStatus(
		            sensor_data.voltage,
		            sensor_data.current,
		            sensor_data.battery_remaining
		        );
		    }
		}

		MAVLink::SendHeartbeat();

	        next_wake += 20U;  //50Hz
	        osDelayUntil(next_wake);
	    }
}

