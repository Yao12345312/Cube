#include "task_Communication.hpp"
#include "MAVLink_bridge.hpp"
#include "MahonyAHRS.hpp"
#include "board.hpp"
#include <string>
#include <cstdio>
extern SD_HandleTypeDef hsd1;
// 定义任务句柄
osThreadId_t communicationTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t communicationTask_attributes = {
    .name = "CommunicationTask",
    .stack_size = 2048,      // 通信任务栈大小
    .priority = (osPriority_t) osPriorityNormal,
};

#define BLOCK_SIZE 512

uint8_t txBuf[BLOCK_SIZE];
uint8_t rxBuf[BLOCK_SIZE];
//SD卡快速测试函数
void SD_Test(void)
{
    uint32_t blockAddr = 0;

    // 填充测试数据
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

    // 校验
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
	auto& oled=Board::getOled();
	auto& buzzer=Board::getBuzzer();
	
	//上电蜂鸣器提示
	buzzer.beep(2000,100);
	
	//没插TF卡的时候注释，否则HAL库会初始化失败
//   SD_Test();
		 
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
	led.setRGBBlink(50,0,0,1);

    while (1) {

    //处理接收到的MAVLink数据
    if (osMessageQueueGet(uart.getMavQueue(), &frame, NULL, 0) == osOK) {
            MAVLink::ParseData(frame.data, frame.len);
        }
	//MAVLink通信成功，设置绿灯1HZ闪烁
	if(MAVLink::get_mavlink_connect_status()){led.setRGBBlink(0,100,0,1);}
	//丢失连接，闪烁红灯
		else{led.setRGBBlink(100,0,0,1);}
		
	//发送心跳包
	MAVLink::SendHeartbeat();
			
        osDelay(100);
    }
}

