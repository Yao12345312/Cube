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
	
	auto& oled = Board::getOled();
	auto& uavcan = Board::getCan();
	auto& esc_node = Board::getESCNode();
	
	//开启OLED显示
	OLED_Display_On(&oled);
	
	ESCNode::ESCStatusCache esc_status[Max_ESC_Num]={0};
	
	while(1)
	{
	
		esc_node.get_esc_status(ESC1_Index,esc_status[ESC1_Index]);
		
		OLED_ShowString(&oled, 0, 0,"vol:");
		OLED_ShowString(&oled, 1, 0,"idx:");
		OLED_ShowString(&oled, 2, 0,"tmp:");
		OLED_ShowString(&oled, 3, 0,"rpm:");
		
		OLED_ShowFloat(&oled,0,5,esc_status[ESC1_Index].voltage,2);	
		OLED_ShowFloat(&oled,1,5,esc_status[ESC1_Index].calib_flag,2);
		OLED_ShowFloat(&oled,2,5,esc_status[ESC1_Index].temperature-273.15f,2);
		OLED_ShowInt32(&oled,3,5,esc_status[ESC1_Index].rpm);
		
		
	osDelay(100);
	}

}

