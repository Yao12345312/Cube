#include "task_Display.hpp"
#include "MAVLink_bridge.hpp"
#include "board.hpp"
#include "menu.hpp"
extern u8g2_t u8g2;
// 定义任务句柄
osThreadId_t DisplayTaskHandle = NULL;

//定义任务属性
const osThreadAttr_t DisplayTask_attributes = {
    .name = "ControlTask",
    .stack_size = 8*1024,      //任务栈大小
    .priority = (osPriority_t) osPriorityNormal,  // 任务优先级
};

void StartDisplayTask(void *argument){
	
	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();
	auto& uavcan = Board::getCan();
	auto& esc_node = Board::getESCNode();
	auto& ina226 = Board::getINA226();
	auto& led =Board::getLedPwm();
	
	ESCNode::ESCStatusCache esc_status[Max_ESC_Num]={0};	
	
	INA226::bat_state bat_status;
	
	MenuState main_menu_flag = MenuState::MAIN_PAGE_STATE;
	//清空缓冲区
	OLED_Clear();
	
	osDelay(100);
	//显示LOGO
	OLED_DisplayLogo();
	//更新显示
	OLED_Update();
	
	osDelay(2000);

	while(1)
	{	
		//清空缓冲区
		OLED_Clear();
	
		//获取按键状态
		key1.update();
		key2.update();
		key3.update();
		
		//获取电池状态
		bat_status = ina226.INA226_get_bat_status(ina226.INA226_ReadBusVoltage());
		
		if(main_menu_flag == MenuState::MAIN_PAGE_STATE)
			
		{	
	
			if(MAVLink::get_mavlink_connect_status())
			{
				led.setRGBBlink(0,100,0,1);
				OLED_ShowBLEON();
				if(bat_status == INA226::bat_state::BAT_FULL_POWER)
					OLED_Fullbattery();
				if(bat_status == INA226::bat_state::BAT_HIGH_POWER)
					OLED_Highbattery();
				if(bat_status == INA226::bat_state::BAT_MID_POWER)
					OLED_Middlebattery;
				if(bat_status == INA226::bat_state::BAT_LOW_POWER)
					OLED_Lowbattery();
			}
			else
			{
				led.setRGBBlink(100,0,0,1);
				OLED_ShowBLEOFF();
				if(bat_status == INA226::bat_state::BAT_FULL_POWER)
					OLED_Fullbattery();
				if(bat_status == INA226::bat_state::BAT_HIGH_POWER)
					OLED_Highbattery();
				if(bat_status == INA226::bat_state::BAT_MID_POWER)
					OLED_Middlebattery;
				if(bat_status == INA226::bat_state::BAT_LOW_POWER)
					OLED_Lowbattery();
			}
			
		}
		
		OLED_ShowChinese(0 ,32 ,"中文显示");
		OLED_ShowChinese(0 ,46 ,"中文显示");
		OLED_DrawRectangle(0 , 32, 120,15,0);
		
		OLED_Update();
		
		osDelay(200);
	}

}

