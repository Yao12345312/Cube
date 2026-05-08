#include "task_Display.hpp"
#include "MAVLink_bridge.hpp"
#include "board.hpp"
#include "menu.hpp"
#include "attitute.hpp"

extern u8g2_t u8g2;

extern osMessageQueueId_t g_dispSensorQueue;

// 任务句柄
osThreadId_t DisplayTaskHandle = NULL;

//任务属性配置
const osThreadAttr_t DisplayTask_attributes = {
    .name = "ControlTask",
    .stack_size = 8*1024,      //任务栈大小
    .priority = (osPriority_t) osPriorityHigh,  // 与控制任务同级，避免饥饿
};

void StartDisplayTask(void *argument){

	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();
	auto& uavcan = Board::getCan();
	auto& esc_node = Board::getESCNode();
	auto& ina226 = Board::getINA226();
	auto& led =Board::getLedPwm();
	auto& buzzer = Board::getBuzzer();

	ESCNode::ESCStatusCache esc_status[Max_ESC_Num]={0};

	INA226::bat_state bat_status;

	int16_t draw_frame_y_pos = 0;

	const int16_t pos_table[3] = {18, 32, 46};  int8_t index = 0; //菜单选项y坐标及当前index

	uint8_t current_frame_y_state=0;

	MenuState main_menu_flag = MenuState::MAIN_PAGE_STATE;
	//清屏缓冲区
	OLED_Clear();

	osDelay(100);
	//显示LOGO
	OLED_DisplayLogo();
	//更新显示
	OLED_Update();

	osDelay(1500);

	uint32_t next_wake = osKernelGetTickCount();

	while(1)
	{
		//清屏缓冲区
		OLED_Clear();

		//读取按键状态
		key1.update();
		key2.update();
		key3.update();


		// Receive sensor data (non-blocking)
		if (g_dispSensorQueue != NULL) {
		    MavSensorData_t sensor_data;
		    if (osMessageQueueGet(g_dispSensorQueue, &sensor_data, NULL, 0) == osOK) {
		        (void)sensor_data;
		    }
		}
		esc_node.get_esc_status(ESC1_Index,esc_status[ESC1_Index]);

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
					OLED_Middlebattery();
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
					OLED_Middlebattery();
				if(bat_status == INA226::bat_state::BAT_LOW_POWER)
					OLED_Lowbattery();
			}
		}
		if(main_menu_flag == MenuState::MAIN_PAGE_STATE)
		{
			//显示选项
			OLED_ShowChinese(0 ,18 ,"立方体姿态");
			OLED_ShowChinese(0 ,32 ,"任务控制");
			OLED_ShowChinese(0 ,46 ,"固件版本");

			//获取当前位置索引
			for(int i = 0; i < 3; i++)
			{
				if(draw_frame_y_pos == pos_table[i])
				{
					index = i;
					break;
				}
			}

			// key1控制菜单（向上）
			if(key1.getEvent() == Key::Event::ShortPress)
			{
				index = (index - 1 + 3) % 3;
				draw_frame_y_pos = pos_table[index];
				//buzzer.beep(500,30);
			}

			// key2控制菜单（向下）
			if(key2.getEvent() == Key::Event::ShortPress)
			{
				index = (index + 1) % 3;
				draw_frame_y_pos = pos_table[index];
				//buzzer.beep(500,30);
			}

			if(draw_frame_y_pos != 0)
				OLED_DrawRectangle(0 , draw_frame_y_pos, 120,15,0);

			if(draw_frame_y_pos == 18 && key3.getEvent() == Key::Event::LongPress)
			{
				main_menu_flag = MenuState::ATT_PAGE_STATE;
				while(key3.isPressed())
				{
					key3.update();
					osDelay(10);
				}
			}

			if(draw_frame_y_pos == 32 && key3.getEvent() == Key::Event::LongPress)
			{
				main_menu_flag = MenuState::CONTROL_PAGE_STATE;
				while(key3.isPressed())
				{
					key3.update();
					osDelay(10);
				}
			}

			if(draw_frame_y_pos == 46 && key3.getEvent() == Key::Event::LongPress)
				main_menu_flag = MenuState::FIRMWARE_PAGE_STATE;
		}


		//子页面模式切换
		switch (main_menu_flag)
		{
		case MenuState::ATT_PAGE_STATE:
			{
				main_menu_flag = menu_att_page();
				break;
			}

		case MenuState::CONTROL_PAGE_STATE:
			{
				main_menu_flag = menu_task_page();
				break;
			}
		case MenuState::FIRMWARE_PAGE_STATE:
			{
				main_menu_flag = menu_firmware_page();
				break;
			}
		case MenuState::CONTROL_RUNNING_STATE:
			{
				main_menu_flag = menu_control_running_page();
				break;
			}
		}

		OLED_Update();

		next_wake += 30U;  //30Hz
		osDelayUntil(next_wake);
	}

}
