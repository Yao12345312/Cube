#include "task_Display.hpp"
#include "MAVLink_bridge.hpp"
#include "board.hpp"
#include "menu.hpp"
#include "attitute.hpp"

extern u8g2_t u8g2;

extern osMessageQueueId_t g_dispSensorQueue;

//定义任务句柄
osThreadId_t DisplayTaskHandle = NULL;

//定义任务属性
const osThreadAttr_t DisplayTask_attributes = {
    .name = "ControlTask",
    .stack_size = 8*1024,
    .priority = (osPriority_t) osPriorityNormal,  // 显示任务不需要高实时性，降低优先级避免阻塞控制任务
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

	static INA226::bat_state bat_status = INA226::bat_state::BAT_MID_POWER;

	// Scrollable menu: 5 items, 3 visible slots
	const int8_t  MENU_COUNT = 5;
	const int16_t VISIBLE_Y[3] = {18, 32, 46};

	const char* menu_names[5] = {
		"立方体姿态",
		"任务控制",
		"固件版本",
		"陀螺仪校准",
		"磁力计校准"
	};

	int16_t draw_frame_y_pos = 0;
	int8_t  selected_idx = 0;
	int8_t  scroll_top   = 0;

	MenuState main_menu_flag = MenuState::MAIN_PAGE_STATE;

	//清除OLED缓存
	OLED_Clear();

	osDelay(100);
	//显示LOGO
	OLED_DisplayLogo();
	//更新缓存
	OLED_Update();

	osDelay(1500);

	//获取按键事件
	key1.update(); key1.getEvent();
	key2.update(); key2.getEvent();
	key3.update(); key3.getEvent();

	uint32_t next_wake = osKernelGetTickCount();

	while(1)
	{
		//清除缓存
		OLED_Clear();

		//更新按键状态
		key1.update();
		key2.update();
		key3.update();


		//队列获取传感器数据
		if (g_dispSensorQueue != NULL) {
		    MavSensorData_t sensor_data;
		    if (osMessageQueueGet(g_dispSensorQueue, &sensor_data, NULL, 0) == osOK) {
		        (void)sensor_data;
		    }
		}
		esc_node.get_esc_status(ESC1_Index,esc_status[ESC1_Index]);

		//获取电池状态
		{
			float bus_v = ina226.INA226_ReadBusVoltage();
			if (bus_v > 0.0f)
				bat_status = ina226.INA226_get_bat_status(bus_v);
		}

		//主界面滚动菜单
		if(main_menu_flag == MenuState::MAIN_PAGE_STATE)
		{
			//显示三个图标（选择框，蓝牙连接状态，电池电量）
			for(int i = 0; i < 3; i++)
			{
				int8_t item_idx = scroll_top + i;
				if(item_idx < MENU_COUNT)
				{
					//菜单名称写入缓存
					OLED_ShowChinese(0, VISIBLE_Y[i], (char*)menu_names[item_idx]);

					//指示校准状态
					if(item_idx == 3 && g_gyro_calibrated)
						OLED_ShowChinese(90, VISIBLE_Y[i], "OK");
					if(item_idx == 4 && g_mag_calibrated)
						OLED_ShowChinese(90, VISIBLE_Y[i], "OK");
				}
			}

			//按键1短按：选择框上移
			if(key1.getEvent() == Key::Event::ShortPress)
			{
				selected_idx = (selected_idx - 1 + MENU_COUNT) % MENU_COUNT;
				//移动选择框
				if(selected_idx < scroll_top)
					scroll_top = selected_idx;
				if(selected_idx >= scroll_top + 3)
					scroll_top = selected_idx - 2;
			}

			//按键2短按：选择框下移
			if(key2.getEvent() == Key::Event::ShortPress)
			{
				selected_idx = (selected_idx + 1) % MENU_COUNT;
				//移动选择框
				if(selected_idx < scroll_top)
					scroll_top = selected_idx;
				if(selected_idx >= scroll_top + 3)
					scroll_top = selected_idx - 2;
			}

			//选择框写入缓存
			draw_frame_y_pos = VISIBLE_Y[selected_idx - scroll_top];
			if(draw_frame_y_pos != 0)
				OLED_DrawRectangle(0, draw_frame_y_pos, 120, 15, 0);

			//按键3长按：确定
			if(key3.getEvent() == Key::Event::LongPress)
			{
				switch(selected_idx)
				{
				case 0:
					main_menu_flag = MenuState::ATT_PAGE_STATE;
					break;
				case 1:
					main_menu_flag = MenuState::CONTROL_PAGE_STATE;
					break;
				case 2:
					main_menu_flag = MenuState::FIRMWARE_PAGE_STATE;
					break;
				case 3:
					main_menu_flag = MenuState::GYRO_CALIB_PAGE_STATE;
					break;
				case 4:
					main_menu_flag = MenuState::MAG_CALIB_PAGE_STATE;
					break;
				}

				//长按后松手检测
				while(key3.isPressed())
				{
					key3.update();
					osDelay(10);
				}
			}
		}


		//子界面切换
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
		case MenuState::GYRO_CALIB_PAGE_STATE:
			{
				main_menu_flag = menu_gyro_calib_page();
				break;
			}
		case MenuState::MAG_CALIB_PAGE_STATE:
			{
				main_menu_flag = menu_mag_calib_page();
				break;
			}
		}

		//蓝牙连接状态标志以及电量图标写入缓存
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

		OLED_Update();

		next_wake += 50U;  // 20Hz
		osDelayUntil(next_wake);
	}

}
