#include "menu.hpp"
#include "board.hpp"
#include "attitute.hpp"

static int16_t select_frame_pos_y = 0;
static int16_t select_frame_pos_x = 0;


MenuState menu_att_page(void)
{
	float roll, pitch, yaw;

	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();

	if(osMutexAcquire(g_att_mutex, 0) == osOK)
	{
		roll  = g_attitude.roll;
		pitch = g_attitude.pitch;
		yaw   = g_attitude.yaw;

		osMutexRelease(g_att_mutex);
	}

	OLED_Clear();

	OLED_ShowString(0,0,"roll:");
	OLED_ShowString(0,16,"pitch:");
	OLED_ShowString(0,32,"yaw:");

	OLED_ShowFloat(58, 0,roll);
	OLED_ShowFloat(58, 16,pitch);
	OLED_ShowFloat(58, 32,yaw);

	OLED_Update();

	if(key3.getEvent() == Key::Event::ShortPress)
		return MenuState::MAIN_PAGE_STATE;

	return MenuState::ATT_PAGE_STATE;
}

MenuState menu_task_page(void)
{
	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();

	static int16_t draw_frame_y_pos = 18;
	const int16_t pos_table[2] = {18, 32};
	static int8_t index = 0;

	// key1: 上移
	{
		Key::Event evt = key1.getEvent();
		if(evt == Key::Event::ShortPress)
		{
			index = (index - 1 + 2) % 2;
			draw_frame_y_pos = pos_table[index];
		}
	}

	// key2: 下移
	{
		Key::Event evt = key2.getEvent();
		if(evt == Key::Event::ShortPress)
		{
			index = (index + 1) % 2;
			draw_frame_y_pos = pos_table[index];
		}
	}

	OLED_Clear();

	OLED_ShowChinese(0, 18, "单边控制");
	OLED_ShowChinese(0, 32, "单点控制");

	OLED_DrawRectangle(0, draw_frame_y_pos, 120, 15, 0);

	// key3: 短按返回 / 长按确认选择
	{
		Key::Event evt = key3.getEvent();
		if(evt == Key::Event::ShortPress)
			return MenuState::MAIN_PAGE_STATE;

		if(evt == Key::Event::LongPress)
		{
			//开启电调输出
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
			
			if(draw_frame_y_pos == 18)
				g_selected_control_mode = 0;  // 单边控制
			else
				g_selected_control_mode = 1;  // 单点控制

			if(g_controlModeSem != NULL)
				osSemaphoreRelease(g_controlModeSem);

			OLED_Update();
			return MenuState::CONTROL_RUNNING_STATE;
		}
	}

	OLED_Update();

	return MenuState::CONTROL_PAGE_STATE;
}

MenuState menu_firmware_page(void)
{
	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();

	OLED_ShowString(0,0,"Version: V1.0.0");

	if(key3.getEvent() == Key::Event::ShortPress)
		return MenuState::MAIN_PAGE_STATE;

	return MenuState::FIRMWARE_PAGE_STATE;
}

MenuState menu_control_running_page(void)
{
	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();

	OLED_Clear();

	OLED_ShowChinese(0, 0, "当前模式:");

	if(g_selected_control_mode == 0)
		OLED_ShowChinese(0, 18, "单边控制");
	else if(g_selected_control_mode == 1)
		OLED_ShowChinese(0, 18, "单点控制");
	else
		OLED_ShowChinese(0, 18, "未选择");

	OLED_ShowString(0, 40, "key3: back");

	OLED_Update();

	// key3: 短按返回主页并退出任务模式
	// 重新采样按键，避免因控制任务抢占导致的按键事件丢失
	key3.update();
	{
		Key::Event evt = key3.getEvent();
		if(evt == Key::Event::ShortPress)
		{
			//关闭电调输出
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET);
			
			g_selected_control_mode = 0xFF;  // 退出任务模式

			if(g_controlModeSem != NULL)
				osSemaphoreRelease(g_controlModeSem);

			return MenuState::MAIN_PAGE_STATE;
		}
	}

	return MenuState::CONTROL_RUNNING_STATE;
}
