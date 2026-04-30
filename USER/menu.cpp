#include "menu.hpp"
#include "board.hpp"
#include "attitute.hpp"

static int16_t select_frame_pos_y = 0;
static int16_t select_frame_pos_x = 0;


MenuState menu_att_page(void)
{	
	float roll, pitch, yaw;
	
	ESCNode::ESCStatusCache esc_status[Max_ESC_Num]={0};
	
	auto& key1 = Board::getKey1();
	auto& key2 = Board::getKey2();
	auto& key3 = Board::getKey3();
	auto& esc_node = Board::getESCNode();
	
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
	OLED_ShowString(0,48,"rpm:");
	
	OLED_ShowFloat(58, 0,roll);
	OLED_ShowFloat(58, 16,pitch);
	OLED_ShowFloat(58, 32,yaw);
	if(esc_node.get_esc_status(ESC1_Index,esc_status[ESC1_Index]))
		OLED_ShowFloat(58, 48,esc_status[ESC1_Index].rpm);
	
	OLED_Update();
	
	if(key3.getEvent() == Key::Event::ShortPress)
		return MenuState::MAIN_PAGE_STATE;
	
	return MenuState::ATT_PAGE_STATE;
}

MenuState menu_esc_page(void)
{
	

	return MenuState::CONTROL_PAGE_STATE;
}
MenuState menu_firmware_page(void)
{
	

	return MenuState::FIRMWARE_PAGE_STATE;
}



