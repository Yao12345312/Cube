#pragma once

#include <stdint.h>

enum class MenuState : uint8_t { //声明底层存储类型为uint8_t
	
	MAIN_PAGE_STATE = 1,
    ATT_PAGE_STATE = 2,
    CONTROL_PAGE_STATE = 3,
    FIRMWARE_PAGE_STATE = 4
	
};


MenuState menu_att_page(void);
MenuState menu_esc_page(void);
MenuState menu_firmware_page(void);











