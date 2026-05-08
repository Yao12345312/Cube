#pragma once

#include <cmsis_os2.h>
#include <stdint.h>

enum class MenuState : uint8_t { //底层状态存储类型为uint8_t

	MAIN_PAGE_STATE = 1,
    ATT_PAGE_STATE = 2,
    CONTROL_PAGE_STATE = 3,
    FIRMWARE_PAGE_STATE = 4,
    CONTROL_RUNNING_STATE = 5

};


MenuState menu_att_page(void);
MenuState menu_task_page(void);
MenuState menu_firmware_page(void);
MenuState menu_control_running_page(void);

// 控制模式选择信号量
extern osSemaphoreId_t g_controlModeSem;
extern uint8_t g_selected_control_mode;  // 0=单边控制, 1=单点控制
