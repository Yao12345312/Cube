#pragma once

#include <cmsis_os2.h>
#include <stdint.h>

enum class MenuState : uint8_t { //底层状态存储类型为uint8_t

	MAIN_PAGE_STATE = 1,
    ATT_PAGE_STATE = 2,
    CONTROL_PAGE_STATE = 3,
    FIRMWARE_PAGE_STATE = 4,
    CONTROL_RUNNING_STATE = 5,
    GYRO_CALIB_PAGE_STATE = 6,
    MAG_CALIB_PAGE_STATE = 7,
    ESC_INDEX_PAGE_STATE = 8

};


MenuState menu_att_page(void);
MenuState menu_task_page(void);
MenuState menu_firmware_page(void);
MenuState menu_control_running_page(void);
MenuState menu_gyro_calib_page(void);
MenuState menu_mag_calib_page(void);
MenuState menu_esc_index_page(void);

// 控制模式选择信号量
extern osSemaphoreId_t g_controlModeSem;
extern uint8_t g_selected_control_mode;  // 0=单边控制, 1=单点控制, 2=单边起跳, 3=单点起跳
extern uint8_t g_rc_control_mode;        // 遥控器控制模式 0xFF=无命令, 0=单边, 1=单点
extern float g_rc_manual_y;

// 校准信号量
extern osSemaphoreId_t g_calibSem;
extern uint8_t g_calib_command;          // 0=idle, 1=gyro, 2=mag
extern bool g_gyro_calibrated;
extern bool g_mag_calibrated;
