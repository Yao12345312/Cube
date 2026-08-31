#pragma once

#include <cmsis_os2.h>
#include <stdint.h>

// =============================================================================
// OLED 菜单 (移植自参考工程 USER/menu.cpp, 适配平衡小车驱动与控制模式)
//
// 页面状态机:
//   MAIN_PAGE  -> 主菜单 (姿态/任务控制/固件版本/陀螺仪校准/电调编号设置)
//   ATT_PAGE   -> 姿态显示 (roll/pitch/yaw)
//   CONTROL_PAGE -> 任务选择 (平衡模式/单点保持/转速测试)
//   FIRMWARE_PAGE -> 固件版本
//   CONTROL_RUNNING -> 任务运行中 (短按 key3 退出)
//   GYRO_CALIB_PAGE -> 陀螺仪零偏校准
//   ESC_CALIB_PAGE  -> 电调调准 (校准电调1/2/3, 全部校准)
//   ESC_INDEX_PAGE  -> 电调编号设置 (2 路)
//
// 按键: key1=上移, key2=下移, key3=短按返回/长按确认
// =============================================================================

enum class MenuState : uint8_t
{
    MAIN_PAGE_STATE       = 1,
    ATT_PAGE_STATE        = 2,
    CONTROL_PAGE_STATE    = 3,
    FIRMWARE_PAGE_STATE   = 4,
    CONTROL_RUNNING_STATE = 5,
    GYRO_CALIB_PAGE_STATE = 6,
    ESC_INDEX_PAGE_STATE  = 7,
    ESC_CALIB_PAGE_STATE  = 8,
};

MenuState menu_att_page(void);
MenuState menu_control_page(void);
MenuState menu_firmware_page(void);
MenuState menu_control_running_page(void);
MenuState menu_gyro_calib_page(void);
MenuState menu_esc_index_page(void);
MenuState menu_esc_calib_page(void);
