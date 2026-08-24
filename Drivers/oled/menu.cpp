#include "menu.hpp"
#include "drv_oled.hpp"
#include "drv_key.hpp"
#include "drv_CubeFOC.hpp"
#include "board.hpp"
#include "task_Control.hpp"

#include <cstdio>

// =============================================================================
// 姿态显示页: roll / pitch / yaw (弧度)
// =============================================================================
MenuState menu_att_page(void)
{
    auto *oled = drv_oled();
    auto *key3 = drv_key(BOARD_KEY_3);

    float roll = 0, pitch = 0, yaw = 0;
    if (g_att_mutex != NULL && osMutexAcquire(g_att_mutex, 0) == osOK)
    {
        roll  = g_attitude.roll;
        pitch = g_attitude.pitch;
        yaw   = g_attitude.yaw;
        osMutexRelease(g_att_mutex);
    }

    oled->showString(0, 16, "roll:");
    oled->showString(0, 32, "pitch:");
    oled->showString(0, 48, "yaw:");

    oled->showFloat(58, 16, roll);
    oled->showFloat(58, 32, pitch);
    oled->showFloat(58, 48, yaw);

    if (key3 && key3->getEvent() == DrvKey::Event::ShortPress)
        return MenuState::MAIN_PAGE_STATE;

    return MenuState::ATT_PAGE_STATE;
}

// =============================================================================
// 任务选择页: 平衡模式(0) / 单点保持(1) / 转速测试(4)
// =============================================================================
MenuState menu_control_page(void)
{
    auto *oled = drv_oled();
    auto *key1 = drv_key(BOARD_KEY_1);
    auto *key2 = drv_key(BOARD_KEY_2);
    auto *key3 = drv_key(BOARD_KEY_3);

    // 菜单项 -> 控制模式号映射 (与 mavlink 命令一致)
    const int8_t MENU_COUNT = 5;
    const int16_t VISIBLE_Y[3] = {18, 32, 46};
    const char *menu_names[5] = {"单边控制", "单点控制", "单边起跳", "单点起跳", "转速测试"};
    const uint8_t mode_map[5] = {0, 1, 2, 3, 4};

    static int8_t selected_idx = 0;
    static int8_t scroll_top   = 0;

    for (int8_t i = 0; i < 3; i++)
    {
        int8_t item_idx = scroll_top + i;
        if (item_idx < MENU_COUNT)
            oled->showChinese(0, VISIBLE_Y[i], menu_names[item_idx]);
    }

    if (key1 && key1->getEvent() == DrvKey::Event::ShortPress)
    {
        selected_idx = (int8_t)((selected_idx - 1 + MENU_COUNT) % MENU_COUNT);
        if (selected_idx < scroll_top)                 scroll_top = selected_idx;
        if (selected_idx >= scroll_top + 3)            scroll_top = (int8_t)(selected_idx - 2);
    }

    if (key2 && key2->getEvent() == DrvKey::Event::ShortPress)
    {
        selected_idx = (int8_t)((selected_idx + 1) % MENU_COUNT);
        if (selected_idx < scroll_top)                 scroll_top = selected_idx;
        if (selected_idx >= scroll_top + 3)            scroll_top = (int8_t)(selected_idx - 2);
    }

    int16_t draw_frame_y_pos = VISIBLE_Y[selected_idx - scroll_top];
    oled->drawRectangle(0, draw_frame_y_pos, 120, 15, 0);

    if (key3)
    {
        DrvKey::Event evt = key3->getEvent();
        if (evt == DrvKey::Event::ShortPress)
            return MenuState::MAIN_PAGE_STATE;

        if (evt == DrvKey::Event::LongPress)
        {
            //开启电调 MOS 管电源 (PE0)
            board_esc_power_enable();

            g_selected_control_mode = mode_map[selected_idx];
            if (g_controlModeSem != NULL)
                osSemaphoreRelease(g_controlModeSem);

            return MenuState::CONTROL_RUNNING_STATE;
        }
    }

    return MenuState::CONTROL_PAGE_STATE;
}

// =============================================================================
// 固件版本页
// =============================================================================
MenuState menu_firmware_page(void)
{
    auto *oled = drv_oled();
    auto *key3 = drv_key(BOARD_KEY_3);

    oled->showString(0, 16, "Version: V1.1.0");

    if (key3 && key3->getEvent() == DrvKey::Event::ShortPress)
        return MenuState::MAIN_PAGE_STATE;

    return MenuState::FIRMWARE_PAGE_STATE;
}

// =============================================================================
// 任务运行中页: 显示当前模式, 短按 key3 退出
// =============================================================================
MenuState menu_control_running_page(void)
{
    auto *oled = drv_oled();
    auto *key3 = drv_key(BOARD_KEY_3);

    oled->showChinese(0, 0, "当前模式:");

    switch (g_selected_control_mode)
    {
        case 0:  oled->showChinese(0, 18, "单边控制"); break;
        case 1:  oled->showChinese(0, 18, "单点控制"); break;
        case 2:  oled->showChinese(0, 18, "单边起跳"); break;
        case 3:  oled->showChinese(0, 18, "单点起跳"); break;
        case 4:  oled->showChinese(0, 18, "转速测试"); break;
        default: oled->showChinese(0, 18, "未选择");   break;
    }

    oled->showString(0, 40, "key3: back");

    if (key3)
    {
        // 重新采样按键, 避免控制任务抢占导致事件丢失
        key3->update();
        DrvKey::Event evt = key3->getEvent();
        if (evt == DrvKey::Event::ShortPress)
        {
            //关闭电调 MOS 管电源 (PE0)
            board_esc_power_disable();

            g_selected_control_mode = 0xFF;   // 退出任务模式
            if (g_controlModeSem != NULL)
                osSemaphoreRelease(g_controlModeSem);

            return MenuState::CONTROL_PAGE_STATE;
        }
    }

    return MenuState::CONTROL_RUNNING_STATE;
}

// =============================================================================
// 陀螺仪零偏校准页: 长按 key3 启动校准
// =============================================================================
MenuState menu_gyro_calib_page(void)
{
    auto *oled = drv_oled();
    auto *key3 = drv_key(BOARD_KEY_3);

    oled->showChinese(0, 0, "陀螺仪校准");

    oled->showChinese(0, 18, "状态:");
    if (g_gyro_calibrated)
        oled->showChinese(36, 18, "已校准");
    else
        oled->showChinese(36, 18, "未校准");

    oled->showChinese(0, 36, "长按key3开始");

    if (key3)
    {
        DrvKey::Event evt = key3->getEvent();
        if (evt == DrvKey::Event::ShortPress)
            return MenuState::MAIN_PAGE_STATE;

        if (evt == DrvKey::Event::LongPress)
        {
            if (g_calibSem != NULL)
            {
                g_calib_command = 1;       // 陀螺仪零偏校准
                osSemaphoreRelease(g_calibSem);
            }
        }
    }

    return MenuState::GYRO_CALIB_PAGE_STATE;
}

// =============================================================================
// 电调编号设置页: 3 路电调
// 注意: 发送设置命令时, CAN 总线上务必只接目标电调, 否则编号会被覆盖
// =============================================================================
MenuState menu_esc_index_page(void)
{
    auto *oled  = drv_oled();
    auto *esc   = drv_cubefoc();
    auto *key1  = drv_key(BOARD_KEY_1);
    auto *key2  = drv_key(BOARD_KEY_2);
    auto *key3  = drv_key(BOARD_KEY_3);

    const int8_t MENU_COUNT = 3;
    const int16_t VISIBLE_Y[3] = {18, 32, 46};
    const char *menu_names[MENU_COUNT] = {"设为电调1", "设为电调2", "设为电调3"};

    static int8_t selected_idx = 0;
    static int8_t scroll_top   = 0;
    static bool   set_success  = false;

    //进入电调编号设置页: 开启电调 MOS 电源 (PE0)
    //电调需上电才能接收编号设置命令, 每帧幂等写入 (仅置位 GPIO)
    board_esc_power_enable();

    if (!set_success)
    {
        for (int8_t i = 0; i < 3; i++)
        {
            int8_t item_idx = scroll_top + i;
            if (item_idx < MENU_COUNT)
                oled->showChinese(0, VISIBLE_Y[i], menu_names[item_idx]);
        }

        if (key1 && key1->getEvent() == DrvKey::Event::ShortPress)
        {
            selected_idx = (int8_t)((selected_idx - 1 + MENU_COUNT) % MENU_COUNT);
            if (selected_idx < scroll_top)      scroll_top = selected_idx;
            if (selected_idx >= scroll_top + 3) scroll_top = (int8_t)(selected_idx - 2);
        }

        if (key2 && key2->getEvent() == DrvKey::Event::ShortPress)
        {
            selected_idx = (int8_t)((selected_idx + 1) % MENU_COUNT);
            if (selected_idx < scroll_top)      scroll_top = selected_idx;
            if (selected_idx >= scroll_top + 3) scroll_top = (int8_t)(selected_idx - 2);
        }

        int16_t draw_frame_y_pos = VISIBLE_Y[selected_idx - scroll_top];
        oled->drawRectangle(0, draw_frame_y_pos, 120, 15, 0);

        if (key3)
        {
            DrvKey::Event evt = key3->getEvent();
            if (evt == DrvKey::Event::ShortPress)
            {
                //退出电调编号设置页: 关闭电调 MOS 电源 (PE0)
                board_esc_power_disable();

                selected_idx = 0;
                scroll_top   = 0;
                return MenuState::MAIN_PAGE_STATE;
            }

            if (evt == DrvKey::Event::LongPress && esc)
            {
                // 目标电调编号 (1-based)
                uint8_t targets[MENU_COUNT] = {ESC1_Index + 1, ESC2_Index + 1, ESC3_Index + 1};
                esc->set_esc_index_command(targets[selected_idx]);
                set_success = true;
            }
        }
    }
    else
    {
        oled->showChinese(0, 18, "设置成功");
        if (key3 && key3->getEvent() == DrvKey::Event::ShortPress)
        {
            //退出电调编号设置页: 关闭电调 MOS 电源 (PE0)
            board_esc_power_disable();

            selected_idx = 0;
            scroll_top   = 0;
            set_success  = false;
            return MenuState::MAIN_PAGE_STATE;
        }
    }

    return MenuState::ESC_INDEX_PAGE_STATE;
}
