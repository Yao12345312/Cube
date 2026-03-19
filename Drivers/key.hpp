#pragma once

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <cstdint>

class Key {
public:
    enum class Event {
        None,
        ShortPress,
        LongPress
    };
    
    enum class ActiveLevel {
        High,   // 按下为高电平（如按键接VCC）
        Low     // 按下为低电平（如按键接GND，常用模式）
    };
    
    /**
     * @brief 按键构造函数
     * @param port GPIO端口
     * @param pin GPIO引脚
     * @param activeLevel 有效电平（按下时的电平）
     * @param longPressTime 长按时间(ms)
     * @param debounceTime 消抖时间(ms)
     */
    Key(GPIO_TypeDef* port,
        uint16_t pin,
        ActiveLevel activeLevel = ActiveLevel::Low,  // 默认低电平有效（常用）
        uint32_t longPressTime = 1000,
        uint32_t debounceTime = 50);
    
    ~Key();
    
    /**
     * @brief 更新按键状态（必须在固定周期调用，建议10ms）
     */
    void update();
    
    /**
     * @brief 获取按键事件（获取后自动清除）
     */
    Event getEvent();
    
    /**
     * @brief 获取当前稳定状态是否按下
     */
    bool isPressed() const;
    
    /**
     * @brief 获取当前物理状态（消抖前）
     */
    GPIO_PinState getPhyState() const { return m_lastPhyState; }

private:
    GPIO_TypeDef* m_port;
    uint16_t m_pin;
    ActiveLevel m_activeLevel;
    uint32_t m_longPressTime;    // 长按时间(ms)
    uint32_t m_debounceTime;     // 消抖时间(ms)
    
    GPIO_PinState m_lastPhyState;    // 上一次物理状态
    GPIO_PinState m_stableState;     // 消抖后的稳定状态
    TickType_t m_lastDebounceTick;   // 上次状态变化时刻
    TickType_t m_pressStartTick;      // 按下起始时刻
    
    bool m_longPressTriggered;   // 是否已触发长按
    Event m_event;               // 当前事件
    
    SemaphoreHandle_t m_mutex;   // 互斥锁
    
    /**
     * @brief 判断指定电平是否为有效（按下）状态
     */
    bool isActive(GPIO_PinState state) const;
    
    /**
     * @brief 计算tick差值（处理溢出）
     */
    TickType_t ticksDiff(TickType_t tick1, TickType_t tick2) const;
};