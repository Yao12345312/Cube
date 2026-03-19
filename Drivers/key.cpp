#include "key.hpp"
#include <cstddef>

Key::Key(GPIO_TypeDef* port,
         uint16_t pin,
         ActiveLevel activeLevel,
         uint32_t longPressTime,
         uint32_t debounceTime)
    : m_port(port)
    , m_pin(pin)
    , m_activeLevel(activeLevel)
    , m_longPressTime(longPressTime)
    , m_debounceTime(debounceTime)
    , m_lastPhyState(GPIO_PIN_RESET)
    , m_stableState(GPIO_PIN_RESET)
    , m_lastDebounceTick(0)
    , m_pressStartTick(0)
    , m_longPressTriggered(false)
    , m_event(Event::None)
    , m_mutex(nullptr)
{
    // 创建互斥锁
    m_mutex = xSemaphoreCreateMutex();
    
    // 读取初始状态
    GPIO_PinState initState = HAL_GPIO_ReadPin(m_port, m_pin);
    m_lastPhyState = initState;
    m_stableState = initState;
    m_lastDebounceTick = xTaskGetTickCount();
}

Key::~Key()
{
    if (m_mutex != nullptr) {
        vSemaphoreDelete(m_mutex);
        m_mutex = nullptr;
    }
}

TickType_t Key::ticksDiff(TickType_t tick1, TickType_t tick2) const
{
    if (tick1 >= tick2) {
        return tick1 - tick2;
    } else {
        // 处理溢出
        return (UINT32_MAX - tick2 + 1) + tick1;
    }
}

bool Key::isActive(GPIO_PinState state) const
{
    if (m_activeLevel == ActiveLevel::High) {
        return state == GPIO_PIN_SET;
    } else {
        return state == GPIO_PIN_RESET;
    }
}

void Key::update()
{
    if (m_mutex != nullptr) {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    }
    
    GPIO_PinState currentState = HAL_GPIO_ReadPin(m_port, m_pin);
    TickType_t now = xTaskGetTickCount();
    
    // 消抖处理
    if (currentState != m_lastPhyState) {
        m_lastDebounceTick = now;
        m_lastPhyState = currentState;
    }
    
    if (ticksDiff(now, m_lastDebounceTick) >= pdMS_TO_TICKS(m_debounceTime)) {
        // 状态稳定
        if (m_stableState != currentState) {
            m_stableState = currentState;
            
            // 检测到按下（考虑极性）
            if (isActive(m_stableState)) {
                // 按下
                m_pressStartTick = now;
                m_longPressTriggered = false;
            } else {
                // 松开
                if (!m_longPressTriggered) {
                    // 如果没有触发过长按，就是短按
                    m_event = Event::ShortPress;
                }
            }
        }
    }
    
    // 长按检测（只在稳定按下状态检测）
    if (isActive(m_stableState) && !m_longPressTriggered) {
        if (ticksDiff(now, m_pressStartTick) >= pdMS_TO_TICKS(m_longPressTime)) {
            m_longPressTriggered = true;
            m_event = Event::LongPress;
        }
    }
    
    if (m_mutex != nullptr) {
        xSemaphoreGive(m_mutex);
    }
}

Key::Event Key::getEvent()
{
    Event e = Event::None;
    
    if (m_mutex != nullptr) {
        xSemaphoreTake(m_mutex, portMAX_DELAY);
    }
    
    e = m_event;
    m_event = Event::None;  // 清除事件
    
    if (m_mutex != nullptr) {
        xSemaphoreGive(m_mutex);
    }
    
    return e;
}

bool Key::isPressed() const
{
    return isActive(m_stableState);
}