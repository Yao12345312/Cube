#include "led.hpp"

/* ================= 构造函数 ================= */

LedPwm::LedPwm(TIM_HandleTypeDef* htim)
    : m_htim(htim)
    , m_isInitialized(false)
    , m_maxPulse(0)
{
}

/* ================= 初始化 ================= */

bool LedPwm::init()
{
    if (m_isInitialized) {
        return true;
    }
    
    if (m_htim == nullptr) {
        return false;
    }
    
    // 获取ARR值（最大脉冲数） 
    m_maxPulse = __HAL_TIM_GET_AUTORELOAD(m_htim);
    
    // 启动PWM输出 [citation:8]
    HAL_TIM_PWM_Start(m_htim, BLUE_CHANNEL);   // PA1 - 蓝色
    HAL_TIM_PWM_Start(m_htim, GREEN_CHANNEL);  // PA2 - 绿色
    HAL_TIM_PWM_Start(m_htim, RED_CHANNEL);    // PA3 - 红色
    
    // 初始关闭所有LED
    turnOff();
    
    m_isInitialized = true;
    return true;
}

/* ================= 百分比转脉冲值 ================= */

uint32_t LedPwm::percentToPulse(uint8_t percent)
{
    if (percent > 100) percent = 100;
    // 映射公式：0-100% → 0 - m_maxPulse 
    return (uint32_t)((float)percent / 100.0f * m_maxPulse);
}

/* ================= 设置RGB ================= */

void LedPwm::setRGB(uint8_t red, uint8_t green, uint8_t blue)
{
    if (!m_isInitialized) return;
    
    setRed(red);
    setGreen(green);
    setBlue(blue);
}

/* ================= 单独设置各通道 ================= */

void LedPwm::setRed(uint8_t value)
{
    if (!m_isInitialized) return;
    
    uint32_t pulse = percentToPulse(value);
    // 直接修改CCR寄存器值 
    __HAL_TIM_SET_COMPARE(m_htim, RED_CHANNEL, pulse);
}

void LedPwm::setGreen(uint8_t value)
{
    if (!m_isInitialized) return;
    
    uint32_t pulse = percentToPulse(value);
    __HAL_TIM_SET_COMPARE(m_htim, GREEN_CHANNEL, pulse);
}

void LedPwm::setBlue(uint8_t value)
{
    if (!m_isInitialized) return;
    
    uint32_t pulse = percentToPulse(value);
    __HAL_TIM_SET_COMPARE(m_htim, BLUE_CHANNEL, pulse);
}

/* ================= 关闭所有LED ================= */

void LedPwm::turnOff()
{
    setRGB(0, 0, 0);
}