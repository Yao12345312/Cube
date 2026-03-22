#include "led.hpp"

/* ================= 构造函数 ================= */

LedPwm::LedPwm(TIM_HandleTypeDef* htim)
    : m_htim(htim)
    , m_isInitialized(false)
    , m_maxPulse(0)
    , m_blinkTaskHandle(NULL)
    , m_blinkRed(0)
    , m_blinkGreen(0)
    , m_blinkBlue(0)
    , m_blinkHalfPeriodMs(0)
    , m_blinkRunning(false)
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

/* ================= 闪烁任务函数（静态） ================= */

void LedPwm::blinkTaskFunc(void* parameter)
{
    LedPwm* pThis = static_cast<LedPwm*>(parameter);
    if (pThis != nullptr) {
        pThis->blinkTask();
    }
}

/* ================= 闪烁任务（成员函数） ================= */

void LedPwm::blinkTask(void)
{
    while (1) {
        // 执行闪烁
        while (m_blinkRunning) {
            // 点亮LED
            setRGB(m_blinkRed, m_blinkGreen, m_blinkBlue);
            
            // 延时半个周期
            osDelay(m_blinkHalfPeriodMs);
            
            if (!m_blinkRunning) break;
            
            // 熄灭LED
            setRGB(0, 0, 0);
            
            // 延时半个周期
            osDelay(m_blinkHalfPeriodMs);
        }
        
        // 闪烁结束，关闭LED
        turnOff();
    }
}

/* ================= 设置RGB闪烁（非阻塞） ================= */

void LedPwm::setRGBBlink(uint8_t red, uint8_t green, uint8_t blue, uint8_t frequency_hz)
{
    if (!m_isInitialized) return;
    
    // 停止当前闪烁
    stopBlink();
    
    // 参数校验
    if (frequency_hz == 0) {
        // 频率为0，直接常亮
        setRGB(red, green, blue);
        return;
    }
    
    // 计算半个周期（毫秒）
    uint16_t period_ms = 1000 / frequency_hz;
    m_blinkHalfPeriodMs = period_ms / 2;
    
    // 保存闪烁参数
    m_blinkRed = red;
    m_blinkGreen = green;
    m_blinkBlue = blue;
    m_blinkRunning = true;
    
    // 创建任务（如果还未创建）
    if (m_blinkTaskHandle == NULL) {
        osThreadAttr_t taskAttr = {
            .name = "LedBlink",
            .stack_size = 128,
            .priority = osPriorityNormal,
        };
        m_blinkTaskHandle = osThreadNew(blinkTaskFunc, this, &taskAttr);
        if (m_blinkTaskHandle == NULL) {
            return;
        }
    }
    
    // 发送通知启动闪烁任务
    osThreadFlagsSet(m_blinkTaskHandle, 0x01);
}

/* ================= 停止闪烁 ================= */

void LedPwm::stopBlink(void)
{
    // 停止闪烁标志
    m_blinkRunning = false;
    
    // 关闭LED
    turnOff();
}

/* ================= 查询是否正在闪烁 ================= */

bool LedPwm::isBlinking(void)
{
    return m_blinkRunning;
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