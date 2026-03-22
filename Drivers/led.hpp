#pragma once

#include "stm32h7xx_hal.h"
#include <cstdint>
#include "cmsis_os2.h"
/**
 * @brief RGB LED PWM驱动类（使用TIM2通道2/3/4）
 * 
 * 三路PWM总和100的设置方法：
 * 设置R/G/B值（0-100），内部自动映射到CCR寄存器（0-999）
 * 例如：setRGB(50, 30, 20) 表示占空比分别为50%、30%、20%
 */
class LedPwm {
public:
    /**
     * @brief 构造函数
     * @param htim HAL库的TIM句柄指针（来自CubeMX生成的&htim2）
     */
    LedPwm(TIM_HandleTypeDef* htim);
    
    /**
     * @brief 初始化PWM输出
     * @return true 成功，false 失败
     */
    bool init();
    
    /**
     * @brief 设置RGB三路PWM占空比（总和100的方式）
     * @param red   红色占比 (0-100)
     * @param green 绿色占比 (0-100)
     * @param blue  蓝色占比 (0-100)
     * @note 三路总和不需要严格等于100，但建议总和≤100避免过亮
     */
    void setRGB(uint8_t red, uint8_t green, uint8_t blue);
	/**
     * @brief 设置RGB三路PWM占空比以特定频率闪烁（总和100的方式）
     * @param red   红色占比 (0-100)
     * @param green 绿色占比 (0-100)
     * @param blue  蓝色占比 (0-100)
     * @note 三路总和不需要严格等于100，但建议总和≤100避免过亮
     */

    void setRGBBlink(uint8_t red, uint8_t green, uint8_t blue, uint8_t frequency_hz);
	
	void stopBlink(void);
	
	bool isBlinking(void);
	
    /**
     * @brief 单独设置红色PWM（PA3/TIM2_CH4）
     * @param value 占空比 (0-100)
     */
    void setRed(uint8_t value);
    
    /**
     * @brief 单独设置绿色PWM（PA2/TIM2_CH3）
     * @param value 占空比 (0-100)
     */
    void setGreen(uint8_t value);
    
    /**
     * @brief 单独设置蓝色PWM（PA1/TIM2_CH2）
     * @param value 占空比 (0-100)
     */
    void setBlue(uint8_t value);
    
    /**
     * @brief 关闭所有LED
     */
    void turnOff();
    
    /**
     * @brief 获取TIM句柄
     */
    TIM_HandleTypeDef* getHandle() { return m_htim; }

private:
	//闪烁任务函数
    static void blinkTaskFunc(void* parameter);
    void blinkTask(void);
    
    osThreadId_t m_blinkTaskHandle;
    uint8_t m_blinkRed;
    uint8_t m_blinkGreen;
    uint8_t m_blinkBlue;
    uint16_t m_blinkHalfPeriodMs;
    volatile bool m_blinkRunning;

    TIM_HandleTypeDef* m_htim;      // TIM句柄
    bool m_isInitialized;            // 初始化标志
    uint16_t m_maxPulse;             // 最大脉冲值（ARR值）
    
    // 通道映射 
    static constexpr uint32_t RED_CHANNEL = TIM_CHANNEL_4;   // PA3
    static constexpr uint32_t GREEN_CHANNEL = TIM_CHANNEL_3; // PA2
    static constexpr uint32_t BLUE_CHANNEL = TIM_CHANNEL_2;  // PA1
    
	
    /**
     * @brief 将百分比(0-100)转换为脉冲值(0-m_maxPulse)
     */
    uint32_t percentToPulse(uint8_t percent);
};