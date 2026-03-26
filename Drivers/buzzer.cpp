#include "buzzer.hpp"
#include "cmsis_os2.h"

extern uint32_t SystemCoreClock;

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint32_t channel)
{
    m_htim = htim;
    m_channel = channel;
}

void Buzzer::init()
{
    HAL_TIM_PWM_Start(m_htim, m_channel);
    off();
}

void Buzzer::on()
{
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, (__HAL_TIM_GET_AUTORELOAD(m_htim) / 2));
}

void Buzzer::off()
{
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, 0);
}

void Buzzer::setFrequency(uint32_t freq)
{
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq() * 2;

    uint32_t psc = 199;  // πÃ∂®
    uint32_t arr = (timer_clk / (psc + 1)) / freq - 1;
	//…Ë÷√∆µ¬ 
    __HAL_TIM_SET_PRESCALER(m_htim, psc);
    __HAL_TIM_SET_AUTORELOAD(m_htim, arr);
}

void Buzzer::setDuty(float duty)
{
    uint32_t compare = duty * (__HAL_TIM_GET_AUTORELOAD(m_htim));
    __HAL_TIM_SET_COMPARE(m_htim, m_channel, compare);
}

void Buzzer::beep(uint32_t freq, uint32_t duration)
{
    setFrequency(freq);
    setDuty(0.5f);
    osDelay(duration);
    off();
}