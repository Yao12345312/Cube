#pragma once

#include "stm32h7xx_hal.h"

class Buzzer
{
public:
    Buzzer(TIM_HandleTypeDef *htim, uint32_t channel);

    void init();
    void on();
    void off();

    void setFrequency(uint32_t freq);
    void setDuty(float duty); // 0~1

    void beep(uint32_t freq, uint32_t duration);

private:
    TIM_HandleTypeDef *m_htim;
    uint32_t m_channel;
};