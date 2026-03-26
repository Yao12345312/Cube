#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <cstdint>
#include "uart3Driver.hpp"

class SPA06
{
public:
    SPA06(SPI_HandleTypeDef* hspi,
          GPIO_TypeDef* cs_port, uint16_t cs_pin);

    bool init();
    bool startBackgroundMode();

    bool read();

    float getPressurePa() const;
    float getTemperatureC() const;
	void Data_Debug();
private:
    /* 硬件 */
    SPI_HandleTypeDef* m_hspi;
    GPIO_TypeDef* m_csPort;
    uint16_t m_csPin;

    osMutexId_t m_mutex;

    /* 原始数据 */
    int32_t raw_pressure;
    int32_t raw_temp;

    /* 校准参数 */
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    int16_t c31, c40;

    /* 结果 */
    float pressure;
    float temperature;

private:
    void csLow();
    void csHigh();

    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    void writeReg(uint8_t reg, uint8_t val);

    void readCoefficients();
    void compensate();

    int32_t convert24bit(uint32_t val);
};