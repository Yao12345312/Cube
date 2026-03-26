#pragma once

#include "stm32h7xx_hal.h"

bool SPI_SetMode(SPI_HandleTypeDef* hspi, uint32_t polarity, uint32_t phase);

void SPI_Unlock();