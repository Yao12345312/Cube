#include "SPI_Manager.hpp"
#include "cmsis_os2.h"

// SPI Manager（全局）
static osMutexId_t spiMutex = osMutexNew(NULL);

static uint32_t currentPolarity = 0xFF;
static uint32_t currentPhase    = 0xFF;


bool SPI_SetMode(SPI_HandleTypeDef* hspi, uint32_t polarity, uint32_t phase)
{
    osMutexAcquire(spiMutex, osWaitForever);
	//如果模式相同则不切换
	if (currentPolarity == polarity && currentPhase == phase)
       {
        osMutexRelease(spiMutex);
        return true;
       } 
	
    /* 等待SPI空闲 */
    while (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY); 

    if(HAL_SPI_DeInit(hspi) != HAL_OK){return false;}
	
    hspi->Init.CLKPolarity = polarity;
    hspi->Init.CLKPhase    = phase;

    if(HAL_SPI_Init(hspi) != HAL_OK){return false;}
	
	currentPolarity = polarity;
    currentPhase    = phase;
	
	return true;
}

void SPI_Unlock()
{
    osMutexRelease(spiMutex);
}