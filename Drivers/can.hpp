#pragma once

#include "stm32h7xx_hal.h"
#include "canard.h"
#include <cstdint>


class UavcanCanDriver
{
public:
    UavcanCanDriver(FDCAN_HandleTypeDef* hfdcan);

    bool init();
	//…œ≤„∞Û∂®canard
	void attach_canard(CanardInstance* canard);

    void process_tx(uint8_t max_frams);
    void process_rx(uint8_t max_frams);

    uint64_t micros64();
	
	void CAN_Send_Test();
	void CAN_Receive_Test();
private:
    void DWT_Init(void);

    FDCAN_HandleTypeDef* hfdcan_;
    CanardInstance* canard_;
};