#pragma once

#include "uart1Driver.hpp"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

class BluetoothDriver
{
public:
    BluetoothDriver(Uart1Driver* uart);

    bool init();

    uint16_t sendAT(const char* at_cmd,
                    char* resp_buf,
                    uint16_t resp_len,
                    uint32_t timeout_ms);

    bool autoBaudScan();

    uint16_t readLine(char* buf, uint16_t buf_len, uint32_t timeout_ms);
		
private:
    Uart1Driver* m_uart;

    uint8_t m_rxByte;


};

extern BluetoothDriver* g_bt;
