#pragma once

#include "uart1Driver.hpp"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

class BluetoothDriver
{
public:
	//类构造函数
    BluetoothDriver(Uart1Driver* uart);

    bool init();
    //发送AT指令
    uint16_t sendAT(const char* cmd, char* resp, uint16_t len);
	//蓝牙串口波特率扫频函数
    bool autoBaudScan();
	//获取蓝牙状态
	bool get_Bluetooth_Status(){return bluetooth_finish_initialization;}
	//读取一行接收到的数据
    uint16_t readLine(char* buf, uint16_t buf_len, uint32_t timeout_ms);
	
private:
    Uart1Driver* m_uart;

    uint8_t m_rxByte;

	volatile bool bluetooth_finish_initialization=false;
};

extern BluetoothDriver* g_bt;
