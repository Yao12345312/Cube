#include "KT6368A.hpp"
#include "uart3Driver.hpp"
#include <string.h>
#include <stdio.h>

/* ================= 全局实例 ================= */
BluetoothDriver* g_bt = nullptr;

/* ================= 构造函数 ================= */

BluetoothDriver::BluetoothDriver(Uart1Driver* uart)
    : m_uart(uart)
{
    g_bt = this;
}

/* ================= 初始化 ================= */

bool BluetoothDriver::init()
{
    if (m_uart == nullptr) return false;

    /* 检查AT队列是否存在 */
    if (m_uart->getAtQueue() == nullptr)
        return false;

    return true;
}


/* ================= 读取一行 ================= */

uint16_t BluetoothDriver::readLine(char* buf,
                                   uint16_t buf_len,
                                   uint32_t timeout_ms)
{
    uint32_t start = osKernelGetTickCount();
    uint16_t idx = 0;
    uint8_t ch;

    while ((osKernelGetTickCount() - start) < timeout_ms)
    {
        if (osMessageQueueGet(
                m_uart->getAtQueue(),
                &ch,
                NULL,
                10) == osOK)
        {
            if (idx < buf_len - 1)
                buf[idx++] = ch;

            if (ch == '\n')
                break;
        }
    }

    buf[idx] = '\0';
    return idx;
}

/* ================= 发送AT并等待响应 ================= */

uint16_t BluetoothDriver::sendAT(const char* cmd, char* resp, uint16_t len)
{
    m_uart->atSend((uint8_t*)cmd, strlen(cmd));

    osDelay(100);

    return m_uart->atRecv((uint8_t*)resp, len, 500);
}


/* ================= 自动波特率扫描 ================= */

bool BluetoothDriver::autoBaudScan()
{
    const uint32_t baud_table[] = {
        2400,4800,9600,19200,38400,57600,
        115200,230400,460800,921600
    };

    char resp[64];

    m_uart->enterAtMode();   // 进入非DMA模式

    for (uint8_t i = 0; i < sizeof(baud_table)/sizeof(uint32_t); i++)
    {
        uint32_t baud = baud_table[i];

        UART_HandleTypeDef* huart = m_uart->getHandle();

        huart->Init.BaudRate = baud;

        HAL_UART_DeInit(huart);
        HAL_UART_Init(huart);

        osDelay(300);

        memset(resp, 0, sizeof(resp));

        uint16_t len = sendAT("AT+QT\r\n", resp, sizeof(resp));

        printf("baud=%ld resp=%s\n", baud, resp);

        if (len > 0 && strstr(resp, "Q") != nullptr)
        {
            sendAT("AT+BMKT6368A\r\n", resp, sizeof(resp));
            osDelay(200);

            sendAT("AT+CR00\r\n", resp, sizeof(resp));
            osDelay(200);

            sendAT("AT+CT09\r\n", resp, sizeof(resp));
            osDelay(200);

            sendAT("AT+CZ\r\n", resp, sizeof(resp));
            osDelay(300);

            /* 切最终波特率 */
            huart->Init.BaudRate = 460800;

            HAL_UART_DeInit(huart);
            HAL_UART_Init(huart);

            m_uart->exitAtMode();   // 恢复DMA

            return true;
        }
    }

    m_uart->exitAtMode();

    return false;
}

