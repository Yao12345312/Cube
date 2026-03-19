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

/* ================= 串口中断接收 ================= */

void BluetoothDriver::onRxByteIRQ()
{
    if (!m_uart) return;

    if (m_uart->isAtMode())
    {
        uint8_t ch = m_rxByte;

        osMessageQueuePut(
            m_uart->getAtQueue(),
            &ch,
            0,
            0   // ISR里必须timeout=0
        );
    }

    HAL_UART_Receive_IT(m_uart->getHandle(), &m_rxByte, 1);
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

uint16_t BluetoothDriver::sendAT(const char* at_cmd,
                                 char* resp_buf,
                                 uint16_t resp_len,
                                 uint32_t timeout_ms)
{
    if (!at_cmd) return 0;

    /* 互斥锁 */
    if (osMutexAcquire(m_uart->getMutex(), 500) != osOK)
        return 0;

    m_uart->setAtMode(true);

    /* 清空队列 */
    uint8_t tmp;
    while (osMessageQueueGet(m_uart->getAtQueue(), &tmp, NULL, 0) == osOK);

    /* 发送 */
    m_uart->send((uint8_t*)at_cmd, strlen(at_cmd));

    osDelay(200);

    uint16_t len = 0;

    if (resp_buf)
    {
        len = readLine(resp_buf, resp_len, timeout_ms);
    }

    m_uart->setAtMode(false);

    osMutexRelease(m_uart->getMutex());

    return len;
}

/* ================= 透传发送 ================= */

bool BluetoothDriver::sendData(uint8_t* data, uint16_t size)
{
    if (!data || size == 0) return false;

    HAL_UART_Transmit(m_uart->getHandle(), data, size, 1000);
    return true;
}
/* ================= 自动波特率扫描 ================= */

bool BluetoothDriver::autoBaudScan()
{
    const uint32_t baud_table[] = {
        2400,4800,9600,19200,38400,57600,
        115200,230400,460800,921600
    };

    char resp[64];

    for (uint8_t i = 0; i < sizeof(baud_table)/sizeof(uint32_t); i++)
    {
        uint32_t baud = baud_table[i];

        UART_HandleTypeDef* huart = m_uart->getHandle();

        huart->Init.BaudRate = baud;
        HAL_UART_Init(huart);

        osDelay(300);

        uint16_t len = sendAT("AT+QT\r\n", resp, sizeof(resp), 200);

        printf("baud=%ld resp=%s\n", baud, resp);

        if (len > 0 && strncmp(resp, "QT+", 3) == 0)
        {
            osDelay(200);

            sendAT("AT+BMKT6368A\r\n", resp, sizeof(resp), 200);
            osDelay(300);

            sendAT("AT+CR00\r\n", resp, sizeof(resp), 200);
            osDelay(300);

            sendAT("AT+CT09\r\n", resp, sizeof(resp), 200);
            osDelay(1000);

            sendAT("AT+CZ\r\n", resp, sizeof(resp), 200);
            osDelay(300);

            /* 切换最终波特率 */
            huart->Init.BaudRate = 460800;
            HAL_UART_Init(huart);

            osDelay(200);

            return true;
        }
    }

    return false;
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (g_bt)
        g_bt->onRxByteIRQ();
}