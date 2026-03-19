#include "uart1Driver.hpp"
#include <string.h>

Uart1Driver* g_uart1Driver = nullptr;

/* ================= 构造 ================= */

Uart1Driver::Uart1Driver(UART_HandleTypeDef* huart)
    : m_huart(huart)
    , m_lastPos(0)
    , m_atQueue(nullptr)
    , m_mavQueue(nullptr)
    , m_inAtMode(false)
{
    g_uart1Driver = this;
}

/* ================= 初始化 ================= */

bool Uart1Driver::init()
{
    /* 创建队列 */
    m_atQueue = osMessageQueueNew(256, sizeof(uint8_t), NULL);
    m_mutex = osMutexNew(NULL);

    if (!m_atQueue || !m_mavQueue) return false;

    /* 创建互斥锁 */
    osMutexAttr_t attr = {
        "uart1_mutex",
        osMutexRecursive | osMutexPrioInherit,
        nullptr,
        0
    };
    m_mutex = osMutexNew(&attr);

    startDMA();

    /* 开启IDLE中断 */
    __HAL_UART_ENABLE_IT(m_huart, UART_IT_IDLE);

    return true;
}

/* ================= 启动DMA ================= */

void Uart1Driver::startDMA()
{
    HAL_UART_Receive_DMA(m_huart, m_dmaBuf, UART1_DMA_RX_BUF_SIZE);

    /* 开启DMA循环模式 */
    __HAL_DMA_DISABLE_IT(m_huart->hdmarx, DMA_IT_HT);
}

/* ================= 发送 ================= */

void Uart1Driver::send(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return;

    osMutexAcquire(m_mutex, osWaitForever);
    HAL_UART_Transmit(m_huart, (uint8_t*)data, len, 1000);
    osMutexRelease(m_mutex);
}

/* ================= AT模式 ================= */

void Uart1Driver::setAtMode(bool enable)
{
    m_inAtMode = enable;
}

/* ================= IDLE中断核心 ================= */

void Uart1Driver::irqHandler()
{
    if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);

        uint16_t dmaRemain = __HAL_DMA_GET_COUNTER(m_huart->hdmarx);
        uint16_t newPos = UART1_DMA_RX_BUF_SIZE - dmaRemain;
        uint16_t start = m_lastPos;

        if (newPos == start) return;

        /* ================= AT模式 ================= */
        if (isAtMode())
        {
            if (newPos > start)
            {
                for (uint16_t i = start; i < newPos; i++)
                {
                    uint8_t ch = m_dmaBuf[i];
                    osMessageQueuePut(m_atQueue, &ch, 0, 0); // ISR必须timeout=0
                }
            }
            else
            {
                for (uint16_t i = start; i < UART1_DMA_RX_BUF_SIZE; i++)
                {
                    uint8_t ch = m_dmaBuf[i];
                    osMessageQueuePut(m_atQueue, &ch, 0, 0);
                }
                for (uint16_t i = 0; i < newPos; i++)
                {
                    uint8_t ch = m_dmaBuf[i];
                    osMessageQueuePut(m_atQueue, &ch, 0, 0);
                }
            }

            m_lastPos = newPos;
            return;
        }

        /* ================= MAVLink模式 ================= */
        MavRxFrame_t frame;
        frame.len = 0;

        if (newPos > start)
        {
            uint16_t len = newPos - start;
            if (len > MAV_RX_BUF_LEN) len = MAV_RX_BUF_LEN;

            memcpy(frame.data, &m_dmaBuf[start], len);
            frame.len = len;
        }
        else
        {
            uint16_t part1 = UART1_DMA_RX_BUF_SIZE - start;
            uint16_t total = part1 + newPos;

            if (total > MAV_RX_BUF_LEN) total = MAV_RX_BUF_LEN;
            if (part1 > total) part1 = total;

            memcpy(frame.data, &m_dmaBuf[start], part1);

            if (total > part1)
            {
                memcpy(&frame.data[part1], &m_dmaBuf[0], total - part1);
            }

            frame.len = total;
        }

        m_lastPos = newPos;

        if (frame.len > 0)
        {
            osMessageQueuePut(m_mavQueue, &frame, 0, 0); // ISR必须timeout=0
        }
    }
}

extern "C" void UART1_IdleCallback(void)
{
    if (g_uart1Driver)
    {
        g_uart1Driver->irqHandler();
    }
}