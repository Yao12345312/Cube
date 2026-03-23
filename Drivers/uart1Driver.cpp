#include "uart1Driver.hpp"
#include <string.h>

extern "C" void Error_Handler(void);
	
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
	m_mavQueue = osMessageQueueNew(16, sizeof(MavRxFrame_t), NULL);
    m_atQueue = osMessageQueueNew(16, sizeof(uint8_t), NULL);
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

void Uart1Driver::enterAtMode()
{
    m_inAtMode = true;

    /* 停DMA */
    HAL_UART_DMAStop(m_huart);

    /* 等待发送完成 */
    while (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_TC) == RESET);

    /* 关闭UART */
    HAL_UART_DeInit(m_huart);

    /* 重新初始化（普通模式） */
    HAL_UART_Init(m_huart);
}

void Uart1Driver::exitAtMode()
{
   m_inAtMode = false;

    HAL_UART_DeInit(m_huart);
    HAL_UART_Init(m_huart);

    startDMA();
}

bool Uart1Driver::atSend(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return false;

    if (HAL_UART_Transmit(m_huart, (uint8_t*)data, len, 1000) != HAL_OK)
        return false;

    return true;
}

int Uart1Driver::atRecv(uint8_t* buf, uint16_t len, uint32_t timeout)
{
    if (!buf || len == 0) return 0;

    uint16_t idx = 0;
    uint8_t ch;

    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout)
    {
        if (HAL_UART_Receive(m_huart, &ch, 1, 50) == HAL_OK) // 50ms等待
        {
            buf[idx++] = ch;
            if (ch == '\n' || idx >= len)
                break;
        }
    }

    return idx;
}

/* ================= 启动DMA ================= */

void Uart1Driver::startDMA()
{	
	memset(m_dmaBuf, 0, UART1_DMA_RX_BUF_SIZE);
	m_lastPos = 0;
	
	if(HAL_UART_Receive_DMA(m_huart, m_dmaBuf, UART1_DMA_RX_BUF_SIZE) != HAL_OK)
    {
	Error_Handler();
	}

    /* 注意不能禁用DMA传输中断，否则DMA收不到数据 */
    //__HAL_DMA_DISABLE_IT(m_huart->hdmarx, DMA_IT_HT);
}
/* ================= 清空DMA状态 ================= */
void Uart1Driver::resetRx()
{
    m_lastPos = 0;
    memset(m_dmaBuf, 0, UART1_DMA_RX_BUF_SIZE);
}

/* ================= 发送 ================= */

void Uart1Driver::send(const uint8_t* data, uint16_t len)
{
    if (!data || len == 0) return;

    osMutexAcquire(m_mutex, osWaitForever);
	
    if(HAL_UART_Transmit_DMA(m_huart, (uint8_t*)data, len )!=HAL_OK)
	{
	Error_Handler();
	}
	
	while (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_TC) == RESET);
	
    osMutexRelease(m_mutex);
}

/* ================= AT模式 ================= */

void Uart1Driver::setAtMode(bool enable)
{
    m_inAtMode = enable;
}

/* ================= IDLE中断核心 ================= */

void Uart1Driver::irqHandler()
{	//判断串口是否进入空闲中断
    if (__HAL_UART_GET_FLAG(m_huart, UART_FLAG_IDLE))
    {	//清空中断标志位
        __HAL_UART_CLEAR_IDLEFLAG(m_huart);
		//计算已接收字节数=BUF_SIZE - 当前DMA剩余计数，获得新数据位置
        uint16_t dmaRemain = __HAL_DMA_GET_COUNTER(m_huart->hdmarx);
        uint16_t newPos = UART1_DMA_RX_BUF_SIZE - dmaRemain;
        uint16_t start = m_lastPos;
		//如果没有数据则返回
        if (newPos == start) return;

        /* ================= AT模式 ================= */
        if (isAtMode())
        {	
			//未发生环形环绕，数据连续
            if (newPos > start)
            {	
				//按顺序逐个字符放入消息队列
                for (uint16_t i = start; i < newPos; i++)
                {
                    uint8_t ch = m_dmaBuf[i];
                    osMessageQueuePut(m_atQueue, &ch, 0, 0); // ISR必须timeout=0，确保在中断中不会阻塞
                }
            }
			//发生环形环绕，先处理缓冲区末尾[start,UART1_DMA_RX_BUF_SIZE],再处理[0,newPos]
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
			//更新lastpose
            m_lastPos = newPos;
            return;
        }

        /* ================= MAVLink模式 ================= */
		//MAVLink属于二进制协议需要以帧为单位处理
        MavRxFrame_t frame;
        frame.len = 0;
		//未发生环绕
        if (newPos > start)
        {	
			//缓冲区保护：如果总数居超过MAV_RX_BUF_LEN，只取前MAV_RX_BUF_LEN个字节
            uint16_t len = newPos - start;
            if (len > MAV_RX_BUF_LEN) len = MAV_RX_BUF_LEN;
			//数据从DMA缓冲区存入frame
            memcpy(frame.data, &m_dmaBuf[start], len);
            frame.len = len;
        }
		//发生环绕，同样先处理尾部再处理头部
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
		//数据帧完整送入队列
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