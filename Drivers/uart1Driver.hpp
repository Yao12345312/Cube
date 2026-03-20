#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

/* ================= 配置 ================= */

#define UART1_DMA_RX_BUF_SIZE   512
#define MAV_RX_BUF_LEN          256

typedef struct {
    uint8_t data[MAV_RX_BUF_LEN];
    uint16_t len;
} MavRxFrame_t;

class Uart1Driver
{
public:
    Uart1Driver(UART_HandleTypeDef* huart);

    bool init();

    /* DMA接收启动 */
    void startDMA();
	/* DMA状态清空 */
	void resetRx();

    /* IDLE中断处理 */
    void irqHandler();

    /* 发送 */
    void send(const uint8_t* data, uint16_t len);

    /* AT模式控制 */
    void setAtMode(bool enable);

    /* 获取队列 */
    osMessageQueueId_t getMavQueue() { return m_mavQueue; }
	
	osMessageQueueId_t getAtQueue() { return m_atQueue; }
	
	UART_HandleTypeDef* getHandle() { return m_huart; }
	
	osMutexId_t getMutex() { return m_mutex; }
	
	bool isAtMode() const { return m_inAtMode; }

private:
    UART_HandleTypeDef* m_huart;

    /* DMA环形缓冲区 */
    uint8_t m_dmaBuf[UART1_DMA_RX_BUF_SIZE];
    volatile uint16_t m_lastPos;
	
	volatile bool m_inAtMode;

    /* 队列 */
    osMessageQueueId_t m_atQueue;
    osMessageQueueId_t m_mavQueue;

    /* 互斥 */
    osMutexId_t m_mutex;

};

/* 全局指针（用于ISR） */
extern Uart1Driver* g_uart1;