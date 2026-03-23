#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include <stdio.h>

class Uart3Driver {
public:
    /**
     * @brief 构造函数
     * @param huart HAL库的UART句柄指针（来自CubeMX生成的&huart3）
     */
    Uart3Driver(UART_HandleTypeDef* huart);
    
    /**
     * @brief 初始化UART3
     * @return true 成功，false 失败
     */
    bool init();
    
    /**
     * @brief 发送字符串（阻塞模式）
     * @param str 要发送的字符串
     */
    void sendString(const char* str);
    
    /**
     * @brief 发送字符串（带超时）
     * @param str 要发送的字符串
     * @param timeout 超时时间（ms）
     */
    void sendString(const char* str, uint32_t timeout);
    
    /**
     * @brief 获取UART句柄
     * @return UART_HandleTypeDef* 
     */
    UART_HandleTypeDef* getHandle() { return m_huart; }
	
	osMutexId_t getMutex(){return m_mutex;}
	
private:
    UART_HandleTypeDef* m_huart;      // UART句柄
    bool m_isInitialized;              // 初始化标志
    osMutexId_t m_mutex;               // 互斥锁（用于多任务安全）
    
    /**
     * @brief 创建互斥锁
     */
    void createMutex();
};

/**
 * @brief C语言接口函数，用于printf重定向
 * @note 需要根据编译器选择正确的实现方式
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 重定向fputc函数到UART3
 * @note 这是Keil MDK标准库的重定向方法
 */
int fputc(int ch, FILE* f);

/**
 * @brief STM32CubeIDE (GCC)的重定向函数
 */
int __io_putchar(int ch);

#ifdef __cplusplus
}
#endif

// 全局UART3驱动实例指针，供fputc使用
extern Uart3Driver* g_uart3Driver;

