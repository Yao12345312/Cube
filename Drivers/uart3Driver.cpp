#include "uart3Driver.hpp"
#include <string.h>

// 全局变量定义
Uart3Driver* g_uart3Driver = nullptr;

/* ================= 构造函数 ================= */

Uart3Driver::Uart3Driver(UART_HandleTypeDef* huart)
    : m_huart(huart)
    , m_isInitialized(false)
    , m_mutex(nullptr)
{
    // 设置全局指针，供fputc使用
    g_uart3Driver = this;
}

/* ================= 初始化 ================= */

bool Uart3Driver::init()
{
    if (m_isInitialized) {
        return true;
    }
    
    if (m_huart == nullptr) {
        return false;
    }
    
    createMutex();
    
    m_isInitialized = true;
    return true;
}

/* ================= 创建互斥锁 ================= */

void Uart3Driver::createMutex()
{
    if (m_mutex == nullptr) {
        osMutexAttr_t mutexAttr = {
            "UART3_Mutex",
            osMutexRecursive | osMutexPrioInherit,
            nullptr,
            0
        };
        m_mutex = osMutexNew(&mutexAttr);
    }
}

/* ================= 发送字符串 ================= */

void Uart3Driver::sendString(const char* str)
{
    sendString(str, 1000);  // 默认超时1000ms
}

void Uart3Driver::sendString(const char* str, uint32_t timeout)
{
    if (!m_isInitialized || str == nullptr) {
        return;
    }
    
    uint16_t len = strlen(str);
    
    // 在FreeRTOS环境中使用互斥锁保护
    if (m_mutex != nullptr) {
        osMutexAcquire(m_mutex, osWaitForever);
    }
    
    HAL_UART_Transmit(m_huart, (uint8_t*)str, len, timeout);
    
    if (m_mutex != nullptr) {
        osMutexRelease(m_mutex);
    }
}

/* ================= printf重定向函数 ================= */

/**
 * @brief 重定向fputc函数到UART3
 * 
 * 这是最常用的重定向方法，适用于Keil MDK环境
 * 
 */
int fputc(int ch, FILE* f)
{
    if (g_uart3Driver != nullptr && 
        g_uart3Driver->getHandle() != nullptr) {
        
        UART_HandleTypeDef* huart = g_uart3Driver->getHandle();
        
        // 发送单个字符
        HAL_UART_Transmit(huart, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    }
    
    return ch;
}

/**
 * @brief STM32CubeIDE (GCC编译器)的重定向函数 
 */
int __io_putchar(int ch)
{
    return fputc(ch, nullptr);
}

/**
 * @brief 另一种常见的重定向函数形式（某些编译器使用）
 */
int _write(int file, char* ptr, int len)
{
    if (g_uart3Driver != nullptr && 
        g_uart3Driver->getHandle() != nullptr) {
        
        UART_HandleTypeDef* huart = g_uart3Driver->getHandle();
        HAL_UART_Transmit(huart, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    }
    
    return len;
}