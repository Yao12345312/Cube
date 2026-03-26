#include "DebugPort.hpp"
#include "uart3Driver.hpp"

DebugPort::Type DebugPort::m_type = DebugPort::Type::UART;

extern Uart3Driver* g_uart3Driver;

void DebugPort::init(Type type)
{
    m_type = type;
}

void DebugPort::write(uint8_t* data, uint16_t len)
{
    if (m_type == Type::UART)
    {
        if (g_uart3Driver)
        {
            HAL_UART_Transmit(
                g_uart3Driver->getHandle(),
                data,
                len,
                HAL_MAX_DELAY
            );
        }
    }
    else
    {
        CDC_Transmit_FS(data, len);
    }
}
