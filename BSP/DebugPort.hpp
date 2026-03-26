#pragma once

#include <stdint.h>
#include "usbd_cdc_if.h"

class DebugPort
{
public:
    enum class Type
    {
        UART,
        USB
    };

    static void init(Type type);
    static void write(uint8_t* data, uint16_t len);

private:
    static Type m_type;
};