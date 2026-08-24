#include "drv_ina226.hpp"

DrvINA226::DrvINA226(BoardI2cPort port, uint8_t dev_addr)
{
    m_port     = port;
    m_dev_addr = dev_addr;
    bat_cells  = static_cast<uint32_t>(BatCellsNum::CELLS_4S);
}

bool DrvINA226::writeReg(uint8_t reg, uint16_t value)
{
    // INA226 寄存器为大端序: [reg, MSB, LSB]
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = (uint8_t)(value >> 8);
    buf[2] = (uint8_t)(value & 0xFF);

    return board_i2c_write(m_port, m_dev_addr, buf, 3, 0.1) == BOARD_OK;
}

bool DrvINA226::readReg(uint8_t reg, uint16_t &value)
{
    uint8_t buf[2];
    if (board_i2c_write_read(m_port, m_dev_addr, &reg, 1, buf, 2, 0.1) != BOARD_OK)
        return false;

    value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

void DrvINA226::init(void)
{
    // 配置寄存器 0x4527 (与参考工程一致)
    // 平均过采样 16 次, Bus/Shunt 转换时间 1.1ms, 模式: Bus+Shunt 连续
    writeReg(INA226_REG_CONFIG, 0x4527);
}

float DrvINA226::readBusVoltage(void)
{
    uint16_t raw;
    if (!readReg(INA226_REG_BUS, raw))
        return -1.0f;

    // Bus 电压分辨率 1.25mV/LSB
    return (float)raw * 0.00125f;
}

DrvINA226::BatState DrvINA226::getBatStatus(float voltage)
{
    if (voltage < bat_cells * LOW_POWER_VEL)
        return BatState::BAT_LOW_POWER;

    if (voltage < bat_cells * MID_POWER_VEL)
        return BatState::BAT_MID_POWER;

    if (voltage < bat_cells * HIGH_POWER_VEL)
        return BatState::BAT_HIGH_POWER;

    if (voltage < bat_cells * FULL_POWER_VEL)
        return BatState::BAT_FULL_POWER;

    return BatState::BAT_MEASURE_ERR;
}

// =============================================================================
// 全局访问函数
// =============================================================================

static DrvINA226 *g_drv_ina226 = nullptr;

void init_drv_ina226()
{
    if (g_drv_ina226)
        return;

    g_drv_ina226 = new DrvINA226(BOARD_I2C_INA226);
    if (g_drv_ina226)
        g_drv_ina226->init();
}

DrvINA226 *drv_ina226()
{
    return g_drv_ina226;
}
