#pragma once

#include "board.hpp"
#include <stdint.h>

// INA226 默认 I2C 7位地址 (board 层内部左移1位)
#define INA226_DEFAULT_ADDR  0x40

// INA226 寄存器
#define INA226_REG_CONFIG  0x00
#define INA226_REG_SHUNT   0x01
#define INA226_REG_BUS     0x02

// INA226 电压/电流/功率 检测驱动 (I2C 接口)
// 仅通过 BSP/board.cpp 提供的 board_i2c_* 接口操作外设
class DrvINA226
{
public:
    // port: 板级逻辑 I2C 端口, dev_addr: 7位设备地址 (board 层内部左移)
    DrvINA226(BoardI2cPort port, uint8_t dev_addr = INA226_DEFAULT_ADDR);

    // 初始化: 写入配置寄存器 (与参考工程一致, 平均过采样 16 次)
    void init(void);

    // 读取 Bus 电压 (V), 失败返回 -1
    float readBusVoltage(void);

    // 电池状态
    enum class BatState : uint8_t {
        BAT_MEASURE_ERR = 0,
        BAT_FULL_POWER  = 1,
        BAT_HIGH_POWER  = 2,
        BAT_MID_POWER   = 3,
        BAT_LOW_POWER   = 4
    };

    // 根据总电压判断电池状态 (芯数取自 g_params.bat_cells, 上位机可在线修改)
    BatState getBatStatus(float voltage);

private:
    // 写 16 位寄存器
    bool writeReg(uint8_t reg, uint16_t value);
    // 读 16 位寄存器
    bool readReg(uint8_t reg, uint16_t &value);

    BoardI2cPort m_port;
    uint8_t      m_dev_addr;

    // 单节电压阈值 (V)
    static constexpr float LOW_POWER_VEL  = 3.70f;
    static constexpr float MID_POWER_VEL  = 3.80f;
    static constexpr float HIGH_POWER_VEL = 4.10f;
    static constexpr float FULL_POWER_VEL = 4.20f;
};

void init_drv_ina226();
DrvINA226 *drv_ina226();
