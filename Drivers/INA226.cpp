#include "INA226.hpp"


INA226::INA226(I2C_HandleTypeDef *hi2c)
{
    i2c = hi2c;
}

void INA226::Init(void)
{
    uint8_t config[3];

    config[0] = INA226_REG_CONFIG;

    // 默认配置（连续测量，平均16次）
    config[1] = 0x45;
    config[2] = 0x27;

    HAL_I2C_Master_Transmit(i2c, INA226_ADDR, config, 3, 100);
}

/**
 * @brief 读取Bus电压
 * @return 电压值 (V)
 */
float INA226::INA226_ReadBusVoltage(void)
{
    uint8_t buf[2];
    uint16_t raw = 0;

    if (HAL_I2C_Mem_Read(i2c,
                         INA226_ADDR,
                         INA226_REG_BUS,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         2,
                         100) != HAL_OK)
    {	
		//如果I2C没有读到值，返回-1
        return -1.0f;
    }

    raw = (buf[0] << 8) | buf[1];

    // 分辨率为1.25mV，读取原始值乘以分辨率得到实际值
    float voltage = raw * 0.00125f;

    return voltage;
}
/**
 * @brief 判断电池状态是否健康
 * @return 电池健康状态标志
 */
bool INA226::INA226_get_bat_status(float bat_vel)
{
	if(bat_vel < BatCellsNum::CELLS_3S *LOW_POWER_VEL )
		return false;
	else{return true;}

}