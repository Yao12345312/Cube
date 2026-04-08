#pragma once

#include "stm32h7xx_hal.h"


#define INA226_ADDR        (0x40 << 1)

#define INA226_REG_CONFIG  0x00
#define INA226_REG_SHUNT   0x01
#define INA226_REG_BUS     0x02


class INA226
{
public:
	
	INA226(I2C_HandleTypeDef *hi2c);
	
	void Init(void);
	
	float INA226_ReadBusVoltage(void);
	
	bool INA226_get_bat_status(float vel);

    enum BatCellsNum {
        CELLS_3S = 3,
        CELLS_4S = 4,
        CELLS_6S = 6
    };
	
private:
		
	static constexpr float LOW_POWER_VEL = 3.6f;

	I2C_HandleTypeDef *i2c;
	
	
};
