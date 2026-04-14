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
	

	enum class BatCellsNum : uint32_t {
        CELLS_3S = 3,
        CELLS_4S = 4,
        CELLS_6S = 6
    };
	
	enum class bat_state : uint8_t {
	
	BAT_MESSURE_ERR= 0,
	BAT_FULL_POWER = 1,
	BAT_HIGH_POWER = 2,
	BAT_MID_POWER  = 3,
	BAT_LOW_POWER  = 4
	};
		
	bat_state INA226_get_bat_status(float vel);

	
private:
	
	
	static constexpr float LOW_POWER_VEL  = 3.70f;
	static constexpr float MID_POWER_VEL  = 3.80f;
	static constexpr float HIGH_POWER_VEL = 4.10f;
	static constexpr float FULL_POWER_VEL = 4.20f;

	I2C_HandleTypeDef *i2c;
	
	
};
