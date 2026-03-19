#pragma once
extern "C"
{
#include "stm32h7xx_hal.h"
#include "system_stm32h7xx.h"
#include "bmi08x.h"
}


class Bmi088
{
public:
    enum Variant
    {
        BMI085 = BMI085_VARIANT,
        BMI088 = BMI088_VARIANT
    };

    Bmi088(SPI_HandleTypeDef *hspi,
           GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
           GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin);

    int8_t init();

    bmi08_dev* getDev();
	
	int8_t getAccelData(float &accel_x, float &accel_y, float &accel_z);
	
	int8_t getGyroData(float &gyro_x, float &gyro_y, float &gyro_z);
private:
	uint8_t m_currentDev;

    SPI_HandleTypeDef *m_hspi;

    GPIO_TypeDef *m_accCsPort;
    uint16_t m_accCsPin;

    GPIO_TypeDef *m_gyroCsPort;
    uint16_t m_gyroCsPin;

    bmi08_dev m_dev;

    uint8_t m_accDevAddr;
    uint8_t m_gyroDevAddr;

    void csLow(uint8_t dev);
    void csHigh(uint8_t dev);

    static BMI08_INTF_RET_TYPE spiRead(uint8_t reg_addr,
                                   uint8_t *reg_data,
                                   uint32_t len,
                                   void *intf_ptr);

    static BMI08_INTF_RET_TYPE spiWrite(uint8_t reg,
                                         const uint8_t *data,
                                         uint32_t len,
                                         void *intf_ptr);

    static void delayUs(uint32_t period, void *intf_ptr);
										 
	float getAccelSensitivity(void);
										 
	float getGyroSensitivity(void);									 
};

