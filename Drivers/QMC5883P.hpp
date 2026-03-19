#pragma once
#include "stm32h7xx_hal.h"

class QMC5883P
{
public:

    struct MagData
    {
        float x;
        float y;
        float z;
    };

    QMC5883P(I2C_HandleTypeDef *hi2c);

    bool init();
    bool readRaw(MagData &data);
    bool readCalibrated(MagData &data);

    void convertMagFrame(MagData &data);

    /* calibration */
    void startCalibration();
    void updateCalibration(const MagData &data);
    void finishCalibration();

private:

    I2C_HandleTypeDef *i2c;

    static constexpr uint8_t I2C_ADDR = 0x2C << 1;

    bool writeReg(uint8_t reg,uint8_t val);
    bool readRegs(uint8_t reg,uint8_t *buf,uint8_t len);

    /* Hard iron */
    float offset[3];

    /* Soft iron */
    float matrix[3][3];

    /* calibration buffer */
    float minV[3];
    float maxV[3];
};