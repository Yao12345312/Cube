#include "QMC5883P.hpp"
#include "cmsis_os2.h"
#include <cstring>

#define QMC_REG_DATA_X_LSB 0x00
#define QMC_REG_CONTROL_1  0x09
#define QMC_REG_CONTROL_2  0x0A


QMC5883P::QMC5883P(I2C_HandleTypeDef *hi2c)
{
    i2c = hi2c;

    // Init calibration: zero offset, identity matrix (no correction)
    memset(offset, 0, sizeof(offset));
    memset(matrix, 0, sizeof(matrix));
    matrix[0][0] = 1.0f;
    matrix[1][1] = 1.0f;
    matrix[2][2] = 1.0f;

    // Calibration collection starts inactive — only enabled by user via menu
    m_collecting  = false;
    m_sampleCount = 0;
    memset(minV, 0, sizeof(minV));
    memset(maxV, 0, sizeof(maxV));
}

bool QMC5883P::init(void)
{
    osDelay(50);

    // OSR=512 RNG=8G ODR=200Hz MODE=Continuous
    if (!writeReg(QMC_REG_CONTROL_1, 0xFF))
        return false;

    osDelay(10);

    if (!writeReg(QMC_REG_CONTROL_2, 0x01))
        return false;

    return true;
}



bool QMC5883P::readRaw(MagData &data)
{
    uint8_t buf[6];

    if (!readRegs(QMC_REG_DATA_X_LSB, buf, 6))
        return false;

    data.x = (int16_t)(buf[1] << 8 | buf[0]);
    data.y = (int16_t)(buf[3] << 8 | buf[2]);
    data.z = (int16_t)(buf[5] << 8 | buf[4]);

    return true;
}

// Read raw + apply hard/soft iron calibration
bool QMC5883P::readCalibrated(MagData &data)
{
    if(!readRaw(data))
        return false;

    convertMagFrame(data);

    // Hard iron correction
    float x = data.x - offset[0];
    float y = data.y - offset[1];
    float z = data.z - offset[2];

    // Soft iron correction
    data.x = matrix[0][0]*x + matrix[0][1]*y + matrix[0][2]*z;
    data.y = matrix[1][0]*x + matrix[1][1]*y + matrix[1][2]*z;
    data.z = matrix[2][0]*x + matrix[2][1]*y + matrix[2][2]*z;

    return true;
}

// Coordinate transform: QMC5883P frame -> BMI088 frame
// QMC5883P rotates 180 deg CW around Z-axis to align with BMI088 accel frame
// Z-axis is same direction between both sensors
// X_acc = -X_mag, Y_acc = -Y_mag, Z_acc = Z_mag
void QMC5883P::convertMagFrame(MagData &data)
{
    data.x = -data.x;
    data.y = -data.y;
    // Z unchanged (same direction)
}


void QMC5883P::startCalibration()
{
    // Reset min/max for 3-axis collection
    minV[0]=minV[1]=minV[2]=1e9f;
    maxV[0]=maxV[1]=maxV[2]=-1e9f;

    m_collecting  = true;
    m_sampleCount = 0;
}


// Collect min/max for each axis during calibration
void QMC5883P::updateCalibration(const MagData &d)
{
    if (!m_collecting)
        return;

    if(d.x<minV[0]) minV[0]=d.x;
    if(d.y<minV[1]) minV[1]=d.y;
    if(d.z<minV[2]) minV[2]=d.z;

    if(d.x>maxV[0]) maxV[0]=d.x;
    if(d.y>maxV[1]) maxV[1]=d.y;
    if(d.z>maxV[2]) maxV[2]=d.z;

    m_sampleCount++;

    // Auto-finish when enough samples collected
    if (m_sampleCount >= CALIB_TARGET_SAMPLES)
    {
        finishCalibration();
        m_collecting = false;
    }
}


// Compute hard iron offset and soft iron scale matrix from collected min/max
void QMC5883P::finishCalibration()
{
    // Hard iron: center of the ellipsoid
    offset[0] = (maxV[0] + minV[0]) * 0.5f;
    offset[1] = (maxV[1] + minV[1]) * 0.5f;
    offset[2] = (maxV[2] + minV[2]) * 0.5f;

    // Soft iron: compute scale factors from axis ranges
    // Diagonal-only approximation (assumes axis-aligned distortion)
    float scaleX = (maxV[0] - minV[0]) * 0.5f;
    float scaleY = (maxV[1] - minV[1]) * 0.5f;
    float scaleZ = (maxV[2] - minV[2]) * 0.5f;

    if (scaleX < 1e-6f) scaleX = 1.0f;
    if (scaleY < 1e-6f) scaleY = 1.0f;
    if (scaleZ < 1e-6f) scaleZ = 1.0f;

    float avg = (scaleX + scaleY + scaleZ) / 3.0f;

    // Reset matrix to identity, then apply diagonal scaling
    memset(matrix, 0, sizeof(matrix));
    matrix[0][0] = avg / scaleX;
    matrix[1][1] = avg / scaleY;
    matrix[2][2] = avg / scaleZ;
}


bool QMC5883P::writeReg(uint8_t reg, uint8_t val)
{
    return (HAL_I2C_Mem_Write(i2c,
                              I2C_ADDR,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              &val,
                              1,
                              100) == HAL_OK);
}

bool QMC5883P::readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    return (HAL_I2C_Mem_Read(i2c,
                             I2C_ADDR,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf,
                             len,
                             100) == HAL_OK);
}
