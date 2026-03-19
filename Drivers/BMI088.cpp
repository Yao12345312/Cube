#include "BMI088.hpp"
#include "cmsis_os2.h"
#include <string.h>
#include "board.hpp"
#include "uart3Driver.hpp"

#define BMI08X_READ_WRITE_LEN  64

extern SPI_HandleTypeDef hspi1;
/* ================= 构造函数 ================= */

Bmi088::Bmi088(SPI_HandleTypeDef *hspi,
               GPIO_TypeDef *acc_cs_port, uint16_t acc_cs_pin,
               GPIO_TypeDef *gyro_cs_port, uint16_t gyro_cs_pin)
{
    m_hspi = hspi;

    m_accCsPort = acc_cs_port;
    m_accCsPin  = acc_cs_pin;

    m_gyroCsPort = gyro_cs_port;
    m_gyroCsPin  = gyro_cs_pin;

    m_currentDev = 0;
}

/* ================= getDev ================= */

bmi08_dev* Bmi088::getDev()
{
    return &m_dev;
}

/* ================= CS 控制 ================= */

void Bmi088::csLow(uint8_t dev)
{
    if (dev == 0)
        HAL_GPIO_WritePin(m_accCsPort, m_accCsPin, GPIO_PIN_RESET);
    else
        HAL_GPIO_WritePin(m_gyroCsPort, m_gyroCsPin, GPIO_PIN_RESET);
}

void Bmi088::csHigh(uint8_t dev)
{
    if (dev == 0)
        HAL_GPIO_WritePin(m_accCsPort, m_accCsPin, GPIO_PIN_SET);
    else
        HAL_GPIO_WritePin(m_gyroCsPort, m_gyroCsPin, GPIO_PIN_SET);
}

/* ================= SPI 读 ================= */

BMI08_INTF_RET_TYPE Bmi088::spiRead(uint8_t reg_addr,
                                   uint8_t *reg_data,
                                   uint32_t len,
                                   void *intf_ptr)
{
    // 从intf_ptr获取Bmi088对象指针
    Bmi088 *self = static_cast<Bmi088 *>(intf_ptr);
    
    uint8_t tx_buf[len + 1];
    uint8_t rx_buf[len + 1];

    tx_buf[0] = reg_addr | 0x80;
    memset(&tx_buf[1], 0xFF, len);

    // 根据当前设备选择片选引脚
    if (self->m_currentDev == 0) {  // 加速度计
        HAL_GPIO_WritePin(self->m_accCsPort, self->m_accCsPin, GPIO_PIN_RESET);
    } else {  // 陀螺仪
        HAL_GPIO_WritePin(self->m_gyroCsPort, self->m_gyroCsPin, GPIO_PIN_RESET);
    }

    HAL_SPI_TransmitReceive(self->m_hspi, tx_buf, rx_buf, len + 1, 100);
     
    if (self->m_currentDev == 0) {
        HAL_GPIO_WritePin(self->m_accCsPort, self->m_accCsPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(self->m_gyroCsPort, self->m_gyroCsPin, GPIO_PIN_SET);
    }

    /* 直接拷贝真实数据*/
    memcpy(reg_data, &rx_buf[1], len);

    return BMI08_INTF_RET_SUCCESS;
}


/* ================= SPI 写 ================= */

BMI08_INTF_RET_TYPE Bmi088::spiWrite(uint8_t reg,
                                     const uint8_t *data,
                                     uint32_t len,
                                     void *intf_ptr)
{
    Bmi088 *self = static_cast<Bmi088 *>(intf_ptr);

    uint8_t tx = reg & 0x7F;

    // 根据当前设备选择片选
    if (self->m_currentDev == 0) {
        HAL_GPIO_WritePin(self->m_accCsPort, self->m_accCsPin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(self->m_gyroCsPort, self->m_gyroCsPin, GPIO_PIN_RESET);
    }

    HAL_SPI_Transmit(self->m_hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(self->m_hspi, (uint8_t*)data, len, HAL_MAX_DELAY);

    if (self->m_currentDev == 0) {
        HAL_GPIO_WritePin(self->m_accCsPort, self->m_accCsPin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(self->m_gyroCsPort, self->m_gyroCsPin, GPIO_PIN_SET);
    }

    return BMI08_OK;
}

float Bmi088::getAccelSensitivity()
{
    float sensitivity = 0.0f;
    
    switch(m_dev.accel_cfg.range) {
        case BMI088_ACCEL_RANGE_24G:
            sensitivity = 1365.0f;  // 根据BMI088数据手册，±24G时灵敏度为1365 LSB/g
            break;
        case BMI088_ACCEL_RANGE_12G:
            sensitivity = 2730.0f;  // ±12G时灵敏度为2730 LSB/g
            break;
        case BMI088_ACCEL_RANGE_6G:
            sensitivity = 5460.0f;  // ±6G时灵敏度为5460 LSB/g
            break;
        case BMI088_ACCEL_RANGE_3G:
            sensitivity = 10920.0f; // ±3G时灵敏度为10920 LSB/g
            break;
        default:
            sensitivity = 1365.0f;   // 默认使用±24G的灵敏度
            break;
    }
    
    return sensitivity;
}

/* ================= 获取陀螺仪灵敏度 ================= */

float Bmi088::getGyroSensitivity()
{
    float sensitivity = 0.0f;
    
    switch(m_dev.gyro_cfg.range) {
        case BMI08_GYRO_RANGE_2000_DPS:
            sensitivity = 16.384f;   // ±2000°/s时灵敏度为16.384 LSB/°/s
            break;
        case BMI08_GYRO_RANGE_1000_DPS:
            sensitivity = 32.768f;   // ±1000°/s时灵敏度为32.768 LSB/°/s
            break;
        case BMI08_GYRO_RANGE_500_DPS:
            sensitivity = 65.536f;   // ±500°/s时灵敏度为65.536 LSB/°/s
            break;
        case BMI08_GYRO_RANGE_250_DPS:
            sensitivity = 131.072f;  // ±250°/s时灵敏度为131.072 LSB/°/s
            break;
        case BMI08_GYRO_RANGE_125_DPS:
            sensitivity = 262.144f;  // ±125°/s时灵敏度为262.144 LSB/°/s
            break;
        default:
            sensitivity = 16.384f;   // 默认使用±2000°/s的灵敏度
            break;
    }
    
    return sensitivity;
}

int8_t Bmi088::getAccelData(float &accel_x, float &accel_y, float &accel_z)
{
    int8_t rslt;
    struct bmi08_sensor_data accel_data;
    
    m_currentDev = 0;  // 切换到加速度计
    
    rslt = bmi08a_get_data(&accel_data, &m_dev);
    
    if (rslt == BMI08_OK) {
        float sensitivity = getAccelSensitivity();
        float g_to_ms2 = 9.80665f;  // 重力加速度转换因子
        
        // 将原始数据转换为g，然后转换为m/s2
        accel_x = (float)accel_data.x / sensitivity * g_to_ms2;
        accel_y = (float)accel_data.y / sensitivity * g_to_ms2;
        accel_z = (float)accel_data.z / sensitivity * g_to_ms2;
    }
    
    return rslt;
}

/* ================= 获取陀螺仪数据 (°/s) ================= */

int8_t Bmi088::getGyroData(float &gyro_x, float &gyro_y, float &gyro_z)
{
    int8_t rslt;
    struct bmi08_sensor_data gyro_data;
    
    m_currentDev = 1;  // 切换到陀螺仪
    
    rslt = bmi08g_get_data(&gyro_data, &m_dev);
    
    if (rslt == BMI08_OK) {
        float sensitivity = getGyroSensitivity();
        
        // 将原始数据转换为°/s
        gyro_x = (float)gyro_data.x / sensitivity;
        gyro_y = (float)gyro_data.y / sensitivity;
        gyro_z = (float)gyro_data.z / sensitivity;
    }
    
    return rslt;
}


/* ================= 延时 ================= */

static void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Bmi088::delayUs(uint32_t period, void *intf_ptr)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}


/* ================= 完整初始化 ================= */

int8_t Bmi088::init()
{
	DWT_Init();
	
    int8_t rslt;

    m_dev.intf = BMI08_SPI_INTF;
    m_dev.read = spiRead;
    m_dev.write = spiWrite;
    m_dev.delay_us = delayUs;
    m_dev.read_write_len = BMI08X_READ_WRITE_LEN;
	m_dev.dummy_byte = 1; 
	
    m_dev.intf_ptr_accel = this;
    m_dev.intf_ptr_gyro  = this;
	
    /* ---------- Accel ---------- */

    m_currentDev = 0;

    rslt = bmi08a_init(&m_dev);
    if (rslt != BMI08_OK) return rslt;

    m_dev.accel_cfg.range = BMI088_ACCEL_RANGE_24G;
    m_dev.accel_cfg.odr   = BMI08_ACCEL_ODR_1600_HZ;
    m_dev.accel_cfg.bw    = BMI08_ACCEL_BW_NORMAL;
	m_dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
	
	rslt = bmi08a_set_power_mode(&m_dev);
    if (rslt != BMI08_OK) return rslt;

    rslt = bmi08a_set_meas_conf(&m_dev);
    if (rslt != BMI08_OK) return rslt;

    osDelay(50);

    /* ---------- Gyro ---------- */

    m_currentDev = 1;

    rslt = bmi08g_init(&m_dev);
    if (rslt != BMI08_OK) return rslt;

    m_dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
    m_dev.gyro_cfg.odr   = BMI08_GYRO_BW_230_ODR_2000_HZ;
	m_dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
	
	rslt = bmi08g_set_power_mode(&m_dev);
	if (rslt != BMI08_OK) return rslt;
	
    rslt = bmi08g_set_meas_conf(&m_dev);
    
    return rslt;
}
