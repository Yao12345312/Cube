#include "BMI088.hpp"
#include "cmsis_os2.h"
#include <string.h>
#include "board.hpp"
#include "uart3Driver.hpp"
#include "SPI_Manager.hpp"

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
	//初始化校准变量
	m_accel_offset[0] = m_accel_offset[1] = m_accel_offset[2] = 0.0f;
    m_gyro_offset[0] = m_gyro_offset[1] = m_gyro_offset[2] = 0.0f;
    m_accel_calibrated = false;
    m_gyro_calibrated = false;
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
    
	//CS拉低引脚前切换模式为mode 0
	//if (!SPI_SetMode(self->m_hspi,SPI_POLARITY_LOW,SPI_PHASE_1EDGE)) {return -1;}
	
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
	
	//释放SPI
	//SPI_Unlock();
	
    return BMI08_INTF_RET_SUCCESS;
}


/* ================= SPI 写 ================= */

BMI08_INTF_RET_TYPE Bmi088::spiWrite(uint8_t reg,
                                     const uint8_t *data,
                                     uint32_t len,
                                     void *intf_ptr)
{
    Bmi088 *self = static_cast<Bmi088 *>(intf_ptr);
	
	//CS拉低引脚前切换模式为mode 0
	//if (!SPI_SetMode(self->m_hspi,SPI_POLARITY_LOW,SPI_PHASE_1EDGE)) {return -1;}
	
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
	
	//释放SPI
	//SPI_Unlock();
	
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
        float g_to_ms2 = 9.7986f;  // 重力加速度转换因子
        
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
/* ================= 加速度计静态校准函数 ================= */
int8_t Bmi088::calibrateAccelStatic()
{
    const float ACC_NORM_THRESHOLD = 0.3f;   // |a|-g 允许误差 (m/s^2)
    const uint16_t REQUIRED_COUNT = 1000;
    const uint32_t TIMEOUT = 10000;

    const float g = 9.7986f;

    float ax, ay, az;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    uint16_t count = 0;
    uint32_t time_ms = 0;

    printf("Accel calibration start...\n");

    while (1)
    {
        if (getAccelData(ax, ay, az) == BMI08_OK)
        {
            float norm = sqrtf(ax*ax + ay*ay + az*az);

            //静止检测
            if (fabs(norm - g) < ACC_NORM_THRESHOLD)
            {
                count++;

                sum_x += ax;
                sum_y += ay;
                sum_z += az;

                if (count >= REQUIRED_COUNT)
                {
                    float mean_x = sum_x / count;
                    float mean_y = sum_y / count;
                    float mean_z = sum_z / count;

                    //关键：根据当前姿态计算理论重力

                    float gx = 0.0f;
                    float gy = 0.0f;
                    float gz = -g;

                    //bias = 测量值 - 理论值
                    m_accel_offset[0] = mean_x - gx;
                    m_accel_offset[1] = mean_y - gy;
                    m_accel_offset[2] = mean_z - gz;

                    m_accel_calibrated = true;

                    printf("Accel cal success:\n");
                    printf("bias x: %.6f\n", m_accel_offset[0]);
                    printf("bias y: %.6f\n", m_accel_offset[1]);
                    printf("bias z: %.6f\n", m_accel_offset[2]);

                    return 0;
                }
            }
            else
            {
                //不稳定 → 清空
                count = 0;
                sum_x = sum_y = sum_z = 0;

                if (time_ms % 200 == 0)
                {
                    printf("Keep IMU static!\n");
                }
            }
        }

        osDelay(1);
        time_ms++;

        if (time_ms > TIMEOUT)
        {
            printf("Accel calibration timeout!\n");
            return -1;
        }
    }
}

/* ================= 获取校准零偏后的加速度计数据 ================= */
int8_t Bmi088::getAccelDataCalibrated(float &accel_x, float &accel_y, float &accel_z)
{
    int8_t rslt = getAccelData(accel_x, accel_y, accel_z);
    
    if (rslt == BMI08_OK && m_accel_calibrated) {
        accel_x -= m_accel_offset[0];
        accel_y -= m_accel_offset[1];
        accel_z -= m_accel_offset[2];
    }
    
    return rslt;
}

/* ================= 陀螺仪静态校准函数 ================= */
int8_t Bmi088::calibrateGyroStatic()
{
    const float GYRO_THRESHOLD = 3.0f;   // deg/s，静止判定阈值
    const uint16_t REQUIRED_COUNT = 1500; // 连续稳定次数
    const uint32_t TIMEOUT = 100000;       // 10s

    float gx, gy, gz;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    uint16_t count = 0;
    uint32_t time_ms = 0;

    printf("Gyro calibration start...\n");

    while (1)
    {
        if (getGyroData(gx, gy, gz) == BMI08_OK)
			{	
			// printf("gx:%.4f,gy:%.4f,gz:%.4f\n",gx,gy,gz);		
            //静止检测
            if (fabs(gx) < GYRO_THRESHOLD &&
                fabs(gy) < GYRO_THRESHOLD &&
                fabs(gz) < GYRO_THRESHOLD)
            {
                count++;

                sum_x += gx;
                sum_y += gy;
                sum_z += gz;

                // 连续稳定成功
                if (count >= REQUIRED_COUNT)
                {
                    m_gyro_offset[0] = sum_x / count;
                    m_gyro_offset[1] = sum_y / count;
                    m_gyro_offset[2] = sum_z / count;

                    m_gyro_calibrated = true;

                    printf("Gyro cal success:\n");
                    printf("gx: %.6f rad/s\n", m_gyro_offset[0]);
                    printf("gy: %.6f rad/s\n", m_gyro_offset[1]);
                    printf("gz: %.6f rad/s\n", m_gyro_offset[2]);

                    return 0;
                }
            }
            else
            {
                // 有抖动 → 清空重来
                count = 0;
                sum_x = sum_y = sum_z = 0;

                if (time_ms % 200 == 0)
                {
                    printf("Keep IMU static!\n");
                }
            }
        }

        osDelay(1);
        time_ms++;

        //  超时保护
        if (time_ms > TIMEOUT)
        {
            printf("Gyro calibration timeout!\n");
            return -1;
        }
    }
}

/* ================= 获取校准后的陀螺仪数据 ================= */
int8_t Bmi088::getGyroDataCalibrated(float &gyro_x, float &gyro_y, float &gyro_z)
{
    int8_t rslt = getGyroData(gyro_x, gyro_y, gyro_z);
    
    if (rslt == BMI08_OK && m_gyro_calibrated) {
        gyro_x -= m_gyro_offset[0];
        gyro_y -= m_gyro_offset[1];
        gyro_z -= m_gyro_offset[2];
    }
    
    return rslt;
}

int8_t Bmi088::calibrateAllStatic()
{
    float accel_off_x, accel_off_y, accel_off_z;
    float gyro_off_x, gyro_off_y, gyro_off_z;
    int8_t ret = 0;
    
    printf("\n静态校准开始\n");
    
    //校准陀螺仪（需要完全静止）
    if (calibrateGyroStatic()!= 0) {
        printf("陀螺仪校准失败\n");
        ret = -1;
    }
    
    osDelay(500);
    
    printf("校准完成\n\n");
    
    return ret;
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
