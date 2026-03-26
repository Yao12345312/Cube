#include "SPA06.hpp"
#include <cstring>
#include <cmath>
#include "SPI_Manager.hpp"

/* ================= 基础 ================= */

SPA06::SPA06(SPI_HandleTypeDef* hspi,
             GPIO_TypeDef* cs_port, uint16_t cs_pin)
{
    m_hspi = hspi;
    m_csPort = cs_port;
    m_csPin = cs_pin;

    m_mutex = osMutexNew(NULL);
}

void SPA06::csLow()  { HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET); }
void SPA06::csHigh() { HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET); }
//符号扩展函数
static int32_t signExtend(uint32_t val, uint8_t bits)
{
    if (val & (1 << (bits - 1)))
        val |= (~0U << bits);
    return (int32_t)val;
}

/* SPI: bit7=1 读 */
uint8_t SPA06::readReg(uint8_t reg)
{	
	//CS拉低前修改SPI 为mode 3
	//SPI_SetMode(m_hspi,SPI_POLARITY_HIGH,SPI_PHASE_2EDGE);
	
    uint8_t tx = static_cast<uint8_t>(reg | 0x80);
    uint8_t rx = 0;

    csLow();
    HAL_SPI_Transmit(m_hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(m_hspi, &rx, 1, HAL_MAX_DELAY);
    csHigh();
	
	//释放SPI
	//SPI_Unlock();
	
    return rx;
}

void SPA06::readRegs(uint8_t reg, uint8_t* buf, uint8_t len)
{	
	//CS拉低前修改SPI 为mode 3
	//SPI_SetMode(m_hspi,SPI_POLARITY_HIGH,SPI_PHASE_2EDGE);
	
    uint8_t tx = static_cast<uint8_t>(reg | 0x80);

    csLow();
    HAL_SPI_Transmit(m_hspi, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(m_hspi, buf, len, HAL_MAX_DELAY);
    csHigh();
	
	//释放SPI
	//SPI_Unlock();
	
}

void SPA06::writeReg(uint8_t reg, uint8_t val)
{	
	//SPI_SetMode(m_hspi, SPI_POLARITY_HIGH, SPI_PHASE_2EDGE);
	
	
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F),static_cast<uint8_t>(val)};

    csLow();
    HAL_SPI_Transmit(m_hspi, tx, 2, HAL_MAX_DELAY);
    csHigh();
	
	//SPI_Unlock();
}

/* ================= 初始化 ================= */

bool SPA06::init()
{
    osMutexAcquire(m_mutex, osWaitForever);

    /* 软复位 */
    writeReg(0x0C, 0x09);
    osDelay(10);

    /* 等待 READY */
    while (!(readReg(0x08) & (1 << 6)))  // SENSOR_RDY
        osDelay(1);

    /* 读取校准参数 */
    readCoefficients();

    osMutexRelease(m_mutex);
    return true;
}

/* ================= 启动测量 ================= */

bool SPA06::startBackgroundMode()
{
    osMutexAcquire(m_mutex, osWaitForever);

    /* PRS_CFG: 4Hz + 16倍过采样 */
    writeReg(0x06, 0x24);

    /* 使用内部温度，OSR=16 */
    writeReg(0x07, 0xA4);

    /* CFG_REG: 使能 shift */
    writeReg(0x09, (1 << 2) | (1 << 3));

    /* 启动 Background 模式 */
    writeReg(0x08, 0x07);

    osMutexRelease(m_mutex);
    return true;
}

/* ================= 读取数据 ================= */

bool SPA06::read()
{
    uint8_t buf[6];

    osMutexAcquire(m_mutex, osWaitForever);

    readRegs(0x00, buf, 6);

    uint32_t p = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    uint32_t t = (buf[3] << 16) | (buf[4] << 8) | buf[5];

    raw_pressure = convert24bit(p);
    raw_temp = convert24bit(t);

    compensate();

    osMutexRelease(m_mutex);
    return true;
}

/* ================= 数据处理 ================= */

int32_t SPA06::convert24bit(uint32_t val)
{
    if (val & 0x800000)
        val |= 0xFF000000;
    return (int32_t)val;
}

void SPA06::readCoefficients()
{
    uint8_t buf[25];
    readRegs(0x10, buf, 25);

    c0  = signExtend(((buf[0] << 4) | (buf[1] >> 4)), 12);
    c1  = signExtend((((buf[1] & 0x0F) << 8) | buf[2]), 12);

    c00 = signExtend(((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4)), 20);
    c10 = signExtend((((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7]), 20);

    c01 = signExtend((buf[8] << 8) | buf[9], 16);
    c11 = signExtend((buf[10] << 8) | buf[11], 16);
    c20 = signExtend((buf[12] << 8) | buf[13], 16);
    c21 = signExtend((buf[14] << 8) | buf[15], 16);
    c30 = signExtend((buf[16] << 8) | buf[17], 16);

    c31 = signExtend((buf[18] << 4) | (buf[19] >> 4), 12);
    c40 = signExtend(((buf[19] & 0x0F) << 8) | buf[20], 12);
}

/* ================= 补偿 ================= */

void SPA06::compensate()
{
    const float kP = 253952.0f;  // 16x
    const float kT = 253952.0f;

    float Praw_sc = raw_pressure / kP;
    float Traw_sc = raw_temp / kT;

    /* 温度 */
    temperature = c0 * 0.5f + c1 * Traw_sc;

    /* 压力 */
    pressure =
        c00 +
        Praw_sc * (c10 + Praw_sc * (c20 + Praw_sc * (c30 + c40 * Praw_sc))) +
        Traw_sc * (c01 + Praw_sc * (c11 + Praw_sc * (c21 + c31 * Praw_sc)));
}

/* ================= 接口 ================= */

float SPA06::getPressurePa() const
{
    return pressure;
}

float SPA06::getTemperatureC() const
{
    return temperature;
}

void SPA06::Data_Debug()
{
	printf("ID=%02X\r\n", readReg(0x0D));

	printf("raw_p=%ld raw_t=%ld\r\n", raw_pressure, raw_temp);

	printf("c0=%d c1=%d c00=%ld c10=%ld\r\n", c0, c1, c00, c10);

	printf("T=%f P=%f\r\n", temperature, pressure);
}
/*调用示例
		//启动测量
		baro.startBackgroundMode();
		//读数
		baro.read();
		pressure=baro.getPressurePa();
		temperature=baro.getTemperatureC();
		printf("p=%.4f,t=%.4f\r\n",pressure,temperature);
		//打印原始读数调试
		baro.Data_Debug();
*/