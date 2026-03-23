#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const uint8_t OLED_F8x16[][16];
	
// OLED描述结构体
typedef struct {
    SPI_HandleTypeDef* spi;        // SPI句柄
    GPIO_TypeDef* cs_port;         // CS引脚端口
    uint16_t cs_pin;               // CS引脚号
    GPIO_TypeDef* dc_port;         // DC引脚端口
    uint16_t dc_pin;               // DC引脚号
    GPIO_TypeDef* rst_port;        // RST引脚端口
    uint16_t rst_pin;              // RST引脚号
} oled_dev_t;

typedef oled_dev_t* oled_desc_t;

// OLED初始化
void OLED_Init(oled_desc_t oled);
// OLED硬件初始化
void OLED_Hw_Init(oled_desc_t oled);
// 写命令/数据
void OLED_WR_Byte(oled_desc_t oled, uint8_t dat, uint8_t cmd);
// 设置显示坐标
void OLED_SetPosition(oled_desc_t oled, uint8_t x, uint8_t y);
// 开启显示
void OLED_Display_On(oled_desc_t oled);
// 关闭显示
void OLED_Display_Off(oled_desc_t oled);
// 清屏
void OLED_Clear(oled_desc_t oled);
// 显示字符（x为行(0-3)，y为列(0-15)）
void OLED_ShowChar(oled_desc_t oled, uint8_t x, uint8_t y, char chr);
// 显示字符串
void OLED_ShowString(oled_desc_t oled, uint8_t x, uint8_t y, const char *str);
// 显示无符号32位整数
void OLED_ShowUInt32(oled_desc_t oled, uint8_t x, uint8_t y, uint32_t num, uint8_t leading_zero);
// 显示有符号32位整数
void OLED_ShowInt32(oled_desc_t oled, uint8_t x, uint8_t y, int32_t num);
// 显示浮点数
void OLED_ShowFloat(oled_desc_t oled, uint8_t x, uint8_t y, float num, uint8_t precision);
// 显示16进制数
void OLED_ShowHex(oled_desc_t oled, uint8_t x, uint8_t y, uint32_t num);

#ifdef __cplusplus
}
#endif
