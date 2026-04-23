#pragma once

#include "stm32h7xx_hal.h"
#include "cmsis_os2.h"
#include "u8g2.h"
#include "board.hpp"

// SPI句柄
extern SPI_HandleTypeDef hspi2;


// 宏定义
#define OLED_CS_HIGH()   HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_SET)
#define OLED_CS_LOW()    HAL_GPIO_WritePin(OLED_CS_PORT, OLED_CS_PIN, GPIO_PIN_RESET)
#define OLED_DC_HIGH()   HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_SET)
#define OLED_DC_LOW()    HAL_GPIO_WritePin(OLED_DC_PORT, OLED_DC_PIN, GPIO_PIN_RESET)
#define OLED_RST_HIGH()  HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_SET)
#define OLED_RST_LOW()   HAL_GPIO_WritePin(OLED_RST_PORT, OLED_RST_PIN, GPIO_PIN_RESET)

// 函数声明
void OLED_SPI_SendByte(uint8_t data);
void OLED_WriteCMD(uint8_t cmd);
void OLED_WriteData(uint8_t data);
void OLED_WriteDataBuffer(uint8_t *data, uint8_t count);
void OLED_Init_Hardware(void);
void OLED_Init(void);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_DisplayLogo(void);
void OLED_ShowBLEON(void);
void OLED_ShowBLEOFF(void);
void OLED_Lowbattery(void);
void OLED_Middlebattery(void);
void OLED_Highbattery(void);
void OLED_Fullbattery(void);
void OLED_Warn(void);
void OLED_ShowString(uint8_t x, uint8_t y, const char *str);
void OLED_ShowImage(uint8_t x, uint8_t y, uint8_t width, uint8_t height, const uint8_t *image);
void OLED_DrawPixel(uint8_t x, uint8_t y);
void OLED_DrawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1);
void OLED_DrawCircle(int16_t x, int16_t y, uint8_t radius, uint8_t is_filled);
void OLED_DrawRectangle(int16_t x, int16_t y, uint8_t width, uint8_t height, uint8_t is_filled);
void OLED_ShowChinese(int16_t x, int16_t y, char *chinese);
void OLED_ShowFloat(uint8_t x, uint8_t y, float num);
// u8g2回调函数
uint8_t u8x8_byte_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8x8_stm32_gpio_and_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
void u8g2_Init_Display(u8g2_t *u8g2);

