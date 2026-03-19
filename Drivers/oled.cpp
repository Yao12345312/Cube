extern "C" {
#include "oled.hpp"
#include "oled_font.hpp"
}
#include <cstdio>
#include <cstring>

#define OLED_DIGIT_MAX_WIDTH 6  // 允许最多6位数

// SPI发送一个字节
static void OLED_SPI_Send(oled_desc_t oled, uint8_t data)
{
    HAL_SPI_Transmit(oled->spi, &data, 1, HAL_MAX_DELAY);
}

// 写命令/数据
void OLED_WR_Byte(oled_desc_t oled, uint8_t dat, uint8_t cmd)
{
    if(cmd)
        HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_SET);   // 数据
    else
        HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_RESET); // 命令

    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_RESET);     // CS低
    OLED_SPI_Send(oled, dat);
    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_SET);       // CS高
}

// 设置显示坐标
void OLED_SetPosition(oled_desc_t oled, uint8_t x, uint8_t y)
{
    OLED_WR_Byte(oled, 0xB0 + (y & 0x07), 0);                  // 页地址
    OLED_WR_Byte(oled, 0x10 | ((x & 0xF0) >> 4), 0);           // 列高4位
    OLED_WR_Byte(oled, 0x00 | (x & 0x0F), 0);                  // 列低4位
}

// OLED硬件初始化
void OLED_Hw_Init(oled_desc_t oled)
{
    // CS引脚初始化
    HAL_GPIO_WritePin(oled->cs_port, oled->cs_pin, GPIO_PIN_SET);
    
    // DC引脚初始化
    HAL_GPIO_WritePin(oled->dc_port, oled->dc_pin, GPIO_PIN_SET);
    
    // RST引脚初始化
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_SET);
    
    // SPI已经在HAL初始化时配置好，这里不需要重复配置
    
    // OLED 上电复位
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_RESET);  // RES = 0
    osDelay(100);
    HAL_GPIO_WritePin(oled->rst_port, oled->rst_pin, GPIO_PIN_SET);    // RES = 1
    osDelay(100);
}

// OLED初始化
void OLED_Init(oled_desc_t oled)
{
    osDelay(100);
    OLED_WR_Byte(oled, 0xAE, 0);    // 关闭显示
    OLED_WR_Byte(oled, 0x00, 0);    // 设置列低地址
    OLED_WR_Byte(oled, 0x10, 0);    // 设置列高地址
    OLED_WR_Byte(oled, 0x40, 0);    // 设置显示起始行
    OLED_WR_Byte(oled, 0x81, 0);    // 设置对比度
    OLED_WR_Byte(oled, 0xCF, 0);    // 对比度值
    OLED_WR_Byte(oled, 0xA1, 0);    // 设置段重映射
    OLED_WR_Byte(oled, 0xC8, 0);    // 设置COM扫描方向
    OLED_WR_Byte(oled, 0xA6, 0);    // 正常显示
    OLED_WR_Byte(oled, 0xA8, 0);    // 设置多路复用
    OLED_WR_Byte(oled, 0x3F, 0);    // 64行
    OLED_WR_Byte(oled, 0xD3, 0);    // 设置显示偏移
    OLED_WR_Byte(oled, 0x00, 0);    // 偏移量为0
    OLED_WR_Byte(oled, 0xD5, 0);    // 设置振荡器频率
    OLED_WR_Byte(oled, 0x80, 0);    // 频率值
    OLED_WR_Byte(oled, 0xD9, 0);    // 设置预充电周期
    OLED_WR_Byte(oled, 0xF1, 0);    // 周期值
    OLED_WR_Byte(oled, 0xDA, 0);    // 设置COM引脚硬件配置
    OLED_WR_Byte(oled, 0x12, 0);    // 配置值
    OLED_WR_Byte(oled, 0xDB, 0);    // 设置VCOMH
    OLED_WR_Byte(oled, 0x40, 0);    // VCOMH值
    OLED_WR_Byte(oled, 0x20, 0);    // 设置内存寻址模式
    OLED_WR_Byte(oled, 0x02, 0);    // 页寻址模式
    OLED_WR_Byte(oled, 0x8D, 0);    // 电荷泵设置
    OLED_WR_Byte(oled, 0x14, 0);    // 使能电荷泵
    OLED_WR_Byte(oled, 0xA4, 0);    // 恢复显示RAM内容
    OLED_WR_Byte(oled, 0xA6, 0);    // 正常显示
    OLED_WR_Byte(oled, 0xAF, 0);    // 开启显示
    
    OLED_Clear(oled);
}

// 开启显示
void OLED_Display_On(oled_desc_t oled)
{
    OLED_WR_Byte(oled, 0x8D, 0);
    OLED_WR_Byte(oled, 0x14, 0);
    OLED_WR_Byte(oled, 0xAF, 0);
}

// 关闭显示
void OLED_Display_Off(oled_desc_t oled)
{
    OLED_WR_Byte(oled, 0x8D, 0);
    OLED_WR_Byte(oled, 0x10, 0);
    OLED_WR_Byte(oled, 0xAE, 0);
}

// 清屏
void OLED_Clear(oled_desc_t oled)
{
    for(uint8_t i = 0; i < 8; i++)
    {
        OLED_SetPosition(oled, 0, i);
        for(uint8_t n = 0; n < 128; n++)
            OLED_WR_Byte(oled, 0x00, 1);
    }
}

// 查表法计算10的n次方
static const uint32_t pow10_table[] = {
    1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000
};

static inline uint32_t pow_10(uint8_t n)
{
    if(n > 9) return 0;
    return pow10_table[n];
}

// 显示字符（x为行，y为列）
void OLED_ShowChar(oled_desc_t oled, uint8_t x, uint8_t y, char chr)
{
    uint8_t c = chr - ' ';
    const uint8_t *p = OLED_F8x16[c];
    uint8_t x_pixel = y * 8; // 列号转像素
    uint8_t y_page = x;      // 行号即页地址

    OLED_SetPosition(oled, x_pixel, y_page * 2);
    for (int i = 0; i < 8; i++) OLED_WR_Byte(oled, p[i], 1);
    OLED_SetPosition(oled, x_pixel, y_page * 2 + 1);
    for (int i = 0; i < 8; i++) OLED_WR_Byte(oled, p[i+8], 1);
}

// 显示字符串（x为行，y为列）
void OLED_ShowString(oled_desc_t oled, uint8_t x, uint8_t y, const char *str)
{
    uint8_t curr_x = x;
    uint8_t curr_y = y;
    while (*str) {
        OLED_ShowChar(oled, curr_x, curr_y, *str++);
        curr_y++;
        if (curr_y > 15) {
            curr_y = 0;
            curr_x++;
            if (curr_x > 3) curr_x = 0;
        }
    }
}

// 显示无符号32位整数
void OLED_ShowUInt32(oled_desc_t oled, uint8_t x, uint8_t y, uint32_t num, uint8_t leading_zero)
{
    char buf[OLED_DIGIT_MAX_WIDTH + 1];
    uint8_t i = 0;
    
    if(num == 0) {
        buf[i++] = '0';
    } else {
        uint8_t digits[10], digit_count = 0;
        while(num > 0) {
            digits[digit_count++] = num % 10;
            num /= 10;
        }
        if(leading_zero) {
            while(digit_count < 4) digits[digit_count++] = 0;
        }
        while(digit_count > 0) buf[i++] = '0' + digits[--digit_count];
    }
    buf[i] = '\0';
    OLED_ShowString(oled, x, y, buf);
}

// 显示有符号32位整数
void OLED_ShowInt32(oled_desc_t oled, uint8_t x, uint8_t y, int32_t num)
{
    if(num < 0) {
        OLED_ShowChar(oled, x, y, '-');
        OLED_ShowUInt32(oled, x, y + 1, -num, 0); // 负号占一列
    } else {
        OLED_ShowUInt32(oled, x, y, num, 0);
    }
}

// 显示浮点数
void OLED_ShowFloat(oled_desc_t oled, uint8_t x, uint8_t y, float num, uint8_t precision)
{
    if(precision > 6) precision = 6;
    if(num < 0) {
        OLED_ShowChar(oled, x, y, '-');
        num = -num;
        y += 1; // 负号占一列
    }
    int32_t int_part = (int32_t)num;
    float decimal_part = num - int_part;

    char int_buf[12];
    int len = 0;
    sprintf(int_buf, "%ld", (long)int_part);
    len = strlen(int_buf);

    OLED_ShowString(oled, x, y, int_buf);

    y += len;
    if(precision > 0) {
        OLED_ShowChar(oled, x, y, '.');
        y += 1;
        uint32_t decimal = (uint32_t)(decimal_part * pow_10(precision));
        char dec_buf[8];
        sprintf(dec_buf, "%0*ld", precision, (long)decimal); // 前导零
        OLED_ShowString(oled, x, y, dec_buf);
    }
}

// 显示16进制数
void OLED_ShowHex(oled_desc_t oled, uint8_t x, uint8_t y, uint32_t num)
{
    char buf[11];
    buf[0] = '0'; buf[1] = 'x';
    for(int i = 9; i >= 2; i--) {
        uint8_t digit = num & 0xF;
        buf[i] = (digit < 10) ? ('0' + digit) : ('A' + digit - 10);
        num >>= 4;
    }
    buf[10] = '\0';
    OLED_ShowString(oled, x, y, buf);
}