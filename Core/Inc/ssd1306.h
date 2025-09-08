/**
 * @file ssd1306.h
 * @brief SSD1306 OLED显示屏驱动头文件
 * @author STM32 Vision Car Project
 * @date 2025
 */

#ifndef __SSD1306_H
#define __SSD1306_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "fonts.h"

/* SSD1306 配置参数 */
#define SSD1306_I2C_ADDR        0x78    // SSD1306 I2C地址 (0x3C << 1)
#define SSD1306_WIDTH           128     // 屏幕宽度
#define SSD1306_HEIGHT          64      // 屏幕高度
#define SSD1306_PAGES           8       // 页数 (64/8=8)

/* SSD1306 命令定义 */
#define SSD1306_SETCONTRAST                     0x81
#define SSD1306_DISPLAYALLON_RESUME             0xA4
#define SSD1306_DISPLAYALLON                    0xA5
#define SSD1306_NORMALDISPLAY                   0xA6
#define SSD1306_INVERTDISPLAY                   0xA7
#define SSD1306_DISPLAYOFF                      0xAE
#define SSD1306_DISPLAYON                       0xAF
#define SSD1306_SETDISPLAYOFFSET                0xD3
#define SSD1306_SETCOMPINS                      0xDA
#define SSD1306_SETVCOMDETECT                   0xDB
#define SSD1306_SETDISPLAYCLOCKDIV              0xD5
#define SSD1306_SETPRECHARGE                    0xD9
#define SSD1306_SETMULTIPLEX                    0xA8
#define SSD1306_SETLOWCOLUMN                    0x00
#define SSD1306_SETHIGHCOLUMN                   0x10
#define SSD1306_SETSTARTLINE                    0x40
#define SSD1306_MEMORYMODE                      0x20
#define SSD1306_COLUMNADDR                      0x21
#define SSD1306_PAGEADDR                        0x22
#define SSD1306_COMSCANINC                      0xC0
#define SSD1306_COMSCANDEC                      0xC8
#define SSD1306_SEGREMAP                        0xA0
#define SSD1306_CHARGEPUMP                      0x8D
#define SSD1306_EXTERNALVCC                     0x1
#define SSD1306_SWITCHCAPVCC                    0x2

/* 颜色定义 */
#define SSD1306_COLOR_BLACK     0x00
#define SSD1306_COLOR_WHITE     0x01
#define SSD1306_COLOR_INVERT    0x02

/* 字体大小定义 */
typedef enum {
    SSD1306_FontSize_7x10 = 0,
    SSD1306_FontSize_11x18,
    SSD1306_FontSize_16x26
} SSD1306_FontSize_t;

/* 函数声明 */
uint8_t SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_ToggleInvert(void);
void SSD1306_Fill(uint8_t Color);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_GotoXY(uint8_t x, uint8_t y);
char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color);
char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color);
void SSD1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void SSD1306_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void SSD1306_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void SSD1306_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint8_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color);
void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint8_t color);

/* 私有函数声明 */
void SSD1306_WriteCommand(uint8_t command);
void SSD1306_WriteData(uint8_t data);
void SSD1306_WriteDataBuffer(uint8_t* data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H */
