/**
 * @file ssd1306.c
 * @brief SSD1306 OLED显示屏驱动源文件
 * @author STM32 Vision Car Project
 * @date 2025
 */

#include "ssd1306.h"
#include <stdlib.h>
#include <string.h>

/* 私有变量 */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint8_t SSD1306_CurrentX = 0;
static uint8_t SSD1306_CurrentY = 0;
static uint8_t SSD1306_Inverted = 0;

/* 私有函数实现 */
void SSD1306_WriteCommand(uint8_t command) {
    uint8_t data[2] = {0x00, command}; // 0x00表示命令
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

void SSD1306_WriteData(uint8_t data) {
    uint8_t data_array[2] = {0x40, data}; // 0x40表示数据
    HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, data_array, 2, HAL_MAX_DELAY);
}

void SSD1306_WriteDataBuffer(uint8_t* data, uint16_t length) {
    uint8_t* buffer = (uint8_t*)malloc((length + 1) * sizeof(uint8_t));
    if (buffer != NULL) {
        buffer[0] = 0x40; // 数据模式
        memcpy(&buffer[1], data, length);
        HAL_I2C_Master_Transmit(&hi2c1, SSD1306_I2C_ADDR, buffer, length + 1, HAL_MAX_DELAY);
        free(buffer);
    }
}

/* 公共函数实现 */
uint8_t SSD1306_Init(void) {
    /* 等待显示屏启动 */
    HAL_Delay(100);
    
    /* 初始化序列 */
    SSD1306_WriteCommand(SSD1306_DISPLAYOFF);                    // 0xAE
    SSD1306_WriteCommand(SSD1306_SETDISPLAYCLOCKDIV);            // 0xD5
    SSD1306_WriteCommand(0x80);                                  // 建议值
    SSD1306_WriteCommand(SSD1306_SETMULTIPLEX);                  // 0xA8
    SSD1306_WriteCommand(SSD1306_HEIGHT - 1);                    // 0x3F
    SSD1306_WriteCommand(SSD1306_SETDISPLAYOFFSET);              // 0xD3
    SSD1306_WriteCommand(0x0);                                   // 无偏移
    SSD1306_WriteCommand(SSD1306_SETSTARTLINE | 0x0);            // 0x40
    SSD1306_WriteCommand(SSD1306_CHARGEPUMP);                    // 0x8D
    SSD1306_WriteCommand(0x14);                                  // 内部VCC
    SSD1306_WriteCommand(SSD1306_MEMORYMODE);                    // 0x20
    SSD1306_WriteCommand(0x00);                                  // 0x0 水平寻址模式
    SSD1306_WriteCommand(SSD1306_SEGREMAP | 0x1);                // 0xA1
    SSD1306_WriteCommand(SSD1306_COMSCANDEC);                    // 0xC8
    SSD1306_WriteCommand(SSD1306_SETCOMPINS);                    // 0xDA
    SSD1306_WriteCommand(0x12);                                  // 0x12
    SSD1306_WriteCommand(SSD1306_SETCONTRAST);                   // 0x81
    SSD1306_WriteCommand(0xCF);                                  // 0xCF
    SSD1306_WriteCommand(SSD1306_SETPRECHARGE);                  // 0xD9
    SSD1306_WriteCommand(0xF1);                                  // 0xF1
    SSD1306_WriteCommand(SSD1306_SETVCOMDETECT);                 // 0xDB
    SSD1306_WriteCommand(0x40);                                  // 0x40
    SSD1306_WriteCommand(SSD1306_DISPLAYALLON_RESUME);           // 0xA4
    SSD1306_WriteCommand(SSD1306_NORMALDISPLAY);                 // 0xA6
    
    /* 清空缓冲区 */
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    
    /* 更新显示 */
    SSD1306_UpdateScreen();
    
    /* 开启显示 */
    SSD1306_WriteCommand(SSD1306_DISPLAYON);                     // 0xAF
    
    return 0; // 成功
}

void SSD1306_UpdateScreen(void) {
    uint8_t m;
    
    for (m = 0; m < SSD1306_PAGES; m++) {
        SSD1306_WriteCommand(0xB0 + m);                          // 设置页地址
        SSD1306_WriteCommand(0x00);                              // 设置低列地址
        SSD1306_WriteCommand(0x10);                              // 设置高列地址
        
        /* 写入一页数据 */
        SSD1306_WriteDataBuffer(&SSD1306_Buffer[SSD1306_WIDTH * m], SSD1306_WIDTH);
    }
}

void SSD1306_ToggleInvert(void) {
    SSD1306_Inverted = !SSD1306_Inverted;
    SSD1306_WriteCommand(SSD1306_Inverted ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
}

void SSD1306_Fill(uint8_t Color) {
    uint32_t i;
    
    for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (Color == SSD1306_COLOR_BLACK) ? 0x00 : 0xFF;
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    
    if (SSD1306_Inverted) {
        color = (uint8_t)!color;
    }
    
    if (color == SSD1306_COLOR_WHITE) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void SSD1306_GotoXY(uint8_t x, uint8_t y) {
    SSD1306_CurrentX = x;
    SSD1306_CurrentY = y;
}

char SSD1306_Putc(char ch, FontDef_t* Font, uint8_t color) {
    uint32_t i, b, j;
    
    if (SSD1306_WIDTH <= (SSD1306_CurrentX + Font->FontWidth) ||
        SSD1306_HEIGHT <= (SSD1306_CurrentY + Font->FontHeight)) {
        return 0;
    }
    
    for (i = 0; i < Font->FontHeight; i++) {
        b = Font->data[(ch - 32) * Font->FontHeight + i];
        for (j = 0; j < Font->FontWidth; j++) {
            if ((b << j) & 0x8000) {
                SSD1306_DrawPixel(SSD1306_CurrentX + j, (SSD1306_CurrentY + i), (uint8_t) color);
            } else {
                SSD1306_DrawPixel(SSD1306_CurrentX + j, (SSD1306_CurrentY + i), (uint8_t)!color);
            }
        }
    }
    
    SSD1306_CurrentX += Font->FontWidth;
    
    return ch;
}

char SSD1306_Puts(char* str, FontDef_t* Font, uint8_t color) {
    char* p = str;
    
    while (*p) {
        if (SSD1306_Putc(*p, Font, color) != *p) {
            return *p;
        }
        p++;
    }
    
    return *p;
}

void SSD1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color) {
    int16_t dx, dy, sx, sy, err, e2;
    
    dx = (x1 < x2) ? (x2 - x1) : (x1 - x2);
    dy = (y1 < y2) ? (y2 - y1) : (y1 - y2);
    sx = (x1 < x2) ? 1 : -1;
    sy = (y1 < y2) ? 1 : -1;
    err = ((dx > dy) ? dx : -dy) / 2;
    
    while (1) {
        SSD1306_DrawPixel(x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        e2 = err;
        if (e2 > -dx) {
            err -= dy;
            x1 += sx;
        }
        if (e2 < dy) {
            err += dx;
            y1 += sy;
        }
    }
}

void SSD1306_DrawRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    SSD1306_DrawLine(x, y, x + w, y, color);         // 上边
    SSD1306_DrawLine(x, y, x, y + h, color);         // 左边
    SSD1306_DrawLine(x + w, y, x + w, y + h, color); // 右边
    SSD1306_DrawLine(x, y + h, x + w, y + h, color); // 下边
}

void SSD1306_DrawFilledRectangle(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color) {
    uint8_t i;
    for (i = 0; i <= h; i++) {
        SSD1306_DrawLine(x, y + i, x + w, y + i, color);
    }
}

void SSD1306_DrawTriangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t x3, uint8_t y3, uint8_t color) {
    SSD1306_DrawLine(x1, y1, x2, y2, color);
    SSD1306_DrawLine(x2, y2, x3, y3, color);
    SSD1306_DrawLine(x3, y3, x1, y1, color);
}

void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    SSD1306_DrawPixel(x0, y0 + r, color);
    SSD1306_DrawPixel(x0, y0 - r, color);
    SSD1306_DrawPixel(x0 + r, y0, color);
    SSD1306_DrawPixel(x0 - r, y0, color);
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        SSD1306_DrawPixel(x0 + x, y0 + y, color);
        SSD1306_DrawPixel(x0 - x, y0 + y, color);
        SSD1306_DrawPixel(x0 + x, y0 - y, color);
        SSD1306_DrawPixel(x0 - x, y0 - y, color);
        SSD1306_DrawPixel(x0 + y, y0 + x, color);
        SSD1306_DrawPixel(x0 - y, y0 + x, color);
        SSD1306_DrawPixel(x0 + y, y0 - x, color);
        SSD1306_DrawPixel(x0 - y, y0 - x, color);
    }
}

void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    
    SSD1306_DrawLine(x0, y0 - r, x0, y0 + r, color);
    
    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        
        SSD1306_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
        SSD1306_DrawLine(x0 - x, y0 - y, x0 + x, y0 - y, color);
        SSD1306_DrawLine(x0 - y, y0 + x, x0 + y, y0 + x, color);
        SSD1306_DrawLine(x0 - y, y0 - x, x0 + y, y0 - x, color);
    }
}

void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint8_t color) {
    int16_t byteWidth = (w + 7) / 8;
    uint8_t byte = 0;
    
    for (int16_t j = 0; j < h; j++, y++) {
        for (int16_t i = 0; i < w; i++) {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = bitmap[j * byteWidth + i / 8];
            }
            if (byte & 0x80) {
                SSD1306_DrawPixel(x + i, y, color);
            }
        }
    }
}
