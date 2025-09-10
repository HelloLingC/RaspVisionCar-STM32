#ifndef __DELAY_H
#define __DELAY_H
#include "stm32f1xx_hal.h"
void systick_delay_ms(uint32_t ms);
void delay_ms(uint32_t ms);
#endif