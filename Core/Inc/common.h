#ifndef __COMMON_H
#define __COMMON_H
#include <stdint.h>

extern volatile uint8_t system_stop_flag;

void uart1_rx_process(void);

void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
#endif