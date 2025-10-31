#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f1xx_hal.h"

void buzzer_init(void);
void buzzer_enqueue(uint16_t duration_ms);
uint16_t buzzer_dequeue(void);
void buzzer_on(uint16_t duration_ms);
void buzzer_off(void);
void buzzer_update(void);

#endif