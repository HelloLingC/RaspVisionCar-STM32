//
// Created by lingc on 11/1/2025.
//

#ifndef RASPVISIONCAR_COMMON_H
#define RASPVISIONCAR_COMMON_H
#include "semphr.h"

#define RX_BUFFER_SIZE 128
extern uint8_t rxBuffer[RX_BUFFER_SIZE];
extern SemaphoreHandle_t xSerialSemaphore;

void usart_info(const char *format, ...);
void usart_error(const char *format, ...);
#endif // RASPVISIONCAR_COMMON_H
