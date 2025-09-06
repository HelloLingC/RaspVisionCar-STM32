#include <string.h>
#include "main.h"
#include "usart.h"
#include "serial_out.h"

// JustFloat Protocol
void send_float_binary(float data) {
    float_union_t float_union;
    float_union.f = data;
    HAL_UART_Transmit(&huart1, float_union.bytes, 4, HAL_MAX_DELAY);
}

void send_tail() {
    uint8_t tail[] = {0x0D, 0x0A}; // CR LF
    HAL_UART_Transmit(&huart1, tail, 2, HAL_MAX_DELAY);
}

void send_frame(struct Frame frame) {
    for (int i = 0; i < CH_COUNT; i++) {
        send_float_binary(frame.fdata[i]);
    }
    send_tail();
}

void send_to_serial(char *ptr) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, strlen(ptr), HAL_MAX_DELAY);
}
