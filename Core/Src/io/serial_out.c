#include <string.h>
#include "main.h"
#include "usart.h"
#include "serial_out.h"

// JustFloart Protocol

void send_float_binary(float data) {
    float_union_t converter;
    converter.f = data;
    
    // Send the 4 bytes of the float
    HAL_UART_Transmit(&huart1, converter.bytes, 4, HAL_MAX_DELAY);
}

void send_tail() {
	uint8_t tail[4];
	tail[0] = 0x00;
    tail[1] = 0x00;
    tail[2] = 0x80;
    tail[3] = 0x7f;
	HAL_UART_Transmit(&huart1, tail, 4, HAL_MAX_DELAY);
}

void send_frame(struct Frame frame) {
	char tail[4];
	tail[0] = 0x00;
    tail[1] = 0x00;
    tail[2] = 0x80;
    tail[3] = 0x7f;
	char *ptr;
 
    for (int i = 0; i <  sizeof(frame.fdata); i++) {
        // Convert each float and move pointer
        ptr += sprintf(ptr, "%.3f", frame.fdata[i]);
        
        
    }
	send_to_serial(ptr);
	send_to_serial(tail);
}

void send_to_serial(char *ptr) {
	HAL_UART_Transmit(&huart1,(uint8_t*)ptr, strlen(ptr), HAL_MAX_DELAY);
}