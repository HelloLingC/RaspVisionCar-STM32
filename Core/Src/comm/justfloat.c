#include <string.h>
#include "main.h"
#include "usart.h"
#include "justfloat.h"

// JustFloat Protocol
void send_float_binary(float data) {
	float_union_t float_union;
	float_union.f = data;
	HAL_UART_Transmit(&huart1, float_union.bytes, 4, HAL_MAX_DELAY);
}

void send_tail(void) {
	uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f}; // JustFloat协议帧尾
	HAL_UART_Transmit(&huart1, tail, 4, HAL_MAX_DELAY);
}

void send_frame(const struct Frame *frame) {
	// 将整帧打包后一次性发送，减少多次HAL调用
	uint8_t buf[CH_COUNT * 4 + 4];
	float_union_t fu;
	for (int i = 0; i < CH_COUNT; i++) {
		fu.f = frame->fdata[i];
		buf[i * 4 + 0] = fu.bytes[0];
		buf[i * 4 + 1] = fu.bytes[1];
		buf[i * 4 + 2] = fu.bytes[2];
		buf[i * 4 + 3] = fu.bytes[3];
	}
	buf[CH_COUNT * 4 + 0] = frame->tail[0];
	buf[CH_COUNT * 4 + 1] = frame->tail[1];
	buf[CH_COUNT * 4 + 2] = frame->tail[2];
	buf[CH_COUNT * 4 + 3] = frame->tail[3];
	HAL_UART_Transmit(&huart1, buf, sizeof(buf), HAL_MAX_DELAY);
}

void send_frame_data(const float *data) {
	struct Frame frame;
	for (int i = 0; i < CH_COUNT; i++) {
		frame.fdata[i] = data[i];
	}
	frame.tail[0] = 0x00;
	frame.tail[1] = 0x00;
	frame.tail[2] = 0x80;
	frame.tail[3] = 0x7f;
	send_frame(&frame);
}

// 便捷函数实现
void init_frame(struct Frame *frame) {
    // 初始化tail为正确的帧尾
    frame->tail[0] = 0x00;
    frame->tail[1] = 0x00;
    frame->tail[2] = 0x80;
    frame->tail[3] = 0x7f;
    
    // 初始化数据为0
    for (int i = 0; i < CH_COUNT; i++) {
        frame->fdata[i] = 0.0f;
    }
}

void create_frame(struct Frame *frame, float *data) {
    // 复制数据
    for (int i = 0; i < CH_COUNT; i++) {
        frame->fdata[i] = data[i];
    }
    
    // 设置正确的帧尾
    frame->tail[0] = 0x00;
    frame->tail[1] = 0x00;
    frame->tail[2] = 0x80;
    frame->tail[3] = 0x7f;
}
