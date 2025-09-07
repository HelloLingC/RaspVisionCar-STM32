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

void send_tail() {
    uint8_t tail[] = {0x00, 0x00, 0x80, 0x7f}; // JustFloat协议帧尾
    HAL_UART_Transmit(&huart1, tail, 4, HAL_MAX_DELAY);
}

void send_frame(struct Frame frame) {
    // 发送浮点数据
    for (int i = 0; i < CH_COUNT; i++) {
        send_float_binary(frame.fdata[i]);
    }
    // 发送帧尾
    HAL_UART_Transmit(&huart1, frame.tail, 4, HAL_MAX_DELAY);
}

// 数据解析函数实现
int find_frame_start(uint8_t *buffer, int buffer_len) {
    // 查找帧尾模式 {0x00, 0x00, 0x80, 0x7f}
    uint8_t tail_pattern[] = {0x00, 0x00, 0x80, 0x7f};
    
    for (int i = 0; i <= buffer_len - 4; i++) {
        if (memcmp(&buffer[i], tail_pattern, 4) == 0) {
            // 找到帧尾，计算帧开始位置
            int frame_start = i - (CH_COUNT * 4);
            if (frame_start >= 0) {
                return frame_start;
            }
        }
    }
    return -1; // 未找到完整帧
}

int validate_frame_tail(uint8_t *buffer, int offset) {
    uint8_t expected_tail[] = {0x00, 0x00, 0x80, 0x7f};
    return memcmp(&buffer[offset], expected_tail, 4) == 0;
}

int parse_frame(uint8_t *buffer, int buffer_len, struct Frame *frame) {
    // 查找帧开始位置
    int frame_start = find_frame_start(buffer, buffer_len);
    if (frame_start < 0) {
        return -1; // 未找到完整帧
    }
    
    // 解析浮点数据
    float_union_t float_union;
    for (int i = 0; i < CH_COUNT; i++) {
        int offset = frame_start + (i * 4);
        if (offset + 4 > buffer_len) {
            return -1; // 数据不完整
        }
        
        // 复制字节数据到union
        memcpy(float_union.bytes, &buffer[offset], 4);
        frame->fdata[i] = float_union.f;
    }
    
    // 验证帧尾
    int tail_offset = frame_start + (CH_COUNT * 4);
    if (!validate_frame_tail(buffer, tail_offset)) {
        return -1; // 帧尾验证失败
    }
    
    // 复制帧尾到结构体
    memcpy(frame->tail, &buffer[tail_offset], 4);
    
    return frame_start + (CH_COUNT * 4) + 4; // 返回下一帧的起始位置
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
