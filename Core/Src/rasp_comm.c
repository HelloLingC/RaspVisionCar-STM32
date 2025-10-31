#include "rasp_comm.h"
#include "main.h"
#include "motor_left.h"
#include "motor_right.h"
// #include "cJSON.h"
#include <stdio.h>
#include <stdarg.h>
#include "buzzer.h"

// 接收缓冲区

#define RX_BUF_SIZE 16

uint8_t rx_buffer[RX_BUF_SIZE];
uint8_t rx_byte;
uint8_t rx_index = 0;

// 初始化通信协议
void rasp_comm_init(void) {
    memset(rx_buffer, 0, RX_BUF_SIZE);

    // 启动UART接收中断
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

const char* starts_with(const char *str, const char *prefix) {
    while (*prefix) {
        if (*prefix++ != *str++)
            return NULL;  // 不匹配
    }
    if (*str == ':')       // 确保冒号存在
        return str + 1;    // 返回冒号后的部分
    return NULL;
}

// I was planning to use cJSON at the beginning, but it's too heavy for this project
int rasp_parse_command(const char* raw_str) {
    // usart_info("rasp_parse_command: %s", raw_str);
    const char* turn_angle = starts_with(raw_str, "ta:");
    if (turn_angle) {
        // usart_info("turn_angle: %s", turn_angle);
        return 1;
    }

    if (strcmp(raw_str, "start") == 0) {
        system_stop_flag = 0;
    } else if (strcmp(raw_str, "stop") == 0) {
        system_stop_flag = 1;
    } else if (strcmp(raw_str, "beep") == 0) {
        buzzer_on(100);
    }
    
    // 初始化命令结构体
    // memset(cmd, 0, sizeof(rasp_command_t));
    // uint8_t cmd_idx = 0;
    // char* cmd_str = malloc(MAX_CMD_LENGTH);

    // for(int i = 0; raw_str[i] != '\0'; i++) {
    //     if(raw_str[i]== ',' || raw_str[i] == ' ' || raw_str[i] == '\n') {
    //         strcpy(cmd->params, cmd_str);
    //         cmd_idx = 0;
    //         memset(cmd_str, 0, MAX_CMD_LENGTH);
    //         rasp_execute_command(cmd);
    //         continue;
    //     }
    //     if(raw_str[i] == ':') {
    //         strcpy(cmd->cmd, cmd_str);
    //         cmd_idx = 0;
    //         memset(cmd_str, 0, MAX_CMD_LENGTH);
    //         continue;
    //     }
    //     cmd_str[cmd_idx++] = raw_str[i];

    // }

    return 1;
}

// UART接收完成 1byte 回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_index < RX_BUF_SIZE - 1) {
            rx_buffer[rx_index++] = rx_byte;
        }

        if (rx_byte == '\n' || rx_byte == '\r') {
            rx_buffer[rx_index-1] = '\0';
            rasp_parse_command((char*)rx_buffer);
            rx_index = 0;
        }

        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// USART printf 相关函数实现

/**
 * @brief 通过USART发送日志消息
 * @param format: 格式化字符串
 * @param ...: 可变参数
 */
void usart_log(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;
    
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        char log_message[300];
        snprintf(log_message, sizeof(log_message), "%s%s\r\n", LOG_PREFIX, buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)log_message, strlen(log_message), HAL_MAX_DELAY);
    }
}

/**
 * @brief 通过USART发送调试信息
 * @param format: 格式化字符串
 * @param ...: 可变参数
 */
void usart_debug(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;
    
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        char debug_message[300];
        snprintf(debug_message, sizeof(debug_message), "DEBUG: %s\r\n", buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)debug_message, strlen(debug_message), HAL_MAX_DELAY);
    }
}

/**
 * @brief 通过USART发送错误信息
 * @param format: 格式化字符串
 * @param ...: 可变参数
 */
void usart_error(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;
    
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        char error_message[300];
        snprintf(error_message, sizeof(error_message), "ERROR: %s\r\n", buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)error_message, strlen(error_message), HAL_MAX_DELAY);
    }
}

/**
 * @brief 通过USART发送信息（带时间戳）
 * @param format: 格式化字符串
 * @param ...: 可变参数
 */
void usart_info(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;
    
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len > 0 && len < sizeof(buffer)) {
        char info_message[300];
        snprintf(info_message, sizeof(info_message), "[%u] INFO: %s\r\n", (unsigned int)HAL_GetTick(), buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)info_message, strlen(info_message), HAL_MAX_DELAY);
    }
}
