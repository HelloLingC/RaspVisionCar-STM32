#include "rasp_comm.h"
#include "main.h"
#include "motor_left.h"
#include "motor_right.h"
// #include "cJSON.h"
#include <stdio.h>
#include <stdarg.h>

// 接收缓冲区
static uint8_t rx_buffer[MAX_CMD_LENGTH];
static uint16_t rx_index = 0;
static uint8_t rx_complete = 0;

// 初始化通信协议
void rasp_comm_init(void) {
    rx_index = 0;
    rx_complete = 0;
    memset(rx_buffer, 0, MAX_CMD_LENGTH);

    // 启动UART接收中断
    HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);

    // usart_info("树莓派通信协议初始化完成");
}

// 处理接收到的数据
void rasp_comm_process(void) {
    if (rx_complete) {
        rx_buffer[rx_index] = '\0'; // 确保字符串结束

        rasp_command_t cmd;
        if (rasp_parse_command(rx_buffer, &cmd) == 1) {
            rasp_execute_command(&cmd);
        } else {
            // usart_error("JSON解析失败: %s", rx_buffer);
        }

        // 重置接收状态
        rx_index = 0;
        rx_complete = 0;
        memset(rx_buffer, 0, MAX_CMD_LENGTH);
        
        // Continue UART receive
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

// I was planning to use cJSON at the beginning, but it's too heavy for this project
int rasp_parse_command(const char* raw_str, rasp_command_t* cmd) {
    // 初始化命令结构体
    memset(cmd, 0, sizeof(rasp_command_t));
    uint8_t cmd_idx = 0;
    char* cmd_str = malloc(MAX_CMD_LENGTH);

    for(int i = 0; raw_str[i] != '\0'; i++) {
        if(raw_str[i]== ',' || raw_str[i] == ' ' || raw_str[i] == '\n') {
            strcpy(cmd->params, cmd_str);
            cmd_idx = 0;
            memset(cmd_str, 0, MAX_CMD_LENGTH);
            rasp_execute_command(cmd);
            continue;
        }
        if(raw_str[i] == ':') {
            strcpy(cmd->cmd, cmd_str);
            cmd_idx = 0;
            memset(cmd_str, 0, MAX_CMD_LENGTH);
            continue;
        }
        cmd_str[cmd_idx++] = raw_str[i];

    }

    return 1;
}

// 执行命令
void rasp_execute_command(const rasp_command_t* cmd) {
    if (strcmp(cmd->cmd, "STOP") == 0) {
        // 设置停止标志
        extern volatile uint8_t system_stop_flag;
        system_stop_flag = 1;
    }
    else if (strcmp(cmd->cmd, "START") == 0) {
        // 清除停止标志
        extern volatile uint8_t system_stop_flag;
        system_stop_flag = 0;
    }
    else if (strcmp(cmd->cmd, "MT") == 0) {
        // handle_motor_turn(&cmd->params);
    }
    else if (strcmp(cmd->cmd, "BEZ") == 0) {
        // handle_buzzer(&cmd->params);
    }
    else {
        // Unknown command
    }
}

// UART接收完成 1byte 回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (rx_buffer[rx_index] == '\n' || rx_buffer[rx_index] == '\r') {
            rx_complete = 1;
        } else if (rx_index < MAX_CMD_LENGTH - 1) {
            rx_index++;
            HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
        } else {
            // 缓冲区溢出，重置
            rx_index = 0;
            rx_complete = 0;
            memset(rx_buffer, 0, MAX_CMD_LENGTH);
            HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
        }
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
