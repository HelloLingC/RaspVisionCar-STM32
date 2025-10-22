#include "rasp_comm.h"
#include "motor.h"
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
            continue;
        }
        if(raw_str[i] == ':') {
            continue;
        }
        cmd_str[cmd_idx++] = raw_str[i];

    }

    return 1;
}

// 执行命令
void rasp_execute_command(const rasp_command_t* cmd) {
    if (strcmp(cmd->cmd, "MF") == 0) {
        handle_motor_forward(&cmd->params);
        rasp_send_ack("电机直行命令执行成功");
    }
    else if (strcmp(cmd->cmd, "MT") == 0) {
        handle_motor_turn(&cmd->params);
        rasp_send_ack("电机转向命令执行成功");
    }
    else if (strcmp(cmd->cmd, "BEZ") == 0) {
        handle_buzzer(&cmd->params);
        rasp_send_ack("蜂鸣器命令执行成功");
    }
    else if (strcmp(cmd->cmd, "SPK") == 0) {
        handle_speaker(&cmd->params);
        rasp_send_ack("声音命令执行成功");
    }
    else {
        rasp_send_error("未知命令");
    }
}

// 处理电机直行命令
void handle_motor_forward(const cmd_params_t* params) {
    if (params->speed >= 0 && params->speed <= 100) {
        Motor_Set_Speed(params->speed);
        char speed_str[16];
        snprintf(speed_str, sizeof(speed_str), "%d", params->speed);
        rasp_send_ack_with_data("电机直行命令执行成功", "speed", speed_str);
    } else {
        rasp_send_error("速度参数超出范围(0-100)");
    }
}

// 处理电机转向命令
void handle_motor_turn(const cmd_params_t* params) {
    // 这里需要根据具体的转向逻辑来实现
    // 暂时使用速度控制
    if (params->speed >= 0 && params->speed <= 100) {
        Motor_Set_Speed(params->speed);
        // TODO: 实现转向逻辑
        
        char response_data[64];
        snprintf(response_data, sizeof(response_data), "电机转向命令执行成功 - 速度:%d, 角度:%d", 
                params->speed, params->angle);
        rasp_send_ack(response_data);
    } else {
        rasp_send_error("速度参数超出范围(0-100)");
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
        snprintf(info_message, sizeof(info_message), "[%lu] INFO: %s\r\n", HAL_GetTick(), buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)info_message, strlen(info_message), HAL_MAX_DELAY);
    }
}
