#include "rasp_comm.h"
#include "motor.h"
#include "cJSON.h"
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
    
    usart_info("树莓派通信协议初始化完成");
}

// 处理接收到的数据
void rasp_comm_process(void) {
    if (rx_complete) {
        rx_buffer[rx_index] = '\0'; // 确保字符串结束
        
        // 检查是否是命令前缀
        if (strncmp((char*)rx_buffer, CMD_PREFIX, strlen(CMD_PREFIX)) == 0) {
            char* json_start = (char*)rx_buffer + strlen(CMD_PREFIX);
            
            usart_debug("收到命令: %s", json_start);
            
            rasp_command_t cmd;
            if (rasp_parse_command(json_start, &cmd) == 0) {
                usart_info("执行命令: %s", cmd.cmd);
                rasp_execute_command(&cmd);
            } else {
                usart_error("JSON解析失败: %s", json_start);
                rasp_send_error("JSON解析失败");
            }
        } else {
            usart_debug("收到非命令数据: %s", (char*)rx_buffer);
        }
        
        // 重置接收状态
        rx_index = 0;
        rx_complete = 0;
        memset(rx_buffer, 0, MAX_CMD_LENGTH);
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_index], 1);
    }
}

// 发送ACK响应
void rasp_send_ack(const char* data) {
    // 创建JSON响应对象
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "ok");
    cJSON_AddStringToObject(json, "data", data);
    
    // 生成JSON字符串
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string != NULL) {
        char response[MAX_RESPONSE_LENGTH];
        snprintf(response, sizeof(response), "%s%s\n", ACK_PREFIX, json_string);
        HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
        
        // 释放内存
        cJSON_free(json_string);
    }
    
    // 清理JSON对象
    cJSON_Delete(json);
}

// 发送带额外数据的ACK响应
void rasp_send_ack_with_data(const char* data, const char* key, const char* value) {
    // 创建JSON响应对象
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "ok");
    cJSON_AddStringToObject(json, "data", data);
    cJSON_AddStringToObject(json, key, value);
    
    // 生成JSON字符串
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string != NULL) {
        char response[MAX_RESPONSE_LENGTH];
        snprintf(response, sizeof(response), "%s%s\n", ACK_PREFIX, json_string);
        HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
        
        // 释放内存
        cJSON_free(json_string);
    }
    
    // 清理JSON对象
    cJSON_Delete(json);
}

// 发送错误响应
void rasp_send_error(const char* error_msg) {
    // 创建JSON错误响应对象
    cJSON *json = cJSON_CreateObject();
    cJSON_AddStringToObject(json, "status", "error");
    cJSON_AddStringToObject(json, "message", error_msg);
    
    // 生成JSON字符串
    char *json_string = cJSON_PrintUnformatted(json);
    if (json_string != NULL) {
        char response[MAX_RESPONSE_LENGTH];
        snprintf(response, sizeof(response), "%s%s\n", ERR_PREFIX, json_string);
        HAL_UART_Transmit(&huart1, (uint8_t*)response, strlen(response), HAL_MAX_DELAY);
        
        // 释放内存
        cJSON_free(json_string);
    }
    
    // 清理JSON对象
    cJSON_Delete(json);
}

// 使用cJSON的JSON解析器
int rasp_parse_command(const char* json_str, rasp_command_t* cmd) {
    // 初始化命令结构体
    memset(cmd, 0, sizeof(rasp_command_t));
    
    // 解析JSON
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        return -1; // JSON解析失败
    }
    
    // 解析cmd字段
    cJSON *cmd_item = cJSON_GetObjectItem(json, "cmd");
    if (cJSON_IsString(cmd_item) && (cmd_item->valuestring != NULL)) {
        strncpy(cmd->cmd, cmd_item->valuestring, sizeof(cmd->cmd) - 1);
        cmd->cmd[sizeof(cmd->cmd) - 1] = '\0';
    } else {
        cJSON_Delete(json);
        return -1;
    }
    
    // 解析timestamp字段
    cJSON *timestamp_item = cJSON_GetObjectItem(json, "timestamp");
    if (cJSON_IsNumber(timestamp_item)) {
        cmd->timestamp = timestamp_item->valuedouble;
    }
    
    // 解析params字段
    cJSON *params_item = cJSON_GetObjectItem(json, "params");
    if (cJSON_IsObject(params_item)) {
        // 解析speed参数
        cJSON *speed_item = cJSON_GetObjectItem(params_item, "speed");
        if (cJSON_IsNumber(speed_item)) {
            cmd->params.speed = (int)speed_item->valuedouble;
        }
        
        // 解析angle参数
        cJSON *angle_item = cJSON_GetObjectItem(params_item, "angle");
        if (cJSON_IsNumber(angle_item)) {
            cmd->params.angle = (int)angle_item->valuedouble;
        }
        
        // 解析direction参数
        cJSON *direction_item = cJSON_GetObjectItem(params_item, "direction");
        if (cJSON_IsString(direction_item) && (direction_item->valuestring != NULL)) {
            strncpy(cmd->params.direction, direction_item->valuestring, sizeof(cmd->params.direction) - 1);
            cmd->params.direction[sizeof(cmd->params.direction) - 1] = '\0';
        }
    }
    
    // 清理JSON对象
    cJSON_Delete(json);
    return 0;
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

// 处理蜂鸣器命令
void handle_buzzer(const cmd_params_t* params) {
    // TODO: 实现蜂鸣器控制
    // 这里可以控制GPIO引脚来驱动蜂鸣器
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // 使用LED作为示例
}

// 处理声音命令
void handle_speaker(const cmd_params_t* params) {
    // TODO: 实现声音播放
    // 这里可以控制PWM或DAC来播放声音
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
