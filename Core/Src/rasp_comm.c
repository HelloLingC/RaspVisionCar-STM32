#include "rasp_comm.h"
#include "main.h"
#include "motor_left.h"
#include "motor_right.h"
// #include "cJSON.h"
#include <stdio.h>
#include <stdarg.h>
#include "buzzer.h"

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
    if (*str == ':')
        return str + 1;    // 返回冒号后的部分
    return NULL;
}

extern volatile int sTurnAngle;
void parse_commands(const char *str, Rasp_Command_t *cmd_list, int *cmd_list_index);
// I was planning to use cJSON at the beginning, but it's too heavy for this project
int rasp_parse_command(const char* raw_str) {
    // const char* turn_angle = starts_with(raw_str, "ta:");
    Rasp_Command_t cmd_list[16];
    int cmd_list_index = 0;
    parse_commands((char*)raw_str, cmd_list, &cmd_list_index);
    for (int i = 0; i < cmd_list_index; i++) {
        if (strcmp(cmd_list[i].cmd, "ta") == 0) {
            sTurnAngle = atoi(cmd_list[i].param);
            // usart_info("turn_angle: %d", sTurnAngle);
        }
    }
    // if (turn_angle) {
    //     sTurnAngle = atoi(turn_angle);
    //     usart_info("turn_angle: %d", sTurnAngle);
    //     return 1;
    // }

    if (strcmp(raw_str, "start") == 0) {
        system_stop_flag = 0;
    } else if (strcmp(raw_str, "stop") == 0) {
        system_stop_flag = 1;
    } else if (strcmp(raw_str, "beep") == 0) {
        buzzer_on(100);
    }
    
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

void parse_commands(const char *str, Rasp_Command_t *cmd_list, int *cmd_list_index) {
    Rasp_Command_t cmd_t = {0};
    const char *p = str;
  
    while (*p && *p != '\n') {
      const char *start = p;
  
      // find separator
      while (*p && *p != ':' && *p != ',' && *p != '\n')
        p++;
  
      size_t len = p - start;
      if (len == 0) {
        if (*p)
          p++;
        continue;
      }
  
      if (*p == ':') {
        // copy command
        snprintf(cmd_t.cmd, sizeof(cmd_t.cmd), "%.*s", (int)len, start);
        p++; // skip ':'
      } else if (*p == ',' || *p == '\0' || *p == '\n') {
        // copy parameter
        snprintf(cmd_t.param, sizeof(cmd_t.param), "%.*s", (int)len, start);
        cmd_list[(*cmd_list_index)++] = cmd_t;
        memset(&cmd_t, 0, sizeof(cmd_t));
  
        if (*p == ',')
          p++; // skip ','
      } else {
        // unexpected character — skip safely
        p++;
      }
    }
  }

// USART printf 相关函数实现

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
