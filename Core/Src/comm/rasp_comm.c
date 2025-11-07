#include "rasp_comm.h"
#include "main.h"
// #include "cJSON.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "buzzer.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"
#include "motor.h"

#define RX_BUF_SIZE 64

uint8_t rx_buffer[RX_BUF_SIZE];
uint8_t rx_index = 0;

extern DMA_HandleTypeDef hdma_usart1_rx;
void rasp_comm_init(void) {
    memset(rx_buffer, 0, RX_BUF_SIZE);

    // Start UART receive interrupt
    HAL_StatusTypeDef status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
    if(status != HAL_OK) {
        usart_error("HAL_UART_Receive_DMA failed");
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    // HAL_UART_Receive_IT(&huart1, rx_buffer, 6);
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

void parse_commands(const char *str, Rasp_Command_t *cmd_list, int *cmd_list_index);
extern int32_t sOdometerLeft;
extern int32_t sOdometerRight;
float turn_error = 0;
uint8_t host_signal = 0;
// I was planning to use cJSON at the beginning, but it's too heavy for this project
int rasp_parse_commands() {
    const char* raw_str = (const char*) rx_buffer;
    // usart_info("Received: %s", raw_str);

    if (strcmp(raw_str, "start\n") == 0) {
        system_stop_flag = 0;
    } else if (strcmp(raw_str, "stop\n") == 0) {
        system_stop_flag = 1;
    } else if (strcmp(raw_str, "beep\n") == 0) {
        buzzer_on(5);
    } else if (strcmp(raw_str, "reset\n") == 0) {
        turn_error = 0;
        sOdometerLeft = 0;
        sOdometerRight = 0;
    }

    // const char* turn_angle = starts_with(raw_str, "ta:");
    Rasp_Command_t cmd_list[16];
    int cmd_list_index = 0;

    parse_commands((char*)raw_str, cmd_list, &cmd_list_index);
    for (int i = 0; i < cmd_list_index; i++) {
        if (strcmp(cmd_list[i].cmd, "ta") == 0) {
            // Turn Angle
            motor_controller_set_turn_angle(atoi(cmd_list[i].param));
            // usart_info("turn_angle: %d", sTurnAngle);
        } else if (strcmp(cmd_list[i].cmd, "lv") == 0) {
            // Left Velocity
            int16_t left_rpm = atoi(cmd_list[i].param);
            motor_controller_set_left_target_rpm(left_rpm);
        } else if (strcmp(cmd_list[i].cmd, "rv") == 0) {
            // Right Velocity
            int16_t right_rpm = atoi(cmd_list[i].param);
            motor_controller_set_right_target_rpm(right_rpm);
        } else if(strcmp(cmd_list[i].cmd, "cv") == 0) {
            // Data from OpenCV module in Host
            turn_error = atof(cmd_list[i].param);
        } else if(strcmp(cmd_list[i].cmd, "sig") == 0) {
            // Signal from the Host
            // 0:red  1:green
            host_signal = atoi(cmd_list[i].param);
            if(host_signal == 0) {

            } else if(host_signal == 1) {
                system_stop_flag = 0;
            }
        }
    }
    return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
    if (huart->Instance == USART1) {
        // notice that some ppl say size may can be real_size-1 or +1
        rx_buffer[size] = '\0'; 
        rasp_parse_commands();
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
         // 关闭DMA过半中断
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart->Instance == USART1) {
//         // rasp_parse_commands();
//         usart_info("Received: %s", rx_buffer);
//         // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, RX_BUF_SIZE);
//         HAL_UART_Receive_IT(&huart1, rx_buffer, 6);
//     }
// }

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
