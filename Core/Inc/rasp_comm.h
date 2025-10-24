#ifndef __RASP_COMM_H__
#define __RASP_COMM_H__

#include "main.h"
#include "usart.h"
// #include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 通信协议配置
#define MAX_CMD_LENGTH 256
#define MAX_RESPONSE_LENGTH 128

#define LOG_PREFIX "RASP_COMM:"

// 命令类型枚举
typedef enum {
    CMD_MF = 0,     // 电机直行
    CMD_MT,         // 电机转向
    CMD_BEZ,        // 蜂鸣器
    CMD_SPK,        // 声音
    CMD_UNKNOWN     // 未知命令
} cmd_type_t;

// 命令结构体
typedef struct {
    char cmd[16];            // 命令字符串
    char params[32];        // 参数
    double timestamp;       // 时间戳
} rasp_command_t;

// 函数声明
void rasp_comm_init(void);
void rasp_comm_process(void);
int rasp_parse_command(const char* json_str, rasp_command_t* cmd);
void rasp_execute_command(const rasp_command_t* cmd);

// 命令处理函数
void handle_motor_forward(const char* params);
void handle_motor_turn(const char* params);

// USART printf
void usart_log(const char *format, ...);
void usart_debug(const char *format, ...);
void usart_error(const char *format, ...);
void usart_info(const char *format, ...);

#endif /* __RASP_COMM_H__ */
