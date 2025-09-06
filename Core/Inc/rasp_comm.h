#ifndef __RASP_COMM_H__
#define __RASP_COMM_H__

#include "main.h"
#include "usart.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// 通信协议配置
#define MAX_CMD_LENGTH 256
#define MAX_RESPONSE_LENGTH 128
#define CMD_PREFIX "CMD:"
#define ACK_PREFIX "ACK:"
#define ERR_PREFIX "ERR:"
#define LOG_PREFIX "LOG:"

// 命令类型枚举
typedef enum {
    CMD_MF = 0,     // 电机直行
    CMD_MT,         // 电机转向
    CMD_BEZ,        // 蜂鸣器
    CMD_SPK,        // 声音
    CMD_UNKNOWN     // 未知命令
} cmd_type_t;

// 命令参数结构体
typedef struct {
    int speed;      // 速度参数
    int angle;      // 角度参数
    char direction[16]; // 方向参数
} cmd_params_t;

// 命令结构体
typedef struct {
    char cmd[8];            // 命令字符串
    double timestamp;       // 时间戳
    cmd_params_t params;    // 参数
} rasp_command_t;

// 响应结构体
typedef struct {
    char status[8];         // 状态: "ok" 或 "error"
    char data[64];          // 响应数据
} rasp_response_t;

// 函数声明
void rasp_comm_init(void);
void rasp_comm_process(void);
void rasp_send_ack(const char* data);
void rasp_send_ack_with_data(const char* data, const char* key, const char* value);
void rasp_send_error(const char* error_msg);
int rasp_parse_command(const char* json_str, rasp_command_t* cmd);
void rasp_execute_command(const rasp_command_t* cmd);

// 命令处理函数
void handle_motor_forward(const cmd_params_t* params);
void handle_motor_turn(const cmd_params_t* params);
void handle_buzzer(const cmd_params_t* params);
void handle_speaker(const cmd_params_t* params);

// USART printf
void usart_log(const char *format, ...);
void usart_debug(const char *format, ...);
void usart_error(const char *format, ...);
void usart_info(const char *format, ...);

#endif /* __RASP_COMM_H__ */
