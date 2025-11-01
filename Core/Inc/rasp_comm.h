#ifndef __RASP_COMM_H__
#define __RASP_COMM_H__

#include "usart.h"
// #include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct {
    char cmd[16];
    char param[16];
} Rasp_Command_t;

void rasp_comm_init(void);
int rasp_parse_command(const char* raw_str);

#endif /* __RASP_COMM_H__ */
