#ifndef INC_COMMAND_H_
#define INC_COMMAND_H_

#include "main.h"
#include <string.h>

typedef struct {
    uint32_t overflow;
    uint32_t len_err;
    uint32_t framing_err;
    uint32_t output_too_small;
} Command_Stats_t;

uint16_t Command_Write(const uint8_t *data, uint16_t length);
uint8_t Command_GetCommand(uint8_t *command, uint8_t command_capacity);
void Command_GetStats(Command_Stats_t *stats);
void Command_ResetStats(void);

#endif /* INC_COMMAND_H_ */
