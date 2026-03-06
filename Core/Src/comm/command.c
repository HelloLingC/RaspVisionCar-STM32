#include "command.h"
#include <stddef.h>
#include <stdint.h>

#define BUFFER_SIZE 128U
#define FRAME_SOF1 0xA5U
#define FRAME_SOF2 0x5AU
#define FRAME_MIN_LEN 8U
#define FRAME_MAX_PAYLOAD 48U
#define FRAME_MAX_LEN (FRAME_MIN_LEN + FRAME_MAX_PAYLOAD)

static uint8_t s_buffer[BUFFER_SIZE];
static uint16_t s_read_index = 0U;
static uint16_t s_write_index = 0U;
static uint16_t s_count = 0U;
static Command_Stats_t s_stats = {0};

static uint8_t buffer_peek(uint16_t offset)
{
    uint16_t index = (uint16_t)((s_read_index + offset) % BUFFER_SIZE);
    return s_buffer[index];
}

static void buffer_drop(uint16_t length)
{
    if (length > s_count) {
        length = s_count;
    }

    s_read_index = (uint16_t)((s_read_index + length) % BUFFER_SIZE);
    s_count = (uint16_t)(s_count - length);
}

static void buffer_pop(uint8_t *out, uint16_t length)
{
    uint16_t i;
    for (i = 0U; i < length; ++i) {
        out[i] = s_buffer[s_read_index];
        s_read_index = (uint16_t)((s_read_index + 1U) % BUFFER_SIZE);
    }
    s_count = (uint16_t)(s_count - length);
}

uint16_t Command_Write(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint16_t written = 0U;
    uint16_t capacity_left = (uint16_t)(BUFFER_SIZE - s_count);

    if (data == NULL || length == 0U) {
        return 0U;
    }

    if (length > capacity_left) {
        s_stats.overflow += (uint32_t)(length - capacity_left);
        length = capacity_left;
    }

    for (i = 0U; i < length; ++i) {
        s_buffer[s_write_index] = data[i];
        s_write_index = (uint16_t)((s_write_index + 1U) % BUFFER_SIZE);
        ++written;
    }

    s_count = (uint16_t)(s_count + written);
    return written;
}

uint8_t Command_GetCommand(uint8_t *command, uint8_t command_capacity)
{
    while (s_count > 0U) {
        uint8_t head = buffer_peek(0U);
        uint8_t payload_len;
        uint8_t total_len;

        if (head != FRAME_SOF1) {
            ++s_stats.framing_err;
            buffer_drop(1U);
            continue;
        }

        if (s_count < 2U) {
            return 0U;
        }

        if (buffer_peek(1U) != FRAME_SOF2) {
            ++s_stats.framing_err;
            buffer_drop(1U);
            continue;
        }

        if (s_count < 6U) {
            return 0U;
        }

        payload_len = buffer_peek(5U);
        if (payload_len > FRAME_MAX_PAYLOAD) {
            ++s_stats.len_err;
            buffer_drop(1U);
            continue;
        }

        total_len = (uint8_t)(FRAME_MIN_LEN + payload_len);
        if (total_len < FRAME_MIN_LEN || total_len > FRAME_MAX_LEN) {
            ++s_stats.len_err;
            buffer_drop(1U);
            continue;
        }

        if (s_count < total_len) {
            return 0U;
        }

        if (total_len > command_capacity) {
            ++s_stats.output_too_small;
            buffer_drop(total_len);
            continue;
        }

        buffer_pop(command, total_len);
        return total_len;
    }

    return 0U;
}

void Command_GetStats(Command_Stats_t *stats)
{
    if (stats == NULL) {
        return;
    }

    *stats = s_stats;
}

void Command_ResetStats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}
