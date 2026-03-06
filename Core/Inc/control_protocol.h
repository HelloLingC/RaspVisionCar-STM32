#ifndef CONTROL_PROTOCOL_H
#define CONTROL_PROTOCOL_H

#include <stdint.h>

#define CONTROL_PROTOCOL_SOF1 0xA5U
#define CONTROL_PROTOCOL_SOF2 0x5AU
#define CONTROL_PROTOCOL_VERSION 0x01U
#define CONTROL_PROTOCOL_MAX_PAYLOAD 48U
#define CONTROL_PROTOCOL_MIN_FRAME_LEN 8U
#define CONTROL_PROTOCOL_MAX_FRAME_LEN (CONTROL_PROTOCOL_MIN_FRAME_LEN + CONTROL_PROTOCOL_MAX_PAYLOAD)

typedef enum {
    CONTROL_MSG_SET_TWIST = 0x01,
    CONTROL_MSG_ARM = 0x02,
    CONTROL_MSG_DISARM = 0x03,
    CONTROL_MSG_ESTOP = 0x04,
    CONTROL_MSG_CLEAR_ESTOP = 0x05,
    CONTROL_MSG_HEARTBEAT = 0x06,
    CONTROL_MSG_STATUS = 0x80,
} control_msg_type_t;

typedef enum {
    CONTROL_DECODE_OK = 0,
    CONTROL_DECODE_TOO_SHORT,
    CONTROL_DECODE_BAD_SOF,
    CONTROL_DECODE_BAD_VERSION,
    CONTROL_DECODE_BAD_LEN,
    CONTROL_DECODE_BAD_CRC,
} control_decode_result_t;

typedef struct {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t payload[CONTROL_PROTOCOL_MAX_PAYLOAD];
} control_frame_t;

uint16_t control_protocol_crc16(const uint8_t *data, uint16_t len);
control_decode_result_t control_protocol_decode_frame(const uint8_t *frame, uint8_t frame_len, control_frame_t *out);
uint8_t control_protocol_build_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint8_t out_capacity);

#endif
