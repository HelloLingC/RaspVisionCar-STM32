#include "control_protocol.h"

#include <stddef.h>
#include <string.h>

uint16_t control_protocol_crc16(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0U; i < len; ++i) {
        uint8_t bit;
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            } else {
                crc <<= 1U;
            }
        }
    }

    return crc;
}

control_decode_result_t control_protocol_decode_frame(const uint8_t *frame, uint8_t frame_len, control_frame_t *out)
{
    uint8_t payload_len;
    uint8_t expected_len;
    uint16_t crc_recv;
    uint16_t crc_calc;

    if (frame == NULL || out == NULL || frame_len < CONTROL_PROTOCOL_MIN_FRAME_LEN) {
        return CONTROL_DECODE_TOO_SHORT;
    }

    if (frame[0] != CONTROL_PROTOCOL_SOF1 || frame[1] != CONTROL_PROTOCOL_SOF2) {
        return CONTROL_DECODE_BAD_SOF;
    }

    if (frame[2] != CONTROL_PROTOCOL_VERSION) {
        return CONTROL_DECODE_BAD_VERSION;
    }

    payload_len = frame[5];
    if (payload_len > CONTROL_PROTOCOL_MAX_PAYLOAD) {
        return CONTROL_DECODE_BAD_LEN;
    }

    expected_len = (uint8_t)(CONTROL_PROTOCOL_MIN_FRAME_LEN + payload_len);
    if (expected_len != frame_len) {
        return CONTROL_DECODE_BAD_LEN;
    }

    crc_recv = (uint16_t)frame[frame_len - 2U] | ((uint16_t)frame[frame_len - 1U] << 8);
    crc_calc = control_protocol_crc16(&frame[2], (uint16_t)(4U + payload_len));
    if (crc_calc != crc_recv) {
        return CONTROL_DECODE_BAD_CRC;
    }

    out->type = frame[3];
    out->seq = frame[4];
    out->len = payload_len;
    if (payload_len > 0U) {
        memcpy(out->payload, &frame[6], payload_len);
    }

    return CONTROL_DECODE_OK;
}

uint8_t control_protocol_build_frame(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t payload_len, uint8_t *out, uint8_t out_capacity)
{
    uint8_t frame_len;
    uint16_t crc;

    if (out == NULL || payload_len > CONTROL_PROTOCOL_MAX_PAYLOAD) {
        return 0U;
    }

    frame_len = (uint8_t)(CONTROL_PROTOCOL_MIN_FRAME_LEN + payload_len);
    if (out_capacity < frame_len) {
        return 0U;
    }

    out[0] = CONTROL_PROTOCOL_SOF1;
    out[1] = CONTROL_PROTOCOL_SOF2;
    out[2] = CONTROL_PROTOCOL_VERSION;
    out[3] = type;
    out[4] = seq;
    out[5] = payload_len;
    if (payload_len > 0U) {
        if (payload == NULL) {
            return 0U;
        }
        memcpy(&out[6], payload, payload_len);
    }

    crc = control_protocol_crc16(&out[2], (uint16_t)(4U + payload_len));
    out[6U + payload_len] = (uint8_t)(crc & 0xFFU);
    out[7U + payload_len] = (uint8_t)((crc >> 8U) & 0xFFU);

    return frame_len;
}
