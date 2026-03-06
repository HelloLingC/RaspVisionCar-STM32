#ifndef CONTROL_SESSION_H
#define CONTROL_SESSION_H

#include "control_protocol.h"
#include "kinematics.h"
#include <stdint.h>

#define CONTROL_SET_TWIST_TIMEOUT_MS 200U

typedef enum {
    CONTROL_STATE_DISARMED = 0,
    CONTROL_STATE_ARMED = 1,
    CONTROL_STATE_ESTOP_LATCHED = 2,
} control_state_t;

typedef enum {
    CONTROL_ERROR_NONE = 0,
    CONTROL_ERROR_BAD_LEN = 1,
    CONTROL_ERROR_BAD_CRC = 2,
    CONTROL_ERROR_BAD_SEQ = 3,
    CONTROL_ERROR_TIMEOUT = 4,
    CONTROL_ERROR_STATE = 5,
    CONTROL_ERROR_BAD_PAYLOAD = 6,
} control_error_t;

typedef struct {
    int16_t v_mmps;
    int16_t w_mradps;
    uint8_t reverse_allowed;
    uint8_t valid;
    uint32_t tick_ms;
} control_command_t;

typedef struct {
    control_state_t control_state;
    control_error_t last_error;
    uint8_t last_rx_seq;
    int16_t cmd_v_mmps;
    int16_t cmd_w_mradps;
    int16_t target_left_rpm;
    int16_t target_right_rpm;
    uint16_t age_set_twist_ms;
    uint16_t age_link_ms;
    uint32_t drop_crc;
    uint32_t drop_seq;
    uint32_t drop_len;
} control_status_t;

typedef struct {
    control_state_t state;
    control_error_t last_error;
    uint8_t last_rx_seq;
    uint8_t seq_initialized;
    uint32_t last_link_tick_ms;
    uint32_t last_set_twist_tick_ms;
    uint32_t drop_crc;
    uint32_t drop_seq;
    uint32_t drop_len;
    control_command_t command;
    kinematics_config_t kinematics_cfg;
    int16_t output_left_rpm;
    int16_t output_right_rpm;
} control_session_t;

void control_session_init(control_session_t *session);
void control_session_note_crc_error(control_session_t *session);
void control_session_note_len_error(control_session_t *session);
void control_session_note_seq_error(control_session_t *session);
void control_session_apply_frame(control_session_t *session, const control_frame_t *frame, uint32_t now_ms);
void control_session_compute_targets(control_session_t *session, uint32_t now_ms, int16_t *left_rpm, int16_t *right_rpm);
void control_session_get_status(const control_session_t *session, uint32_t now_ms, control_status_t *status_out);

#endif
