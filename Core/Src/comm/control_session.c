#include "control_session.h"

#include <string.h>

static uint16_t saturate_u16(uint32_t value)
{
    if (value > 0xFFFFU) {
        return 0xFFFFU;
    }
    return (uint16_t)value;
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void set_error(control_session_t *session, control_error_t err)
{
    if (session == 0) {
        return;
    }
    session->last_error = err;
}

static uint8_t is_seq_valid(control_session_t *session, uint8_t seq)
{
    uint8_t delta;

    if (session->seq_initialized == 0U) {
        return 1U;
    }

    delta = (uint8_t)(seq - session->last_rx_seq);
    if (delta == 0U || delta > 127U) {
        return 0U;
    }

    return 1U;
}

void control_session_init(control_session_t *session)
{
    if (session == 0) {
        return;
    }

    memset(session, 0, sizeof(*session));
    session->state = CONTROL_STATE_DISARMED;
    session->last_error = CONTROL_ERROR_NONE;
    kinematics_get_default_config(&session->kinematics_cfg);
}

void control_session_note_crc_error(control_session_t *session)
{
    if (session == 0) {
        return;
    }

    ++session->drop_crc;
    set_error(session, CONTROL_ERROR_BAD_CRC);
}

void control_session_note_len_error(control_session_t *session)
{
    if (session == 0) {
        return;
    }

    ++session->drop_len;
    set_error(session, CONTROL_ERROR_BAD_LEN);
}

void control_session_note_seq_error(control_session_t *session)
{
    if (session == 0) {
        return;
    }

    ++session->drop_seq;
    set_error(session, CONTROL_ERROR_BAD_SEQ);
}

void control_session_apply_frame(control_session_t *session, const control_frame_t *frame, uint32_t now_ms)
{
    if (session == 0 || frame == 0) {
        return;
    }

    if (is_seq_valid(session, frame->seq) == 0U) {
        control_session_note_seq_error(session);
        return;
    }

    session->seq_initialized = 1U;
    session->last_rx_seq = frame->seq;
    session->last_link_tick_ms = now_ms;

    switch (frame->type) {
    case CONTROL_MSG_SET_TWIST:
        if (session->state == CONTROL_STATE_ESTOP_LATCHED) {
            set_error(session, CONTROL_ERROR_STATE);
            return;
        }
        if (frame->len != 6U) {
            control_session_note_len_error(session);
            return;
        }
        session->command.v_mmps = read_i16_le(&frame->payload[0]);
        session->command.w_mradps = read_i16_le(&frame->payload[2]);
        session->command.reverse_allowed = (uint8_t)(frame->payload[4] & 0x01U);
        session->command.valid = 1U;
        session->command.tick_ms = now_ms;
        session->last_set_twist_tick_ms = now_ms;
        set_error(session, CONTROL_ERROR_NONE);
        break;
    case CONTROL_MSG_ARM:
        if (frame->len != 0U) {
            control_session_note_len_error(session);
            return;
        }
        if (session->state == CONTROL_STATE_DISARMED) {
            session->state = CONTROL_STATE_ARMED;
            set_error(session, CONTROL_ERROR_NONE);
        } else {
            set_error(session, CONTROL_ERROR_STATE);
        }
        break;
    case CONTROL_MSG_DISARM:
        if (frame->len != 0U) {
            control_session_note_len_error(session);
            return;
        }
        session->state = CONTROL_STATE_DISARMED;
        session->output_left_rpm = 0;
        session->output_right_rpm = 0;
        set_error(session, CONTROL_ERROR_NONE);
        break;
    case CONTROL_MSG_ESTOP:
        if (frame->len != 0U) {
            control_session_note_len_error(session);
            return;
        }
        session->state = CONTROL_STATE_ESTOP_LATCHED;
        session->output_left_rpm = 0;
        session->output_right_rpm = 0;
        set_error(session, CONTROL_ERROR_NONE);
        break;
    case CONTROL_MSG_CLEAR_ESTOP:
        if (frame->len != 0U) {
            control_session_note_len_error(session);
            return;
        }
        if (session->state == CONTROL_STATE_ESTOP_LATCHED) {
            session->state = CONTROL_STATE_DISARMED;
            set_error(session, CONTROL_ERROR_NONE);
        } else {
            set_error(session, CONTROL_ERROR_STATE);
        }
        break;
    case CONTROL_MSG_HEARTBEAT:
        if (frame->len != 0U) {
            control_session_note_len_error(session);
            return;
        }
        set_error(session, CONTROL_ERROR_NONE);
        break;
    default:
        set_error(session, CONTROL_ERROR_BAD_PAYLOAD);
        break;
    }
}

void control_session_compute_targets(control_session_t *session, uint32_t now_ms, int16_t *left_rpm, int16_t *right_rpm)
{
    int16_t target_left = 0;
    int16_t target_right = 0;

    if (left_rpm == 0 || right_rpm == 0) {
        return;
    }

    if (session == 0) {
        *left_rpm = 0;
        *right_rpm = 0;
        return;
    }

    if (session->state == CONTROL_STATE_ARMED && session->command.valid != 0U) {
        uint32_t age = now_ms - session->command.tick_ms;
        if (age <= CONTROL_SET_TWIST_TIMEOUT_MS) {
            kinematics_twist_to_wheel_rpm(&session->kinematics_cfg,
                                          session->command.v_mmps,
                                          session->command.w_mradps,
                                          session->command.reverse_allowed,
                                          &target_left,
                                          &target_right);

            target_left = kinematics_slew_limit(session->output_left_rpm, target_left, session->kinematics_cfg.max_rpm_step_per_cycle);
            target_right = kinematics_slew_limit(session->output_right_rpm, target_right, session->kinematics_cfg.max_rpm_step_per_cycle);
        } else {
            set_error(session, CONTROL_ERROR_TIMEOUT);
        }
    }

    session->output_left_rpm = target_left;
    session->output_right_rpm = target_right;
    *left_rpm = target_left;
    *right_rpm = target_right;
}

void control_session_get_status(const control_session_t *session, uint32_t now_ms, control_status_t *status_out)
{
    if (session == 0 || status_out == 0) {
        return;
    }

    memset(status_out, 0, sizeof(*status_out));
    status_out->control_state = session->state;
    status_out->last_error = session->last_error;
    status_out->last_rx_seq = session->last_rx_seq;
    status_out->cmd_v_mmps = session->command.v_mmps;
    status_out->cmd_w_mradps = session->command.w_mradps;
    status_out->target_left_rpm = session->output_left_rpm;
    status_out->target_right_rpm = session->output_right_rpm;
    status_out->age_set_twist_ms = session->command.valid ? saturate_u16(now_ms - session->command.tick_ms) : 0xFFFFU;
    status_out->age_link_ms = (session->seq_initialized != 0U) ? saturate_u16(now_ms - session->last_link_tick_ms) : 0xFFFFU;
    status_out->drop_crc = session->drop_crc;
    status_out->drop_seq = session->drop_seq;
    status_out->drop_len = session->drop_len;
}
