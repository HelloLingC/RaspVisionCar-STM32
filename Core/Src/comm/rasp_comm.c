#include "rasp_comm.h"

#include "command.h"
#include "common.h"
#include "control_protocol.h"
#include "control_session.h"
#include "encoder.h"
#include "motor.h"
#include "stm32f1xx_hal_dma.h"
#include "stm32f1xx_hal_uart.h"
#include "usart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define RASP_RX_BUF_SIZE 64U
#define RASP_FRAME_BUF_SIZE CONTROL_PROTOCOL_MAX_FRAME_LEN
#define RASP_STATUS_PAYLOAD_SIZE 36U
#define RASP_STATUS_PERIOD_MS 50U

static uint8_t s_rx_dma_buffer[RASP_RX_BUF_SIZE];
static uint8_t s_tx_buffer[CONTROL_PROTOCOL_MAX_FRAME_LEN];
static rasp_status_t s_status = {0};
static control_session_t s_session;
static uint8_t s_status_seq = 0U;
static uint32_t s_last_status_tx_tick = 0U;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern int32_t sOdometerLeft;
extern int32_t sOdometerRight;

static void write_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void write_i16_le(uint8_t *dst, int16_t value)
{
    write_u16_le(dst, (uint16_t)value);
}

static void write_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8U) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16U) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static void write_i32_le(uint8_t *dst, int32_t value)
{
    write_u32_le(dst, (uint32_t)value);
}

static void update_status(uint32_t now_ms)
{
    control_status_t control_status;
    Command_Stats_t command_stats;

    int16_t measured_left_rpm = 0;
    int16_t measured_right_rpm = 0;

    control_session_get_status(&s_session, now_ms, &control_status);
    Command_GetStats(&command_stats);
    encoder_get_motor_speed(&measured_left_rpm, &measured_right_rpm);

    s_status.control_state = (uint8_t)control_status.control_state;
    s_status.last_error = (uint8_t)control_status.last_error;
    s_status.last_rx_seq = control_status.last_rx_seq;
    s_status.reserved = 0U;
    s_status.target_left_rpm = control_status.target_left_rpm;
    s_status.target_right_rpm = control_status.target_right_rpm;
    s_status.meas_left_rpm = measured_left_rpm;
    s_status.meas_right_rpm = measured_right_rpm;
    s_status.cmd_v_mmps = control_status.cmd_v_mmps;
    s_status.cmd_w_mradps = control_status.cmd_w_mradps;
    s_status.odom_left = sOdometerLeft;
    s_status.odom_right = sOdometerRight;
    s_status.age_set_twist_ms = control_status.age_set_twist_ms;
    s_status.age_link_ms = control_status.age_link_ms;
    s_status.drop_crc = control_status.drop_crc;
    s_status.drop_seq = control_status.drop_seq;
    s_status.drop_len = control_status.drop_len;
    s_status.framing_err = command_stats.framing_err;
    s_status.overflow = command_stats.overflow;
}

static void try_send_status_frame(uint32_t now_ms)
{
    uint8_t payload[RASP_STATUS_PAYLOAD_SIZE] = {0};
    uint8_t frame_len;

    if ((now_ms - s_last_status_tx_tick) < RASP_STATUS_PERIOD_MS) {
        return;
    }

    payload[0] = s_status.control_state;
    payload[1] = s_status.last_error;
    payload[2] = s_status.last_rx_seq;
    payload[3] = s_status.reserved;
    write_i16_le(&payload[4], s_status.target_left_rpm);
    write_i16_le(&payload[6], s_status.target_right_rpm);
    write_i16_le(&payload[8], s_status.meas_left_rpm);
    write_i16_le(&payload[10], s_status.meas_right_rpm);
    write_i16_le(&payload[12], s_status.cmd_v_mmps);
    write_i16_le(&payload[14], s_status.cmd_w_mradps);
    write_i32_le(&payload[16], s_status.odom_left);
    write_i32_le(&payload[20], s_status.odom_right);
    write_u16_le(&payload[24], s_status.age_set_twist_ms);
    write_u16_le(&payload[26], s_status.age_link_ms);
    write_u32_le(&payload[28], s_status.drop_crc);
    write_u32_le(&payload[32], s_status.drop_seq);

    frame_len = control_protocol_build_frame((uint8_t)CONTROL_MSG_STATUS,
                                             s_status_seq++,
                                             payload,
                                             RASP_STATUS_PAYLOAD_SIZE,
                                             s_tx_buffer,
                                             sizeof(s_tx_buffer));
    if (frame_len == 0U) {
        return;
    }

    if (huart1.gState == HAL_UART_STATE_READY) {
        if (HAL_UART_Transmit_IT(&huart1, s_tx_buffer, frame_len) == HAL_OK) {
            s_last_status_tx_tick = now_ms;
        }
    }
}

void rasp_comm_init(void)
{
    memset(s_rx_dma_buffer, 0, sizeof(s_rx_dma_buffer));
    memset(&s_status, 0, sizeof(s_status));
    control_session_init(&s_session);
    Command_ResetStats();
    s_last_status_tx_tick = HAL_GetTick();

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma_buffer, RASP_RX_BUF_SIZE) != HAL_OK) {
        usart_error("HAL_UARTEx_ReceiveToIdle_DMA failed");
        Error_Handler();
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

void rasp_comm_poll(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint8_t frame_buffer[RASP_FRAME_BUF_SIZE];
    uint8_t frame_len;
    int16_t target_left = 0;
    int16_t target_right = 0;

    while ((frame_len = Command_GetCommand(frame_buffer, sizeof(frame_buffer))) > 0U) {
        control_frame_t frame;
        control_decode_result_t decode_result = control_protocol_decode_frame(frame_buffer, frame_len, &frame);

        if (decode_result == CONTROL_DECODE_OK) {
            control_session_apply_frame(&s_session, &frame, now_ms);
        } else if (decode_result == CONTROL_DECODE_BAD_CRC) {
            control_session_note_crc_error(&s_session);
        } else if (decode_result == CONTROL_DECODE_BAD_LEN) {
            control_session_note_len_error(&s_session);
        } else {
            control_session_note_len_error(&s_session);
        }
    }

    control_session_compute_targets(&s_session, now_ms, &target_left, &target_right);
    motor_controller_set_comm_target_rpm(target_left, target_right);

    update_status(now_ms);
    try_send_status_frame(now_ms);

    system_stop_flag = (s_status.control_state == (uint8_t)CONTROL_STATE_ARMED) ? 0U : 1U;
}

void rasp_comm_get_status(rasp_status_t *out)
{
    if (out == 0) {
        return;
    }
    *out = s_status;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART1) {
        uint16_t written = Command_Write(s_rx_dma_buffer, size);
        s_status.rx_bytes += written;

        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_rx_dma_buffer, RASP_RX_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
}

void usart_error(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < (int)sizeof(buffer)) {
        char error_message[300];
        snprintf(error_message, sizeof(error_message), "ERROR: %s\r\n", buffer);
        HAL_UART_Transmit(&huart1, (uint8_t *)error_message, strlen(error_message), HAL_MAX_DELAY);
    }
}

void usart_info(const char *format, ...)
{
    char buffer[256];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < (int)sizeof(buffer)) {
        char info_message[300];
        snprintf(info_message, sizeof(info_message), "[%u] INFO: %s\r\n", (unsigned int)HAL_GetTick(), buffer);
        HAL_UART_Transmit_IT(&huart1, (uint8_t *)info_message, strlen(info_message));
    }
}
