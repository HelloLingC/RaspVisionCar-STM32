#ifndef __RASP_COMM_H__
#define __RASP_COMM_H__

#include <stdint.h>

typedef struct {
    uint8_t control_state;
    uint8_t last_error;
    uint8_t last_rx_seq;
    uint8_t reserved;
    int16_t target_left_rpm;
    int16_t target_right_rpm;
    int16_t meas_left_rpm;
    int16_t meas_right_rpm;
    int16_t cmd_v_mmps;
    int16_t cmd_w_mradps;
    int32_t odom_left;
    int32_t odom_right;
    uint16_t age_set_twist_ms;
    uint16_t age_link_ms;
    uint32_t drop_crc;
    uint32_t drop_seq;
    uint32_t drop_len;
    uint32_t rx_bytes;
    uint32_t framing_err;
    uint32_t overflow;
} rasp_status_t;

void rasp_comm_init(void);
void rasp_comm_poll(void);
void rasp_comm_get_status(rasp_status_t *out);

void usart_log(const char *format, ...);
void usart_debug(const char *format, ...);
void usart_error(const char *format, ...);
void usart_info(const char *format, ...);

#endif
