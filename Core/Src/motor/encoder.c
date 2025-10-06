#include "main.h"
#include "tim.h"
#include "encoder.h"

#define encoder_timer_left &htim2
#define encoder_timer_right &htim4

typedef struct {
    TIM_HandleTypeDef* timer;
	uint16_t last_count;
	int16_t rpm;
	int16_t filter_rpm[5];
} Encoder_t;

static Encoder_t encoder_left, encoder_right;

void init_encoders(void) {
	encoder_left.timer = encoder_timer_left;
	encoder_left.last_count = (encoder_left.timer->Instance->CNT);
	encoder_left.rpm = 0;
	// Initialize filter array to prevent garbage values
	for (int i = 0; i < 5; i++) {
		encoder_left.filter_rpm[i] = 0;
	}

	encoder_right.timer = encoder_timer_right;
	encoder_right.last_count = (encoder_right.timer->Instance->CNT);
	encoder_right.rpm = 0;

	HAL_TIM_Encoder_Start(encoder_timer_left, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(encoder_timer_right, TIM_CHANNEL_ALL);
}

#define ENCODER_PPR 13
#define MOTOR_GEAR_RATIO 28 // MG513X
#define PULSE_PER_REV (ENCODER_PPR * 4 * MOTOR_GEAR_RATIO)
// 每转脉冲数 4倍频模式（Encoder Mode: T1 T2），实际每转脉冲数 = PPR * 4
#define delta_t 10 // unit: ms

static void encoder_update_one(Encoder_t* enc) {
	uint16_t cnt = enc->timer->Instance->CNT; // 0 ~ 65535
	int32_t diff = (int32_t)(cnt - enc->last_count);

	if (diff > 32767) {
        diff -= 65536;  // Negative overflow
    } else if (diff < -32768) {
        diff += 65536;  // Positive overflow
    }
	int32_t pulse_per_second = diff;

	int32_t rpm = pulse_per_second / PULSE_PER_REV * 60;

	// 平均值滤波
	for (int i = 0; i < 4; i++) {
		enc->filter_rpm[i] = enc->filter_rpm[i+1];
    }
    enc->filter_rpm[4] = (int16_t)rpm;

	int16_t filtered_rpm = (enc->filter_rpm[0] + enc->filter_rpm[1] + enc->filter_rpm[2] + enc->filter_rpm[3] + enc->filter_rpm[4]) / 5;
	enc->rpm = filtered_rpm;
	enc->last_count = cnt;
}

void encoder_update_10ms(void) {
	encoder_update_one(&encoder_left);
	encoder_update_one(&encoder_right);
	// the left encoder value is negative, so we need to reverse it
	encoder_left.rpm = -encoder_left.rpm;
}

void encoder_get_motor_speed(int16_t* left_speed_rpm, int16_t* right_speed_rpm)
{
    if (left_speed_rpm)  { *left_speed_rpm = encoder_left.rpm; }
    if (right_speed_rpm) { *right_speed_rpm = encoder_right.rpm; }
}