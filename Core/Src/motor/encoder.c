#include "main.h"
#include "tim.h"
#include "encoder.h"

#define ENCODER_PPR 13
#define MOTOR_GEAR_RATIO 28.0f

#define encoder_timer_left &htim2
#define encoder_timer_right &htim4

typedef struct {
    TIM_HandleTypeDef* timer;
	uint16_t last_count;
	int16_t rpm;
} Encoder_t;

static Encoder_t encoder_left, encoder_right;

void init_encoders(void) {
	encoder_left.timer = encoder_timer_left;
	encoder_left.last_count = (encoder_left.timer->Instance->CNT);
	encoder_left.rpm = 0;

	encoder_right.timer = encoder_timer_right;
	encoder_right.last_count = (encoder_right.timer->Instance->CNT);
	encoder_right.rpm = 0;
	HAL_TIM_Encoder_Start(encoder_timer_left, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(encoder_timer_right, TIM_CHANNEL_ALL);
}

static void encoder_update_one(Encoder_t* enc) {
	uint16_t cnt = enc->timer->Instance->CNT; // 0 ~ 65535
	int16_t diff = (int16_t)(cnt - enc->last_count);

	if (diff > 32767) {
        diff -= 65536;  // Negative overflow
    } else if (diff < -32767) {
        diff += 65536;  // Positive overflow
    }

	int16_t pulse_per_sec = diff * 100.0f; // 10ms 调用一次 -> ×100 得到每秒脉冲数
	// 4倍频模式（Encoder Mode: TI12），实际每转脉冲数 = PPR × 4
	float rpm_f = (float)pulse_per_sec * 60.0f / (float)(ENCODER_PPR * 4);
    // MGX513X 减速比 28:1
    rpm_f = rpm_f / MOTOR_GEAR_RATIO;

	enc->rpm = (int16_t)rpm_f;
	enc->last_count = cnt;
}

// 应在10ms周期调用一次
void encoder_update_10ms(void) {
	encoder_update_one(&encoder_left);
	// I don't know why, but the left encoder is reversed
	// Maybe the A/B wire is reversed
	encoder_left.rpm = -encoder_left.rpm;
	encoder_update_one(&encoder_right);
}

void encoder_get_motor_speed(int16_t* left_speed_rpm, int16_t* right_speed_rpm)
{
    if (left_speed_rpm)  { *left_speed_rpm = encoder_left.rpm; }
    if (right_speed_rpm) { *right_speed_rpm = encoder_right.rpm; }
}