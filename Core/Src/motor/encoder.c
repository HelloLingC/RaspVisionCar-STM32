#include "main.h"
#include "tim.h"

#define ENCODER_PPR 13
#define MOTOR_GEAR_RATIO 28.0f

typedef struct {
    TIM_HandleTypeDef* timer;
	int32_t last_count;
	int16_t rpm;
} Encoder_t;

static Encoder_t encoder_left, encoder_right;

void init_encoders(void) {
	encoder_left.timer = &htim2;
	encoder_left.last_count = 0;
	encoder_left.rpm = 0;
	
	encoder_right.timer = &htim4;
	encoder_right.last_count = 0;
	encoder_right.rpm = 0;
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
}

static void encoder_update_one(Encoder_t* enc) {
	int32_t current = (int32_t)(enc->timer->Instance->CNT);
	int32_t diff = current - enc->last_count;
	// 处理16位计数器溢出/下溢
	if(diff > 32767) {
		diff -= 65536;
	} else if (diff < -32768) {
		diff += 65536;
	}
	int32_t pulse_per_sec = diff * 10; // 100ms 调用一次 -> ×10 得到每秒脉冲数
	// 4倍频模式（Encoder Mode: TI12），实际每转脉冲数 = PPR × 4
	float rpm_f = (float)pulse_per_sec * 60.0f / (float)(ENCODER_PPR * 4);
    // MGX513X 减速比 28:1
    rpm_f = rpm_f / MOTOR_GEAR_RATIO;

    if (rpm_f > 32767.0f) rpm_f = 32767.0f;
	if (rpm_f < -32768.0f) rpm_f = -32768.0f;
	enc->rpm = (int16_t)rpm_f;
	enc->last_count = current;
}

// 应在100ms周期调用一次
void encoder_update_100ms(void) {
	encoder_update_one(&encoder_left);
	encoder_update_one(&encoder_right);
}

void encoder_get_motor_speed(int16_t* left_speed_rpm, int16_t* right_speed_rpm)
{
    if (left_speed_rpm)  { *left_speed_rpm = encoder_left.rpm; }
    if (right_speed_rpm) { *right_speed_rpm = encoder_right.rpm; }
}