#include "main.h"
#include "tim.h"
#include "encoder.h"

#define ENCODER_PPR 13
#define MOTOR_GEAR_RATIO 28.0f

#define encoder_timer_left &htim2
#define encoder_timer_right &htim4

typedef struct {
    TIM_HandleTypeDef* timer;
	int32_t last_count;
	int16_t rpm;
} Encoder_t;

static Encoder_t encoder_left, encoder_right;

void init_encoders(void) {
	encoder_left.timer = encoder_timer_left;
	encoder_left.last_count = (int32_t)(encoder_left.timer->Instance->CNT);
	encoder_left.rpm = 0;
	
	encoder_right.timer = encoder_timer_right;
	encoder_right.last_count = (int32_t)(encoder_right.timer->Instance->CNT);
	encoder_right.rpm = 0;
	
	HAL_TIM_Encoder_Start(encoder_timer_left, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(encoder_timer_right, TIM_CHANNEL_ALL);
}

static void encoder_update_one(Encoder_t* enc) {
	int16_t cnt = enc->timer->Instance->CNT;
	int32_t current = (int32_t)cnt;
	int32_t diff = current - enc->last_count;
	int32_t pulse_per_sec = diff * 10; // 100ms 调用一次 -> ×10 得到每秒脉冲数
	// 4倍频模式（Encoder Mode: TI12），实际每转脉冲数 = PPR × 4
	float rpm_f = (float)pulse_per_sec * 60.0f / (float)(ENCODER_PPR * 4);
    // MGX513X 减速比 28:1
    rpm_f = rpm_f / MOTOR_GEAR_RATIO;

	// 抑制抖动：接近零的小数值归零
	// if (rpm_f > -3.0f && rpm_f < 3.0f) {
	// 	rpm_f = 0.0f;
	// }
	int16_t rpm_r = (int16_t)rpm_f;
	enc->rpm = rpm_r;
	enc->last_count = current;
}

// 应在100ms周期调用一次
void encoder_update_100ms(void) {
	encoder_update_one(&encoder_left);
	//encoder_update_one(&encoder_right);
}

void encoder_get_motor_speed(int16_t* left_speed_rpm, int16_t* right_speed_rpm)
{
    if (left_speed_rpm)  { *left_speed_rpm = encoder_left.rpm; }
    if (right_speed_rpm) { *right_speed_rpm = encoder_right.rpm; }
}