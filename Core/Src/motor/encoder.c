#include "main.h"
#include "tim.h"

#define ENCODER_PPR 13

typedef struct {
    TIM_HandleTypeDef* timer;
    int32_t count;
	int32_t last_count;
} Encoder_t;

Encoder_t encoder_left, encoder_right;

void init_encoders() {
	encoder_left.timer = &htim2;
    encoder_left.count = 0;
	
	encoder_right.timer = &htim4;
    encoder_right.count = 0;
	
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	
	// 启动编码器定时器的中断
  HAL_TIM_Base_Start_IT(&htim2);
	
	
}

// Cacl PPR
void update(int32_t *count, int32_t *last_count, int32_t *speed) {
    *count = TIM2->CNT; // Read 32-bit value from register
    int32_t diff = *count - *last_count;
	 // 处理16位计数器溢出/下溢
    if(diff > 32767) {
        diff -= 65536;
    } else if (diff < -32768) {
        diff += 65536;
    }
	int32_t Pulse_Count_Per_Second = diff * 10; // 100ms intervals, so multiply 10
	
	// 3. 转换为RPM
    // RPM = (脉冲数/秒) × (60秒/分钟) ÷ (脉冲数/转)
    //          = Pulse_Count_Per_Second × 60 / ENCODER_PPR
	// 4倍频模式（Encoder Mode: Tl1 and Tl2），实际每转脉冲数 = PPR × 4
    int32_t Motor_RPM = (float)Pulse_Count_Per_Second * 60.0f / (float)(ENCODER_PPR * 4);
	
    *last_count = *count;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// 如果是TIM2的中断
	if (htim->Instance == TIM2) {
		int32_t speed;
		update(&encoder_left.count, &encoder_left.last_count, &speed);
	}
}


void encoder_get_motor_speed(int16_t* left_speed, int16_t* right_speed)
{
    *left_speed = encoder_left.count;
    *right_speed = encoder_right.count;
}