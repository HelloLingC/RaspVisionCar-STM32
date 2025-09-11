#include "tim.h"
#include <stdlib.h>
#include "motor.h"
#include "motor_left.h"
#include "motor_right.h"

typedef struct {
	uint8_t is_initialized;
	int8_t current_left_speed; // -100 to 100
	int8_t current_right_speed;
	int isStopped;
} Motor_HandleTypeDef;

Motor_HandleTypeDef hmotor;

/**
 * @brief Initialize Left&Right Motor
 */
void Motor_Init(void) {
	// 退出待机模式
	Motor_Wakeup();

	// 设置初始占空比为0
	__HAL_TIM_SET_COMPARE(MOTOR_A_TIMER, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_B_TIMER, TIM_CHANNEL_2, 0);

	// 启动PWM定时器
	HAL_TIM_PWM_Start(MOTOR_A_TIMER, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(MOTOR_B_TIMER, TIM_CHANNEL_2);

	Motor_Left_ALL_RESET();
	Motor_Right_ALL_RESET();
}

void Motor_Set_Speed(int8_t speed) {
	Motor_Left_Set_Speed(speed);
	Motor_Right_Set_Speed(speed);
}


/**
 * @brief 进入待机模式
 */
void Motor_Standby(void) {
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 退出待机模式
 */
void Motor_Wakeup(void) {
  HAL_GPIO_WritePin(MOTOR_STBY_PORT, MOTOR_STBY_PIN, GPIO_PIN_SET);
}



