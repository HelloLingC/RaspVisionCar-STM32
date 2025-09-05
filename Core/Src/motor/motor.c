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

	// 初始状态：停止
	HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);

	HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
	
	// 启动PWM定时器
	HAL_TIM_PWM_Start(MOTOR_A_TIMER, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(MOTOR_B_TIMER, TIM_CHANNEL_2);

	// 设置初始占空比为0
	__HAL_TIM_SET_COMPARE(MOTOR_A_TIMER, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(MOTOR_B_TIMER, TIM_CHANNEL_2, 0);

	//hmotor.is_initialized = 1;
	//hmotor.current_speed = 0;
}

void Motor_Set_Speed(int8_t speed) {
	
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



