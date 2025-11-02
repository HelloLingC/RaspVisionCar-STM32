#include "motor.h"

#include <stdlib.h>

void Motor_Left_Set_Raw_Speed(int16_t pwm_value) {
  pwm_value = (pwm_value < -3600) ? -3600 : (pwm_value > 3600) ? 3600 : pwm_value;
  if(pwm_value > 0) {
    // 正转
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
  } else if (pwm_value < 0) {
    // 反转
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
    pwm_value = -pwm_value;
  } else {
    // 停止
    Motor_Left_ALL_RESET();
  }
  __HAL_TIM_SET_COMPARE(MOTOR_A_TIMER, TIM_CHANNEL_1, pwm_value);
}

void Motor_Left_ALL_RESET(void) {
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
}
