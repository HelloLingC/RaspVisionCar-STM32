#include "motor_right.h"
#include "motor.h"

#include <stdlib.h>

void Motor_Right_Set_Raw_Speed(int16_t pwm_value) {
  pwm_value = (pwm_value < -1000) ? -1000 : (pwm_value > 1000) ? 1000 : pwm_value;
  if(pwm_value > 0) {
    // 正转
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
  } else if (pwm_value < 0) {
    // 反转
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
    pwm_value = -pwm_value;
  } else {
    // 停止
    Motor_Right_ALL_RESET();
  }
  __HAL_TIM_SET_COMPARE(MOTOR_B_TIMER, TIM_CHANNEL_2, pwm_value);
}


void Motor_Right_ALL_RESET(void) {
  HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
}
