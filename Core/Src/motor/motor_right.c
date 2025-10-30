#include "motor_right.h"
#include "motor.h"

#include <stdlib.h>

void Motor_Right_Set_Raw_Speed(int16_t pwm_value) {
  pwm_value = (pwm_value < -1000) ? -1000 : (pwm_value > 1000) ? 1000 : pwm_value;
  HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(MOTOR_B_TIMER, TIM_CHANNEL_2, pwm_value);
}

/**
 * @brief 设置 Right 电机速度和方向
 * @param speed: 速度值 (-100 到 100)
 *       负值反转，正值正转，0停止
 */
void Motor_Right_Set_Speed(int8_t speed) {
  // 限制速度范围
  speed = (speed < -100) ? -100 : (speed > 100) ? 100 : speed;
  
  // CCRv
  uint16_t pwm_value = abs(speed) * (htim3.Init.Period + 1) / 100;
  
  if (speed > 0) {
    // 正转 Forward
	  HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
  } else if (speed < 0) {
    // 反转 Backward
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
  } else {
    // 停止 Stop
    Motor_Right_ALL_RESET();
  }
  
  // Set PWM value
  __HAL_TIM_SET_COMPARE(MOTOR_B_TIMER, TIM_CHANNEL_2, pwm_value);
}

void Motor_Right_ALL_RESET(void) {
  HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
}
