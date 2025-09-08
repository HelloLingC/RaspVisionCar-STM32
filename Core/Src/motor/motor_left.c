#include "motor_left.h"
#include "motor.h"

#include <stdlib.h>

/**
 * @brief 设置电机速度和方向
 * @param speed: 速度值 (-100 到 100)
 *       负值反转，正值正转，0停止
 */
void Motor_Left_Set_Speed(int8_t speed) {
  // 限制速度范围
  speed = (speed < -100) ? -100 : (speed > 100) ? 100 : speed;
  
  // 计算PWM占空比
  uint16_t pwm_value = (abs(speed) * (htim3.Init.Period + 1)) / 100;
  
  if (speed > 0) {
    // 正转
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
  } else if (speed < 0) {
    // 反转
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_SET);
  } else {
    // 停止
    HAL_GPIO_WritePin(MOTOR_AIN1_PORT, MOTOR_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_AIN2_PORT, MOTOR_AIN2_PIN, GPIO_PIN_RESET);
  }
  
  // 设置PWM值
  __HAL_TIM_SET_COMPARE(MOTOR_A_TIMER, TIM_CHANNEL_1, pwm_value);
  
  // hmotor.current_speed = speed;
}
