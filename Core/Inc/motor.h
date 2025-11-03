#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "tim.h" 

// Left Motor
#define MOTOR_A_TIMER &htim3
#define MOTOR_AIN1_PIN GPIO_PIN_0
#define MOTOR_AIN1_PORT GPIOB
#define MOTOR_AIN2_PIN GPIO_PIN_1
#define MOTOR_AIN2_PORT GPIOB

#define MOTOR_STBY_PIN GPIO_PIN_15
#define MOTOR_STBY_PORT GPIOC

// Right Motor
#define MOTOR_B_TIMER &htim3
#define MOTOR_BIN1_PIN GPIO_PIN_4
#define MOTOR_BIN1_PORT GPIOA
#define MOTOR_BIN2_PIN GPIO_PIN_5
#define MOTOR_BIN2_PORT GPIOA

void motor_controller_set_left_target_rpm(int16_t left_rpm);
void motor_controller_set_right_target_rpm(int16_t right_rpm);
void motor_controller_set_target_rpm(int16_t left_rpm, int16_t right_rpm);

void motor_controller_set_turn_angle(int16_t turn_angle);
int16_t motor_controller_get_turn_angle();
void motor_controller_get_real_target_rpm(int16_t *left_rpm, int16_t *right_rpm);
void motor_controller_update();

void Motor_Init(void) ;
void Motor_Set_Speed(int16_t speed);
void Motor_Wakeup(void);

void Motor_Left_Set_Raw_Speed(int16_t l_pwm);
void Motor_Right_Set_Raw_Speed(int16_t r_pwm);

void Motor_Left_ALL_RESET(void);
void Motor_Right_ALL_RESET(void);

#endif

