#include "main.h"

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "stm32f1xx_hal.h"
#include "tim.h" 

// Left Motor
#define MOTOR_A_TIMER &htim3
#define MOTOR_AIN1_PIN GPIO_PIN_4
#define MOTOR_AIN1_PORT GPIOA
#define MOTOR_AIN2_PIN GPIO_PIN_2
#define MOTOR_AIN2_PORT GPIOA

#define MOTOR_STBY_PIN GPIO_PIN_5
#define MOTOR_STBY_PORT GPIOA

// Right Motor
#define MOTOR_B_TIMER &htim3
#define MOTOR_BIN1_PIN GPIO_PIN_0
#define MOTOR_BIN1_PORT GPIOB
#define MOTOR_BIN2_PIN GPIO_PIN_1
#define MOTOR_BIN2_PORT GPIOB

#endif

void Motor_Init(void) ;
void Motor_Set_Speed(int8_t speed);
void Motor_Brake(void);
void Motor_Wakeup(void);

