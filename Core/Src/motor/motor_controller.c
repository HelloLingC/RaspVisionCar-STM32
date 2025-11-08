#include <stdint.h>
#include "encoder.h"
#include "main.h"
#include "pid_controller.h"
#include "feedforward_controller.h"
#include "motor.h"
#include "stm32f1xx_hal.h"
#include "rasp_comm.h"
#include <math.h>
#include "buzzer.h"

int16_t target_left_rpm = 0;
int16_t target_right_rpm = 0;
int16_t sTurnAngle = 0;

int16_t real_target_left_rpm = 0;
int16_t real_target_right_rpm = 0;

void motor_controller_set_left_target_rpm(int16_t left_rpm) {
  target_left_rpm = left_rpm;
}

void motor_controller_set_right_target_rpm(int16_t right_rpm) {
  target_right_rpm = right_rpm;
}

void motor_controller_set_target_rpm(int16_t left_rpm, int16_t right_rpm) {
  motor_controller_set_left_target_rpm(left_rpm);
  motor_controller_set_right_target_rpm(right_rpm);
}

int16_t motor_controller_get_turn_angle() {
  return sTurnAngle;
}

void motor_controller_set_turn_angle(int16_t turn_angle) {
  sTurnAngle = turn_angle;
}

void motor_controller_get_real_target_rpm(int16_t *left_rpm, int16_t *right_rpm) {
  *left_rpm = real_target_left_rpm;
  *right_rpm = real_target_right_rpm;
}


// 里程计
int32_t sOdometerLeft = 0;
int32_t sOdometerRight = 0;
extern float yaw;
extern float turn_error;
extern uint8_t slow_accelerate_flag;
extern uint8_t motor_program_stop_flag;
uint32_t stop_at = 0;

void motor_movements() {
    // Go straight
    if(sOdometerLeft <= 5800) {
      target_left_rpm = 120;
      target_right_rpm = 120;
      slow_accelerate_flag = 1;
    } else if(sOdometerLeft > 5800 && sOdometerLeft < 20000) {
      // 准备进入弯道
      slow_accelerate_flag = 0;
      sTurnAngle = (int) turn_error * 0.38;
    } else if(sOdometerLeft > 19000 && sOdometerLeft < 24000) {
      // 斑马线停止
      if(stop_at == 0) {
        stop_at = HAL_GetTick();
        sTurnAngle = 0;
        target_left_rpm = 0;
        target_right_rpm = 0;
        buzzer_on(5);
      }
      if(HAL_GetTick() - stop_at > 3000) {
        // 斑马线恢复
        sTurnAngle = (int) turn_error * 0.38;
        target_left_rpm = 120;
        target_right_rpm = 120;
      }
    } else if (sOdometerLeft > 65000 && sOdometerLeft < 69000) {
      // 减速
      sTurnAngle = (int) turn_error * 0.38;
      target_left_rpm = 50;
      target_right_rpm = 50;
      buzzer_on(2);
    } else {
      sTurnAngle = (int) turn_error * 0.38;
      target_left_rpm = 120;
      target_right_rpm = 120;
    }
}


extern volatile uint8_t system_stop_flag;
void motor_controller_update() {
  int16_t l_rpm = 0, r_rpm = 0;
  encoder_get_motor_speed(&l_rpm, &r_rpm);

  // we havent consider backward situation
  sOdometerLeft += l_rpm;
  sOdometerRight += r_rpm;

  if(!motor_program_stop_flag) {
    motor_movements();
  }

  // sTurnAngle = (int) turn_error * 0.53;

  real_target_left_rpm = target_left_rpm;
  real_target_right_rpm = target_right_rpm;

  if (sTurnAngle != 0) {
    // positive: 左转时，左轮转速减小，右轮转速增大
    real_target_left_rpm -= sTurnAngle;
    real_target_right_rpm += sTurnAngle;
  }

  int16_t ff_left_pwm = 0, ff_right_pwm = 0;
  int16_t pid_left_pwm = 0, pid_right_pwm = 0;
  pid_update(l_rpm, r_rpm, &pid_left_pwm, &pid_right_pwm);
  ff_update(real_target_left_rpm, real_target_right_rpm, &ff_left_pwm,
            &ff_right_pwm);

//   ff_left_pwm =0; ff_right_pwm=0;

  Motor_Left_Set_Raw_Speed(pid_left_pwm + ff_left_pwm);
  Motor_Right_Set_Raw_Speed(pid_right_pwm + ff_right_pwm);

  // char message[100];
  // snprintf(message, sizeof(message), "<main%u>:%d,%d,%d,%d,%d,%d\n", HAL_GetTick(),
  //          l_rpm, r_rpm, pid_left_pwm, pid_right_pwm, sOdometerLeft, sOdometerRight);
  // HAL_UART_Transmit_IT(&huart1, message, strlen(message));

}