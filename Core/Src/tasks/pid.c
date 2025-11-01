//
// Created by lingc on 11/1/2025.
//
#include "encoder.h"
#include "pid_controller.h"

void task_pid_update() {
  encoder_update_10ms();
  int16_t l_rpm = 0, r_rpm = 0;
  encoder_get_motor_speed(&l_rpm, &r_rpm);
  // PID_Calc(l_rpm, r_rpm, &left_output, &right_output);
  pid_update(l_rpm, r_rpm);

  // char message[100];
  // snprintf(message, sizeof(message), "<main%u>:%d,%d.%d\n", HAL_GetTick(), l_rpm, r_rpm, sTurnAngle);
  // HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
}