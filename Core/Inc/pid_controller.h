#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stm32f1xx_hal.h"

typedef struct {
    float kP;
    float kI;
    float kD;
    float integrator_limit;
    float output_limit;     // absolute limit, e.g. 100 for PWM percent
} PID_Params;

void pid_init_default(void);
void pid_init(const PID_Params* left_params, const PID_Params* right_params);

// Set target wheel speeds in RPM
void pid_set_target_rpm(int16_t left_target_rpm, int16_t right_target_rpm);

// Call every 100 ms with measured RPM; internally sets Motor_* speeds
void pid_update_100ms(int16_t left_meas_rpm, int16_t right_meas_rpm);

// Optional helpers
void pid_reset(void);

#endif // PID_CONTROLLER_H


