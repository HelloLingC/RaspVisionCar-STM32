#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stm32f1xx_hal.h"

typedef struct {
    float kP;
    float kI;
    float kD;
    float integrator_limit;
} PID_Params;

typedef struct {
    PID_Params left_params;
    PID_Params right_params;

    int16_t left_integral;
    int16_t right_integral;

    int16_t last_left_err; // e[n-1]
    int16_t last_last_left_err; // e[n-2]
    int16_t last_right_err;
    int16_t last_last_right_err;

    float left_output;
    float right_output;

    int16_t target_left_rpm;
    int16_t target_right_rpm;
} PID_Handle;

extern PID_Handle s_pid;

void pid_init_default(void);
void pid_init(const PID_Params* left_params, const PID_Params* right_params);

// Set target wheel speeds in RPM
void pid_set_target_rpm(int16_t left_target_rpm, int16_t right_target_rpm);

void pid_update(int16_t left_meas_rpm, int16_t right_meas_rpm,
     int16_t *left_pwm, int16_t *right_pwm);

// Optional helpers
void pid_reset(void);

#endif // PID_CONTROLLER_H


