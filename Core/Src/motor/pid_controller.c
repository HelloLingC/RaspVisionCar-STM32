#include "pid_controller.h"
#include "main.h"
#include "motor.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>

static float clampf(float x, float limit_abs)
{
    if (x > limit_abs) return limit_abs;
    if (x < -limit_abs) return -limit_abs;
    return x;
}

PID_Handle s_pid;

extern TIM_HandleTypeDef htim1;

extern UART_HandleTypeDef huart1;

void pid_init_default(void)
{
    // 3.3 0.5 0.6
    PID_Params left_p = {
        .kP = 0.0f,
        .kI = 0.39f,
        .kD = 0.0f,
    };
    PID_Params right_p = {
        .kP = 0.0f,
        .kI = 0.46f,
        .kD = 0.0f,
    };
    pid_init(&left_p, &right_p);
}

void pid_init(const PID_Params* left_p, const PID_Params* right_p)
{
    s_pid.left_params = *left_p;
    s_pid.right_params = *right_p;
    pid_reset();
}

void pid_reset(void)
{
    s_pid.last_left_err = 0;
    s_pid.last_last_left_err = 0;

    s_pid.last_right_err = 0;
    s_pid.last_last_right_err = 0;

    // Critical: Initialize output accumulators for incremental PID
    s_pid.left_output = 0.0f;
    s_pid.right_output = 0.0f;
}

void pid_update(int16_t left_meas_rpm, int16_t right_meas_rpm,
     int16_t *left_pwm, int16_t *right_pwm) {
    //static const float dt = 0.01f;  // 采样时间

    int16_t target_left_rpm = 0;
    int16_t target_right_rpm = 0;
    motor_controller_get_real_target_rpm(&target_left_rpm, &target_right_rpm);
    
    int16_t l_err = target_left_rpm - left_meas_rpm;
    int16_t r_err = target_right_rpm - right_meas_rpm;

    // 增量式PID计算
    float l_delta = s_pid.left_params.kP * (l_err - s_pid.last_left_err) +
            s_pid.left_params.kI * l_err +
            s_pid.left_params.kD * (l_err - 2 * s_pid.last_left_err + s_pid.last_last_left_err);
    float r_delta = s_pid.right_params.kP * (r_err - s_pid.last_right_err) +
            s_pid.right_params.kI * r_err +
            s_pid.right_params.kD * (r_err - 2 * s_pid.last_right_err + s_pid.last_last_right_err);

    s_pid.left_output += l_delta;
    s_pid.right_output += r_delta;

    s_pid.left_output = clampf(s_pid.left_output, 3599.0f);
    s_pid.right_output = clampf(s_pid.right_output, 3599.0f);

    s_pid.last_last_left_err = s_pid.last_left_err;
    s_pid.last_left_err = l_err;
    s_pid.last_last_right_err = s_pid.last_right_err;
    s_pid.last_right_err = r_err;

    // char message[100];
    // snprintf(message, sizeof(message), "%d, %d, %d, %d, %d, %d, %d\n", s_pid.target_left_rpm, left_meas_rpm, (int16_t)s_pid.left_output, right_meas_rpm, (int16_t)s_pid.right_output, (int16_t)l_err, (int16_t)r_err);
    // HAL_UART_Transmit_IT(&huart1, message, strlen(message));

    *left_pwm = (int16_t)s_pid.left_output;
    *right_pwm = (int16_t)s_pid.right_output;
}

// void PID_Calc(int16_t left_current, int16_t right_current, float* left_output, float* right_output) {
//     // Left motor PID (位置式)
//     float l_error = (float)s_pid.target_left_rpm - (float)left_current;
//     s_pid.left_integral += (int16_t)l_error;
//     float l_derivative = l_error - (float)s_pid.last_left_err;

//     if (s_pid.left_params.kI != 0.0f) {
//         float l_integralMax = 1000.0f / s_pid.left_params.kI;
//         if (s_pid.left_integral > l_integralMax) s_pid.left_integral = (int16_t)l_integralMax;
//         if (s_pid.left_integral < -l_integralMax) s_pid.left_integral = (int16_t)(-l_integralMax);
//     }

//     float l_out = s_pid.left_params.kP * l_error +
//                   s_pid.left_params.kI * (float)s_pid.left_integral +
//                   s_pid.left_params.kD * l_derivative;

//     s_pid.last_left_err = (int16_t)l_error;

//     // Right motor PID (位置式)
//     float r_error = (float)s_pid.target_right_rpm - (float)right_current;
//     s_pid.right_integral += (int16_t)r_error;
//     float r_derivative = r_error - (float)s_pid.last_right_err;

//     if (s_pid.right_params.kI != 0.0f) {
//         float r_integralMax = 1000.0f / s_pid.right_params.kI;
//         if (s_pid.right_integral > r_integralMax) s_pid.right_integral = (int16_t)r_integralMax;
//         if (s_pid.right_integral < -r_integralMax) s_pid.right_integral = (int16_t)(-r_integralMax);
//     }

//     float r_out = s_pid.right_params.kP * r_error +
//                   s_pid.right_params.kI * (float)s_pid.right_integral +
//                   s_pid.right_params.kD * r_derivative;

//     s_pid.last_right_err = (int16_t)r_error;

//     *left_output = l_out;
//     *right_output = r_out;
//     char message[100];
//     snprintf(message, sizeof(message), "<pid>:%d,%d,%d,%d\n", (int16_t)l_error, (int16_t)r_error, (int16_t)l_out, (int16_t)r_out);
//     HAL_UART_Transmit(&huart1, message, strlen(message), HAL_MAX_DELAY);
// }

