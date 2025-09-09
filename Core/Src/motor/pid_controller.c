#include "pid_controller.h"
#include "motor_left.h"
#include "motor_right.h"

typedef struct {
    PID_Params left_params;
    PID_Params right_params;
    float left_integral;
    float right_integral;

    int16_t last_left_err; // e[n-1]
    int16_t last_last_left_err; // e[n-2]
    int16_t last_right_err;
    int16_t last_last_right_err;

    int16_t target_left_rpm;
    int16_t target_right_rpm;
} PID_Handle;

static PID_Handle s_pid;

static float clampf(float x, float limit_abs)
{
    if (x > limit_abs) return limit_abs;
    if (x < -limit_abs) return -limit_abs;
    return x;
}

void pid_init_default(void)
{
    PID_Params left_p = {
        .kP = 0.6f,
        .kI = 0.0f,
        .kD = 0.0f,
        .integrator_limit = 50.0f,
        .output_limit = 100.0f,
    };
    PID_Params right_p = {
        .kP = 0.6f,
        .kI = 0.0f,
        .kD = 0.0f,
        .integrator_limit = 50.0f,
        .output_limit = 100.0f,
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
    s_pid.left_integral = 0.0f;
    s_pid.right_integral = 0.0f;
    s_pid.target_left_rpm = 0;
    s_pid.target_right_rpm = 0;

    s_pid.last_left_err = 0;
    s_pid.last_last_left_err = 0;

    s_pid.last_right_err = 0;
    s_pid.last_last_right_err = 0;
}

void pid_set_target_rpm(int16_t left_target_rpm, int16_t right_target_rpm)
{
    s_pid.target_left_rpm = left_target_rpm;
    s_pid.target_right_rpm = right_target_rpm;
}

int8_t pid_compute_one(int16_t left_meas_rpm, int16_t right_meas_rpm, int8_t* left_pwm, int8_t* right_pwm) {
    int16_t l_err = s_pid.target_left_rpm - left_meas_rpm;
    int16_t r_err = s_pid.target_right_rpm - right_meas_rpm;

    // PI-D with simple discrete form (dt = 0.1s)
    *left_pwm = s_pid.left_params.kP * (l_err - s_pid.last_left_err)
    + s_pid.left_params.kI * l_err
    + s_pid.left_params.kD * (l_err - 2 * s_pid.last_left_err + s_pid.last_last_left_err);

    *right_pwm = s_pid.right_params.kP * (r_err - s_pid.last_right_err)
    + s_pid.right_params.kI * r_err
    + s_pid.right_params.kD * (r_err - 2 * s_pid.last_right_err + s_pid.last_last_right_err);

    // Map directly to PWM percentage [-100, 100]
    *left_pwm = clampf(*left_pwm, s_pid.left_params.output_limit);
    *right_pwm = clampf(*right_pwm, s_pid.right_params.output_limit);

    s_pid.last_last_left_err = s_pid.last_left_err;
    s_pid.last_left_err = l_err;
    s_pid.last_last_right_err = s_pid.last_right_err;
    s_pid.last_right_err = r_err;
}

void pid_update_100ms(int16_t left_meas_rpm, int16_t right_meas_rpm)
{
    int8_t left_pwm, right_pwm;
    pid_compute_one(left_meas_rpm, right_meas_rpm, &left_pwm, &right_pwm);
    
    Motor_Left_Set_Speed(left_pwm);
    Motor_Right_Set_Speed(right_pwm);
}


