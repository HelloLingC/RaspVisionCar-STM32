#include <stdint.h>

#include "encoder.h"
#include "feedforward_controller.h"
#include "motor.h"
#include "pid_controller.h"

static int16_t s_target_left_rpm = 0;
static int16_t s_target_right_rpm = 0;
static int16_t s_turn_angle = 0;

static int16_t s_real_target_left_rpm = 0;
static int16_t s_real_target_right_rpm = 0;

int32_t sOdometerLeft = 0;
int32_t sOdometerRight = 0;

void motor_controller_set_left_target_rpm(int16_t left_rpm)
{
    s_target_left_rpm = left_rpm;
}

void motor_controller_set_right_target_rpm(int16_t right_rpm)
{
    s_target_right_rpm = right_rpm;
}

void motor_controller_set_target_rpm(int16_t left_rpm, int16_t right_rpm)
{
    motor_controller_set_left_target_rpm(left_rpm);
    motor_controller_set_right_target_rpm(right_rpm);
}

void motor_controller_set_comm_target_rpm(int16_t left_rpm, int16_t right_rpm)
{
    s_target_left_rpm = left_rpm;
    s_target_right_rpm = right_rpm;
    s_turn_angle = 0;
}

void motor_controller_set_turn_angle(int16_t turn_angle)
{
    s_turn_angle = turn_angle;
}

int16_t motor_controller_get_turn_angle()
{
    return s_turn_angle;
}

void motor_controller_get_real_target_rpm(int16_t *left_rpm, int16_t *right_rpm)
{
    if (left_rpm != 0) {
        *left_rpm = s_real_target_left_rpm;
    }
    if (right_rpm != 0) {
        *right_rpm = s_real_target_right_rpm;
    }
}

void motor_controller_update()
{
    int16_t l_rpm = 0;
    int16_t r_rpm = 0;
    int16_t ff_left_pwm = 0;
    int16_t ff_right_pwm = 0;
    int16_t pid_left_pwm = 0;
    int16_t pid_right_pwm = 0;

    encoder_get_motor_speed(&l_rpm, &r_rpm);
    sOdometerLeft += l_rpm;
    sOdometerRight += r_rpm;

    s_real_target_left_rpm = s_target_left_rpm;
    s_real_target_right_rpm = s_target_right_rpm;

    pid_update(l_rpm, r_rpm, &pid_left_pwm, &pid_right_pwm);
    ff_update(s_real_target_left_rpm, s_real_target_right_rpm, &ff_left_pwm, &ff_right_pwm);

    Motor_Left_Set_Raw_Speed(pid_left_pwm + ff_left_pwm);
    Motor_Right_Set_Raw_Speed(pid_right_pwm + ff_right_pwm);
}
