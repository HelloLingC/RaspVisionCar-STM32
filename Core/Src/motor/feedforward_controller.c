#include "feedforward_controller.h"
#include "motor.h"
#include <stdint.h>

typedef struct {
    FF_Params left_params;
    FF_Params right_params;
} FF_Handle;

static FF_Handle s_ff;

uint8_t slow_accelerate_flag = 0;

void ff_init_default(void) {
    FF_Params left_p = {
        .kV = 17.84f,    // 占空比/每RPM 的比例
        .kS = 3.49f,     // 根据实车标定再调
        .kA = 0.0f,     // 加速度前馈
    };
    FF_Params right_p = {
        .kV = 20.4f,    // 占空比/每RPM 的比例
        .kS = -57.76f,     // 根据实车标定再调
        .kA = 0.0f,     // 加速度前馈
    };
    s_ff.left_params = left_p;
    s_ff.right_params = right_p;
}

static int16_t ff_compute_left_pwm(float rpm_cmd) {
    float sign = (rpm_cmd > 0.0f) - (rpm_cmd < 0.0f);
    float u;
    // u = s_ff.left_params.kS * sign + s_ff.left_params.kV * rpm_cmd;

    // 防止速度太快，小车前翘
    if(slow_accelerate_flag && rpm_cmd > 130) {
        u = s_ff.left_params.kS * sign + s_ff.left_params.kV * 130;
    } else {
        u = s_ff.left_params.kS * sign + s_ff.left_params.kV * rpm_cmd;
    }
    return (int16_t)u;
}

static int16_t ff_compute_right_pwm(float rpm_cmd) {
    float sign = (rpm_cmd > 0.0f) - (rpm_cmd < 0.0f);
    float u;
    if(slow_accelerate_flag && rpm_cmd > 120) {
       u = s_ff.left_params.kS * sign + s_ff.left_params.kV * 120;
    } else {
        u = s_ff.right_params.kS * sign + s_ff.right_params.kV * rpm_cmd;
    }
    // u = s_ff.right_params.kS * sign + s_ff.right_params.kV * rpm_cmd;

    // kA*accel 可后续加入
    return (int16_t)u;
}

void ff_update(int16_t left_target_rpm, int16_t right_target_rpm, 
int16_t *left_pwm, int16_t *right_pwm) {
    // 纯前馈：忽略测量，仅由目标给定计算占空比。
    // 如需稳态误差更小，可在此加入微小比例项 e.g. u += kP*(target - meas)
    *left_pwm = ff_compute_left_pwm((float)left_target_rpm);
    *right_pwm = ff_compute_right_pwm((float)right_target_rpm);
}


