#include "feedforward_controller.h"
#include "motor.h"
#include "motor_left.h"
#include "motor_right.h"

typedef struct {
    FF_Params params;
    int16_t target_left_rpm;
    int16_t target_right_rpm;
} FF_Handle;

static FF_Handle s_ff;

void ff_init_default(void) {
    FF_Params p = {
        .kS = 8.0f,     // 根据实车标定再调
        .kV = 0.08f,    // 占空比/每RPM 的比例（示例）
        .kA = 0.0f,     // 初期可不使用加速度前馈
        .max_rpm = 300, // 示例上限
    };
    ff_init(&p);
}

void ff_init(const FF_Params* params) {
    s_ff.params = *params;
    s_ff.target_left_rpm = 0;
    s_ff.target_right_rpm = 0;
}

void ff_set_target_rpm(int16_t left_target_rpm, int16_t right_target_rpm) {
    s_ff.target_left_rpm = left_target_rpm;
    s_ff.target_right_rpm = right_target_rpm;
}

static int8_t ff_compute_pwm_from_rpm(float rpm_cmd) {
    float sign = (rpm_cmd > 0.0f) - (rpm_cmd < 0.0f);
    float u = s_ff.params.kS * sign + s_ff.params.kV * rpm_cmd; // kA*accel 可后续加入
    // 将u映射为占空比百分比，并限幅到[-100,100]
    if (u > 100.0f) u = 100.0f;
    if (u < -100.0f) u = -100.0f;
    return (int8_t)u;
}

void ff_update_100ms(int16_t left_meas_rpm, int16_t right_meas_rpm) {
    // 纯前馈：忽略测量，仅由目标给定计算占空比。
    // 如需稳态误差更小，可在此加入微小比例项 e.g. u += kP*(target - meas)
    int8_t left_pwm = ff_compute_pwm_from_rpm((float)s_ff.target_left_rpm);
    int8_t right_pwm = ff_compute_pwm_from_rpm((float)s_ff.target_right_rpm);

    Motor_Left_Set_Speed(left_pwm);
    Motor_Right_Set_Speed(right_pwm);
}


