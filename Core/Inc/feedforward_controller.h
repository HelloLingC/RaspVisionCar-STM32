#ifndef FEEDFORWARD_CONTROLLER_H
#define FEEDFORWARD_CONTROLLER_H
#include <stdint.h>


typedef struct {
    float kS;   // 静摩擦补偿项系数（常量项）
    float kV;   // 速度项系数（V per RPM）
    float kA;   // 加速度项系数（可选）
} FF_Params;

void ff_init_default(void);

// 调用一次，输入目标转速（RPM），内部计算PWM并下发到电机
void ff_update(int16_t left_target_rpm, int16_t right_target_rpm, 
                int16_t *left_pwm, int16_t *right_pwm);

#endif // FEEDFORWARD_CONTROLLER_H


