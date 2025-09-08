#ifndef FEEDFORWARD_CONTROLLER_H
#define FEEDFORWARD_CONTROLLER_H

#include "stm32f1xx_hal.h"

typedef struct {
    float kS;   // 静摩擦补偿项系数（常量项）
    float kV;   // 速度项系数（V per RPM）
    float kA;   // 加速度项系数（可选）
    int16_t max_rpm; // 机械最大RPM，用于限幅/换算
} FF_Params;

void ff_init_default(void);
void ff_init(const FF_Params* params);

// 设置目标转速（RPM）
void ff_set_target_rpm(int16_t left_target_rpm, int16_t right_target_rpm);

// 每100ms调用一次，输入测得转速（RPM），内部计算PWM并下发到电机
void ff_update_100ms(int16_t left_meas_rpm, int16_t right_meas_rpm);

#endif // FEEDFORWARD_CONTROLLER_H


