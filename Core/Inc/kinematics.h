#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <stdint.h>

typedef struct {
    float wheel_radius_mm;
    float track_width_mm;
    int16_t max_linear_mmps;
    int16_t max_angular_mradps;
    int16_t max_wheel_rpm;
    uint16_t max_rpm_step_per_cycle;
} kinematics_config_t;

void kinematics_get_default_config(kinematics_config_t *cfg);
void kinematics_twist_to_wheel_rpm(const kinematics_config_t *cfg,
                                   int16_t v_mmps,
                                   int16_t w_mradps,
                                   uint8_t reverse_allowed,
                                   int16_t *left_rpm,
                                   int16_t *right_rpm);
int16_t kinematics_slew_limit(int16_t current, int16_t target, uint16_t max_step);

#endif
