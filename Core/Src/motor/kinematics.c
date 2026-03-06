#include "kinematics.h"

#include <math.h>

#define PI_F 3.1415926f

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void kinematics_get_default_config(kinematics_config_t *cfg)
{
    if (cfg == 0) {
        return;
    }

    cfg->wheel_radius_mm = 33.0f;
    cfg->track_width_mm = 140.0f;
    cfg->max_linear_mmps = 900;
    cfg->max_angular_mradps = 4500;
    cfg->max_wheel_rpm = 220;
    cfg->max_rpm_step_per_cycle = 30U;
}

void kinematics_twist_to_wheel_rpm(const kinematics_config_t *cfg,
                                   int16_t v_mmps,
                                   int16_t w_mradps,
                                   uint8_t reverse_allowed,
                                   int16_t *left_rpm,
                                   int16_t *right_rpm)
{
    float v_l_mmps;
    float v_r_mmps;
    float w_radps;
    float wheel_circumference;
    int16_t v_cmd;
    int16_t w_cmd;
    int16_t l_rpm;
    int16_t r_rpm;

    if (cfg == 0 || left_rpm == 0 || right_rpm == 0) {
        return;
    }

    v_cmd = clamp_i16(v_mmps, (int16_t)-cfg->max_linear_mmps, cfg->max_linear_mmps);
    if (reverse_allowed == 0U && v_cmd < 0) {
        v_cmd = 0;
    }
    w_cmd = clamp_i16(w_mradps, (int16_t)-cfg->max_angular_mradps, cfg->max_angular_mradps);
    w_radps = ((float)w_cmd) / 1000.0f;

    v_l_mmps = (float)v_cmd - (w_radps * cfg->track_width_mm * 0.5f);
    v_r_mmps = (float)v_cmd + (w_radps * cfg->track_width_mm * 0.5f);
    wheel_circumference = 2.0f * PI_F * cfg->wheel_radius_mm;

    l_rpm = (int16_t)lroundf((v_l_mmps / wheel_circumference) * 60.0f);
    r_rpm = (int16_t)lroundf((v_r_mmps / wheel_circumference) * 60.0f);

    l_rpm = clamp_i16(l_rpm, (int16_t)-cfg->max_wheel_rpm, cfg->max_wheel_rpm);
    r_rpm = clamp_i16(r_rpm, (int16_t)-cfg->max_wheel_rpm, cfg->max_wheel_rpm);

    *left_rpm = l_rpm;
    *right_rpm = r_rpm;
}

int16_t kinematics_slew_limit(int16_t current, int16_t target, uint16_t max_step)
{
    int32_t delta = (int32_t)target - (int32_t)current;

    if (delta > (int32_t)max_step) {
        return (int16_t)(current + (int16_t)max_step);
    }
    if (delta < -(int32_t)max_step) {
        return (int16_t)(current - (int16_t)max_step);
    }

    return target;
}
