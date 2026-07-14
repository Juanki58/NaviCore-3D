#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * PIDController — control proporcional-integral-derivativo en float (FPU).
 * Zero-heap, sin memoria dinámica. Anti-windup por clamping de integral.
 */

struct PIDController {
    float kp;
    float ki;
    float kd;
    float out_min;
    float out_max;
    float integral;
    float prev_error;
    bool first_sample;

    void init(float kp_in, float ki_in, float kd_in, float out_min_in, float out_max_in);
    void reset();
    float update(float setpoint, float measurement, float dt_s);
    float update_yaw(float setpoint_rad, float measurement_rad, float dt_s);
};

float pid_normalize_angle_rad(float angle_rad);
