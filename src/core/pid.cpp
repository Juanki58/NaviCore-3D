#include "pid.hpp"

#include <math.h>

namespace {

float pid_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float pid_compute_step(
    PIDController *pid,
    float error,
    float dt_s)
{
    if (pid == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    float derivative = 0.0f;
    if (!pid->first_sample) {
        derivative = (error - pid->prev_error) / dt_s;
    } else {
        pid->first_sample = false;
    }

    const float integral_candidate = pid->integral + (error * dt_s);
    const float output_unsat =
        (pid->kp * error)
        + (pid->ki * integral_candidate)
        + (pid->kd * derivative);
    const float output = pid_clampf(output_unsat, pid->out_min, pid->out_max);

    if (output == output_unsat) {
        pid->integral = integral_candidate;
    }

    pid->prev_error = error;
    return output;
}

} /* namespace */

float pid_normalize_angle_rad(float angle_rad)
{
    float normalized = fmodf(angle_rad + static_cast<float>(M_PI), 2.0f * static_cast<float>(M_PI));
    if (normalized < 0.0f) {
        normalized += 2.0f * static_cast<float>(M_PI);
    }
    return normalized - static_cast<float>(M_PI);
}

void PIDController::init(float kp_in, float ki_in, float kd_in, float out_min_in, float out_max_in)
{
    kp = kp_in;
    ki = ki_in;
    kd = kd_in;
    out_min = out_min_in;
    out_max = out_max_in;
    integral = 0.0f;
    prev_error = 0.0f;
    first_sample = true;
}

void PIDController::reset()
{
    integral = 0.0f;
    prev_error = 0.0f;
    first_sample = true;
}

float PIDController::update(float setpoint, float measurement, float dt_s)
{
    const float error = setpoint - measurement;
    return pid_compute_step(this, error, dt_s);
}

float PIDController::update_yaw(float setpoint_rad, float measurement_rad, float dt_s)
{
    const float delta = pid_normalize_angle_rad(setpoint_rad - measurement_rad);
    return pid_compute_step(this, delta, dt_s);
}
