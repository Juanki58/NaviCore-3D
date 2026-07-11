#include "slip_compensation.hpp"

#include "math_utils.hpp"

#include <math.h>

static uint8_t slip_comp_clamp_health_score(int32_t value)
{
    if (value < (int32_t)DIAG_HEALTH_SCORE_MIN) {
        return DIAG_HEALTH_SCORE_MIN;
    }
    if (value > (int32_t)DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return (uint8_t)value;
}

static NavHealthMode slip_comp_mode_from_score(uint8_t health_score)
{
    if (health_score <= DIAG_HEALTH_SCORE_CRITICAL_MAX) {
        return HEALTH_CRITICAL;
    }
    if (health_score < DIAG_HEALTH_SCORE_NOMINAL_MIN) {
        return HEALTH_DEGRADED;
    }
    return HEALTH_NOMINAL;
}

static void slip_comp_penalize_monitor(
    SystemHealthMonitor *monitor,
    float slip_ratio)
{
    if (monitor == NULL) {
        return;
    }

    monitor->last_slip_comp_error = SLIP_COMP_ERROR_SLIP;
    monitor->last_slip_ratio = slip_ratio;

    const int32_t penalized =
        (int32_t)monitor->health_score - (int32_t)SLIP_COMP_HEALTH_PENALTY;
    monitor->health_score = slip_comp_clamp_health_score(penalized);
    monitor->mode = slip_comp_mode_from_score(monitor->health_score);
    monitor->update_count++;
}

static float slip_comp_resolve_imu_speed_mps(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return 0.0f;
    }

    if (filter->imu_speed_prediction_valid) {
        return filter->imu_predicted_speed_mps;
    }

    return navstate_speed_mps(&filter->state);
}

static float slip_comp_compute_ratio(float imu_speed_mps, float wheel_speed_mps)
{
    const float imu_abs = fabsf(imu_speed_mps);
    const float wheel_abs = fabsf(wheel_speed_mps);
    float reference_mps = imu_abs;

    if (wheel_abs > reference_mps) {
        reference_mps = wheel_abs;
    }

    if (reference_mps <= NAVICORE_EPS_SPEED_MPS) {
        return 0.0f;
    }

    return fabsf(imu_abs - wheel_abs) / reference_mps;
}

static float slip_comp_exponential_noise_scale(float slip_ratio)
{
    const float excess = slip_ratio - NAVICORE_SLIP_RATIO_THRESHOLD;

    if (excess <= 0.0f) {
        return 1.0f;
    }

    float scale = expf(excess * SLIP_COMP_EXP_NOISE_GAIN);

    if (scale > SLIP_COMP_MAX_NOISE_SCALE) {
        scale = SLIP_COMP_MAX_NOISE_SCALE;
    }

    if (scale < 1.0f) {
        scale = 1.0f;
    }

    return scale;
}

void slip_compensation_evaluate(
    DeadReckoningFilter *filter,
    float wheel_speed_mps,
    SystemHealthMonitor *monitor)
{
    if (filter == NULL) {
        return;
    }

    const float imu_speed_mps = slip_comp_resolve_imu_speed_mps(filter);
    const float slip_ratio = slip_comp_compute_ratio(imu_speed_mps, wheel_speed_mps);

    filter->slip_ratio = slip_ratio;

    if (slip_ratio > NAVICORE_SLIP_RATIO_THRESHOLD) {
        filter->slip_fault_active = true;
        filter->odom_noise_covariance_scale = slip_comp_exponential_noise_scale(slip_ratio);
        filter->quality |= NAVICORE_QUALITY_ODOM_FAULT;
        slip_comp_penalize_monitor(monitor, slip_ratio);
        return;
    }

    filter->slip_fault_active = false;
    filter->odom_noise_covariance_scale = 1.0f;

    if (monitor != NULL) {
        monitor->last_slip_comp_error = SLIP_COMP_ERROR_NONE;
        monitor->last_slip_ratio = slip_ratio;
    }
}

float slip_compensation_get_last_ratio(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return 0.0f;
    }

    return filter->slip_ratio;
}
