#include "divergence_guard.hpp"

#include "NavState.h"

#include <math.h>

static struct {
    uint32_t consecutive_over_threshold_ticks;
    bool divergence_latched;
} g_divergence_guard{};

static uint8_t divergence_guard_clamp_health_score(int32_t value)
{
    if (value < (int32_t)DIAG_HEALTH_SCORE_MIN) {
        return DIAG_HEALTH_SCORE_MIN;
    }
    if (value > (int32_t)DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return (uint8_t)value;
}

static NavHealthMode divergence_guard_mode_from_score(uint8_t health_score)
{
    if (health_score <= DIAG_HEALTH_SCORE_CRITICAL_MAX) {
        return HEALTH_CRITICAL;
    }
    if (health_score < DIAG_HEALTH_SCORE_NOMINAL_MIN) {
        return HEALTH_DEGRADED;
    }
    return HEALTH_NOMINAL;
}

static bool divergence_guard_is_finite_speed(float speed_mps)
{
    return !isnan(speed_mps) && !isinf(speed_mps);
}

static float divergence_guard_filter_speed_mps(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return 0.0f;
    }

    return navstate_speed_mps(&filter->state);
}

static void divergence_guard_inject_imu_gps_error(
    SystemHealthMonitor *monitor,
    float innovation_sq)
{
    if (monitor == NULL) {
        return;
    }

    monitor->last_divergence_error = DIVERGENCE_ERROR_IMU_GPS;
    monitor->last_divergence_innovation_sq = innovation_sq;

    const int32_t penalized =
        (int32_t)monitor->health_score - (int32_t)DIVERGENCE_GUARD_HEALTH_PENALTY;
    monitor->health_score = divergence_guard_clamp_health_score(penalized);
    monitor->mode = divergence_guard_mode_from_score(monitor->health_score);
    monitor->update_count++;
}

void divergence_guard_reset(void)
{
    g_divergence_guard.consecutive_over_threshold_ticks = 0U;
    g_divergence_guard.divergence_latched = false;
}

bool divergence_guard_check(
    DeadReckoningFilter *filter,
    float gps_speed_mps,
    SystemHealthMonitor *monitor)
{
    if (filter == NULL) {
        return false;
    }

    const float filter_speed_mps = divergence_guard_filter_speed_mps(filter);

    if (!divergence_guard_is_finite_speed(filter_speed_mps) ||
        !divergence_guard_is_finite_speed(gps_speed_mps)) {
        divergence_guard_reset();
        if (monitor != NULL) {
            monitor->last_divergence_error = DIVERGENCE_ERROR_NONE;
        }
        return false;
    }

    const float innovation_mps = filter_speed_mps - gps_speed_mps;
    const float innovation_sq = innovation_mps * innovation_mps;

    if (monitor != NULL) {
        monitor->last_divergence_innovation_sq = innovation_sq;
    }

    if (innovation_sq > NAVICORE_DIVERGENCE_INNOVATION_SQ_THRESHOLD) {
        g_divergence_guard.consecutive_over_threshold_ticks++;

        if (g_divergence_guard.consecutive_over_threshold_ticks >
            DIVERGENCE_CONSECUTIVE_TICKS_REQUIRED) {
            if (!g_divergence_guard.divergence_latched) {
                divergence_guard_inject_imu_gps_error(monitor, innovation_sq);
                g_divergence_guard.divergence_latched = true;
            }
            return true;
        }

        return false;
    }

    divergence_guard_reset();
    if (monitor != NULL) {
        monitor->last_divergence_error = DIVERGENCE_ERROR_NONE;
    }

    return false;
}
