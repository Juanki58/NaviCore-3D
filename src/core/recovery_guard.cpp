#include "recovery_guard.hpp"

#include "divergence_guard.hpp"
#include "NavState.h"

#include <math.h>

/** Estado estatico del guard — unico contador, cero heap. */
static struct {
    uint32_t consecutive_clean_ticks;
} g_recovery_guard{};

static bool recovery_guard_is_filter_stopped(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return false;
    }

    return navstate_speed_mps(&filter->state) == 0.0f;
}

static bool recovery_guard_conditions_met(
    const DeadReckoningFilter *filter,
    const SystemHealthMonitor *monitor,
    float current_innovation_sq)
{
    if (filter == NULL || monitor == NULL) {
        return false;
    }

    if (monitor->shutdown_latched) {
        return false;
    }

    if (monitor->mode != HEALTH_CRITICAL) {
        return false;
    }

    if (!recovery_guard_is_filter_stopped(filter)) {
        return false;
    }

    if (isnan(current_innovation_sq) || isinf(current_innovation_sq)) {
        return false;
    }

    return current_innovation_sq < RECOVERY_GUARD_INNOVATION_SQ_THRESHOLD;
}

static void recovery_guard_reset_filter_covariance_p(DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->position_prior_variance_m2 =
        NAVICORE_GPS_MEASUREMENT_VARIANCE_M2 * RECOVERY_GUARD_COVARIANCE_GPS_FACTOR;
    filter->gps_noise_covariance_scale = 1.0f;
    filter->odom_noise_covariance_scale = 1.0f;
}

static void recovery_guard_clear_monitor_error_flags(SystemHealthMonitor *monitor)
{
    if (monitor == NULL) {
        return;
    }

    monitor->last_time_guard_error = TIME_GUARD_ERROR_NONE;
    monitor->last_slip_comp_error = SLIP_COMP_ERROR_NONE;
    monitor->last_geometry_error = GEOMETRY_ERROR_NONE;
    monitor->last_divergence_error = DIVERGENCE_ERROR_NONE;
    monitor->last_execution_ticks = 0U;
    monitor->last_max_allowed_ticks = 0U;
    monitor->last_slip_ratio = 0.0f;
    monitor->last_geometry_step_m = 0.0f;
    monitor->last_divergence_innovation_sq = 0.0f;
}

static void recovery_guard_apply_hot_restart(
    DeadReckoningFilter *filter,
    SystemHealthMonitor *monitor)
{
    dead_reckoning_reset_imu_bias_state(filter);
    recovery_guard_reset_filter_covariance_p(filter);

    monitor->health_score = RECOVERY_GUARD_RECOVERED_HEALTH_SCORE;
    monitor->mode = HEALTH_NOMINAL;
    recovery_guard_clear_monitor_error_flags(monitor);
    divergence_guard_reset();
    monitor->update_count++;
}

void recovery_guard_reset(void)
{
    g_recovery_guard.consecutive_clean_ticks = 0U;
}

bool recovery_guard_step(
    DeadReckoningFilter *filter,
    SystemHealthMonitor *monitor,
    float current_innovation_sq)
{
    if (monitor != NULL && monitor->shutdown_latched) {
        recovery_guard_reset();
        return false;
    }

    if (!recovery_guard_conditions_met(filter, monitor, current_innovation_sq)) {
        recovery_guard_reset();
        return false;
    }

    g_recovery_guard.consecutive_clean_ticks++;

    if (g_recovery_guard.consecutive_clean_ticks < RECOVERY_GUARD_CLEAN_TICKS_REQUIRED) {
        return false;
    }

    recovery_guard_apply_hot_restart(filter, monitor);
    recovery_guard_reset();
    return true;
}
