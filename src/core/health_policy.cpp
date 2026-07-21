#include "health_policy.hpp"

bool health_policy_imu_silence_is_degraded(uint32_t silence_ms)
{
    return silence_ms >= HEALTH_POLICY_IMU_SILENCE_DEGRADE_MS;
}

bool health_policy_gnss_silence_is_degraded(uint32_t silence_ms)
{
    return silence_ms >= HEALTH_POLICY_GNSS_SILENCE_DEGRADE_MS;
}

bool health_policy_uart_overflow_rate_trips(uint16_t events_in_window)
{
    return events_in_window > HEALTH_POLICY_UART_OVERFLOW_PER_S_MAX;
}

bool health_policy_task_starvation_is_critical(uint64_t idle_us, uint32_t max_idle_us)
{
    if (max_idle_us == 0U) {
        return false;
    }
    return idle_us > static_cast<uint64_t>(max_idle_us);
}

bool health_policy_nav_update_allowed(uint32_t pending_before_consume)
{
    if (pending_before_consume == 0U) {
        return false;
    }
    const uint32_t backlog = pending_before_consume - 1U;
    /* Mirrors PICO2_FT_MISSED_TICKS_INVALID_MAX = 2 */
    return backlog <= 2U;
}

HealthPolicyLevel health_policy_classify_level(const HealthPolicyInput *in)
{
    if (in == nullptr) {
        return HEALTH_POLICY_NOMINAL;
    }

    const bool imu_silent = health_policy_imu_silence_is_degraded(in->imu_silence_ms);
    const bool gnss_silent = health_policy_gnss_silence_is_degraded(in->gnss_silence_ms);

    if (in->last_loop_us >= HEALTH_POLICY_LOOP_CRITICAL_US
        || in->max_tick_backlog >= HEALTH_POLICY_TICK_BACKLOG_CRITICAL
        || in->power_offline
        || health_policy_task_starvation_is_critical(in->task_idle_us, in->task_max_idle_us)
        || in->i2c_recoveries > HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX) {
        return HEALTH_POLICY_CRITICAL;
    }

    if (in->last_loop_us >= HEALTH_POLICY_LOOP_DEGRADED_US
        || in->max_tick_backlog >= HEALTH_POLICY_TICK_BACKLOG_DEGRADED
        || in->consecutive_overrun >= HEALTH_POLICY_LOOP_OVERRUN_DEGRADED
        || in->imu_degraded
        || in->gnss_degraded
        || imu_silent
        || gnss_silent
        || in->imu_cross_check_fail
        || in->uart0_overflows_total >= HEALTH_POLICY_RING_OVERFLOW_DEGRADE
        || in->uart1_overflows_total >= HEALTH_POLICY_RING_OVERFLOW_DEGRADE
        || health_policy_uart_overflow_rate_trips(in->uart0_overflows_in_window)
        || health_policy_uart_overflow_rate_trips(in->uart1_overflows_in_window)) {
        return HEALTH_POLICY_DEGRADED;
    }

    return HEALTH_POLICY_NOMINAL;
}

HealthPolicyDecision health_policy_evaluate(const HealthPolicyInput *in)
{
    HealthPolicyDecision d{};
    d.level = HEALTH_POLICY_NOMINAL;
    d.primary_event = "nominal";
    d.nav_update_allowed = true;

    if (in == nullptr) {
        return d;
    }

    d.level = health_policy_classify_level(in);
    d.imu_should_degrade =
        health_policy_uart_overflow_rate_trips(in->uart0_overflows_in_window)
        || health_policy_imu_silence_is_degraded(in->imu_silence_ms)
        || in->imu_cross_check_fail;
    d.gnss_should_degrade =
        health_policy_uart_overflow_rate_trips(in->uart1_overflows_in_window)
        || health_policy_gnss_silence_is_degraded(in->gnss_silence_ms);
    d.power_should_force_offline =
        in->i2c_recoveries > HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX;
    d.wifi_should_disable = false; /* rate window of loop overruns — caller tracks */
    d.controlled_restart = false;
    d.nav_update_allowed = true;

    if (in->power_offline) {
        d.primary_event = "power_offline";
    } else if (health_policy_task_starvation_is_critical(in->task_idle_us, in->task_max_idle_us)) {
        d.primary_event = "task_starvation";
        d.controlled_restart = true;
    } else if (in->i2c_recoveries > HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX) {
        d.primary_event = "i2c_recoveries";
    } else if (in->last_loop_us >= HEALTH_POLICY_LOOP_CRITICAL_US) {
        d.primary_event = "loop_duration_critical";
    } else if (in->imu_cross_check_fail) {
        d.primary_event = "imu_cross_check_fail";
    } else if (health_policy_imu_silence_is_degraded(in->imu_silence_ms)) {
        d.primary_event = "imu_silence";
    } else if (health_policy_uart_overflow_rate_trips(in->uart0_overflows_in_window)) {
        d.primary_event = "uart0_overflow_rate";
    } else if (health_policy_uart_overflow_rate_trips(in->uart1_overflows_in_window)) {
        d.primary_event = "uart1_overflow_rate";
    } else if (health_policy_gnss_silence_is_degraded(in->gnss_silence_ms)) {
        d.primary_event = "gnss_silence";
    } else if (d.level == HEALTH_POLICY_DEGRADED) {
        d.primary_event = "degraded_aggregate";
    }

    return d;
}
