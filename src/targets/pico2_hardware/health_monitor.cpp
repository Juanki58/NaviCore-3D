#include "health_monitor.hpp"

#include "bsp_power.hpp"
#include "bsp_sensors.hpp"
#include "hw_config.hpp"
#include "safe_log.hpp"
#include "task_monitor.hpp"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

namespace {

/*
 * Tabla evento → clasificación → recuperación.
 * Umbrales en hw_config.hpp: SUPUESTO — pendiente P3 (campaña WCET).
 */
constexpr HealthPolicyDescriptor k_policy_table[] = {
    {
        "loop_budget_exceeded_rate",
        HealthPolicyRecovery::Permanent,
        SystemHealth::DEGRADED,
    },
    {
        "uart0_overflow_rate",
        HealthPolicyRecovery::Permanent,
        SystemHealth::DEGRADED,
    },
    {
        "uart1_overflow_rate",
        HealthPolicyRecovery::Permanent,
        SystemHealth::DEGRADED,
    },
    {
        "tick_backlog",
        HealthPolicyRecovery::Recoverable,
        SystemHealth::DEGRADED,
    },
    {
        "i2c_recoveries",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "loop_duration_streak",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "housekeeping_starvation",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "rx_pump_starvation",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "nav_tick_starvation",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "wifi_starvation",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
    {
        "loop_starvation",
        HealthPolicyRecovery::Permanent,
        SystemHealth::CRITICAL,
    },
};

struct RateWindow {
    uint32_t window_start_ms = 0U;
    uint32_t baseline_total = 0U;
    uint16_t events_in_window = 0U;
};

struct EventWindow {
    uint32_t window_start_ms = 0U;
    uint16_t events_in_window = 0U;
};

SystemHealth g_system_health = SystemHealth::NOMINAL;
bool g_wifi_disabled = false;
uint8_t g_loop_over_20ms_streak = 0U;
EventWindow g_loop_overrun_window{};
RateWindow g_uart0_overflow_window{};
RateWindow g_uart1_overflow_window{};

void health_monitor_set_system_health(SystemHealth health)
{
    if (static_cast<uint8_t>(health) > static_cast<uint8_t>(g_system_health)) {
        g_system_health = health;
    }
}

SystemHealth health_monitor_evaluate(
    uint32_t last_loop_us,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline)
{
    const RuntimeHealth *health = loop_metrics_health();
    if (health == nullptr) {
        return SystemHealth::NOMINAL;
    }

    if (last_loop_us >= PICO2_LOOP_CRITICAL_US
        || health->max_tick_backlog >= PICO2_TICK_BACKLOG_CRITICAL
        || power_offline) {
        return SystemHealth::CRITICAL;
    }

    if (last_loop_us >= PICO2_LOOP_DEGRADED_US
        || health->max_tick_backlog >= PICO2_TICK_BACKLOG_DEGRADED
        || loop_metrics_consecutive_overrun() >= PICO2_LOOP_OVERRUN_DEGRADED
        || imu_degraded
        || gnss_degraded
        || health->uart0_overflows >= PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD
        || health->uart1_overflows >= PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD) {
        return SystemHealth::DEGRADED;
    }

    return SystemHealth::NOMINAL;
}

void event_window_tick(EventWindow *window, uint32_t timestamp_ms, bool event)
{
    if (window == nullptr) {
        return;
    }

    if (window->window_start_ms == 0U) {
        window->window_start_ms = timestamp_ms;
    }

    if ((timestamp_ms - window->window_start_ms) >= PICO2_FT_RATE_WINDOW_MS) {
        window->window_start_ms = timestamp_ms;
        window->events_in_window = 0U;
    }

    if (event && window->events_in_window < 0xFFFFU) {
        ++window->events_in_window;
    }
}

void rate_window_update(RateWindow *window, uint32_t timestamp_ms, uint32_t total_count)
{
    if (window == nullptr) {
        return;
    }

    if (window->window_start_ms == 0U) {
        window->window_start_ms = timestamp_ms;
        window->baseline_total = total_count;
    }

    if (total_count > window->baseline_total) {
        window->events_in_window = static_cast<uint16_t>(
            window->events_in_window + (total_count - window->baseline_total));
        window->baseline_total = total_count;
    }

    if ((timestamp_ms - window->window_start_ms) >= PICO2_FT_RATE_WINDOW_MS) {
        window->window_start_ms = timestamp_ms;
        window->events_in_window = 0U;
        window->baseline_total = total_count;
    }
}

void health_monitor_disable_wifi(void)
{
    if (!g_wifi_disabled) {
        g_wifi_disabled = true;
        health_monitor_set_system_health(SystemHealth::DEGRADED);
        safe_logf(
            "HM: Wi-Fi deshabilitado (loop_budget_exceeded > %u en %u ms)\n",
            PICO2_FT_LOOP_OVERRUN_DEGRADED_MAX,
            PICO2_FT_RATE_WINDOW_MS);
    }
}

void health_monitor_request_controlled_restart(void)
{
    health_monitor_set_system_health(SystemHealth::CRITICAL);
    safe_logf(
        "HM: CRITICAL — reinicio controlado (loop > %u us repetido)\n",
        PICO2_LOOP_RESTART_US);
    safe_log_flush_pending();
    watchdog_reboot(0U, 0U);
}

} /* namespace */

void health_monitor_init(void)
{
    g_system_health = SystemHealth::NOMINAL;
    g_wifi_disabled = false;
    g_loop_over_20ms_streak = 0U;
    g_loop_overrun_window = EventWindow{};
    g_uart0_overflow_window = RateWindow{};
    g_uart1_overflow_window = RateWindow{};
}

bool health_monitor_wifi_poll_allowed(void)
{
    return !g_wifi_disabled;
}

bool health_monitor_nav_update_allowed(uint32_t pending_before_consume)
{
    if (pending_before_consume == 0U) {
        return false;
    }

    const uint32_t backlog = pending_before_consume - 1U;
    return backlog <= PICO2_FT_MISSED_TICKS_INVALID_MAX;
}

void health_monitor_check_task_deadline(TaskId id, uint32_t max_idle_us, const char *label)
{
    const uint64_t idle_us = task_monitor_idle_us(id);
    if (idle_us > static_cast<uint64_t>(max_idle_us)) {
        health_monitor_set_system_health(SystemHealth::CRITICAL);
        safe_logf(
            "HM: CRITICAL — %s sin ejecutar en %u ms\n",
            label,
            max_idle_us / 1000U);
        safe_log_flush_pending();
        watchdog_reboot(0U, 0U);
    }
}

void health_monitor_on_loop_complete(
    uint32_t loop_us,
    uint32_t nav_timestamp_ms,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline)
{
    const SystemHealth evaluated = health_monitor_evaluate(
        loop_us,
        imu_degraded,
        gnss_degraded,
        power_offline);
    health_monitor_set_system_health(evaluated);

    const RuntimeHealth *health = loop_metrics_health();
    if (health == nullptr) {
        return;
    }

    const bool loop_budget_exceeded = (loop_us > PICO2_LOOP_BUDGET_US);
    event_window_tick(&g_loop_overrun_window, nav_timestamp_ms, loop_budget_exceeded);

    if (g_loop_overrun_window.events_in_window > PICO2_FT_LOOP_OVERRUN_DEGRADED_MAX) {
        health_monitor_disable_wifi();
    }

    rate_window_update(
        &g_uart0_overflow_window,
        nav_timestamp_ms,
        health->uart0_overflows);
    rate_window_update(
        &g_uart1_overflow_window,
        nav_timestamp_ms,
        health->uart1_overflows);

    if (g_uart0_overflow_window.events_in_window > PICO2_FT_UART_OVERFLOW_PER_S_MAX) {
        pico2_bsp_sensors_set_imu_degraded(true);
    }

    if (g_uart1_overflow_window.events_in_window > PICO2_FT_UART_OVERFLOW_PER_S_MAX) {
        pico2_bsp_sensors_set_gnss_degraded(true);
    }

    if (health->i2c_recoveries > PICO2_FT_I2C_RECOVERY_OFFLINE_MAX) {
        pico2_bsp_power_force_offline();
    }

    if (loop_us > PICO2_LOOP_RESTART_US) {
        if (g_loop_over_20ms_streak < 255U) {
            ++g_loop_over_20ms_streak;
        }
    } else {
        g_loop_over_20ms_streak = 0U;
    }

    if (g_loop_over_20ms_streak >= PICO2_LOOP_RESTART_STREAK) {
        health_monitor_request_controlled_restart();
    }
}

const RuntimeHealth *health_monitor_runtime(void)
{
    return loop_metrics_health();
}

SystemHealth health_monitor_system_health(void)
{
    return g_system_health;
}

const HealthPolicyDescriptor *health_monitor_policy_table(uint8_t *count)
{
    if (count != nullptr) {
        *count = static_cast<uint8_t>(sizeof(k_policy_table) / sizeof(k_policy_table[0]));
    }

    return k_policy_table;
}
