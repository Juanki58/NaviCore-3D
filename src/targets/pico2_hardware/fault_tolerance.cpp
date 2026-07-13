#include "fault_tolerance.hpp"

#include "bsp_power.hpp"
#include "bsp_sensors.hpp"
#include "hw_config.hpp"
#include "loop_metrics.hpp"
#include "safe_log.hpp"

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

namespace {

struct RateWindow {
    uint32_t window_start_ms = 0U;
    uint32_t baseline_total = 0U;
    uint16_t events_in_window = 0U;
};

struct EventWindow {
    uint32_t window_start_ms = 0U;
    uint16_t events_in_window = 0U;
};

bool g_wifi_disabled = false;
uint8_t g_loop_over_20ms_streak = 0U;
EventWindow g_loop_overrun_window{};
RateWindow g_uart0_overflow_window{};
RateWindow g_uart1_overflow_window{};

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

void fault_tolerance_disable_wifi(void)
{
    if (!g_wifi_disabled) {
        g_wifi_disabled = true;
        loop_metrics_set_system_health(SystemHealth::DEGRADED);
        safe_logf(
            "FT: Wi-Fi deshabilitado (loop_budget_exceeded > %u en %u ms)\n",
            PICO2_FT_LOOP_OVERRUN_DEGRADED_MAX,
            PICO2_FT_RATE_WINDOW_MS);
    }
}

void fault_tolerance_request_controlled_restart(void)
{
    loop_metrics_set_system_health(SystemHealth::CRITICAL);
    safe_logf(
        "FT: CRITICAL — reinicio controlado (loop > %u us repetido)\n",
        PICO2_LOOP_RESTART_US);
    safe_log_flush_pending();
    watchdog_reboot(0U, 0U);
}

} /* namespace */

void fault_tolerance_init(void)
{
    g_wifi_disabled = false;
    g_loop_over_20ms_streak = 0U;
    g_loop_overrun_window = EventWindow{};
    g_uart0_overflow_window = RateWindow{};
    g_uart1_overflow_window = RateWindow{};
}

bool fault_tolerance_wifi_poll_allowed(void)
{
    return !g_wifi_disabled;
}

bool fault_tolerance_nav_update_allowed(uint32_t pending_before_consume)
{
    if (pending_before_consume == 0U) {
        return false;
    }

    const uint32_t backlog = pending_before_consume - 1U;
    return backlog <= PICO2_FT_MISSED_TICKS_INVALID_MAX;
}

void fault_tolerance_on_loop_complete(uint32_t loop_us, uint32_t nav_timestamp_ms)
{
    const RuntimeHealth *health = loop_metrics_health();
    if (health == nullptr) {
        return;
    }

    const bool loop_budget_exceeded = (loop_us > PICO2_LOOP_BUDGET_US);
    event_window_tick(&g_loop_overrun_window, nav_timestamp_ms, loop_budget_exceeded);

    if (g_loop_overrun_window.events_in_window > PICO2_FT_LOOP_OVERRUN_DEGRADED_MAX) {
        fault_tolerance_disable_wifi();
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
        fault_tolerance_request_controlled_restart();
    }
}
