#include "loop_metrics.hpp"
#include "hw_config.hpp"

namespace {

RuntimeHealth g_health{};
SystemHealth g_system_health = SystemHealth::NOMINAL;
uint8_t g_consecutive_loop_overrun = 0U;
uint32_t g_last_loop_us = 0U;

void health_update_max(uint32_t *field, uint32_t value)
{
    if (value > *field) {
        *field = value;
    }
}

SystemHealth health_evaluate(
    uint32_t last_loop_us,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline)
{
    if (last_loop_us >= PICO2_LOOP_CRITICAL_US
        || g_health.max_tick_backlog >= PICO2_TICK_BACKLOG_CRITICAL
        || power_offline) {
        return SystemHealth::CRITICAL;
    }

    if (last_loop_us >= PICO2_LOOP_DEGRADED_US
        || g_health.max_tick_backlog >= PICO2_TICK_BACKLOG_DEGRADED
        || g_consecutive_loop_overrun >= PICO2_LOOP_OVERRUN_DEGRADED
        || imu_degraded
        || gnss_degraded
        || g_health.uart0_overflows >= PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD
        || g_health.uart1_overflows >= PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD) {
        return SystemHealth::DEGRADED;
    }

    return SystemHealth::NOMINAL;
}

} /* namespace */

void loop_metrics_init(void)
{
    g_health = RuntimeHealth{};
    g_system_health = SystemHealth::NOMINAL;
    g_consecutive_loop_overrun = 0U;
    g_last_loop_us = 0U;
}

void loop_metrics_on_loop_complete(uint64_t loop_time_us)
{
    const uint32_t loop_us = static_cast<uint32_t>(loop_time_us);
    g_last_loop_us = loop_us;
    health_update_max(&g_health.max_loop_us, loop_us);
    if (loop_us > PICO2_LOOP_BUDGET_US) {
        ++g_health.loop_budget_exceeded;
        if (g_consecutive_loop_overrun < 255U) {
            ++g_consecutive_loop_overrun;
        }
    } else {
        g_consecutive_loop_overrun = 0U;
    }
}

void loop_metrics_report_due(void)
{
    (void)PICO2_LOOP_METRICS_REPORT_MS;
}

uint32_t loop_metrics_max_loop_time_us(void)
{
    return g_health.max_loop_us;
}

void loop_metrics_record_rx_pump_us(uint32_t elapsed_us)
{
    health_update_max(&g_health.max_rxpump_us, elapsed_us);
}

void loop_metrics_record_tick_us(uint32_t elapsed_us)
{
    health_update_max(&g_health.max_tick_us, elapsed_us);
}

void loop_metrics_record_housekeeping_us(uint32_t elapsed_us)
{
    health_update_max(&g_health.max_housekeeping_us, elapsed_us);
}

void loop_metrics_record_wifi_us(uint32_t elapsed_us)
{
    health_update_max(&g_health.max_wifi_us, elapsed_us);
}

void loop_metrics_record_logging_us(uint32_t elapsed_us)
{
    health_update_max(&g_health.max_logging_us, elapsed_us);
}

void loop_metrics_add_missed_ticks(uint32_t count)
{
    g_health.missed_ticks += count;
}

void loop_metrics_record_tick_backlog(uint32_t pending_before_consume)
{
    if (pending_before_consume <= 1U) {
        return;
    }

    const uint32_t backlog = pending_before_consume - 1U;
    health_update_max(&g_health.missed_ticks, backlog);
    health_update_max(&g_health.max_tick_backlog, backlog);
}

void loop_metrics_add_wifi_skipped(void)
{
    ++g_health.wifi_skipped_budget;
}

void loop_metrics_add_i2c_recovery(void)
{
    ++g_health.i2c_recoveries;
}

void loop_metrics_sync_uart_overflows(uint32_t uart0_total, uint32_t uart1_total)
{
    g_health.uart0_overflows = uart0_total;
    g_health.uart1_overflows = uart1_total;
}

const RuntimeHealth *loop_metrics_health(void)
{
    return &g_health;
}

void loop_metrics_update_system_health(
    uint32_t last_loop_us,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline)
{
    const SystemHealth evaluated = health_evaluate(
        last_loop_us,
        imu_degraded,
        gnss_degraded,
        power_offline);

    if (static_cast<uint8_t>(evaluated) > static_cast<uint8_t>(g_system_health)) {
        g_system_health = evaluated;
    }
}

void loop_metrics_set_system_health(SystemHealth health)
{
    if (static_cast<uint8_t>(health) > static_cast<uint8_t>(g_system_health)) {
        g_system_health = health;
    }
}

SystemHealth loop_metrics_system_health(void)
{
    return g_system_health;
}
