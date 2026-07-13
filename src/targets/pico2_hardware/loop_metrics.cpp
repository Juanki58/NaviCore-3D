#include "loop_metrics.hpp"
#include "hw_config.hpp"

namespace {

RuntimeHealth g_health{};

void health_update_max(uint32_t *field, uint32_t value)
{
    if (value > *field) {
        *field = value;
    }
}

} /* namespace */

void loop_metrics_init(void)
{
    g_health = RuntimeHealth{};
}

void loop_metrics_on_loop_complete(uint64_t loop_time_us)
{
    const uint32_t loop_us = static_cast<uint32_t>(loop_time_us);
    health_update_max(&g_health.max_loop_us, loop_us);
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
