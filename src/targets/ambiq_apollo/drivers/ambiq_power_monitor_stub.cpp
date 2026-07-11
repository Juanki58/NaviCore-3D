/**
 * @file ambiq_power_monitor_stub.cpp
 * @brief Stub metricas de energia del tick activo
 */
#include "ambiq_power_monitor.hpp"

static PowerMetrics g_power_metrics = {1800.0f, 4.2f, 12500U};

void ambiq_power_monitor_init(void)
{
    g_power_metrics.voltage_core_mv = 1800.0f;
    g_power_metrics.current_draw_ua = 4.2f;
    g_power_metrics.active_cycles = 12500U;
}

void ambiq_power_add_cycles(uint32_t cycles)
{
    g_power_metrics.active_cycles += cycles;
}

void ambiq_power_set_current_ua(float current_ua)
{
    g_power_metrics.current_draw_ua = current_ua;
}

void ambiq_power_get_metrics(PowerMetrics *metrics_out)
{
    if (metrics_out == NULL) {
        return;
    }

    *metrics_out = g_power_metrics;
}
