#include "loop_metrics.hpp"
#include "hw_config.hpp"
#include "safe_log.hpp"

#include "pico/stdlib.h"

namespace {

uint32_t g_max_loop_time_us = 0U;
uint32_t g_window_max_loop_time_us = 0U;
uint64_t g_last_report_us = 0U;

} /* namespace */

void loop_metrics_init(void)
{
    g_max_loop_time_us = 0U;
    g_window_max_loop_time_us = 0U;
    g_last_report_us = time_us_64();
}

void loop_metrics_on_loop_complete(uint64_t loop_time_us)
{
    const uint32_t loop_us = static_cast<uint32_t>(loop_time_us);

    if (loop_us > g_max_loop_time_us) {
        g_max_loop_time_us = loop_us;
    }
    if (loop_us > g_window_max_loop_time_us) {
        g_window_max_loop_time_us = loop_us;
    }
}

void loop_metrics_report_due(void)
{
    const uint64_t now_us = time_us_64();
    if ((now_us - g_last_report_us) < (static_cast<uint64_t>(PICO2_LOOP_METRICS_REPORT_MS) * 1000ULL)) {
        return;
    }

    g_last_report_us = now_us;

    safe_logf("WCET loop max_loop_time_us=%u (ventana %u ms)\n", g_window_max_loop_time_us, PICO2_LOOP_METRICS_REPORT_MS);

    g_window_max_loop_time_us = 0U;
}

uint32_t loop_metrics_max_loop_time_us(void)
{
    return g_max_loop_time_us;
}
