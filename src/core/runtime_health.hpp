#pragma once

#include <stdint.h>

/*
 * Instantánea de salud de tiempo real — portable core / Pico2 / simulador PC.
 * Sin heap; la FSM de misión evalúa estos campos cada tick.
 */
struct RuntimeHealth {
    uint32_t max_loop_us;
    uint32_t max_tick_us;
    uint32_t max_wifi_us;
    uint32_t max_housekeeping_us;
    uint32_t max_rxpump_us;
    uint32_t max_logging_us;
    uint32_t uart0_overflows;
    uint32_t uart1_overflows;
    uint32_t missed_ticks;
    uint32_t max_tick_backlog;
    uint32_t loop_budget_exceeded;
    uint32_t wifi_skipped_budget;
    uint32_t i2c_recoveries;
};
