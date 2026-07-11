/**
 * @file ambiq_power_monitor.hpp
 * @brief Monitor de consumo por tick (SPOT® / corriente core)
 */
#pragma once

#include <stdint.h>

#include "../bsp_sensors.hpp"

void ambiq_power_monitor_init(void);
void ambiq_power_add_cycles(uint32_t cycles);
void ambiq_power_set_current_ua(float current_ua);
void ambiq_power_get_metrics(PowerMetrics *metrics_out);
