#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/fusion.hpp"

typedef struct {
    bool imu_degraded;
    bool gnss_degraded;
} SensorConfidenceFlags;

bool pico2_bsp_sensors_init(void);
void pico2_bsp_sensors_rx_pump(void);
void pico2_bsp_sensors_housekeeping(uint32_t nav_tick_count);
bool pico2_bsp_sensors_can_sleep(void);
uint32_t pico2_bsp_uart_get_overflow_count(uint8_t uart_id);
void pico2_bsp_sensors_get_confidence_flags(SensorConfidenceFlags *flags_out);
bool pico2_bsp_sensors_tick(
    DeadReckoningFilter *nav_filter,
    uint32_t timestamp_ms,
    bool *gps_fix_valid_out);
