#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/fusion.hpp"

bool pico2_bsp_sensors_init(void);
void pico2_bsp_sensors_rx_pump(void);
void pico2_bsp_sensors_housekeeping(uint32_t nav_tick_count);
bool pico2_bsp_sensors_can_sleep(void);
bool pico2_bsp_sensors_tick(
    DeadReckoningFilter *nav_filter,
    uint32_t timestamp_ms,
    bool *gps_fix_valid_out);
