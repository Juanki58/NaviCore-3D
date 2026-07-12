#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/fusion.hpp"

bool pico_bsp_sensors_init(void);
bool pico_bsp_sensors_tick(DeadReckoningFilter *nav_filter, uint32_t timestamp_ms, bool *gps_fix_valid_out);
