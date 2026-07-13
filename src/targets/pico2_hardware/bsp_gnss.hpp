#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/sensor_types.hpp"

bool pico2_bsp_gnss_init(void);
bool pico2_bsp_gnss_rx_pending(void);
void pico2_bsp_gnss_rx_pump(uint16_t byte_budget);
bool pico2_bsp_gnss_poll(GpsSample *gps_out);
