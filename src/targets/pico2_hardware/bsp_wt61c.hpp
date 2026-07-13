#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/sensor_types.hpp"

bool pico2_bsp_wt61c_init(void);
bool pico2_bsp_wt61c_rx_pending(void);
void pico2_bsp_wt61c_rx_pump(uint16_t byte_budget);
bool pico2_bsp_wt61c_poll(ImuSample *imu_out);
