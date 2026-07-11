/**
 * @file ambiq_gpio_gnss.hpp
 * @brief GPIO / interrupcion GNSS (PPS / UART ready)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_types.hpp"

void ambiq_gpio_gnss_init(void);
bool ambiq_gpio_gnss_data_ready(void);
bool ambiq_gpio_gnss_read_fix(GpsSample *gnss_out, uint32_t tick_index);
