#pragma once

#include <stdbool.h>
#include <stdint.h>

bool pico2_bsp_power_init(void);
void pico2_bsp_power_poll(uint32_t nav_tick_count);
bool pico2_bsp_power_is_offline(void);
bool pico2_bsp_power_consume_battery(uint16_t *battery_mv_out);
void pico2_bsp_power_force_offline(void);
