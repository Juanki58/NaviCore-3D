#pragma once

#include "../../core/navigation_state.hpp"

#include <stdbool.h>
#include <stdint.h>

bool pico2_nav_state_udp_init(const char *host, uint16_t port);
bool pico2_nav_state_udp_is_ready(void);
bool pico2_nav_state_udp_send(const NavigationState *state);
void pico2_nav_state_udp_close(void);
