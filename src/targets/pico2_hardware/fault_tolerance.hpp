#pragma once

#include <stdbool.h>
#include <stdint.h>

void fault_tolerance_init(void);
bool fault_tolerance_wifi_poll_allowed(void);
bool fault_tolerance_nav_update_allowed(uint32_t pending_before_consume);
void fault_tolerance_on_loop_complete(uint32_t loop_us, uint32_t nav_timestamp_ms);
