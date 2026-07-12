#pragma once

#include <cstdint>

#include "../../core/diagnostic.hpp"
#include "../../core/fusion.hpp"
#include "../../core/navigation_cortex.hpp"

void pico_navigation_cortex_tick(
    NavigationCortexState *cortex_state,
    DeadReckoningFilter *nav,
    SystemHealthMonitor *health,
    bool gps_fix_valid,
    uint8_t filter_quality_u8,
    uint8_t worst_bsp_bus,
    uint32_t timestamp_ms,
    NavigationDecision *decision);
