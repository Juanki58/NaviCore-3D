#pragma once

#include "sensor_types.hpp"

class INaviFilter;
struct TelemetryInterface;

typedef void (*SlalomNavEmitFn)(
    const INaviFilter *filter,
    uint32_t timestamp_ms,
    const GpsSample *gps,
    bool dead_reckoning);

void run_slalom_scenario(
    TelemetryInterface *telemetry,
    SlalomNavEmitFn emit_nav = nullptr);
