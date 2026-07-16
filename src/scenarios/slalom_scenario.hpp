#pragma once

#include "sensor_types.hpp"

struct InsEkfFilter;
struct TelemetryInterface;

typedef void (*SlalomNavEmitFn)(
    const InsEkfFilter *ekf,
    uint32_t timestamp_ms,
    const GpsSample *gps,
    bool dead_reckoning);

void run_slalom_scenario(
    TelemetryInterface *telemetry,
    SlalomNavEmitFn emit_nav = nullptr);
