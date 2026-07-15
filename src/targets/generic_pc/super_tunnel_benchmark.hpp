#pragma once

#include <cstdint>

struct TelemetryInterface;

struct SuperTunnelPassResult {
    float drift_exit_tunnel_m;
    float drift_final_m;
    uint32_t nhc_updates;
};

SuperTunnelPassResult super_tunnel_run_pass(bool nhc_enabled, bool verbose = false);
void run_super_tunnel_scenario(TelemetryInterface *telemetry);
