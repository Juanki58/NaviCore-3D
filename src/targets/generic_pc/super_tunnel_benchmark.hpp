#pragma once

#include <cstdint>

/* Benchmark NHC (regresion): apagon GPS 10-55 s en recta 90 km/h. */
constexpr uint32_t SUPER_TUNNEL_GPS_OFF_START_MS = 10000U;
constexpr uint32_t SUPER_TUNNEL_GPS_OFF_END_MS = 55000U;

struct SuperTunnelOutageRms {
    float position_m;
    float velocity_mps;
    float yaw_deg;
    uint32_t sample_count;
};

struct SuperTunnelNhcInnovationMax {
    float lateral_mps;
    float vertical_mps;
    float norm_mps;
};

struct SuperTunnelPassResult {
    float drift_exit_tunnel_m;
    float drift_final_m;
    uint32_t nhc_updates;
    SuperTunnelOutageRms outage_rms;
    SuperTunnelNhcInnovationMax nhc_innovation_max;
};

enum SuperTunnelImuMode : uint8_t {
    SUPER_TUNNEL_IMU_DIRTY_FULL = 0,
    SUPER_TUNNEL_IMU_NO_SCALE_MISALIGN = 1,
    SUPER_TUNNEL_IMU_IDEAL = 2,
};

SuperTunnelPassResult super_tunnel_run_pass(
    bool nhc_enabled,
    bool verbose = false,
    SuperTunnelImuMode imu_mode = SUPER_TUNNEL_IMU_DIRTY_FULL,
    uint32_t rng_seed = 0U,
    float nhc_lateral_std_mps = 0.0f,
    float nhc_vertical_std_mps = 0.0f);
void run_super_tunnel_nhc_benchmark();
