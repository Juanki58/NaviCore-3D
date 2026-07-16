#pragma once

#include "ins_ekf.hpp"

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
    float innov_mean_norm_mps;
    float innov_final_norm_mps;
    InsEkfNhcRunSummary nhc_summary_all;
    InsEkfNhcRunSummary nhc_summary_window;
    bool nhc_summary_all_valid;
    bool nhc_summary_window_valid;
};

enum SuperTunnelImuMode : uint8_t {
    SUPER_TUNNEL_IMU_DIRTY_FULL = 0,
    SUPER_TUNNEL_IMU_NO_SCALE_MISALIGN = 1,
    SUPER_TUNNEL_IMU_IDEAL = 2,
};

enum SuperTunnelNhcPolicy : uint8_t {
    SUPER_TUNNEL_NHC_OFF = 0,
    SUPER_TUNNEL_NHC_ALWAYS = 1,
    /* NHC solo en apagon GPS y sin aceleracion ni giroscopio significativos. */
    SUPER_TUNNEL_NHC_CONSTANT_VEL_ONLY = 2,
    /* NHC solo sin fix GNSS (OFF cuando gps.fix_valid). */
    SUPER_TUNNEL_NHC_NO_GNSS_FIX = 3,
};

struct SuperTunnelRunConfig {
    const char *experiment_id;
    SuperTunnelNhcPolicy nhc_policy;
    SuperTunnelImuMode imu_mode;
    uint32_t rng_seed;
    /* Multiplicadores sobre NAVICORE_INS_EKF_NHC_*_STD_MPS (si override <= 0). */
    float nhc_r_lateral_multiplier;
    float nhc_r_vertical_multiplier;
    /* Si > 0, sigma explicito en m/s (ignora default lateral/vertical). */
    float nhc_lateral_std_override_mps;
    float nhc_vertical_std_override_mps;
    bool verbose;
    const char *nhc_trace_csv_path;
};

SuperTunnelPassResult super_tunnel_run_pass(
    bool nhc_enabled,
    bool verbose = false,
    SuperTunnelImuMode imu_mode = SUPER_TUNNEL_IMU_DIRTY_FULL,
    uint32_t rng_seed = 0U,
    float nhc_lateral_std_mps = 0.0f,
    float nhc_vertical_std_mps = 0.0f);

SuperTunnelPassResult super_tunnel_run_with_config(const SuperTunnelRunConfig &config);

void run_super_tunnel_nhc_benchmark();
int run_super_tunnel_nhc_experiments();
