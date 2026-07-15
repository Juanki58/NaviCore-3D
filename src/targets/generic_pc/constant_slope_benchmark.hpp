#pragma once

#include <cstdint>

constexpr uint32_t TC03_GPS_OFF_START_MS = 15000U;
constexpr uint32_t TC03_GPS_OFF_END_MS = 30000U;
constexpr uint32_t TC03_DURATION_MS = 40000U;
constexpr float TC03_SPEED_MPS = 25.0f;
constexpr float TC03_GRADE_PERCENT = 10.0f;
constexpr float TC03_PITCH_RAD = 0.099668652f; /* atan(10%) ≈ 5.71 deg */
constexpr float TC03_COURSE_DEG = 90.0f;

struct ConstantSlopeOutageRms {
    float position_m;
    float velocity_mps;
    float velocity_d_mps;
    float yaw_deg;
    uint32_t sample_count;
};

struct ConstantSlopeNhcInnovationMax {
    float lateral_mps;
    float vertical_mps;
    float norm_mps;
};

struct ConstantSlopePassResult {
    float drift_exit_outage_m;
    float drift_final_m;
    uint32_t nhc_updates;
    ConstantSlopeOutageRms outage_rms;
    ConstantSlopeNhcInnovationMax nhc_innovation_max;
};

ConstantSlopePassResult constant_slope_run_pass(bool nhc_enabled, bool verbose = false);
