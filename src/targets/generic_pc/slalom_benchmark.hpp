#pragma once

#include <cstdint>

constexpr uint32_t TC04_GPS_OFF_START_MS = 10000U;
constexpr uint32_t TC04_GPS_OFF_END_MS = 20000U;
constexpr uint32_t TC04_DURATION_MS = 25000U;
constexpr float TC04_SPEED_KMH = 50.0f;
constexpr float TC04_SPEED_MPS = TC04_SPEED_KMH / 3.6f;
constexpr float TC04_MAX_LATERAL_ACCEL_MPS2 = 3.0f;
constexpr float TC04_SLALOM_PERIOD_S = 4.0f;
constexpr float TC04_BASE_COURSE_DEG = 90.0f;

struct SlalomOutageRms {
    float position_m;
    float velocity_mps;
    float yaw_deg;
    uint32_t sample_count;
};

struct SlalomNhcInnovationMax {
    float lateral_mps;
    float vertical_mps;
    float norm_mps;
};

struct SlalomPassResult {
    float drift_exit_outage_m;
    float drift_final_m;
    uint32_t nhc_updates;
    SlalomOutageRms outage_rms;
    SlalomNhcInnovationMax nhc_innovation_max;
};

SlalomPassResult slalom_run_pass(bool nhc_enabled, bool verbose = false);
