#pragma once

#include <cstdint>

#include "sensor_types.hpp"

class INaviFilter;
struct TelemetryInterface;

/** Fases cronológicas del escenario reproducible TUNNEL_STRESS. */
enum class TunnelStressPhase : uint8_t {
    NOMINAL_GPS = 0,       /* t ∈ [0, 10) s  — GPS nominal, calibración sesgos */
    TUNNEL_ENTRY = 1,      /* t ∈ [10, 20) s — GPS_LOSS + NHC (dead reckoning) */
    TRAFFIC_LIGHT_STOP = 2,/* t ∈ [20, 25) s — v=0 + ZUPT */
    TUNNEL_RESUME = 3,     /* t ∈ [25, 30) s — marcha sin GPS + NHC */
    TUNNEL_EXIT = 4,       /* t ≥ 30 s       — GPS + glitch FDE */
};

/* --- Tiempos (ms) --- */
constexpr uint32_t TUNNEL_STRESS_T0_MS = 0U;
constexpr uint32_t TUNNEL_STRESS_PHASE1_END_MS = 10000U;
constexpr uint32_t TUNNEL_STRESS_GPS_OFF_START_MS = 10000U;
constexpr uint32_t TUNNEL_STRESS_TUNNEL_ENTRY_END_MS = 20000U;
constexpr uint32_t TUNNEL_STRESS_ZUPT_START_MS = 20000U;
constexpr uint32_t TUNNEL_STRESS_ZUPT_END_MS = 25000U;
constexpr uint32_t TUNNEL_STRESS_RESUME_START_MS = 25000U;
constexpr uint32_t TUNNEL_STRESS_GPS_OFF_END_MS = 30000U;
constexpr uint32_t TUNNEL_STRESS_GPS_GLITCH_MS = 30000U;
constexpr uint32_t TUNNEL_STRESS_DURATION_MS = 45000U;

constexpr float TUNNEL_STRESS_CRUISE_SPEED_MPS = 25.0f; /* 90 km/h */
constexpr float TUNNEL_STRESS_COURSE_DEG = 90.0f;
constexpr float TUNNEL_STRESS_GPS_GLITCH_OFFSET_M = 50.0f;

/** Segundos (float) del tramo sin GPS en túnel — para visualizadores Python. */
constexpr float TUNNEL_STRESS_TUNNEL_GPS_LOSS_T0_S = 10.0f;
constexpr float TUNNEL_STRESS_TUNNEL_GPS_LOSS_T1_S = 30.0f;

/** Umbral de deriva horizontal para considerar convergencia GPS tras salida del túnel. */
constexpr float TUNNEL_STRESS_GPS_RECOVERY_DRIFT_M = 1.0f;

struct TunnelStressResult {
    float drift_at_gps_loss_m;
    float drift_at_zupt_start_m;
    float drift_at_resume_m;
    float max_vel_during_zupt_mps;
    float drift_at_gps_return_m;
    float gps_recovery_time_s;
    float glitch_nis;
    bool glitch_rejected;
    bool gps_recovered;
    uint32_t nhc_updates;
    uint32_t zupt_updates;
    uint32_t gnss_accepts;
    uint32_t gnss_rejects;
};

typedef void (*TunnelStressNavEmitFn)(
    const INaviFilter *filter,
    uint32_t timestamp_ms,
    const GpsSample *gps,
    bool dead_reckoning);

TunnelStressPhase tunnel_stress_phase_at_ms(uint32_t t_ms);
const char *tunnel_stress_phase_name(TunnelStressPhase phase);
bool tunnel_stress_gps_outage_at_ms(uint32_t t_ms);

class TunnelStressScenario {
public:
    void run(
        TelemetryInterface *telemetry,
        TunnelStressNavEmitFn emit_nav = nullptr,
        uint32_t seed = 71U);
};

/** Punto de entrada del simulador (--scenario TUNNEL_STRESS). */
void run_tunnel_stress_scenario(
    TelemetryInterface *telemetry,
    TunnelStressNavEmitFn emit_nav = nullptr,
    uint32_t seed = 71U);
