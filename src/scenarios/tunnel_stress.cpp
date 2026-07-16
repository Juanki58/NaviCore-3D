#include "tunnel_stress.hpp"

#include "ins_ekf.hpp"
#include "mission.hpp"
#include "NavState.h"
#include "sensors_sim.hpp"
#include "telemetry_interface.hpp"
#include "vector3d.h"

#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

namespace {

constexpr uint32_t kEkfStepMs = 10U;
constexpr float kEmergencyStopDecelMps2 = 12.5f;
constexpr float kZuptSpeedThresholdMps = 0.5f;

TunnelStressPhase phase_from_ms(uint32_t t_ms)
{
    if (t_ms < TUNNEL_STRESS_PHASE1_END_MS) {
        return TunnelStressPhase::NOMINAL_GPS;
    }
    if (t_ms < TUNNEL_STRESS_TUNNEL_ENTRY_END_MS) {
        return TunnelStressPhase::TUNNEL_ENTRY;
    }
    if (t_ms < TUNNEL_STRESS_ZUPT_END_MS) {
        return TunnelStressPhase::TRAFFIC_LIGHT_STOP;
    }
    if (t_ms < TUNNEL_STRESS_GPS_OFF_END_MS) {
        return TunnelStressPhase::TUNNEL_RESUME;
    }
    return TunnelStressPhase::TUNNEL_EXIT;
}

void gps_truth_to_ned_m(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    const GpsSample *gps_truth,
    float *north_m,
    float *east_m,
    float *down_m)
{
    if (gps_truth == NULL || north_m == NULL || east_m == NULL || down_m == NULL) {
        return;
    }

    const float dlat_m = (gps_truth->position.x - ref_lat_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float lat_rad = (ref_lat_deg + gps_truth->position.x) * 0.5f
        * (static_cast<float>(M_PI) / 180.0f);
    const float dlon_m = (gps_truth->position.y - ref_lon_deg)
        * NAVICORE_METERS_PER_DEG_LAT * std::cos(lat_rad);

    *north_m = dlat_m;
    *east_m = dlon_m;
    *down_m = ref_alt_m - gps_truth->position.z;
}

float ekf_horizontal_drift_m(
    const InsEkfFilter *ekf,
    float truth_n_m,
    float truth_e_m)
{
    float est_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_position_ned(ekf, est_ned);
    const float dn = est_ned[0] - truth_n_m;
    const float de = est_ned[1] - truth_e_m;
    return std::sqrt((dn * dn) + (de * de));
}

const char *nav_mode_name(NavMode mode)
{
    switch (mode) {
    case NAV_MODE_INITIALIZING:
        return "INITIALIZING";
    case NAV_MODE_GPS:
        return "GPS";
    case NAV_MODE_DEAD_RECKONING:
        return "DEAD_RECKONING";
    case NAV_MODE_HYBRID:
        return "HYBRID";
    default:
        return "UNKNOWN";
    }
}

void prepare_imu_sample(SensorsSimulation *sensors, ImuSample *imu, uint32_t sim_clock_ms)
{
    if (sensors == NULL || imu == NULL) {
        return;
    }

    if (sensors->gps.speed_mps > kZuptSpeedThresholdMps) {
        imu->accel_mps2[0] = sensors->imu.commanded_forward_accel_mps2;
        imu->accel_mps2[1] = 0.0f;
        imu->accel_mps2[2] = 9.80665f;
    } else {
        imu->accel_mps2[0] = 0.0f;
        imu->accel_mps2[1] = 0.0f;
        imu->accel_mps2[2] = 9.80665f;
    }

    imu->gyro_radps[0] = 0.0f;
    imu->gyro_radps[1] = 0.0f;
    imu->gyro_radps[2] = 0.0f;
    imu_simulator_apply_measurement_model(&sensors->imu, imu, sim_clock_ms);
    imu->valid = true;
}

void seed_velocity_from_gps(InsEkfFilter *ekf, const GpsSample *gps)
{
    if (ekf == NULL || gps == NULL) {
        return;
    }

    const float course_rad = static_cast<float>(gps->course_deg * M_PI / 180.0);
    ekf->vel_[0] = gps->speed_mps * std::cos(course_rad);
    ekf->vel_[1] = gps->speed_mps * std::sin(course_rad);
    ekf->vel_[2] = 0.0f;
}

void apply_gps_glitch_from_ekf(
    GpsSample *gps,
    const InsEkfFilter *ekf,
    float offset_east_m)
{
    if (gps == NULL || ekf == NULL || !ekf->initialized || offset_east_m == 0.0f) {
        return;
    }

    float est_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_position_ned(ekf, est_ned);

    const float lat_deg = ekf->ref_lat_deg + (est_ned[0] / NAVICORE_METERS_PER_DEG_LAT);
    const float lat_rad = lat_deg * (static_cast<float>(M_PI) / 180.0f);
    const float meters_per_deg_lon = NAVICORE_METERS_PER_DEG_LAT * std::cos(lat_rad);
    const float lon_deg =
        ekf->ref_lon_deg + (est_ned[1] / meters_per_deg_lon) + (offset_east_m / meters_per_deg_lon);
    const float alt_m = ekf->ref_alt_m - est_ned[2];

    gps->position.x = lat_deg;
    gps->position.y = lon_deg;
    gps->position.z = alt_m;
}

void apply_vehicle_profile(SensorsSimulation *sensors, TunnelStressPhase phase)
{
    if (sensors == NULL) {
        return;
    }

    switch (phase) {
    case TunnelStressPhase::TRAFFIC_LIGHT_STOP:
        if (sensors->gps.speed_mps > 0.0f) {
            sensors->gps.speed_mps = std::fmax(
                0.0f,
                sensors->gps.speed_mps - (kEmergencyStopDecelMps2 * kEkfStepMs * 0.001f));
        }
        sensors->gps.yaw_rate_radps = 0.0f;
        sensors->imu.commanded_forward_accel_mps2 =
            (sensors->gps.speed_mps > 0.0f) ? -kEmergencyStopDecelMps2 : 0.0f;
        sensors->imu.commanded_yaw_rate_radps = 0.0f;
        break;

    case TunnelStressPhase::TUNNEL_RESUME:
        if (sensors->gps.speed_mps < TUNNEL_STRESS_CRUISE_SPEED_MPS) {
            sensors->gps.speed_mps = std::fmin(
                TUNNEL_STRESS_CRUISE_SPEED_MPS,
                sensors->gps.speed_mps + (kEmergencyStopDecelMps2 * kEkfStepMs * 0.001f));
            sensors->imu.commanded_forward_accel_mps2 = kEmergencyStopDecelMps2;
        } else {
            sensors->imu.commanded_forward_accel_mps2 = 0.0f;
        }
        sensors->imu.commanded_yaw_rate_radps = 0.0f;
        break;

    case TunnelStressPhase::NOMINAL_GPS:
    case TunnelStressPhase::TUNNEL_ENTRY:
    case TunnelStressPhase::TUNNEL_EXIT:
    default:
        sensors->imu.commanded_forward_accel_mps2 = 0.0f;
        sensors->imu.commanded_yaw_rate_radps = 0.0f;
        break;
    }
}

} /* namespace */

TunnelStressPhase tunnel_stress_phase_at_ms(uint32_t t_ms)
{
    return phase_from_ms(t_ms);
}

const char *tunnel_stress_phase_name(TunnelStressPhase phase)
{
    switch (phase) {
    case TunnelStressPhase::NOMINAL_GPS:
        return "NOMINAL_GPS";
    case TunnelStressPhase::TUNNEL_ENTRY:
        return "TUNNEL_ENTRY";
    case TunnelStressPhase::TRAFFIC_LIGHT_STOP:
        return "TRAFFIC_LIGHT_STOP";
    case TunnelStressPhase::TUNNEL_RESUME:
        return "TUNNEL_RESUME";
    case TunnelStressPhase::TUNNEL_EXIT:
        return "TUNNEL_EXIT";
    default:
        return "UNKNOWN";
    }
}

bool tunnel_stress_gps_outage_at_ms(uint32_t t_ms)
{
    return (t_ms >= TUNNEL_STRESS_GPS_OFF_START_MS) && (t_ms < TUNNEL_STRESS_GPS_OFF_END_MS);
}

void TunnelStressScenario::run(
    TelemetryInterface *telemetry,
    TunnelStressNavEmitFn emit_nav,
    uint32_t seed)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    SensorsSimulation sensors{};
    sensors_simulation_init(
        &sensors,
        SCENARIO_CLEAN,
        origin,
        TUNNEL_STRESS_CRUISE_SPEED_MPS,
        TUNNEL_STRESS_COURSE_DEG,
        seed);
    sensors.imu.commanded_forward_accel_mps2 = 0.0f;
    sensors.imu.commanded_yaw_rate_radps = 0.0f;

    InsEkfFilter ekf{};
    bool ekf_seeded = false;
    bool glitch_applied = false;
    TunnelStressPhase last_logged_phase = TunnelStressPhase::NOMINAL_GPS;
    bool logged_glitch = false;

    TunnelStressResult result{};
    result.gps_recovery_time_s = -1.0f;
    result.gps_recovered = false;

    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_TUNNEL_STRESS;
    if (telemetry != NULL) {
        telemetry->bind_sources(&bindings);
    }

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: TUNNEL_STRESS — perfil reproducible (5 fases)\n");
    std::printf("  Semilla RNG: %u\n", seed);
    std::printf("  Duracion: %.0f s | Crucero: %.0f km/h (%.1f m/s)\n",
                static_cast<float>(TUNNEL_STRESS_DURATION_MS) * 0.001f,
                TUNNEL_STRESS_CRUISE_SPEED_MPS * 3.6f,
                TUNNEL_STRESS_CRUISE_SPEED_MPS);
    std::printf("  Fase 1 [0-10 s]:   GPS nominal (estabilizacion sesgos IMU)\n");
    std::printf("  Fase 2 [10-20 s]:  GPS_LOSS + NHC (entrada tunel / dead reckoning)\n");
    std::printf("  Fase 3 [20-25 s]:  Parada semaforo v=0 + ZUPT\n");
    std::printf("  Fase 4 [25-30 s]:  Reanudacion marcha sin GPS + NHC\n");
    std::printf("  Fase 5 [>30 s]:    Salida tunel — GPS + glitch %.0f m (FDE/NIS)\n",
                TUNNEL_STRESS_GPS_GLITCH_OFFSET_M);
    std::printf("================================================================\n");

    for (uint32_t t_ms = 0U; t_ms <= TUNNEL_STRESS_DURATION_MS; t_ms += kEkfStepMs) {
        const TunnelStressPhase phase = phase_from_ms(t_ms);
        apply_vehicle_profile(&sensors, phase);

        ImuSample imu{};
        GpsSample gps_meas{};
        GpsSample gps_truth{};

        if (!gps_simulator_read(&sensors.gps, t_ms, &gps_meas)) {
            continue;
        }

        imu_simulator_step_bias_random_walk(&sensors.imu);
        prepare_imu_sample(&sensors, &imu, t_ms);
        sensors_simulation_apply_step_faults(&sensors, &imu, &gps_meas);

        (void)gps_simulator_get_truth(&sensors.gps, &gps_truth);

        GpsSample gps = gps_meas;
        const bool gps_outage = tunnel_stress_gps_outage_at_ms(t_ms);
        if (gps_outage) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        const bool gps_restored = (t_ms >= TUNNEL_STRESS_GPS_OFF_END_MS) && gps_truth.fix_valid;
        if (gps_restored && !glitch_applied && t_ms == TUNNEL_STRESS_GPS_GLITCH_MS && ekf_seeded) {
            apply_gps_glitch_from_ekf(&gps, &ekf, TUNNEL_STRESS_GPS_GLITCH_OFFSET_M);
            glitch_applied = true;
        }

        const bool zupt_active =
            (phase == TunnelStressPhase::TRAFFIC_LIGHT_STOP)
            && (sensors.gps.speed_mps <= kZuptSpeedThresholdMps);
        const bool nhc_active = gps_outage;

        if (phase != last_logged_phase) {
            std::printf(
                "TUNNEL_STRESS: Fase %s @ t=%.1f s\n",
                tunnel_stress_phase_name(phase),
                static_cast<float>(t_ms) * 0.001f);
            last_logged_phase = phase;
        }

        if (gps.fix_valid && !ekf_seeded) {
            const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, false);
            seed_velocity_from_gps(&ekf, &gps);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ref_alt_m = gps.position.z;
            ekf_seeded = true;

            if (t_ms == 0U) {
                std::printf(
                    "TUNNEL_STRESS: EKF inicializado | ref=(%.6f, %.6f) | v=%.1f m/s\n",
                    ref_lat_deg,
                    ref_lon_deg,
                    TUNNEL_STRESS_CRUISE_SPEED_MPS);
            }
        }

        bool gnss_update_this_cycle = false;
        float tick_nis = 0.0f;
        float tick_innov_ned[3] = {0.0f, 0.0f, 0.0f};
        bool zupt_this_cycle = false;

        if (ekf_seeded && imu.valid) {
            ins_ekf_set_nhc_enabled(&ekf, nhc_active);
            ins_ekf_predict(&ekf, &imu);

            if (zupt_active) {
                zupt_this_cycle = ins_ekf_update_zupt(&ekf);
            }

            if (gps.fix_valid) {
                gnss_update_this_cycle = true;
                const bool accepted = ins_ekf_update_gnss(&ekf, &gps);
                tick_nis = ins_ekf_last_nis(&ekf);
                ins_ekf_get_gnss_innovation(&ekf, tick_innov_ned);
                if (accepted) {
                    ins_ekf_clear_outlier_flag(&ekf);
                }

                if (glitch_applied && t_ms == TUNNEL_STRESS_GPS_GLITCH_MS) {
                    result.glitch_nis = tick_nis;
                    result.glitch_rejected = !accepted;
                }
            }

            float truth_n = 0.0f;
            float truth_e = 0.0f;
            float truth_d = 0.0f;
            gps_truth_to_ned_m(
                ref_lat_deg,
                ref_lon_deg,
                ref_alt_m,
                &gps_truth,
                &truth_n,
                &truth_e,
                &truth_d);
            (void)truth_d;

            if (t_ms == TUNNEL_STRESS_GPS_OFF_START_MS) {
                result.drift_at_gps_loss_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "TUNNEL_STRESS: GPS APAGADO | deriva=%.2f m | NHC=ON\n",
                    result.drift_at_gps_loss_m);
            }

            if (t_ms == TUNNEL_STRESS_ZUPT_START_MS) {
                result.drift_at_zupt_start_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                float vel_ned[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_velocity_ned(&ekf, vel_ned);
                const float speed_mps = std::sqrt(
                    (vel_ned[0] * vel_ned[0]) + (vel_ned[1] * vel_ned[1]));
                std::printf(
                    "TUNNEL_STRESS: PARADA EN SECO | deriva=%.2f m | |v|=%.2f m/s\n",
                    result.drift_at_zupt_start_m,
                    speed_mps);
            }

            if (phase == TunnelStressPhase::TRAFFIC_LIGHT_STOP
                && sensors.gps.speed_mps <= kZuptSpeedThresholdMps) {
                float vel_ned[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_velocity_ned(&ekf, vel_ned);
                const float speed_mps = std::sqrt(
                    (vel_ned[0] * vel_ned[0]) + (vel_ned[1] * vel_ned[1]) + (vel_ned[2] * vel_ned[2]));
                if (speed_mps > result.max_vel_during_zupt_mps) {
                    result.max_vel_during_zupt_mps = speed_mps;
                }
            }

            if (t_ms == TUNNEL_STRESS_ZUPT_END_MS) {
                std::printf(
                    "TUNNEL_STRESS: FIN ZUPT | updates=%u | max|v|=%.3f m/s\n",
                    ins_ekf_zupt_update_count(&ekf),
                    result.max_vel_during_zupt_mps);
            }

            if (t_ms == TUNNEL_STRESS_RESUME_START_MS) {
                result.drift_at_resume_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "TUNNEL_STRESS: REANUDACION MARCHA (sin GPS) | deriva=%.2f m\n",
                    result.drift_at_resume_m);
            }

            if (t_ms == TUNNEL_STRESS_GPS_OFF_END_MS) {
                result.drift_at_gps_return_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "TUNNEL_STRESS: SALIDA TUNEL — GPS restaurado | deriva=%.2f m\n",
                    result.drift_at_gps_return_m);
            }

            if (glitch_applied && t_ms == TUNNEL_STRESS_GPS_GLITCH_MS && !logged_glitch) {
                std::printf(
                    "TUNNEL_STRESS: GLITCH +%.0f m E | NIS=%.2f | %s\n",
                    TUNNEL_STRESS_GPS_GLITCH_OFFSET_M,
                    result.glitch_nis,
                    result.glitch_rejected ? "RECHAZADO (FDE)" : "ACEPTADO");
                if (result.glitch_rejected) {
                    std::printf(
                        "TUNNEL_STRESS: ALERTA INTEGRIDAD — GNSS_OUTLIER activo (Chi-cuadrado)\n");
                }
                logged_glitch = true;
            }
        }

        if (ekf_seeded) {
            float truth_n = 0.0f;
            float truth_e = 0.0f;
            float truth_d = 0.0f;
            gps_truth_to_ned_m(
                ref_lat_deg,
                ref_lon_deg,
                ref_alt_m,
                &gps_truth,
                &truth_n,
                &truth_e,
                &truth_d);
            (void)truth_d;

            NavState nav_state{};
            ins_ekf_export_nav_state(
                &ekf,
                &nav_state,
                t_ms,
                gps.fix_valid ? &gps : NULL);
            if (gps_outage) {
                nav_state.mode = NAV_MODE_DEAD_RECKONING;
            }

            if (telemetry != NULL) {
                bindings.ekf = &ekf;
                ekf_tick.gnss_update_this_cycle = gnss_update_this_cycle;
                ekf_tick.nis = tick_nis;
                ekf_tick.innov_ned[0] = tick_innov_ned[0];
                ekf_tick.innov_ned[1] = tick_innov_ned[1];
                ekf_tick.innov_ned[2] = tick_innov_ned[2];
                const float drift_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                bindings.drift_m = drift_m;
                bindings.drift_valid = true;

                if (!result.gps_recovered
                    && t_ms >= TUNNEL_STRESS_GPS_OFF_END_MS
                    && drift_m < TUNNEL_STRESS_GPS_RECOVERY_DRIFT_M) {
                    result.gps_recovered = true;
                    result.gps_recovery_time_s =
                        (static_cast<float>(t_ms - TUNNEL_STRESS_GPS_OFF_END_MS)) * 0.001f;
                }

                telemetry->broadcast(nav_state, MISSION_STATE_INIT);
            }

            if (emit_nav != NULL) {
                emit_nav(&ekf, t_ms, gps.fix_valid ? &gps : NULL, gps_outage);
            }

            if (telemetry != NULL && (t_ms % 2000U) == 0U) {
                float truth_n = 0.0f;
                float truth_e = 0.0f;
                float truth_d = 0.0f;
                gps_truth_to_ned_m(
                    ref_lat_deg,
                    ref_lon_deg,
                    ref_alt_m,
                    &gps_truth,
                    &truth_n,
                    &truth_e,
                    &truth_d);
                (void)truth_d;
                const float drift_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "[t=%5.1fs] fase=%s mode=%s nhc=%s zupt=%s gnss=%s nis=%.2f drift=%.2f m\n",
                    static_cast<float>(t_ms) * 0.001f,
                    tunnel_stress_phase_name(phase),
                    nav_mode_name(nav_state.mode),
                    nhc_active ? "ON" : "OFF",
                    zupt_this_cycle ? "TICK" : (zupt_active ? "ARM" : "OFF"),
                    gnss_update_this_cycle
                        ? (ins_ekf_outlier_detected(&ekf) ? "REJ" : "OK")
                        : "SKIP",
                    gnss_update_this_cycle ? tick_nis : 0.0f,
                    drift_m);
            }
        }
    }

    if (ekf_seeded) {
        result.nhc_updates = ins_ekf_nhc_update_count(&ekf);
        result.zupt_updates = ins_ekf_zupt_update_count(&ekf);
        result.gnss_accepts = ins_ekf_gnss_accept_count(&ekf);
        result.gnss_rejects = ins_ekf_gnss_reject_count(&ekf);
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(" RESULTADO TUNNEL_STRESS\n");
    std::printf("  Deriva al apagar GPS (10 s):       %8.2f m\n", result.drift_at_gps_loss_m);
    std::printf("  Deriva al parada ZUPT (20 s):     %8.2f m\n", result.drift_at_zupt_start_m);
    std::printf("  Deriva al reanudar (25 s):        %8.2f m\n", result.drift_at_resume_m);
    std::printf("  Max |v| durante ZUPT (20-25 s):   %7.3f m/s\n", result.max_vel_during_zupt_mps);
    std::printf("  Deriva al salir tunel (30 s):     %8.2f m\n", result.drift_at_gps_return_m);
    if (result.gps_recovered) {
        std::printf(
            "  Recovery Time (GPS < %.1f m):      %8.2f s\n",
            TUNNEL_STRESS_GPS_RECOVERY_DRIFT_M,
            result.gps_recovery_time_s);
    } else {
        std::printf(
            "  Recovery Time (GPS < %.1f m):          TIMEOUT\n",
            TUNNEL_STRESS_GPS_RECOVERY_DRIFT_M);
    }
    std::printf("  Glitch 50 m — NIS:                 %8.2f | %s\n",
                result.glitch_nis,
                result.glitch_rejected ? "RECHAZADO" : "ACEPTADO");
    std::printf("  Updates NHC / ZUPT:              %u / %u\n",
                result.nhc_updates,
                result.zupt_updates);
    std::printf("  GNSS aceptados / rechazados:     %u / %u\n",
                result.gnss_accepts,
                result.gnss_rejects);
    std::printf("----------------------------------------------------------------\n");
}

void run_tunnel_stress_scenario(
    TelemetryInterface *telemetry,
    TunnelStressNavEmitFn emit_nav,
    uint32_t seed)
{
    TunnelStressScenario scenario{};
    scenario.run(telemetry, emit_nav, seed);
}
