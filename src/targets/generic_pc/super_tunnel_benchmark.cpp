#include "super_tunnel_benchmark.hpp"

#include "ins_ekf.hpp"
#include "sensors_sim.hpp"
#include "telemetry_interface.hpp"

#include <cmath>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr uint32_t kEkfStepMs = 10U;
constexpr uint32_t kSuperTunnelDurationMs = 60000U;
constexpr uint32_t kSuperTunnelGpsOffStartMs = 10000U;
constexpr uint32_t kSuperTunnelGpsOffEndMs = 55000U;
constexpr float kSuperTunnelSpeedMps = 25.0f;
constexpr float kSuperTunnelCourseDeg = 90.0f;

void gps_position_to_ned_m(
    float ref_lat_deg,
    float ref_lon_deg,
    const Vector3D *position,
    float *north_m,
    float *east_m)
{
    if (position == NULL || north_m == NULL || east_m == NULL) {
        return;
    }

    const float dlat_m = (position->x - ref_lat_deg) * 111132.954f;
    const float lat_rad = (ref_lat_deg + position->x) * 0.5f * (static_cast<float>(M_PI) / 180.0f);
    const float dlon_m = (position->y - ref_lon_deg) * 111132.954f * std::cos(lat_rad);
    *north_m = dlat_m;
    *east_m = dlon_m;
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

void super_tunnel_sanitize_imu(ImuSample *imu)
{
    if (imu == NULL) {
        return;
    }

    imu->accel_mps2[0] = 0.0f;
    imu->accel_mps2[1] = 0.0f;
    imu->accel_mps2[2] = 9.80665f;
    imu->gyro_radps[0] = 0.0f;
    imu->gyro_radps[1] = 0.0f;
    imu->gyro_radps[2] = 0.0f;
    imu->valid = true;
}

void super_tunnel_seed_velocity_from_gps(InsEkfFilter *ekf, const GpsSample *gps)
{
    if (ekf == NULL || gps == NULL) {
        return;
    }

    const float course_rad = static_cast<float>(gps->course_deg * M_PI / 180.0);
    ekf->vel_[0] = gps->speed_mps * std::cos(course_rad);
    ekf->vel_[1] = gps->speed_mps * std::sin(course_rad);
    ekf->vel_[2] = 0.0f;
}

} /* namespace */

SuperTunnelPassResult super_tunnel_run_pass(bool nhc_enabled, bool verbose)
{
    SuperTunnelPassResult result{};

    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    SensorsSimulation sensors{};
    sensors_simulation_init(
        &sensors,
        SCENARIO_CLEAN,
        origin,
        kSuperTunnelSpeedMps,
        kSuperTunnelCourseDeg,
        71U);
    sensors.imu.commanded_forward_accel_mps2 = 0.0f;
    sensors.imu.commanded_yaw_rate_radps = 0.0f;

    InsEkfFilter ekf{};
    bool ekf_seeded = false;

    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;

    for (uint32_t t_ms = 0U; t_ms <= kSuperTunnelDurationMs; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps_truth{};
        GpsSample gps{};

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps_truth)) {
            continue;
        }

        super_tunnel_sanitize_imu(&imu);

        gps = gps_truth;
        const bool gps_outage =
            (t_ms >= kSuperTunnelGpsOffStartMs) && (t_ms < kSuperTunnelGpsOffEndMs);
        if (gps_outage) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        if (gps.fix_valid && !ekf_seeded) {
            const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            super_tunnel_seed_velocity_from_gps(&ekf, &gps);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ekf_seeded = true;

            if (verbose && t_ms == 0U) {
                std::printf(
                    "SUPER_TUNNEL [%s]: EKF inicializado | ref=(%.6f, %.6f) | v=%.1f m/s (90 km/h)\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    ref_lat_deg,
                    ref_lon_deg,
                    kSuperTunnelSpeedMps);
            }
        }

        if (ekf_seeded && imu.valid) {
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            ins_ekf_predict(&ekf, &imu);

            if (verbose && t_ms == kSuperTunnelGpsOffStartMs) {
                float truth_n = 0.0f;
                float truth_e = 0.0f;
                gps_position_to_ned_m(
                    ref_lat_deg,
                    ref_lon_deg,
                    &gps_truth.position,
                    &truth_n,
                    &truth_e);
                const float drift_entry_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "SUPER_TUNNEL [%s]: GPS APAGADO @ t=%.1f s (inicio tunel) | deriva=%.2f m\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    static_cast<float>(t_ms) * 0.001f,
                    drift_entry_m);
            }

            if (t_ms == kSuperTunnelGpsOffEndMs) {
                float truth_n = 0.0f;
                float truth_e = 0.0f;
                gps_position_to_ned_m(
                    ref_lat_deg,
                    ref_lon_deg,
                    &gps_truth.position,
                    &truth_n,
                    &truth_e);
                result.drift_exit_tunnel_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                if (verbose) {
                    std::printf(
                        "SUPER_TUNNEL [%s]: salida tunel @ t=%.1f s | deriva=%.2f m\n",
                        nhc_enabled ? "NHC ON" : "NHC OFF",
                        static_cast<float>(t_ms) * 0.001f,
                        result.drift_exit_tunnel_m);
                }
            }

            if (gps.fix_valid) {
                (void)ins_ekf_update_gnss(&ekf, &gps);
            }
        }
    }

    if (ekf_seeded) {
        ImuSample imu{};
        GpsSample final_truth{};
        (void)sensors_simulation_tick(&sensors, kSuperTunnelDurationMs, &imu, &final_truth);

        float truth_n = 0.0f;
        float truth_e = 0.0f;
        gps_position_to_ned_m(
            ref_lat_deg,
            ref_lon_deg,
            &final_truth.position,
            &truth_n,
            &truth_e);
        result.drift_final_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
        result.nhc_updates = ins_ekf_nhc_update_count(&ekf);
    }

    return result;
}

void run_super_tunnel_scenario(TelemetryInterface *telemetry)
{
    (void)telemetry;

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: SUPER_TUNNEL — recta 90 km/h, apagon GPS 10-55 s\n");
    std::printf("  Duracion: %.0f s | Velocidad: %.0f km/h (%.1f m/s)\n",
                static_cast<float>(kSuperTunnelDurationMs) * 0.001f,
                kSuperTunnelSpeedMps * 3.6f,
                kSuperTunnelSpeedMps);
    std::printf("  Tunel:    t=%.1f s -> t=%.1f s (%.0f s sin GPS)\n",
                static_cast<float>(kSuperTunnelGpsOffStartMs) * 0.001f,
                static_cast<float>(kSuperTunnelGpsOffEndMs) * 0.001f,
                static_cast<float>(kSuperTunnelGpsOffEndMs - kSuperTunnelGpsOffStartMs) * 0.001f);
    std::printf("  NHC:      v_lateral=0 (r=0.1 m/s) | v_vertical=0 (r=0.05 m/s)\n");
    std::printf("================================================================\n");

    const SuperTunnelPassResult without_nhc = super_tunnel_run_pass(false, true);
    const SuperTunnelPassResult with_nhc = super_tunnel_run_pass(true, true);

    const float improvement_exit_m = without_nhc.drift_exit_tunnel_m - with_nhc.drift_exit_tunnel_m;
    const float improvement_final_m = without_nhc.drift_final_m - with_nhc.drift_final_m;

    std::printf("----------------------------------------------------------------\n");
    std::printf(" RESULTADO SUPER_TUNNEL — comparativa deriva horizontal (m)\n");
    std::printf("                    | sin NHC   | con NHC   | mejora\n");
    std::printf("  Salida tunel (55s)| %8.2f | %8.2f | %+.2f\n",
                without_nhc.drift_exit_tunnel_m,
                with_nhc.drift_exit_tunnel_m,
                improvement_exit_m);
    std::printf("  Final trayecto(60s)| %8.2f | %8.2f | %+.2f\n",
                without_nhc.drift_final_m,
                with_nhc.drift_final_m,
                improvement_final_m);
    std::printf("  Updates NHC       |        — | %8u | —\n", with_nhc.nhc_updates);
    std::printf("----------------------------------------------------------------\n");
}
