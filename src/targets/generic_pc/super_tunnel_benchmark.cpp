#include "super_tunnel_benchmark.hpp"

#include "ins_ekf.hpp"
#include "sensors_sim.hpp"

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
constexpr uint32_t kSuperTunnelDurationMs = 60000U;
constexpr float kSuperTunnelSpeedMps = 25.0f;
constexpr float kSuperTunnelCourseDeg = 90.0f;
constexpr float kRadToDegF = 180.0f / static_cast<float>(M_PI);

struct OutageRmsAccumulator {
    double sum_position_sq_m2;
    double sum_velocity_sq_mps2;
    double sum_yaw_sq_rad2;
    uint32_t sample_count;
};

void outage_rms_init(OutageRmsAccumulator *acc)
{
    if (acc == NULL) {
        return;
    }

    acc->sum_position_sq_m2 = 0.0;
    acc->sum_velocity_sq_mps2 = 0.0;
    acc->sum_yaw_sq_rad2 = 0.0;
    acc->sample_count = 0U;
}

float wrap_angle_rad(float angle_rad)
{
    while (angle_rad > static_cast<float>(M_PI)) {
        angle_rad -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle_rad < -static_cast<float>(M_PI)) {
        angle_rad += 2.0f * static_cast<float>(M_PI);
    }
    return angle_rad;
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

void gps_truth_velocity_ned_mps(const GpsSample *gps_truth, float vel_ned[3])
{
    if (gps_truth == NULL || vel_ned == NULL) {
        return;
    }

    const float course_rad = static_cast<float>(gps_truth->course_deg * M_PI / 180.0);
    vel_ned[0] = gps_truth->speed_mps * std::cos(course_rad);
    vel_ned[1] = gps_truth->speed_mps * std::sin(course_rad);
    vel_ned[2] = 0.0f;
}

void outage_rms_accumulate_sample(
    OutageRmsAccumulator *acc,
    const InsEkfFilter *ekf,
    float truth_n_m,
    float truth_e_m,
    float truth_d_m,
    const float truth_vel_ned[3],
    float truth_yaw_rad)
{
    if (acc == NULL || ekf == NULL || truth_vel_ned == NULL) {
        return;
    }

    float est_pos[3] = {0.0f, 0.0f, 0.0f};
    float est_vel[3] = {0.0f, 0.0f, 0.0f};
    float est_roll = 0.0f;
    float est_pitch = 0.0f;
    float est_yaw = 0.0f;

    ins_ekf_get_position_ned(ekf, est_pos);
    ins_ekf_get_velocity_ned(ekf, est_vel);
    ins_ekf_get_attitude_rad(ekf, &est_roll, &est_pitch, &est_yaw);

    const float dn = est_pos[0] - truth_n_m;
    const float de = est_pos[1] - truth_e_m;
    const float dd = est_pos[2] - truth_d_m;
    const float dvn = est_vel[0] - truth_vel_ned[0];
    const float dve = est_vel[1] - truth_vel_ned[1];
    const float dvd = est_vel[2] - truth_vel_ned[2];
    const float dyaw = wrap_angle_rad(est_yaw - truth_yaw_rad);

    acc->sum_position_sq_m2 += static_cast<double>((dn * dn) + (de * de) + (dd * dd));
    acc->sum_velocity_sq_mps2 += static_cast<double>((dvn * dvn) + (dve * dve) + (dvd * dvd));
    acc->sum_yaw_sq_rad2 += static_cast<double>(dyaw * dyaw);
    ++acc->sample_count;
}

void outage_rms_finalize(const OutageRmsAccumulator *acc, SuperTunnelOutageRms *out)
{
    if (acc == NULL || out == NULL || acc->sample_count == 0U) {
        return;
    }

    const double inv_count = 1.0 / static_cast<double>(acc->sample_count);
    out->position_m = static_cast<float>(std::sqrt(acc->sum_position_sq_m2 * inv_count));
    out->velocity_mps = static_cast<float>(std::sqrt(acc->sum_velocity_sq_mps2 * inv_count));
    out->yaw_deg = static_cast<float>(
        std::sqrt(acc->sum_yaw_sq_rad2 * inv_count) * static_cast<double>(kRadToDegF));
    out->sample_count = acc->sample_count;
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

SuperTunnelPassResult super_tunnel_run_pass(
    bool nhc_enabled,
    bool verbose,
    SuperTunnelImuMode imu_mode,
    uint32_t rng_seed,
    float nhc_lateral_std_mps,
    float nhc_vertical_std_mps)
{
    SuperTunnelPassResult result{};

    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const uint32_t seed = (rng_seed != 0U)
        ? rng_seed
        : sensors_simulation_get_default_seed();

    SensorsSimulation sensors{};
    sensors_simulation_init(
        &sensors,
        SCENARIO_CLEAN,
        origin,
        kSuperTunnelSpeedMps,
        kSuperTunnelCourseDeg,
        seed);
    sensors.imu.commanded_forward_accel_mps2 = 0.0f;
    sensors.imu.commanded_yaw_rate_radps = 0.0f;
    imu_simulator_set_scale_misalign_enabled(
        &sensors.imu,
        imu_mode == SUPER_TUNNEL_IMU_DIRTY_FULL);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;
    OutageRmsAccumulator outage_rms{};

    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;
    outage_rms_init(&outage_rms);

    for (uint32_t t_ms = 0U; t_ms <= kSuperTunnelDurationMs; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps_meas{};
        GpsSample gps_truth{};

        if (!gps_simulator_read(&sensors.gps, t_ms, &gps_meas)) {
            continue;
        }

        if (imu_mode != SUPER_TUNNEL_IMU_IDEAL) {
            imu_simulator_step_bias_random_walk(&sensors.imu);
        }
        super_tunnel_sanitize_imu(&imu);
        if (imu_mode == SUPER_TUNNEL_IMU_IDEAL) {
            imu.timestamp_ms = t_ms;
        } else {
            imu_simulator_apply_measurement_model(&sensors.imu, &imu, t_ms);
        }
        sensors_simulation_apply_step_faults(&sensors, &imu, &gps_meas);

        (void)gps_simulator_get_truth(&sensors.gps, &gps_truth);

        GpsSample gps = gps_meas;
        const bool gps_outage =
            (t_ms >= SUPER_TUNNEL_GPS_OFF_START_MS) && (t_ms < SUPER_TUNNEL_GPS_OFF_END_MS);
        if (gps_outage) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        if (gps.fix_valid && !ekf_seeded) {
            const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            if (nhc_lateral_std_mps > 0.0f) {
                ekf.nhc_lateral_var_m2 = nhc_lateral_std_mps * nhc_lateral_std_mps;
            }
            if (nhc_vertical_std_mps > 0.0f) {
                ekf.nhc_vertical_var_m2 = nhc_vertical_std_mps * nhc_vertical_std_mps;
            }
            super_tunnel_seed_velocity_from_gps(&ekf, &gps);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ref_alt_m = gps.position.z;
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

            if (gps_outage) {
                float truth_n = 0.0f;
                float truth_e = 0.0f;
                float truth_d = 0.0f;
                float truth_vel[3] = {0.0f, 0.0f, 0.0f};
                gps_truth_to_ned_m(
                    ref_lat_deg,
                    ref_lon_deg,
                    ref_alt_m,
                    &gps_truth,
                    &truth_n,
                    &truth_e,
                    &truth_d);
                gps_truth_velocity_ned_mps(&gps_truth, truth_vel);
                const float truth_yaw_rad = static_cast<float>(
                    gps_truth.course_deg * M_PI / 180.0);
                outage_rms_accumulate_sample(
                    &outage_rms,
                    &ekf,
                    truth_n,
                    truth_e,
                    truth_d,
                    truth_vel,
                    truth_yaw_rad);
            }

            if (verbose && t_ms == SUPER_TUNNEL_GPS_OFF_START_MS) {
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
                const float drift_entry_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
                std::printf(
                    "SUPER_TUNNEL [%s]: GPS APAGADO @ t=%.1f s (inicio tunel) | deriva=%.2f m\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    static_cast<float>(t_ms) * 0.001f,
                    drift_entry_m);
            }

            if (t_ms == SUPER_TUNNEL_GPS_OFF_END_MS) {
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
        GpsSample final_meas{};
        GpsSample final_truth{};
        (void)sensors_simulation_tick(&sensors, kSuperTunnelDurationMs, &imu, &final_meas);
        (void)gps_simulator_get_truth(&sensors.gps, &final_truth);

        float truth_n = 0.0f;
        float truth_e = 0.0f;
        float truth_d = 0.0f;
        gps_truth_to_ned_m(
            ref_lat_deg,
            ref_lon_deg,
            ref_alt_m,
            &final_truth,
            &truth_n,
            &truth_e,
            &truth_d);
        (void)truth_d;
        result.drift_final_m = ekf_horizontal_drift_m(&ekf, truth_n, truth_e);
        result.nhc_updates = ins_ekf_nhc_update_count(&ekf);
        outage_rms_finalize(&outage_rms, &result.outage_rms);
        ins_ekf_get_nhc_innovation_max(
            &ekf,
            &result.nhc_innovation_max.lateral_mps,
            &result.nhc_innovation_max.vertical_mps,
            &result.nhc_innovation_max.norm_mps);
    }

    return result;
}

void run_super_tunnel_nhc_benchmark()
{
    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" BENCHMARK: SUPER_TUNNEL — recta 90 km/h, apagon GPS 10-55 s\n");
    std::printf("  Duracion: %.0f s | Velocidad: %.0f km/h (%.1f m/s)\n",
                static_cast<float>(kSuperTunnelDurationMs) * 0.001f,
                kSuperTunnelSpeedMps * 3.6f,
                kSuperTunnelSpeedMps);
    std::printf("  Tunel:    t=%.1f s -> t=%.1f s (%.0f s sin GPS)\n",
                static_cast<float>(SUPER_TUNNEL_GPS_OFF_START_MS) * 0.001f,
                static_cast<float>(SUPER_TUNNEL_GPS_OFF_END_MS) * 0.001f,
                static_cast<float>(SUPER_TUNNEL_GPS_OFF_END_MS - SUPER_TUNNEL_GPS_OFF_START_MS)
                    * 0.001f);
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
    std::printf("  RMS pos (apagon)  | %8.2f | %8.2f | %+.2f\n",
                without_nhc.outage_rms.position_m,
                with_nhc.outage_rms.position_m,
                without_nhc.outage_rms.position_m - with_nhc.outage_rms.position_m);
    std::printf("  RMS vel (apagon)  | %8.2f | %8.2f | %+.2f\n",
                without_nhc.outage_rms.velocity_mps,
                with_nhc.outage_rms.velocity_mps,
                without_nhc.outage_rms.velocity_mps - with_nhc.outage_rms.velocity_mps);
    std::printf("  RMS yaw (apagon)  | %8.2f | %8.2f | %+.2f\n",
                without_nhc.outage_rms.yaw_deg,
                with_nhc.outage_rms.yaw_deg,
                without_nhc.outage_rms.yaw_deg - with_nhc.outage_rms.yaw_deg);
    std::printf("  Updates NHC       |        — | %8u | —\n", with_nhc.nhc_updates);
    std::printf("  Innov NHC max (on)|        — | lat=%.3f vert=%.3f norm=%.3f m/s\n",
                with_nhc.nhc_innovation_max.lateral_mps,
                with_nhc.nhc_innovation_max.vertical_mps,
                with_nhc.nhc_innovation_max.norm_mps);
    std::printf("----------------------------------------------------------------\n");
}
