#include "slalom_scenario.hpp"

#include "ins_ekf.hpp"
#include "mission.hpp"
#include "NavState.h"
#include "slalom_benchmark.hpp"
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

#ifndef NAVICORE_INS_EKF_GRAVITY_MPS2
#define NAVICORE_INS_EKF_GRAVITY_MPS2 9.80665f
#endif

namespace {

constexpr uint32_t kEkfStepMs = 10U;
constexpr float kGravityMps2 = NAVICORE_INS_EKF_GRAVITY_MPS2;
constexpr float kRadToDegF = 180.0f / static_cast<float>(M_PI);
constexpr float kTwoPiF = 2.0f * static_cast<float>(M_PI);
constexpr float kBaseCourseRad = TC04_BASE_COURSE_DEG * (static_cast<float>(M_PI) / 180.0f);
constexpr float kSlalomOmegaRadps = kTwoPiF / TC04_SLALOM_PERIOD_S;
constexpr float kYawAmplitudeRad =
    TC04_MAX_LATERAL_ACCEL_MPS2 / (TC04_SPEED_MPS * kSlalomOmegaRadps);

struct TruthState {
    float pos_ned[3];
    float vel_ned[3];
    float accel_ned[3];
    float yaw_rad;
    float yaw_rate_radps;
};

void quat_normalize(float q[4])
{
    const float norm = std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
    if (norm <= 1.0e-8f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    const float inv_norm = 1.0f / norm;
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

void yaw_to_quat(float yaw_rad, float q[4])
{
    const float half_yaw = 0.5f * yaw_rad;
    q[0] = std::cos(half_yaw);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = std::sin(half_yaw);
    quat_normalize(q);
}

void quat_to_dcm_bn(const float q[4], float dcm[3][3])
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    dcm[0][0] = (qw * qw) + (qx * qx) - (qy * qy) - (qz * qz);
    dcm[0][1] = 2.0f * ((qx * qy) - (qw * qz));
    dcm[0][2] = 2.0f * ((qx * qz) + (qw * qy));
    dcm[1][0] = 2.0f * ((qx * qy) + (qw * qz));
    dcm[1][1] = (qw * qw) - (qx * qx) + (qy * qy) - (qz * qz);
    dcm[1][2] = 2.0f * ((qy * qz) - (qw * qx));
    dcm[2][0] = 2.0f * ((qx * qz) - (qw * qy));
    dcm[2][1] = 2.0f * ((qy * qz) + (qw * qx));
    dcm[2][2] = (qw * qw) - (qx * qx) - (qy * qy) + (qz * qz);
}

void ned_to_body(const float dcm[3][3], const float ned[3], float body[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        body[i] = (dcm[0][i] * ned[0]) + (dcm[1][i] * ned[1]) + (dcm[2][i] * ned[2]);
    }
}

void slalom_kinematics_at_time(float t_s, TruthState *truth)
{
    if (truth == NULL) {
        return;
    }

    const float phase = kSlalomOmegaRadps * t_s;
    const float heading_offset = kYawAmplitudeRad * std::sin(phase);
    truth->yaw_rad = kBaseCourseRad + heading_offset;
    truth->yaw_rate_radps = kYawAmplitudeRad * kSlalomOmegaRadps * std::cos(phase);

    const float sin_h = std::sin(truth->yaw_rad);
    const float cos_h = std::cos(truth->yaw_rad);

    truth->vel_ned[0] = TC04_SPEED_MPS * cos_h;
    truth->vel_ned[1] = TC04_SPEED_MPS * sin_h;
    truth->vel_ned[2] = 0.0f;

    truth->accel_ned[0] = -TC04_SPEED_MPS * sin_h * truth->yaw_rate_radps;
    truth->accel_ned[1] = TC04_SPEED_MPS * cos_h * truth->yaw_rate_radps;
    truth->accel_ned[2] = 0.0f;
}

void truth_propagate(TruthState *truth, float dt_s)
{
    if (truth == NULL || dt_s <= 0.0f) {
        return;
    }

    truth->pos_ned[0] += truth->vel_ned[0] * dt_s;
    truth->pos_ned[1] += truth->vel_ned[1] * dt_s;
    truth->pos_ned[2] += truth->vel_ned[2] * dt_s;
}

void truth_to_gps_sample(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    const TruthState *truth,
    GpsSample *gps)
{
    if (truth == NULL || gps == NULL) {
        return;
    }

    const float lat_rad = ref_lat_deg * (static_cast<float>(M_PI) / 180.0f);
    const float cos_lat = std::cos(lat_rad);

    gps->position.x = ref_lat_deg + (truth->pos_ned[0] / NAVICORE_METERS_PER_DEG_LAT);
    gps->position.y = ref_lon_deg + (truth->pos_ned[1] / (NAVICORE_METERS_PER_DEG_LAT * cos_lat));
    gps->position.z = ref_alt_m - truth->pos_ned[2];
    gps->speed_mps = TC04_SPEED_MPS;
    gps->course_deg = truth->yaw_rad * kRadToDegF;
    gps->satellites = 12U;
    gps->fix_valid = true;
}

void make_ideal_slalom_imu(const TruthState *truth, uint32_t timestamp_ms, ImuSample *imu)
{
    if (truth == NULL || imu == NULL) {
        return;
    }

    float q[4]{};
    yaw_to_quat(truth->yaw_rad, q);

    float dcm[3][3]{};
    quat_to_dcm_bn(q, dcm);

    const float specific_ned[3] = {
        truth->accel_ned[0],
        truth->accel_ned[1],
        truth->accel_ned[2] + kGravityMps2,
    };

    float a_body[3]{};
    ned_to_body(dcm, specific_ned, a_body);

    imu->accel_mps2[0] = a_body[0];
    imu->accel_mps2[1] = a_body[1];
    imu->accel_mps2[2] = a_body[2];
    imu->gyro_radps[0] = 0.0f;
    imu->gyro_radps[1] = 0.0f;
    imu->gyro_radps[2] = truth->yaw_rate_radps;
    imu->timestamp_ms = timestamp_ms;
    imu->valid = true;
}

void seed_ekf_from_truth(InsEkfFilter *ekf, const TruthState *truth)
{
    if (ekf == NULL || truth == NULL) {
        return;
    }

    yaw_to_quat(truth->yaw_rad, ekf->q_att_);
    ekf->vel_[0] = truth->vel_ned[0];
    ekf->vel_[1] = truth->vel_ned[1];
    ekf->vel_[2] = truth->vel_ned[2];
}

float lateral_drift_m(const InsEkfFilter *ekf, const TruthState *truth)
{
    float est_pos[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_position_ned(ekf, est_pos);
    const float dn = est_pos[0] - truth->pos_ned[0];
    const float de = est_pos[1] - truth->pos_ned[1];
    const float sin_h = std::sin(truth->yaw_rad);
    const float cos_h = std::cos(truth->yaw_rad);
    return (-sin_h * dn) + (cos_h * de);
}

} /* namespace */

void run_slalom_scenario(
    TelemetryInterface *telemetry,
    SlalomNavEmitFn emit_nav)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;
    float max_lateral_drift_m = 0.0f;

    TruthState truth{};
    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;
    float prev_t_s = 0.0f;

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_SLALOM;
    if (telemetry != NULL) {
        telemetry->bind_sources(&bindings);
    }

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: SLALOM — seguimiento en curvas cerradas (NHC ON)\n");
    std::printf("  Duracion: %.0f s | Velocidad: %.0f km/h | a_lat max: %.1f m/s2\n",
                static_cast<float>(TC04_DURATION_MS) * 0.001f,
                TC04_SPEED_KMH,
                TC04_MAX_LATERAL_ACCEL_MPS2);
    std::printf("================================================================\n");

    for (uint32_t t_ms = 0U; t_ms <= TC04_DURATION_MS; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps{};

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        const float dt_s = (t_ms > 0U) ? (t_s - prev_t_s) : 0.0f;
        slalom_kinematics_at_time(t_s, &truth);
        if (dt_s > 0.0f) {
            truth_propagate(&truth, dt_s);
        }
        prev_t_s = t_s;

        truth_to_gps_sample(ref_lat_deg, ref_lon_deg, ref_alt_m, &truth, &gps);
        gps.timestamp_ms = t_ms;
        make_ideal_slalom_imu(&truth, t_ms, &imu);

        if (gps.fix_valid && !ekf_seeded) {
            ins_ekf_init(&ekf, gps.position, truth.yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, true);
            seed_ekf_from_truth(&ekf, &truth);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ref_alt_m = gps.position.z;
            ekf_seeded = true;
        }

        bool gnss_update_this_cycle = false;
        float tick_nis = 0.0f;
        float tick_innov_ned[3] = {0.0f, 0.0f, 0.0f};

        if (ekf_seeded && imu.valid) {
            ins_ekf_set_nhc_enabled(&ekf, true);
            ins_ekf_predict(&ekf, &imu);

            if (gps.fix_valid) {
                gnss_update_this_cycle = true;
                (void)ins_ekf_update_gnss(&ekf, &gps);
                tick_nis = ins_ekf_last_nis(&ekf);
                ins_ekf_get_gnss_innovation(&ekf, tick_innov_ned);
                if (!ins_ekf_outlier_detected(&ekf)) {
                    ins_ekf_clear_outlier_flag(&ekf);
                }
            }

            const float lateral = std::fabs(lateral_drift_m(&ekf, &truth));
            if (lateral > max_lateral_drift_m) {
                max_lateral_drift_m = lateral;
            }

            bindings.drift_m = lateral;
            bindings.drift_valid = true;

            NavState nav_state{};
            ins_ekf_export_nav_state(&ekf, &nav_state, t_ms, &gps);

            if (telemetry != NULL) {
                bindings.ekf = &ekf;
                ekf_tick.gnss_update_this_cycle = gnss_update_this_cycle;
                ekf_tick.nis = tick_nis;
                ekf_tick.innov_ned[0] = tick_innov_ned[0];
                ekf_tick.innov_ned[1] = tick_innov_ned[1];
                ekf_tick.innov_ned[2] = tick_innov_ned[2];
                telemetry->broadcast(nav_state, MISSION_STATE_INIT);
            }

            if (emit_nav != NULL) {
                emit_nav(&ekf, t_ms, &gps, false);
            }

            if ((t_ms % 2000U) == 0U) {
                std::printf(
                    "[t=%5.1fs] lateral_drift=%.3f m | max=%.3f m\n",
                    t_s,
                    lateral,
                    max_lateral_drift_m);
            }
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(" RESULTADO SLALOM\n");
    std::printf("  Max deriva lateral (curvas):     %8.3f m\n", max_lateral_drift_m);
    std::printf("  Umbral benchmark:                 %8.3f m\n", 0.15f);
    std::printf("----------------------------------------------------------------\n");
}
