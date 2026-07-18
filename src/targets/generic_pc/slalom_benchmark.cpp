#include "slalom_benchmark.hpp"

#include "geodesy.hpp"
#include "ins_ekf.hpp"

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

struct OutageRmsAccumulator {
    double sum_position_sq_m2;
    double sum_velocity_sq_mps2;
    double sum_yaw_sq_rad2;
    uint32_t sample_count;
};

struct TruthState {
    float pos_ned[3];
    float vel_ned[3];
    float accel_ned[3];
    float yaw_rad;
    float yaw_rate_radps;
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

    const geodesy::LLA ref = geodesy::lla(ref_lat_deg, ref_lon_deg, ref_alt_m);
    const geodesy::NED ned{truth->pos_ned[0], truth->pos_ned[1], truth->pos_ned[2]};
    const geodesy::LLA point = geodesy::ned_to_lla(ned, ref);

    gps->position.x = point.lat_deg;
    gps->position.y = point.lon_deg;
    gps->position.z = point.alt_m;
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

void outage_rms_accumulate_sample(
    OutageRmsAccumulator *acc,
    const InsEkfFilter *ekf,
    const TruthState *truth)
{
    if (acc == NULL || ekf == NULL || truth == NULL) {
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

    const float dn = est_pos[0] - truth->pos_ned[0];
    const float de = est_pos[1] - truth->pos_ned[1];
    const float dd = est_pos[2] - truth->pos_ned[2];
    const float dvn = est_vel[0] - truth->vel_ned[0];
    const float dve = est_vel[1] - truth->vel_ned[1];
    const float dvd = est_vel[2] - truth->vel_ned[2];
    const float dyaw = wrap_angle_rad(est_yaw - truth->yaw_rad);

    acc->sum_position_sq_m2 += static_cast<double>((dn * dn) + (de * de) + (dd * dd));
    acc->sum_velocity_sq_mps2 += static_cast<double>((dvn * dvn) + (dve * dve) + (dvd * dvd));
    acc->sum_yaw_sq_rad2 += static_cast<double>(dyaw * dyaw);
    ++acc->sample_count;
}

void outage_rms_finalize(const OutageRmsAccumulator *acc, SlalomOutageRms *out)
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

float horizontal_drift_m(const InsEkfFilter *ekf, const TruthState *truth)
{
    float est_pos[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_position_ned(ekf, est_pos);
    const float dn = est_pos[0] - truth->pos_ned[0];
    const float de = est_pos[1] - truth->pos_ned[1];
    return std::sqrt((dn * dn) + (de * de));
}

} /* namespace */

SlalomPassResult slalom_run_pass(bool nhc_enabled, bool verbose)
{
    SlalomPassResult result{};

    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;
    OutageRmsAccumulator outage_rms{};
    outage_rms_init(&outage_rms);

    TruthState truth{};
    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;
    float prev_t_s = 0.0f;

    for (uint32_t t_ms = 0U; t_ms <= TC04_DURATION_MS; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps_truth{};
        GpsSample gps{};

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        const float dt_s = (t_ms > 0U) ? (t_s - prev_t_s) : 0.0f;
        slalom_kinematics_at_time(t_s, &truth);
        if (dt_s > 0.0f) {
            truth_propagate(&truth, dt_s);
        }
        prev_t_s = t_s;

        truth_to_gps_sample(ref_lat_deg, ref_lon_deg, ref_alt_m, &truth, &gps_truth);
        gps_truth.timestamp_ms = t_ms;
        make_ideal_slalom_imu(&truth, t_ms, &imu);

        gps = gps_truth;
        const bool gps_outage =
            (t_ms >= TC04_GPS_OFF_START_MS) && (t_ms < TC04_GPS_OFF_END_MS);
        if (gps_outage) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        if (gps.fix_valid && !ekf_seeded) {
            ins_ekf_init(&ekf, gps.position, truth.yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            seed_ekf_from_truth(&ekf, &truth);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ref_alt_m = gps.position.z;
            ekf_seeded = true;

            if (verbose && t_ms == 0U) {
                std::printf(
                    "TC-04 [%s]: slalom %.0f km/h | a_lat max=%.1f m/s2 | periodo=%.1f s\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    TC04_SPEED_KMH,
                    TC04_MAX_LATERAL_ACCEL_MPS2,
                    TC04_SLALOM_PERIOD_S);
            }
        }

        if (ekf_seeded && imu.valid) {
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            ins_ekf_predict(&ekf, &imu);

            if (gps_outage) {
                outage_rms_accumulate_sample(&outage_rms, &ekf, &truth);
            }

            if (verbose && t_ms == TC04_GPS_OFF_START_MS) {
                std::printf(
                    "TC-04 [%s]: GPS APAGADO @ t=%.1f s | deriva=%.2f m\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    t_s,
                    horizontal_drift_m(&ekf, &truth));
            }

            if (t_ms == TC04_GPS_OFF_END_MS) {
                result.drift_exit_outage_m = horizontal_drift_m(&ekf, &truth);
                if (verbose) {
                    std::printf(
                        "TC-04 [%s]: fin apagon @ t=%.1f s | deriva=%.2f m\n",
                        nhc_enabled ? "NHC ON" : "NHC OFF",
                        t_s,
                        result.drift_exit_outage_m);
                }
            }

            if (gps.fix_valid) {
                (void)ins_ekf_update_gnss(&ekf, &gps);
            }
        }
    }

    if (ekf_seeded) {
        result.drift_final_m = horizontal_drift_m(&ekf, &truth);
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
