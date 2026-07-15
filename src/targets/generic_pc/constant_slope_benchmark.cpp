#include "constant_slope_benchmark.hpp"

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

struct OutageRmsAccumulator {
    double sum_position_sq_m2;
    double sum_velocity_sq_mps2;
    double sum_velocity_d_sq_mps2;
    double sum_yaw_sq_rad2;
    uint32_t sample_count;
};

struct TruthState {
    float pos_ned[3];
    float vel_ned[3];
    float yaw_rad;
    float pitch_rad;
};

void outage_rms_init(OutageRmsAccumulator *acc)
{
    if (acc == NULL) {
        return;
    }

    acc->sum_position_sq_m2 = 0.0;
    acc->sum_velocity_sq_mps2 = 0.0;
    acc->sum_velocity_d_sq_mps2 = 0.0;
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

void yaw_pitch_to_quat(float yaw_rad, float pitch_rad, float q[4])
{
    const float cy = std::cos(yaw_rad * 0.5f);
    const float sy = std::sin(yaw_rad * 0.5f);
    const float cp = std::cos(pitch_rad * 0.5f);
    const float sp = std::sin(pitch_rad * 0.5f);

    q[0] = cp * cy;
    q[1] = -sp * sy;
    q[2] = sp * cy;
    q[3] = cp * sy;
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

void truth_state_at_time(float t_s, TruthState *truth)
{
    if (truth == NULL) {
        return;
    }

    const float along_slope_m = TC03_SPEED_MPS * t_s;
    const float cos_pitch = std::cos(TC03_PITCH_RAD);
    const float sin_pitch = std::sin(TC03_PITCH_RAD);
    const float course_rad = static_cast<float>(TC03_COURSE_DEG * M_PI / 180.0);

    truth->pos_ned[0] = along_slope_m * cos_pitch * std::cos(course_rad);
    truth->pos_ned[1] = along_slope_m * cos_pitch * std::sin(course_rad);
    truth->pos_ned[2] = -along_slope_m * sin_pitch;
    truth->vel_ned[0] = TC03_SPEED_MPS * cos_pitch * std::cos(course_rad);
    truth->vel_ned[1] = TC03_SPEED_MPS * cos_pitch * std::sin(course_rad);
    truth->vel_ned[2] = -TC03_SPEED_MPS * sin_pitch;
    truth->yaw_rad = course_rad;
    truth->pitch_rad = TC03_PITCH_RAD;
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
    gps->speed_mps = TC03_SPEED_MPS;
    gps->course_deg = TC03_COURSE_DEG;
    gps->satellites = 12U;
    gps->fix_valid = true;
}

void make_ideal_slope_imu(uint32_t timestamp_ms, ImuSample *imu)
{
    if (imu == NULL) {
        return;
    }

    const float yaw_rad = static_cast<float>(TC03_COURSE_DEG * M_PI / 180.0);
    float q[4]{};
    yaw_pitch_to_quat(yaw_rad, TC03_PITCH_RAD, q);

    float dcm[3][3]{};
    quat_to_dcm_bn(q, dcm);

    const float g_ned[3] = {0.0f, 0.0f, kGravityMps2};
    float a_body[3]{};
    ned_to_body(dcm, g_ned, a_body);

    imu->accel_mps2[0] = a_body[0];
    imu->accel_mps2[1] = a_body[1];
    imu->accel_mps2[2] = a_body[2];
    imu->gyro_radps[0] = 0.0f;
    imu->gyro_radps[1] = 0.0f;
    imu->gyro_radps[2] = 0.0f;
    imu->timestamp_ms = timestamp_ms;
    imu->valid = true;
}

void seed_ekf_from_truth(InsEkfFilter *ekf, const TruthState *truth, const GpsSample *gps)
{
    if (ekf == NULL || truth == NULL || gps == NULL) {
        return;
    }

    yaw_pitch_to_quat(truth->yaw_rad, truth->pitch_rad, ekf->q_att_);
    ekf->vel_[0] = truth->vel_ned[0];
    ekf->vel_[1] = truth->vel_ned[1];
    ekf->vel_[2] = truth->vel_ned[2];
    (void)gps;
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
    acc->sum_velocity_d_sq_mps2 += static_cast<double>(dvd * dvd);
    acc->sum_yaw_sq_rad2 += static_cast<double>(dyaw * dyaw);
    ++acc->sample_count;
}

void outage_rms_finalize(const OutageRmsAccumulator *acc, ConstantSlopeOutageRms *out)
{
    if (acc == NULL || out == NULL || acc->sample_count == 0U) {
        return;
    }

    const double inv_count = 1.0 / static_cast<double>(acc->sample_count);
    out->position_m = static_cast<float>(std::sqrt(acc->sum_position_sq_m2 * inv_count));
    out->velocity_mps = static_cast<float>(std::sqrt(acc->sum_velocity_sq_mps2 * inv_count));
    out->velocity_d_mps = static_cast<float>(std::sqrt(acc->sum_velocity_d_sq_mps2 * inv_count));
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

ConstantSlopePassResult constant_slope_run_pass(bool nhc_enabled, bool verbose)
{
    ConstantSlopePassResult result{};

    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const float yaw_rad = static_cast<float>(TC03_COURSE_DEG * M_PI / 180.0);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;
    OutageRmsAccumulator outage_rms{};
    outage_rms_init(&outage_rms);

    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;

    for (uint32_t t_ms = 0U; t_ms <= TC03_DURATION_MS; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps_truth{};
        GpsSample gps{};
        TruthState truth{};

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        truth_state_at_time(t_s, &truth);
        truth_to_gps_sample(ref_lat_deg, ref_lon_deg, ref_alt_m, &truth, &gps_truth);
        gps_truth.timestamp_ms = t_ms;
        make_ideal_slope_imu(t_ms, &imu);

        gps = gps_truth;
        const bool gps_outage = (t_ms >= TC03_GPS_OFF_START_MS) && (t_ms < TC03_GPS_OFF_END_MS);
        if (gps_outage) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        if (gps.fix_valid && !ekf_seeded) {
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            seed_ekf_from_truth(&ekf, &truth, &gps);
            ref_lat_deg = gps.position.x;
            ref_lon_deg = gps.position.y;
            ref_alt_m = gps.position.z;
            ekf_seeded = true;

            if (verbose && t_ms == 0U) {
                std::printf(
                    "TC-03 [%s]: EKF en pendiente %.1f%% | pitch=%.2f deg | v=%.1f m/s\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    TC03_GRADE_PERCENT,
                    TC03_PITCH_RAD * kRadToDegF,
                    TC03_SPEED_MPS);
            }
        }

        if (ekf_seeded && imu.valid) {
            ins_ekf_set_nhc_enabled(&ekf, nhc_enabled);
            ins_ekf_predict(&ekf, &imu);

            if (gps_outage) {
                outage_rms_accumulate_sample(&outage_rms, &ekf, &truth);
            }

            if (verbose && t_ms == TC03_GPS_OFF_START_MS) {
                std::printf(
                    "TC-03 [%s]: GPS APAGADO @ t=%.1f s | deriva=%.2f m\n",
                    nhc_enabled ? "NHC ON" : "NHC OFF",
                    t_s,
                    horizontal_drift_m(&ekf, &truth));
            }

            if (t_ms == TC03_GPS_OFF_END_MS) {
                result.drift_exit_outage_m = horizontal_drift_m(&ekf, &truth);
                if (verbose) {
                    std::printf(
                        "TC-03 [%s]: fin apagon @ t=%.1f s | deriva=%.2f m\n",
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
        TruthState final_truth{};
        truth_state_at_time(static_cast<float>(TC03_DURATION_MS) * 0.001f, &final_truth);
        result.drift_final_m = horizontal_drift_m(&ekf, &final_truth);
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
