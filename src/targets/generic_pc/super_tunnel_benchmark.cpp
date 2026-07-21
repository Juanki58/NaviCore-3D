#include "super_tunnel_benchmark.hpp"

#include "geodesy.hpp"
#include "ins_ekf.hpp"
#include "ins_ekf_15_state.hpp"
#include "interfaces/INaviFilter.hpp"
#include "sensors_sim.hpp"

#include <cmath>
#include <cstdio>
#include <memory>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

#ifndef NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS
#define NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS 0.1f
#endif

#ifndef NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS
#define NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS 0.5f
#endif

namespace {

constexpr uint32_t kEkfStepMs = 10U;
constexpr uint32_t kSuperTunnelDurationMs = 60000U;
constexpr float kSuperTunnelSpeedMps = 25.0f;
constexpr float kSuperTunnelCourseDeg = 90.0f;
constexpr float kRadToDegF = 180.0f / static_cast<float>(M_PI);
constexpr uint32_t kExperimentSeed = 424242U;
constexpr float kConstantVelMaxBodyAccelMps2 = 0.08f;
constexpr float kConstantVelMaxGyroRadps = 0.02f;

struct OutageRmsAccumulator {
    double sum_position_sq_m2;
    double sum_velocity_sq_mps2;
    double sum_yaw_sq_rad2;
    uint32_t sample_count;
};

struct NhcSummaryAccumulator {
    uint32_t count;
    double sum_innov_y;
    double sum_innov_z;
    double sum_innov_y_sq;
    double sum_innov_z_sq;
    double sum_k_y;
    double sum_k_z;
    double sum_nis;
    double sum_v_body_y;
    double sum_v_body_z;
    float max_nis;
    uint32_t same_sign_count;
};

void nhc_summary_init(NhcSummaryAccumulator *acc)
{
    if (acc == NULL) {
        return;
    }

    acc->count = 0U;
    acc->sum_innov_y = 0.0;
    acc->sum_innov_z = 0.0;
    acc->sum_innov_y_sq = 0.0;
    acc->sum_innov_z_sq = 0.0;
    acc->sum_k_y = 0.0;
    acc->sum_k_z = 0.0;
    acc->sum_nis = 0.0;
    acc->sum_v_body_y = 0.0;
    acc->sum_v_body_z = 0.0;
    acc->max_nis = 0.0f;
    acc->same_sign_count = 0U;
}

void nhc_summary_accumulate(
    NhcSummaryAccumulator *acc,
    const InsEkfNhcUpdateDetail *detail)
{
    if (acc == NULL || detail == NULL) {
        return;
    }

    acc->sum_innov_y += static_cast<double>(detail->innov_y_mps);
    acc->sum_innov_z += static_cast<double>(detail->innov_z_mps);
    acc->sum_innov_y_sq +=
        static_cast<double>(detail->innov_y_mps) * static_cast<double>(detail->innov_y_mps);
    acc->sum_innov_z_sq +=
        static_cast<double>(detail->innov_z_mps) * static_cast<double>(detail->innov_z_mps);
    acc->sum_k_y += static_cast<double>(detail->k_y);
    acc->sum_k_z += static_cast<double>(detail->k_z);
    acc->sum_nis += static_cast<double>(detail->nis);
    acc->sum_v_body_y += static_cast<double>(detail->v_body_y_mps);
    acc->sum_v_body_z += static_cast<double>(detail->v_body_z_mps);
    if (detail->nis > acc->max_nis) {
        acc->max_nis = detail->nis;
    }
    if ((detail->innov_y_mps * detail->dx_att_y_rad) > 0.0f) {
        ++acc->same_sign_count;
    }
    ++acc->count;
}

bool nhc_summary_finalize(
    const NhcSummaryAccumulator *acc,
    InsEkfNhcRunSummary *out_summary)
{
    if (acc == NULL || out_summary == NULL || acc->count == 0U) {
        return false;
    }

    const double inv_n = 1.0 / static_cast<double>(acc->count);
    const double mean_y = acc->sum_innov_y * inv_n;
    const double mean_z = acc->sum_innov_z * inv_n;
    const double var_y = (acc->sum_innov_y_sq * inv_n) - (mean_y * mean_y);
    const double var_z = (acc->sum_innov_z_sq * inv_n) - (mean_z * mean_z);

    out_summary->sample_count = acc->count;
    out_summary->mean_innov_y_mps = static_cast<float>(mean_y);
    out_summary->mean_innov_z_mps = static_cast<float>(mean_z);
    out_summary->std_innov_y_mps = static_cast<float>(std::sqrt(std::fmax(0.0, var_y)));
    out_summary->std_innov_z_mps = static_cast<float>(std::sqrt(std::fmax(0.0, var_z)));
    out_summary->mean_k_y = static_cast<float>(acc->sum_k_y * inv_n);
    out_summary->mean_k_z = static_cast<float>(acc->sum_k_z * inv_n);
    out_summary->frac_same_sign_corr =
        static_cast<float>(acc->same_sign_count) * static_cast<float>(inv_n);
    out_summary->mean_nis = static_cast<float>(acc->sum_nis * inv_n);
    out_summary->max_nis = acc->max_nis;
    out_summary->mean_v_body_y_mps = static_cast<float>(acc->sum_v_body_y * inv_n);
    out_summary->mean_v_body_z_mps = static_cast<float>(acc->sum_v_body_z * inv_n);
    return true;
}

void print_nhc_summary_table(
    const char *title,
    const InsEkfNhcRunSummary *summary)
{
    if (title == NULL || summary == NULL || summary->sample_count == 0U) {
        return;
    }

    std::printf("  [%s] n=%u\n", title, summary->sample_count);
    std::printf("    media(innov_y)              %10.6f m/s\n", summary->mean_innov_y_mps);
    std::printf("    media(innov_z)              %10.6f m/s\n", summary->mean_innov_z_mps);
    std::printf("    std(innov_y)                %10.6f m/s\n", summary->std_innov_y_mps);
    std::printf("    std(innov_z)                %10.6f m/s\n", summary->std_innov_z_mps);
    std::printf("    media(K_y)                  %10.6f\n", summary->mean_k_y);
    std::printf("    media(K_z)                  %10.6f\n", summary->mean_k_z);
    std::printf("    media(v_body_y)             %10.6f m/s\n", summary->mean_v_body_y_mps);
    std::printf("    media(v_body_z)             %10.6f m/s\n", summary->mean_v_body_z_mps);
    std::printf(
        "    mismo signo innov/dx_att_y    %10.1f%%\n",
        summary->frac_same_sign_corr * 100.0f);
    std::printf("    NIS medio                   %10.4f\n", summary->mean_nis);
    std::printf("    NIS maximo                  %10.4f\n", summary->max_nis);
}

void write_nhc_summary_json_field(
    FILE *fp,
    const char *key,
    const InsEkfNhcRunSummary *summary,
    bool trailing_comma)
{
    if (fp == NULL || key == NULL || summary == NULL) {
        return;
    }

    std::fprintf(fp, "  \"%s\": {\n", key);
    std::fprintf(fp, "    \"sample_count\": %u,\n", summary->sample_count);
    std::fprintf(
        fp,
        "    \"mean_innov_y_mps\": %.8f,\n"
        "    \"mean_innov_z_mps\": %.8f,\n"
        "    \"std_innov_y_mps\": %.8f,\n"
        "    \"std_innov_z_mps\": %.8f,\n"
        "    \"mean_k_y\": %.8f,\n"
        "    \"mean_k_z\": %.8f,\n"
        "    \"mean_v_body_y_mps\": %.8f,\n"
        "    \"mean_v_body_z_mps\": %.8f,\n"
        "    \"frac_same_sign_corr\": %.8f,\n"
        "    \"mean_nis\": %.8f,\n"
        "    \"max_nis\": %.8f\n",
        summary->mean_innov_y_mps,
        summary->mean_innov_z_mps,
        summary->std_innov_y_mps,
        summary->std_innov_z_mps,
        summary->mean_k_y,
        summary->mean_k_z,
        summary->mean_v_body_y_mps,
        summary->mean_v_body_z_mps,
        summary->frac_same_sign_corr,
        summary->mean_nis,
        summary->max_nis);
    std::fprintf(fp, "  }%s\n", trailing_comma ? "," : "");
}

bool write_nhc_summary_json(
    const char *experiment_id,
    const SuperTunnelPassResult *result,
    const char *json_path)
{
    if (experiment_id == NULL || result == NULL || json_path == NULL || json_path[0] == '\0') {
        return false;
    }

    FILE *fp = std::fopen(json_path, "w");
    if (fp == NULL) {
        return false;
    }

    std::fprintf(fp, "{\n");
    std::fprintf(fp, "  \"experiment_id\": \"%s\",\n", experiment_id);
    std::fprintf(
        fp,
        "  \"drift_exit_m\": %.4f,\n"
        "  \"drift_final_m\": %.4f,\n"
        "  \"nhc_updates\": %u,\n",
        result->drift_exit_tunnel_m,
        result->drift_final_m,
        result->nhc_updates);

    bool wrote_section = false;
    if (result->nhc_summary_all_valid) {
        write_nhc_summary_json_field(fp, "summary_all", &result->nhc_summary_all, false);
        wrote_section = true;
    }
    if (result->nhc_summary_window_valid) {
        if (wrote_section) {
            std::fprintf(fp, ",\n");
        }
        write_nhc_summary_json_field(
            fp,
            "summary_outage_const_vel",
            &result->nhc_summary_window,
            false);
        wrote_section = true;
    }
    if (!wrote_section) {
        std::fprintf(fp, "  \"summary_all\": null\n");
    }

    std::fprintf(fp, "}\n");
    std::fclose(fp);
    return true;
}

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

    geodesy::lla_to_ned(
        ref_lat_deg,
        ref_lon_deg,
        ref_alt_m,
        gps_truth->position.x,
        gps_truth->position.y,
        gps_truth->position.z,
        north_m,
        east_m,
        down_m);
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
    const INaviFilter *filter,
    float truth_n_m,
    float truth_e_m,
    float truth_d_m,
    const float truth_vel_ned[3],
    float truth_yaw_rad)
{
    const InsEkfFilter *ekf = navi_filter_try_get_ins_ekf(filter);
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

float filter_horizontal_drift_m(
    const INaviFilter *filter,
    float truth_n_m,
    float truth_e_m)
{
    if (filter == nullptr) {
        return 0.0f;
    }

    const NaviState state = filter->get_state();
    const float dn = static_cast<float>(state.pos_ned[0]) - truth_n_m;
    const float de = static_cast<float>(state.pos_ned[1]) - truth_e_m;
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


bool is_constant_velocity_cruise(const ImuSample *imu)
{
    if (imu == NULL || !imu->valid) {
        return false;
    }

    return (std::fabs(imu->accel_mps2[0]) <= kConstantVelMaxBodyAccelMps2)
        && (std::fabs(imu->accel_mps2[1]) <= kConstantVelMaxBodyAccelMps2)
        && (std::fabs(imu->gyro_radps[0]) <= kConstantVelMaxGyroRadps)
        && (std::fabs(imu->gyro_radps[1]) <= kConstantVelMaxGyroRadps)
        && (std::fabs(imu->gyro_radps[2]) <= kConstantVelMaxGyroRadps);
}

bool nhc_allowed_this_tick(
    SuperTunnelNhcPolicy policy,
    bool gps_outage,
    bool gps_fix_valid,
    const ImuSample *imu)
{
    switch (policy) {
    case SUPER_TUNNEL_NHC_OFF:
        return false;
    case SUPER_TUNNEL_NHC_ALWAYS:
        return true;
    case SUPER_TUNNEL_NHC_CONSTANT_VEL_ONLY:
        return gps_outage && is_constant_velocity_cruise(imu);
    case SUPER_TUNNEL_NHC_NO_GNSS_FIX:
        return !gps_fix_valid;
    default:
        return false;
    }
}

void append_nhc_trace_row(
    FILE *trace_fp,
    const InsEkfFilter *ekf,
    uint32_t t_ms,
    bool gps_outage,
    bool constant_vel)
{
    if (trace_fp == NULL || ekf == NULL) {
        return;
    }

    InsEkfNhcUpdateDetail detail{};
    if (!ins_ekf_get_nhc_last_update_detail(ekf, &detail)) {
        return;
    }
    if (detail.timestamp_ms != t_ms) {
        return;
    }

    const float yaw_deg = detail.yaw_rad * kRadToDegF;
    std::fprintf(
        trace_fp,
        "%u,%u,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u\n",
        t_ms,
        gps_outage ? 1U : 0U,
        constant_vel ? 1U : 0U,
        detail.vel_n_mps,
        detail.vel_e_mps,
        detail.vel_d_mps,
        yaw_deg,
        detail.v_body_x_mps,
        detail.v_body_y_mps,
        detail.v_body_z_mps,
        detail.innov_y_mps,
        detail.innov_z_mps,
        detail.innov_norm_mps,
        detail.k_y,
        detail.k_z,
        detail.k_max,
        detail.nis,
        detail.dx_vel_n_mps,
        detail.dx_vel_e_mps,
        detail.dx_vel_d_mps,
        detail.dx_att_x_rad,
        detail.dx_att_y_rad,
        detail.dx_att_z_rad,
        detail.dx_vel_norm_mps,
        detail.dx_att_norm_rad,
        detail.dx_pos_norm_m,
        ins_ekf_nhc_update_count(ekf));
}

void cov_block_frob(
    const InsEkfFilter *ekf,
    uint8_t row0,
    uint8_t col0,
    uint8_t n_rows,
    uint8_t n_cols,
    float *out_frob)
{
    if (out_frob == NULL) {
        return;
    }
    *out_frob = 0.0f;
    if (ekf == NULL || !ekf->initialized) {
        return;
    }
    double sum_sq = 0.0;
    for (uint8_t r = 0U; r < n_rows; ++r) {
        for (uint8_t c = 0U; c < n_cols; ++c) {
            const float v = ekf->cov.P[row0 + r][col0 + c];
            sum_sq += static_cast<double>(v) * static_cast<double>(v);
        }
    }
    *out_frob = static_cast<float>(std::sqrt(sum_sq));
}

void append_anatomy_row(
    FILE *anatomy_fp,
    const InsEkfFilter *ekf,
    const INaviFilter *filter,
    uint32_t t_ms,
    bool gps_outage,
    bool nhc_applied,
    float truth_n,
    float truth_e)
{
    if (anatomy_fp == NULL || ekf == NULL || !ekf->initialized) {
        return;
    }

    float p_vv_frob = 0.0f;
    float p_pv_frob = 0.0f;
    cov_block_frob(ekf, INS_ERR_VEL_N, INS_ERR_VEL_N, 3U, 3U, &p_vv_frob);
    cov_block_frob(ekf, INS_ERR_POS_N, INS_ERR_VEL_N, 3U, 3U, &p_pv_frob);
    const float p_vv_trace =
        ekf->cov.P[INS_ERR_VEL_N][INS_ERR_VEL_N]
        + ekf->cov.P[INS_ERR_VEL_E][INS_ERR_VEL_E]
        + ekf->cov.P[INS_ERR_VEL_D][INS_ERR_VEL_D];

    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_velocity_ned(ekf, vel_ned);
    const float vel_norm = std::sqrt(
        vel_ned[0] * vel_ned[0] + vel_ned[1] * vel_ned[1] + vel_ned[2] * vel_ned[2]);
    const float drift_h =
        (filter != nullptr) ? filter_horizontal_drift_m(filter, truth_n, truth_e) : 0.0f;

    float dx_pos = 0.0f;
    float dx_vel = 0.0f;
    float k_max = 0.0f;
    float innov_norm = 0.0f;
    if (nhc_applied) {
        InsEkfNhcUpdateDetail detail{};
        if (ins_ekf_get_nhc_last_update_detail(ekf, &detail)
            && detail.timestamp_ms == t_ms) {
            dx_pos = detail.dx_pos_norm_m;
            dx_vel = detail.dx_vel_norm_mps;
            k_max = detail.k_max;
            innov_norm = detail.innov_norm_mps;
        }
    }

    std::fprintf(
        anatomy_fp,
        "%u,%u,%u,%.9e,%.9e,%.9e,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        t_ms,
        gps_outage ? 1U : 0U,
        nhc_applied ? 1U : 0U,
        p_vv_frob,
        p_pv_frob,
        p_vv_trace,
        vel_norm,
        drift_h,
        dx_pos,
        dx_vel,
        k_max,
        innov_norm);
}

} /* namespace */

SuperTunnelPassResult super_tunnel_run_with_config(const SuperTunnelRunConfig &config)
{
    SuperTunnelPassResult result{};

    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const uint32_t seed = (config.rng_seed != 0U)
        ? config.rng_seed
        : sensors_simulation_get_default_seed();

    const float lateral_std = (config.nhc_lateral_std_override_mps > 0.0f)
        ? config.nhc_lateral_std_override_mps
        : (NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS * config.nhc_r_lateral_multiplier);
    const float vertical_std = (config.nhc_vertical_std_override_mps > 0.0f)
        ? config.nhc_vertical_std_override_mps
        : (NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS * config.nhc_r_vertical_multiplier);

    FILE *trace_fp = NULL;
    if (config.nhc_trace_csv_path != NULL && config.nhc_trace_csv_path[0] != '\0') {
        trace_fp = std::fopen(config.nhc_trace_csv_path, "w");
        if (trace_fp != NULL) {
            std::fprintf(
                trace_fp,
                "t_ms,gps_outage,constant_vel,vel_n,vel_e,vel_d,yaw_deg,"
                "vbx,vby,vbz,innov_y,innov_z,innov_norm,k_y,k_z,k_max,nis,"
                "dx_vel_n,dx_vel_e,dx_vel_d,dx_att_x,dx_att_y,dx_att_z,"
                "dx_vel_norm,dx_att_norm,dx_pos_norm,nhc_count\n");
        }
    }

    FILE *anatomy_fp = NULL;
    if (config.anatomy_csv_path != NULL && config.anatomy_csv_path[0] != '\0') {
        anatomy_fp = std::fopen(config.anatomy_csv_path, "w");
        if (anatomy_fp != NULL) {
            std::fprintf(
                anatomy_fp,
                "t_ms,gps_outage,nhc_applied,P_vv_frob,P_pv_frob,P_vv_trace,"
                "vel_norm_mps,drift_h_m,dx_pos_norm,dx_vel_norm,k_max,innov_norm\n");
        }
    }

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
        config.imu_mode == SUPER_TUNNEL_IMU_DIRTY_FULL);

    std::unique_ptr<INaviFilter> nav_filter = create_default_navi_filter();
    InsEkf15State *filter_impl = dynamic_cast<InsEkf15State *>(nav_filter.get());
    INaviFilter *filter = nav_filter.get();
    bool ekf_seeded = false;
    OutageRmsAccumulator outage_rms{};
    NhcSummaryAccumulator nhc_window_stats{};
    double innov_norm_sum = 0.0;
    uint32_t innov_norm_count = 0U;
    float innov_final_norm = 0.0f;

    float ref_lat_deg = origin.x;
    float ref_lon_deg = origin.y;
    float ref_alt_m = origin.z;
    outage_rms_init(&outage_rms);
    nhc_summary_init(&nhc_window_stats);

    for (uint32_t t_ms = 0U; t_ms <= kSuperTunnelDurationMs; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps_meas{};
        GpsSample gps_truth{};

        if (!gps_simulator_read(&sensors.gps, t_ms, &gps_meas)) {
            continue;
        }

        if (config.imu_mode != SUPER_TUNNEL_IMU_IDEAL) {
            imu_simulator_step_bias_random_walk(&sensors.imu);
        }
        super_tunnel_sanitize_imu(&imu);
        if (config.imu_mode == SUPER_TUNNEL_IMU_IDEAL) {
            imu.timestamp_ms = t_ms;
        } else {
            imu_simulator_apply_measurement_model(&sensors.imu, &imu, t_ms);
            /* Reloj deterministico del benchmark (evita descartar filas de traza D). */
            imu.timestamp_ms = t_ms;
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

        if (gps.fix_valid && !ekf_seeded && filter_impl != nullptr) {
            if (filter_impl->seed_from_gnss_sample(gps, NAVICORE_DOMAIN_AIR)) {
                filter_impl->set_nhc_measurement_stds(lateral_std, vertical_std);
                ref_lat_deg = gps.position.x;
                ref_lon_deg = gps.position.y;
                ref_alt_m = gps.position.z;
                ekf_seeded = true;

                if (config.verbose) {
                    std::printf(
                        "SUPER_TUNNEL: %s init | policy=%u | R_lat=%.3f m/s | R_vert=%.3f m/s | IMU=%u\n",
                        filter->get_filter_name().c_str(),
                        static_cast<unsigned>(config.nhc_policy),
                        lateral_std,
                        vertical_std,
                        static_cast<unsigned>(config.imu_mode));
                }
            }
        }

        if (ekf_seeded && imu.valid && filter != nullptr && filter_impl != nullptr) {
            const bool allow_nhc =
                nhc_allowed_this_tick(config.nhc_policy, gps_outage, gps.fix_valid, &imu);
            const bool constant_vel = is_constant_velocity_cruise(&imu);
            const float nhc_lateral = allow_nhc ? lateral_std : 0.0f;
            const float nhc_vertical = allow_nhc ? vertical_std : 0.0f;
            filter->apply_constraints(false, nhc_lateral, nhc_vertical);
            filter_impl->sync_simulation_clock_ms(t_ms);

            InsEkfFilter &ekf = filter_impl->native();
            const uint32_t nhc_before = ins_ekf_nhc_update_count(&ekf);
            filter->predict(
                static_cast<double>(kEkfStepMs) * 0.001,
                imu.accel_mps2,
                imu.gyro_radps);
            const uint32_t nhc_after = ins_ekf_nhc_update_count(&ekf);

            if (nhc_after > nhc_before) {
                InsEkfNhcUpdateDetail detail{};
                if (ins_ekf_get_nhc_last_update_detail(&ekf, &detail)) {
                    if (gps_outage) {
                        innov_norm_sum += static_cast<double>(detail.innov_norm_mps);
                        ++innov_norm_count;
                        innov_final_norm = detail.innov_norm_mps;
                    }
                    if (gps_outage && constant_vel) {
                        nhc_summary_accumulate(&nhc_window_stats, &detail);
                    }
                }
                append_nhc_trace_row(trace_fp, &ekf, t_ms, gps_outage, constant_vel);
            }

            if (anatomy_fp != NULL) {
                float truth_n_a = 0.0f;
                float truth_e_a = 0.0f;
                float truth_d_a = 0.0f;
                gps_truth_to_ned_m(
                    ref_lat_deg,
                    ref_lon_deg,
                    ref_alt_m,
                    &gps_truth,
                    &truth_n_a,
                    &truth_e_a,
                    &truth_d_a);
                append_anatomy_row(
                    anatomy_fp,
                    &ekf,
                    filter,
                    t_ms,
                    gps_outage,
                    nhc_after > nhc_before,
                    truth_n_a,
                    truth_e_a);
            }

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
                    filter,
                    truth_n,
                    truth_e,
                    truth_d,
                    truth_vel,
                    truth_yaw_rad);
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
                result.drift_exit_tunnel_m = filter_horizontal_drift_m(filter, truth_n, truth_e);
                if (config.verbose) {
                    std::printf(
                        "SUPER_TUNNEL: salida tunel @ %.1f s | deriva=%.2f m | NHC updates=%u\n",
                        static_cast<float>(t_ms) * 0.001f,
                        result.drift_exit_tunnel_m,
                        ins_ekf_nhc_update_count(&ekf));
                }
            }

            if (gps.fix_valid) {
                (void)filter_impl->update_gnss_from_sample(gps);
            }
        }
    }

    if (ekf_seeded && filter != nullptr && filter_impl != nullptr) {
        const InsEkfFilter &ekf = filter_impl->native();
        GpsSample final_truth{};
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
        result.drift_final_m = filter_horizontal_drift_m(filter, truth_n, truth_e);
        result.nhc_updates = ins_ekf_nhc_update_count(&ekf);
        outage_rms_finalize(&outage_rms, &result.outage_rms);
        ins_ekf_get_nhc_innovation_max(
            &ekf,
            &result.nhc_innovation_max.lateral_mps,
            &result.nhc_innovation_max.vertical_mps,
            &result.nhc_innovation_max.norm_mps);
        result.innov_final_norm_mps = innov_final_norm;
        result.innov_mean_norm_mps = (innov_norm_count > 0U)
            ? static_cast<float>(innov_norm_sum / static_cast<double>(innov_norm_count))
            : 0.0f;
        result.nhc_summary_all_valid =
            ins_ekf_get_nhc_run_summary(&ekf, &result.nhc_summary_all);
        result.nhc_summary_window_valid =
            nhc_summary_finalize(&nhc_window_stats, &result.nhc_summary_window);
    }

    if (trace_fp != NULL) {
        std::fclose(trace_fp);
    }
    if (anatomy_fp != NULL) {
        std::fclose(anatomy_fp);
    }

    return result;
}

SuperTunnelPassResult super_tunnel_run_pass(
    bool nhc_enabled,
    bool verbose,
    SuperTunnelImuMode imu_mode,
    uint32_t rng_seed,
    float nhc_lateral_std_mps,
    float nhc_vertical_std_mps)
{
    SuperTunnelRunConfig config{};
    config.nhc_policy = nhc_enabled ? SUPER_TUNNEL_NHC_ALWAYS : SUPER_TUNNEL_NHC_OFF;
    config.imu_mode = imu_mode;
    config.rng_seed = rng_seed;
    config.nhc_r_lateral_multiplier = 1.0f;
    config.nhc_r_vertical_multiplier = 1.0f;
    config.nhc_lateral_std_override_mps = nhc_lateral_std_mps;
    config.nhc_vertical_std_override_mps = nhc_vertical_std_mps;
    config.verbose = verbose;
    config.nhc_trace_csv_path = NULL;
    config.anatomy_csv_path = NULL;

    return super_tunnel_run_with_config(config);
}

namespace {

void run_traced_experiment(
    const char *experiment_id,
    SuperTunnelRunConfig config,
    char trace_path[256]);

void run_g_r_experiment(
    const char *experiment_id,
    SuperTunnelRunConfig base,
    float lateral_std_mps,
    float vertical_std_mps,
    char trace_path[256])
{
    SuperTunnelRunConfig config = base;
    config.nhc_policy = SUPER_TUNNEL_NHC_NO_GNSS_FIX;
    config.nhc_lateral_std_override_mps = lateral_std_mps;
    config.nhc_vertical_std_override_mps = vertical_std_mps;
    run_traced_experiment(experiment_id, config, trace_path);
}

void run_traced_experiment(
    const char *experiment_id,
    SuperTunnelRunConfig config,
    char trace_path[256])
{
    config.experiment_id = experiment_id;
    config.nhc_trace_csv_path = NULL;
    trace_path[0] = '\0';

    if (config.nhc_policy != SUPER_TUNNEL_NHC_OFF) {
        std::snprintf(
            trace_path,
            256,
            "docs/nhc_experiments/%s_trace.csv",
            experiment_id);
        config.nhc_trace_csv_path = trace_path;
    }

    const SuperTunnelPassResult result = super_tunnel_run_with_config(config);

    std::printf(
        "  %-14s drift_exit=%8.2f m | drift_final=%8.2f m | NHC=%5u",
        experiment_id,
        result.drift_exit_tunnel_m,
        result.drift_final_m,
        result.nhc_updates);
    if (config.nhc_trace_csv_path != NULL) {
        std::printf(" | trace=%s", config.nhc_trace_csv_path);
    }
    std::printf("\n");

    if (config.nhc_policy != SUPER_TUNNEL_NHC_OFF) {
        if (result.nhc_summary_all_valid) {
            print_nhc_summary_table("resumen: toda la corrida", &result.nhc_summary_all);
        }
        if (result.nhc_summary_window_valid) {
            print_nhc_summary_table(
                "resumen: apagon + vel.const",
                &result.nhc_summary_window);
        }

        char summary_path[256];
        std::snprintf(
            summary_path,
            256,
            "docs/nhc_experiments/%s_summary.json",
            experiment_id);
        if (write_nhc_summary_json(experiment_id, &result, summary_path)) {
            std::printf("  summary=%s\n", summary_path);
        }
    }
}

} /* namespace */

void run_super_tunnel_nhc_benchmark()
{
    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" BENCHMARK: SUPER_TUNNEL — recta 90 km/h, apagon GPS 10-55 s\n");
    std::printf("================================================================\n");

    const SuperTunnelPassResult without_nhc = super_tunnel_run_pass(false, true);
    const SuperTunnelPassResult with_nhc = super_tunnel_run_pass(true, true);

    std::printf(
        "  Salida tunel: sin NHC=%.2f m | con NHC=%.2f m\n",
        without_nhc.drift_exit_tunnel_m,
        with_nhc.drift_exit_tunnel_m);
}

int run_super_tunnel_nhc_experiments()
{
#ifdef _WIN32
    (void)_mkdir("docs");
    (void)_mkdir("docs\\nhc_experiments");
#else
    (void)mkdir("docs", 0755);
    (void)mkdir("docs/nhc_experiments", 0755);
#endif

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" SUPER_TUNNEL: sintonia R_nhc asimetrico + politica G\n");
    std::printf("  Jacobiano corregido | seed=%u | IMU ideal\n", kExperimentSeed);
    std::printf("  R lateral (Y cuerpo) bajo | R vertical (Z cuerpo) inflado\n");
    std::printf("  Politica G: NHC OFF si gps.fix_valid\n");
    std::printf("  Apagon GPS: %.1f-%.1f s | baseline A ~493 m\n",
                static_cast<float>(SUPER_TUNNEL_GPS_OFF_START_MS) * 0.001f,
                static_cast<float>(SUPER_TUNNEL_GPS_OFF_END_MS) * 0.001f);
    std::printf("================================================================\n");
    std::printf("  Analisis: python run_nhc_experiments.py\n");
    std::printf("----------------------------------------------------------------\n");

    SuperTunnelRunConfig base{};
    base.experiment_id = NULL;
    base.imu_mode = SUPER_TUNNEL_IMU_IDEAL;
    base.rng_seed = kExperimentSeed;
    base.nhc_r_lateral_multiplier = 1.0f;
    base.nhc_r_vertical_multiplier = 1.0f;
    base.nhc_lateral_std_override_mps = 0.0f;
    base.nhc_vertical_std_override_mps = 0.0f;
    base.verbose = false;
    base.nhc_trace_csv_path = NULL;
    base.anatomy_csv_path = NULL;

    char trace_path[256];

    std::printf("\n[A] Referencia sin NHC\n");
    SuperTunnelRunConfig exp_a = base;
    exp_a.nhc_policy = SUPER_TUNNEL_NHC_OFF;
    run_traced_experiment("A", exp_a, trace_path);

    std::printf("\n[G] Barrido R_lat x R_vert (politica G, sigma en m/s)\n");
    static const struct {
        const char *id;
        float lateral_std_mps;
        float vertical_std_mps;
    } kGRGrid[] = {
        {"G_l01_v05", 0.1f, 0.5f},
        {"G_l01_v10", 0.1f, 1.0f},
        {"G_l02_v05", 0.2f, 0.5f},
        {"G_l02_v10", 0.2f, 1.0f},
        {"G_l10_v05", 1.0f, 0.5f},
        {"G_l10_v10", 1.0f, 1.0f},
    };

    for (size_t i = 0U; i < (sizeof(kGRGrid) / sizeof(kGRGrid[0])); ++i) {
        std::printf(
            "  -> %s | sigma_lat=%.2f | sigma_vert=%.2f\n",
            kGRGrid[i].id,
            kGRGrid[i].lateral_std_mps,
            kGRGrid[i].vertical_std_mps);
        run_g_r_experiment(
            kGRGrid[i].id,
            base,
            kGRGrid[i].lateral_std_mps,
            kGRGrid[i].vertical_std_mps,
            trace_path);
    }

    std::printf("\n----------------------------------------------------------------\n");
    std::printf("Harness D: traza CSV + resumen JSON por experimento\n");
    std::printf("  innov_y/z, k_y/k_z, NIS, vby/vbz, dx_* | *_summary.json\n");
    std::printf("  Nota: sesgo constante requiere estados en el modelo (montaje/bias);\n");
    std::printf("  el NHC actual no los estima — no es limite universal del NHC.\n");
    std::printf("================================================================\n");

    return 0;
}

int run_super_tunnel_bd_isolation_rerun()
{
#ifdef _WIN32
    (void)_mkdir("docs");
    (void)_mkdir("docs\\benchmarks");
    (void)_mkdir("docs\\benchmarks\\super_tunnel_bd_rerun");
#else
    (void)mkdir("docs", 0755);
    (void)mkdir("docs/benchmarks", 0755);
    (void)mkdir("docs/benchmarks/super_tunnel_bd_rerun", 0755);
#endif

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" SUPER_TUNNEL B/B_dirty + N_always isolation (preregistered)\n");
    std::printf("  Protocol: docs/diagnostics/16-super-tunnel-ieee952-rerun-protocol.md\n");
    std::printf("  Jacobiano NHC coeficientes = bf2bfbd | seed=%u | ZUPT=false hardcoded\n",
                kExperimentSeed);
    std::printf("  Binario actual != corrida original 481/1416 (ver protocolo §0)\n");
    std::printf("  Out: docs/benchmarks/super_tunnel_bd_rerun/\n");
    std::printf("================================================================\n");

    static const struct {
        const char *id;
        SuperTunnelNhcPolicy nhc_policy;
        SuperTunnelImuMode imu_mode;
    } kArms[] = {
        {"A", SUPER_TUNNEL_NHC_OFF, SUPER_TUNNEL_IMU_IDEAL},
        {"A_dirty", SUPER_TUNNEL_NHC_OFF, SUPER_TUNNEL_IMU_DIRTY_FULL},
        {"B", SUPER_TUNNEL_NHC_CONSTANT_VEL_ONLY, SUPER_TUNNEL_IMU_IDEAL},
        {"B_dirty", SUPER_TUNNEL_NHC_CONSTANT_VEL_ONLY, SUPER_TUNNEL_IMU_DIRTY_FULL},
        {"N_always", SUPER_TUNNEL_NHC_ALWAYS, SUPER_TUNNEL_IMU_IDEAL},
        {"N_always_dirty", SUPER_TUNNEL_NHC_ALWAYS, SUPER_TUNNEL_IMU_DIRTY_FULL},
    };

    FILE *results_fp = std::fopen(
        "docs/benchmarks/super_tunnel_bd_rerun/results.csv", "w");
    if (results_fp != NULL) {
        std::fprintf(
            results_fp,
            "experiment_id,nhc_policy,imu_mode,drift_exit_m,drift_final_m,nhc_updates,"
            "innov_max_norm_mps,anatomy_csv,trace_csv\n");
    }

    for (size_t i = 0U; i < (sizeof(kArms) / sizeof(kArms[0])); ++i) {
        SuperTunnelRunConfig config{};
        config.experiment_id = kArms[i].id;
        config.nhc_policy = kArms[i].nhc_policy;
        config.imu_mode = kArms[i].imu_mode;
        config.rng_seed = kExperimentSeed;
        config.nhc_r_lateral_multiplier = 1.0f;
        config.nhc_r_vertical_multiplier = 1.0f;
        config.nhc_lateral_std_override_mps = 0.0f;
        config.nhc_vertical_std_override_mps = 0.0f;
        config.verbose = false;

        char anatomy_path[256];
        char trace_path[256];
        std::snprintf(
            anatomy_path,
            sizeof(anatomy_path),
            "docs/benchmarks/super_tunnel_bd_rerun/%s_anatomy.csv",
            kArms[i].id);
        config.anatomy_csv_path = anatomy_path;

        if (kArms[i].nhc_policy != SUPER_TUNNEL_NHC_OFF) {
            std::snprintf(
                trace_path,
                sizeof(trace_path),
                "docs/benchmarks/super_tunnel_bd_rerun/%s_trace.csv",
                kArms[i].id);
            config.nhc_trace_csv_path = trace_path;
        } else {
            trace_path[0] = '\0';
            config.nhc_trace_csv_path = NULL;
        }

        const SuperTunnelPassResult result = super_tunnel_run_with_config(config);

        const char *policy_name = "off";
        if (kArms[i].nhc_policy == SUPER_TUNNEL_NHC_CONSTANT_VEL_ONLY) {
            policy_name = "constant_vel_only";
        } else if (kArms[i].nhc_policy == SUPER_TUNNEL_NHC_ALWAYS) {
            policy_name = "always";
        }
        const char *imu_name =
            (kArms[i].imu_mode == SUPER_TUNNEL_IMU_DIRTY_FULL) ? "dirty_full" : "ideal";

        std::printf(
            "  %-14s drift_exit=%8.2f m | drift_final=%8.2f m | NHC=%5u | imu=%s | policy=%s\n",
            kArms[i].id,
            result.drift_exit_tunnel_m,
            result.drift_final_m,
            result.nhc_updates,
            imu_name,
            policy_name);
        std::printf("    anatomy=%s\n", anatomy_path);

        if (results_fp != NULL) {
            std::fprintf(
                results_fp,
                "%s,%s,%s,%.6f,%.6f,%u,%.6f,%s,%s\n",
                kArms[i].id,
                policy_name,
                imu_name,
                result.drift_exit_tunnel_m,
                result.drift_final_m,
                result.nhc_updates,
                result.nhc_innovation_max.norm_mps,
                anatomy_path,
                (config.nhc_trace_csv_path != NULL) ? config.nhc_trace_csv_path : "");
        }

        if (kArms[i].nhc_policy != SUPER_TUNNEL_NHC_OFF) {
            char summary_path[256];
            std::snprintf(
                summary_path,
                sizeof(summary_path),
                "docs/benchmarks/super_tunnel_bd_rerun/%s_summary.json",
                kArms[i].id);
            (void)write_nhc_summary_json(kArms[i].id, &result, summary_path);
        }
    }

    if (results_fp != NULL) {
        std::fclose(results_fp);
        std::printf("  results=docs/benchmarks/super_tunnel_bd_rerun/results.csv\n");
    }
    std::printf("================================================================\n");
    std::printf("Next: python tools/audit_super_tunnel_bd_rerun.py\n");
    std::printf("================================================================\n");
    return 0;
}
