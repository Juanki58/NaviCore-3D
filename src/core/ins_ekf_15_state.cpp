#include "ins_ekf_15_state.hpp"

#include "geodesy.hpp"
#include "vector3d.h"

#include <cmath>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float kRadToDegF = 180.0f / static_cast<float>(M_PI);

} /* namespace */

InsEkf15State::InsEkf15State()
    : timestamp_s_(0.0)
    , run_zupt_after_predict_(false)
    , pending_lateral_std_mps_(NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS)
    , pending_vertical_std_mps_(NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS)
{
    std::memset(&ekf_, 0, sizeof(ekf_));
}

void InsEkf15State::sync_timestamp_from_ms(uint32_t t_ms)
{
    timestamp_s_ = static_cast<double>(t_ms) * 0.001;
}

void InsEkf15State::body_velocity_from_ned(float out_body[3]) const
{
    if (out_body == nullptr) {
        return;
    }

    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_velocity_ned(&ekf_, vel_ned);

    InsEkfMat3 dcm_bn{};
    ins_ekf_kinematics_quat_to_dcm_bn(ekf_.q_att_, dcm_bn);
    ins_ekf_kinematics_ned_to_body(dcm_bn, vel_ned, out_body);
}

bool InsEkf15State::is_initialized() const
{
    return ekf_.initialized;
}

const InsEkfFilter &InsEkf15State::native() const
{
    return ekf_;
}

InsEkfFilter &InsEkf15State::native()
{
    return ekf_;
}

void InsEkf15State::set_nhc_measurement_stds(float lateral_std_mps, float vertical_std_mps)
{
    pending_lateral_std_mps_ = lateral_std_mps;
    pending_vertical_std_mps_ = vertical_std_mps;
    ekf_.nhc_lateral_var_m2 = lateral_std_mps * lateral_std_mps;
    ekf_.nhc_vertical_var_m2 = vertical_std_mps * vertical_std_mps;
}

bool InsEkf15State::seed_from_gnss_sample(const GpsSample &gps, NavDomain domain)
{
    if (!gps.fix_valid) {
        return false;
    }

    const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
    ins_ekf_init(&ekf_, gps.position, yaw_rad, domain);
    ins_ekf_set_nhc_enabled(&ekf_, false);

    const float course_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
    ekf_.vel_[0] = gps.speed_mps * std::cos(course_rad);
    ekf_.vel_[1] = gps.speed_mps * std::sin(course_rad);
    ekf_.vel_[2] = 0.0f;

    sync_timestamp_from_ms(gps.timestamp_ms);
    return true;
}

bool InsEkf15State::seed_from_ned_fix(const double pos_ned_m[3], NavDomain domain)
{
    const double barcelona_ref_deg[3] = {41.3874, 2.1686, 12.0};
    return seed_from_ned_fix(pos_ned_m, barcelona_ref_deg, domain);
}

bool InsEkf15State::seed_from_ned_fix(
    const double pos_ned_m[3],
    const double ref_lla_deg[3],
    NavDomain domain)
{
    if (pos_ned_m == nullptr || ref_lla_deg == nullptr) {
        return false;
    }

    const Vector3D ref = vector3d_make(
        static_cast<float>(ref_lla_deg[0]),
        static_cast<float>(ref_lla_deg[1]),
        static_cast<float>(ref_lla_deg[2]));
    ins_ekf_init(&ekf_, ref, 0.0f, domain);
    ins_ekf_set_nhc_enabled(&ekf_, false);

    ekf_.pos_[0] = static_cast<float>(pos_ned_m[0]);
    ekf_.pos_[1] = static_cast<float>(pos_ned_m[1]);
    ekf_.pos_[2] = static_cast<float>(pos_ned_m[2]);
    ekf_.vel_[0] = 0.0f;
    ekf_.vel_[1] = 0.0f;
    ekf_.vel_[2] = 0.0f;

    /* Pos/vel: primera update GNSS ajusta P; actitud sin referencia inicial generosa. */
    ekf_.cov.P[INS_ERR_ATT_X][INS_ERR_ATT_X] = NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2;
    ekf_.cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Y] = NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2;
    ekf_.cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z] = NAVICORE_INS_EKF_INIT_ATT_YAW_VAR_RAD2;

    return true;
}

void InsEkf15State::sync_simulation_clock_ms(uint32_t t_ms)
{
    sync_timestamp_from_ms(t_ms);
}

void InsEkf15State::initialize(const NaviState &initial_state)
{
    if (!ekf_.initialized) {
        return;
    }

    timestamp_s_ = initial_state.timestamp_s;
    ekf_.pos_[0] = static_cast<float>(initial_state.pos_ned[0]);
    ekf_.pos_[1] = static_cast<float>(initial_state.pos_ned[1]);
    ekf_.pos_[2] = static_cast<float>(initial_state.pos_ned[2]);

    ekf_.bias_a_[0] = initial_state.accel_bias[0];
    ekf_.bias_a_[1] = initial_state.accel_bias[1];
    ekf_.bias_a_[2] = initial_state.accel_bias[2];
    ekf_.bias_g_[0] = initial_state.gyro_bias[0];
    ekf_.bias_g_[1] = initial_state.gyro_bias[1];
    ekf_.bias_g_[2] = initial_state.gyro_bias[2];
}

void InsEkf15State::predict(
    double dt_s,
    const float accel_mps2[3],
    const float gyro_rads[3])
{
    if (!ekf_.initialized || accel_mps2 == nullptr || gyro_rads == nullptr) {
        return;
    }

    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = static_cast<uint32_t>(timestamp_s_ * 1000.0);
    imu.accel_mps2[0] = accel_mps2[0];
    imu.accel_mps2[1] = accel_mps2[1];
    imu.accel_mps2[2] = accel_mps2[2];
    imu.gyro_radps[0] = gyro_rads[0];
    imu.gyro_radps[1] = gyro_rads[1];
    imu.gyro_radps[2] = gyro_rads[2];

    (void)ins_ekf_predict(&ekf_, &imu);

    if (run_zupt_after_predict_) {
        const float vel_before_zupt[3] = {
            ekf_.vel_[0],
            ekf_.vel_[1],
            ekf_.vel_[2],
        };
        if (ins_ekf_update_zupt(&ekf_)) {
            ekf_.vel_pipeline_audit_last_.zupt_applied = true;
            for (uint8_t i = 0U; i < 3U; ++i) {
                ekf_.vel_pipeline_audit_last_.vel_after_zupt[i] = ekf_.vel_[i];
                ekf_.vel_pipeline_audit_last_.dv_zupt[i] =
                    ekf_.vel_[i] - vel_before_zupt[i];
            }
        }
        run_zupt_after_predict_ = false;
    }

    if (!ekf_.vel_pipeline_audit_last_.zupt_applied) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            ekf_.vel_pipeline_audit_last_.vel_after_zupt[i] =
                ekf_.vel_pipeline_audit_last_.vel_after_nhc[i];
            ekf_.vel_pipeline_audit_last_.dv_zupt[i] = 0.0f;
        }
    }
}

void InsEkf15State::update_gnss(const double pos_ned_m[3], const float std_dev_m[3])
{
    if (!ekf_.initialized || pos_ned_m == nullptr) {
        return;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    float alt_m = 0.0f;
    geodesy::ned_to_lla(
        ekf_.ref_lat_deg,
        ekf_.ref_lon_deg,
        ekf_.ref_alt_m,
        static_cast<float>(pos_ned_m[0]),
        static_cast<float>(pos_ned_m[1]),
        static_cast<float>(pos_ned_m[2]),
        &lat_deg,
        &lon_deg,
        &alt_m);

    GpsSample gps{};
    gps.fix_valid = true;
    gps.position = vector3d_make(lat_deg, lon_deg, alt_m);
    gps.timestamp_ms = static_cast<uint32_t>(timestamp_s_ * 1000.0);
    gps.satellites = 12U;

    if (std_dev_m != nullptr && std_dev_m[0] > 0.0f) {
        ekf_.gnss_pos_var_m2 = std_dev_m[0] * std_dev_m[0];
    }

    (void)ins_ekf_update_gnss(&ekf_, &gps);
}

void InsEkf15State::update_gnss_with_velocity(
    const double pos_ned_m[3],
    const float std_dev_m[3],
    float speed_mps,
    float course_deg,
    bool has_velocity_obs)
{
    if (!ekf_.initialized || pos_ned_m == nullptr) {
        return;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    float alt_m = 0.0f;
    geodesy::ned_to_lla(
        ekf_.ref_lat_deg,
        ekf_.ref_lon_deg,
        ekf_.ref_alt_m,
        static_cast<float>(pos_ned_m[0]),
        static_cast<float>(pos_ned_m[1]),
        static_cast<float>(pos_ned_m[2]),
        &lat_deg,
        &lon_deg,
        &alt_m);

    GpsSample gps{};
    gps.fix_valid = true;
    gps.position = vector3d_make(lat_deg, lon_deg, alt_m);
    gps.timestamp_ms = static_cast<uint32_t>(timestamp_s_ * 1000.0);
    gps.satellites = 12U;
    gps.speed_mps = has_velocity_obs ? speed_mps : 0.0f;
    gps.course_deg = has_velocity_obs ? course_deg : 0.0f;

    if (std_dev_m != nullptr && std_dev_m[0] > 0.0f) {
        ekf_.gnss_pos_var_m2 = std_dev_m[0] * std_dev_m[0];
    }

    (void)ins_ekf_update_gnss(&ekf_, &gps);
}

bool InsEkf15State::update_gnss_from_sample(const GpsSample &gps)
{
    if (!ekf_.initialized || !gps.fix_valid) {
        return false;
    }

    return ins_ekf_update_gnss(&ekf_, &gps);
}

void InsEkf15State::apply_constraints(
    bool is_stopping,
    float lateral_std_mps,
    float vertical_std_mps)
{
    if (!ekf_.initialized) {
        return;
    }

    pending_lateral_std_mps_ = lateral_std_mps;
    pending_vertical_std_mps_ = vertical_std_mps;
    ekf_.nhc_lateral_var_m2 = lateral_std_mps * lateral_std_mps;
    ekf_.nhc_vertical_var_m2 = vertical_std_mps * vertical_std_mps;

    const bool enable_nhc = (lateral_std_mps > 0.0f);
    ins_ekf_set_nhc_enabled(&ekf_, enable_nhc);
    run_zupt_after_predict_ = is_stopping;
}

NaviState InsEkf15State::get_state() const
{
    NaviState state{};
    state.timestamp_s = timestamp_s_;

    if (!ekf_.initialized) {
        return state;
    }

    float pos_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_position_ned(&ekf_, pos_ned);
    state.pos_ned[0] = static_cast<double>(pos_ned[0]);
    state.pos_ned[1] = static_cast<double>(pos_ned[1]);
    state.pos_ned[2] = static_cast<double>(pos_ned[2]);

    body_velocity_from_ned(state.vel_body);

    ins_ekf_get_attitude_rad(
        &ekf_,
        &state.att_euler[0],
        &state.att_euler[1],
        &state.att_euler[2]);

    ins_ekf_get_bias(&ekf_, state.accel_bias, state.gyro_bias);

    state.cov_pos_diag[0] = ins_ekf_get_covariance_flat(&ekf_, 0U);
    state.cov_pos_diag[1] = ins_ekf_get_covariance_flat(&ekf_, 16U);
    state.cov_pos_diag[2] = ins_ekf_get_covariance_flat(&ekf_, 32U);
    state.cov_att_diag[0] = ins_ekf_get_covariance_flat(&ekf_, 96U);
    state.cov_att_diag[1] = ins_ekf_get_covariance_flat(&ekf_, 112U);
    state.cov_att_diag[2] = ins_ekf_get_covariance_flat(&ekf_, 128U);

    state.nis = ins_ekf_last_nis(&ekf_);
    return state;
}

std::string InsEkf15State::get_filter_name() const
{
    return "InsEkf15State";
}

std::unique_ptr<INaviFilter> create_default_navi_filter()
{
    return std::make_unique<InsEkf15State>();
}

const InsEkfFilter *navi_filter_try_get_ins_ekf(const INaviFilter *filter)
{
    if (filter == nullptr) {
        return nullptr;
    }

    const InsEkf15State *impl = dynamic_cast<const InsEkf15State *>(filter);
    if (impl == nullptr || !impl->is_initialized()) {
        return nullptr;
    }

    return &impl->native();
}

InsEkfFilter *navi_filter_try_get_ins_ekf_mut(INaviFilter *filter)
{
    if (filter == nullptr) {
        return nullptr;
    }

    InsEkf15State *impl = dynamic_cast<InsEkf15State *>(filter);
    if (impl == nullptr || !impl->is_initialized()) {
        return nullptr;
    }

    return &impl->native();
}
