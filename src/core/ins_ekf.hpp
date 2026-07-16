#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
#include "navigation_state.hpp"
#include "sensor_types.hpp"

/*
 * INS / ESKF 15 estados de error — core agnostico de plataforma.
 *
 * Estado nominal (integrado a 100 Hz): pos_, vel_, q_att_, bias_a_, bias_g_.
 * Vector de error delta_x_[15]: media del error; reset a cero tras cada inyeccion GNSS.
 * Covarianza P_[15x15]: incertidumbre del error.
 *
 * Politica: float exclusivo, zero-heap, matrices planas con bucles inline.
 */

#define INS_EKF_STATE_DIM 15U
#define INS_EKF_QUAT_DIM  4U

typedef float InsEkfMat15[INS_EKF_STATE_DIM][INS_EKF_STATE_DIM];
typedef float InsEkfMat3[3][3];
typedef float InsEkfVec3[3];

#ifndef NAVICORE_INS_EKF_DT_S
#define NAVICORE_INS_EKF_DT_S 0.01f
#endif

#ifndef NAVICORE_INS_EKF_GRAVITY_MPS2
#define NAVICORE_INS_EKF_GRAVITY_MPS2 9.80665f
#endif

#ifndef NAVICORE_INS_EKF_NIS_THRESHOLD
#define NAVICORE_INS_EKF_NIS_THRESHOLD 11.345f
#endif

#ifndef NAVICORE_INS_EKF_GNSS_POS_VAR_M2
#define NAVICORE_INS_EKF_GNSS_POS_VAR_M2 6.0f
#endif

#ifndef NAVICORE_INS_EKF_ACCEL_NOISE_STD_MPS2
/* Ruido blanco IMU real (log coche): sigma_a = 0.05 m/s^2 -> Q_vel = sigma_a^2 * dt. */
#define NAVICORE_INS_EKF_ACCEL_NOISE_STD_MPS2 0.05f
#endif

#ifndef NAVICORE_INS_EKF_ACCEL_NOISE_VAR
#define NAVICORE_INS_EKF_ACCEL_NOISE_VAR \
    (NAVICORE_INS_EKF_ACCEL_NOISE_STD_MPS2 * NAVICORE_INS_EKF_ACCEL_NOISE_STD_MPS2)
#endif

#ifndef NAVICORE_INS_EKF_GYRO_NOISE_STD_RADPS
/* Vibracion de soporte: sigma_g = 0.002 rad/s -> Q_att = sigma_g^2 * dt. */
#define NAVICORE_INS_EKF_GYRO_NOISE_STD_RADPS 0.002f
#endif

#ifndef NAVICORE_INS_EKF_GYRO_NOISE_VAR
#define NAVICORE_INS_EKF_GYRO_NOISE_VAR \
    (NAVICORE_INS_EKF_GYRO_NOISE_STD_RADPS * NAVICORE_INS_EKF_GYRO_NOISE_STD_RADPS)
#endif

#ifndef NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2
/* Actitud inicial moderada cuando no hay referencia de inclinacion (0.1 rad). */
#define NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2 0.01f
#endif

#ifndef NAVICORE_INS_EKF_INIT_ATT_YAW_VAR_RAD2
/* Yaw desconocido al arrancar parado sin brujula (0.8 rad ~ 46 deg). */
#define NAVICORE_INS_EKF_INIT_ATT_YAW_VAR_RAD2 0.64f
#endif

#ifndef NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR
/* PSD alineada con IMU sim: (sigma_tick^2) / dt, sigma=1e-4 m/s^2/tick @100Hz */
#define NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR 1.0e-6f
#endif

#ifndef NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR
/* PSD alineada con IMU sim: (sigma_tick^2) / dt, sigma=1e-6 rad/s/tick @100Hz */
#define NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR 1.0e-10f
#endif

#ifndef NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS
/* Log real coche: tolerancia alta a vibracion lateral del soporte movil. */
#define NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS 0.5f
#endif

#ifndef NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS
/* Suspension / pitch: aun mas tolerante que lateral. */
#define NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS 1.0f
#endif

#ifndef NAVICORE_INS_EKF_NHC_EVERY_N_TICKS
#define NAVICORE_INS_EKF_NHC_EVERY_N_TICKS 1U
#endif

#ifndef NAVICORE_INS_EKF_ZUPT_VEL_STD_MPS
#define NAVICORE_INS_EKF_ZUPT_VEL_STD_MPS 0.05f
#endif

#ifndef NAVICORE_INS_EKF_ZUPT_MAX_GAIN
/* Limita ganancia por eje en ZUPT para evitar correccion de un solo tick ~100%. */
#define NAVICORE_INS_EKF_ZUPT_MAX_GAIN 0.85f
#endif

enum InsEkfErrorIdx : uint8_t {
    INS_ERR_POS_N = 0,
    INS_ERR_POS_E = 1,
    INS_ERR_POS_D = 2,
    INS_ERR_VEL_N = 3,
    INS_ERR_VEL_E = 4,
    INS_ERR_VEL_D = 5,
    INS_ERR_ATT_X = 6,
    INS_ERR_ATT_Y = 7,
    INS_ERR_ATT_Z = 8,
    INS_ERR_BIAS_AX = 9,
    INS_ERR_BIAS_AY = 10,
    INS_ERR_BIAS_AZ = 11,
    INS_ERR_BIAS_GX = 12,
    INS_ERR_BIAS_GY = 13,
    INS_ERR_BIAS_GZ = 14,
};

/* Alias retrocompatible con integraciones existentes */
enum InsEkfStateIdx : uint8_t {
    INS_POS_N = INS_ERR_POS_N,
    INS_POS_E = INS_ERR_POS_E,
    INS_POS_D = INS_ERR_POS_D,
    INS_VEL_N = INS_ERR_VEL_N,
    INS_VEL_E = INS_ERR_VEL_E,
    INS_VEL_D = INS_ERR_VEL_D,
    INS_ATT_ROLL = INS_ERR_ATT_X,
    INS_ATT_PITCH = INS_ERR_ATT_Y,
    INS_ATT_YAW = INS_ERR_ATT_Z,
    INS_BIAS_AX = INS_ERR_BIAS_AX,
    INS_BIAS_AY = INS_ERR_BIAS_AY,
    INS_BIAS_AZ = INS_ERR_BIAS_AZ,
    INS_BIAS_GX = INS_ERR_BIAS_GX,
    INS_BIAS_GY = INS_ERR_BIAS_GY,
    INS_BIAS_GZ = INS_ERR_BIAS_GZ,
};

struct InsEkfCovariance {
    float P[INS_EKF_STATE_DIM][INS_EKF_STATE_DIM];
};

typedef struct {
    float dt_s;
    float imu_body_mps2[3];
    float a_corr_mps2[3];
    float a_nav_mps2[3];
    float a_lin_mps2[3];
    float w_corr_radps[3];
    float vel_ned_mps[3];
    float pos_ned_m[3];
    float bias_a_mps2[3];
    float bias_g_radps[3];
    float dcm_bn[3][3];
    bool valid;
} InsEkfPredictAudit;

typedef struct {
    float dt_s;
    float gyro_raw_radps[3];
    float gyro_bias_radps[3];
    float gyro_corr_radps[3];
    float delta_theta_integrated_rad[3];
    float delta_theta_integrated_mag_rad;
    float q_before[4];
    float q_after[4];
    float roll_before_rad;
    float pitch_before_rad;
    float yaw_before_rad;
    float roll_after_rad;
    float pitch_after_rad;
    float yaw_after_rad;
    bool valid;
} InsEkfAttitudePropAudit;

struct InsEkfFilter {
    /* --- Estado nominal absoluto (ESKF) --- */
    float pos_[3];    /* NED local [m] */
    float vel_[3];    /* NED [m/s] */
    float q_att_[4];  /* Cuaternión body->NED [w, x, y, z] */
    float bias_a_[3]; /* Sesgo acelerometro [m/s^2] */
    float bias_g_[3]; /* Sesgo giroscopio [rad/s] */

    /* --- Media del error ESKF (reset a cero tras inyeccion GNSS) --- */
    float delta_x_[INS_EKF_STATE_DIM];

    /* --- Covarianza del error (15x15) --- */
    InsEkfCovariance cov;

    /* --- Scratch 15x15 (Joseph / propagate) — evita ~4 KB en pila --- */
    InsEkfMat15 scratch_a_;
    InsEkfMat15 scratch_b_;

    /* --- Marco de referencia geodesico --- */
    float ref_lat_deg;
    float ref_lon_deg;
    float ref_alt_m;

    /* --- Ruido de proceso / medicion --- */
    float accel_noise_var;
    float gyro_noise_var;
    float bias_accel_rw_var;
    float bias_gyro_rw_var;
    float gnss_pos_var_m2;
    float nis_threshold;

    /* --- Diagnostico GNSS (esqueleto update) --- */
    float gnss_nis_last;
    float gnss_innovation_last[3];
    float gnss_innovation_cov_last[3][3];

    uint32_t last_imu_timestamp_ms;
    uint32_t last_gnss_accept_ms;
    uint32_t gnss_accept_count;
    uint32_t gnss_reject_count;
    uint32_t nhc_update_count;
    uint32_t zupt_update_count;
    uint8_t nhc_tick_counter;
    float nhc_innovation_last[2];
    float nhc_innovation_max_lateral_mps;
    float nhc_innovation_max_vertical_mps;
    float nhc_innovation_max_norm_mps;

    uint32_t nhc_last_update_timestamp_ms;
    float nhc_last_innov_y_mps;
    float nhc_last_innov_z_mps;
    float nhc_last_innov_norm_mps;
    float nhc_last_k_max;
    float nhc_last_dx_vel_norm_mps;
    float nhc_last_dx_att_norm_rad;
    float nhc_last_dx_pos_norm_m;
    float nhc_last_v_body_x_mps;
    float nhc_last_v_body_y_mps;
    float nhc_last_v_body_z_mps;
    float nhc_last_vel_n_mps;
    float nhc_last_vel_e_mps;
    float nhc_last_vel_d_mps;
    float nhc_last_yaw_rad;
    float nhc_last_dx_vel_n_mps;
    float nhc_last_dx_vel_e_mps;
    float nhc_last_dx_vel_d_mps;
    float nhc_last_dx_att_x_rad;
    float nhc_last_dx_att_y_rad;
    float nhc_last_dx_att_z_rad;
    float nhc_last_k_y;
    float nhc_last_k_z;
    float nhc_last_nis;

    double nhc_stat_sum_innov_y;
    double nhc_stat_sum_innov_z;
    double nhc_stat_sum_innov_y_sq;
    double nhc_stat_sum_innov_z_sq;
    double nhc_stat_sum_k_y;
    double nhc_stat_sum_k_z;
    double nhc_stat_sum_nis;
    double nhc_stat_sum_v_body_y;
    double nhc_stat_sum_v_body_z;
    float nhc_stat_max_nis;
    uint32_t nhc_stat_same_sign_count;

    NavDomain domain;
    bool initialized;
    bool outlier_detected;
    bool nhc_enabled;
    float nhc_lateral_var_m2;
    float nhc_vertical_var_m2;
    float zupt_vel_var_m2;

    InsEkfPredictAudit predict_audit_last_;
    InsEkfAttitudePropAudit attitude_prop_audit_last_;

    void predict(const ImuSample &imu_sample, float dt_s);
    bool update_gnss(const GpsSample &gps_sample, float *out_nis);
    bool update_nhc();
    bool update_zupt();
};

NAVICORE_STATIC_ASSERT(sizeof(InsEkfCovariance) == (INS_EKF_STATE_DIM * INS_EKF_STATE_DIM * sizeof(float)),
    "InsEkfCovariance float-only");

void ins_ekf_init(
    InsEkfFilter *filter,
    Vector3D initial_position,
    float initial_yaw_rad,
    NavDomain domain);

bool ins_ekf_predict(InsEkfFilter *filter, const ImuSample *imu);
bool ins_ekf_update_gnss(InsEkfFilter *filter, const GpsSample *gps);
bool ins_ekf_update_nhc(InsEkfFilter *filter);
void ins_ekf_set_nhc_enabled(InsEkfFilter *filter, bool enabled);
bool ins_ekf_nhc_enabled(const InsEkfFilter *filter);
uint32_t ins_ekf_nhc_update_count(const InsEkfFilter *filter);
void ins_ekf_get_nhc_innovation_last(
    const InsEkfFilter *filter,
    float *out_lateral_mps,
    float *out_vertical_mps);
void ins_ekf_get_nhc_innovation_max(
    const InsEkfFilter *filter,
    float *out_lateral_mps,
    float *out_vertical_mps,
    float *out_norm_mps);

typedef struct {
    uint32_t timestamp_ms;
    float innov_y_mps;
    float innov_z_mps;
    float innov_norm_mps;
    float k_max;
    float k_y;
    float k_z;
    float nis;
    float dx_vel_norm_mps;
    float dx_att_norm_rad;
    float dx_pos_norm_m;
    float v_body_x_mps;
    float v_body_y_mps;
    float v_body_z_mps;
    float vel_n_mps;
    float vel_e_mps;
    float vel_d_mps;
    float yaw_rad;
    float dx_vel_n_mps;
    float dx_vel_e_mps;
    float dx_vel_d_mps;
    float dx_att_x_rad;
    float dx_att_y_rad;
    float dx_att_z_rad;
} InsEkfNhcUpdateDetail;

bool ins_ekf_get_nhc_last_update_detail(
    const InsEkfFilter *filter,
    InsEkfNhcUpdateDetail *out_detail);

typedef struct {
    uint32_t sample_count;
    float mean_innov_y_mps;
    float mean_innov_z_mps;
    float std_innov_y_mps;
    float std_innov_z_mps;
    float mean_k_y;
    float mean_k_z;
    float frac_same_sign_corr;
    float mean_nis;
    float max_nis;
    float mean_v_body_y_mps;
    float mean_v_body_z_mps;
} InsEkfNhcRunSummary;

bool ins_ekf_get_nhc_run_summary(
    const InsEkfFilter *filter,
    InsEkfNhcRunSummary *out_summary);

bool ins_ekf_update_zupt(InsEkfFilter *filter);
uint32_t ins_ekf_zupt_update_count(const InsEkfFilter *filter);

bool ins_ekf_outlier_detected(const InsEkfFilter *filter);
void ins_ekf_clear_outlier_flag(InsEkfFilter *filter);
float ins_ekf_last_nis(const InsEkfFilter *filter);
void ins_ekf_get_gnss_innovation(const InsEkfFilter *filter, float out_ned[3]);
void ins_ekf_get_bias(
    const InsEkfFilter *filter,
    float out_accel_bias[3],
    float out_gyro_bias[3]);
void ins_ekf_get_position_ned(const InsEkfFilter *filter, float out_ned[3]);
void ins_ekf_get_velocity_ned(const InsEkfFilter *filter, float out_ned[3]);
float ins_ekf_get_covariance_flat(const InsEkfFilter *filter, uint16_t linear_idx);
void ins_ekf_get_attitude_rad(
    const InsEkfFilter *filter,
    float *roll_rad,
    float *pitch_rad,
    float *yaw_rad);
bool ins_ekf_compute_roll_pitch_from_gravity_body(
    const float accel_body_mps2[3],
    float *roll_rad,
    float *pitch_rad);
bool ins_ekf_apply_gravity_tilt_init(
    InsEkfFilter *filter,
    const float mean_accel_body_mps2[3],
    const float mean_gyro_body_radps[3]);
bool ins_ekf_get_last_predict_audit(
    const InsEkfFilter *filter,
    InsEkfPredictAudit *out_audit);
bool ins_ekf_get_last_attitude_prop_audit(
    const InsEkfFilter *filter,
    InsEkfAttitudePropAudit *out_audit);
uint32_t ins_ekf_gnss_accept_count(const InsEkfFilter *filter);
uint32_t ins_ekf_gnss_reject_count(const InsEkfFilter *filter);
void ins_ekf_export_nav_state(
    const InsEkfFilter *filter,
    NavState *out_state,
    uint32_t timestamp_ms,
    const GpsSample *last_gps);

bool ins_ekf_pack_navigation_state(
    const InsEkfFilter *filter,
    uint32_t timestamp_ms,
    uint32_t health_flags,
    NavigationState *out_state);

/*
 * Acoplamiento actitud-velocidad para NHC (perturbacion derecha q' = q * dq).
 * d(v_body)/d(delta_theta) = +[v_body]x; filas de H para y = [-v_y, -v_z].
 */
void ins_ekf_fill_nhc_attitude_coupling(const float v_body[3], float h_rows_att[2][3]);

void ins_ekf_kinematics_quat_to_dcm_bn(const float q[4], float dcm[3][3]);
void ins_ekf_kinematics_ned_to_body(const float dcm[3][3], const float ned[3], float body[3]);
void ins_ekf_kinematics_quat_apply_small_angle_error(float q[4], const float dtheta[3]);
