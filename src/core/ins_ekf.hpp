#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
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

#ifndef NAVICORE_INS_EKF_ACCEL_NOISE_VAR
#define NAVICORE_INS_EKF_ACCEL_NOISE_VAR 0.05f
#endif

#ifndef NAVICORE_INS_EKF_GYRO_NOISE_VAR
#define NAVICORE_INS_EKF_GYRO_NOISE_VAR 0.001f
#endif

#ifndef NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR
#define NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR 1.0e-5f
#endif

#ifndef NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR
#define NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR 1.0e-6f
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

    NavDomain domain;
    bool initialized;
    bool outlier_detected;

    void predict(const ImuSample &imu_sample, float dt_s);
    bool update_gnss(const GpsSample &gps_sample, float *out_nis);
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
uint32_t ins_ekf_gnss_accept_count(const InsEkfFilter *filter);
uint32_t ins_ekf_gnss_reject_count(const InsEkfFilter *filter);
void ins_ekf_export_nav_state(
    const InsEkfFilter *filter,
    NavState *out_state,
    uint32_t timestamp_ms,
    const GpsSample *last_gps);
