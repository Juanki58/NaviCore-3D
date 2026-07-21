#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

#ifndef NAVICORE_INS_EKF_GNSS_VEL_STD_MPS
#define NAVICORE_INS_EKF_GNSS_VEL_STD_MPS 1.5f
#endif

#ifndef NAVICORE_INS_EKF_GNSS_VEL_VAR_M2
#define NAVICORE_INS_EKF_GNSS_VEL_VAR_M2 \
    (NAVICORE_INS_EKF_GNSS_VEL_STD_MPS * NAVICORE_INS_EKF_GNSS_VEL_STD_MPS)
#endif

#ifndef NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF
#define NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF 9.210f /* chi2 2 DoF @ 99% */
#endif

#ifndef NAVICORE_INS_EKF_NIS_THRESHOLD_POS_VEL_5DOF
#define NAVICORE_INS_EKF_NIS_THRESHOLD_POS_VEL_5DOF 15.086f /* chi2 5 DoF @ 99% */
#endif

enum InsEkfGnssObsMode : uint8_t {
    INS_EKF_GNSS_OBS_POS = 0,
    INS_EKF_GNSS_OBS_POS_VEL = 1,
    INS_EKF_GNSS_OBS_VEL_ONLY = 2,
};

enum InsEkfPpvPolicy : uint8_t {
    INS_EKF_PPV_POLICY_NONE = 0,
    INS_EKF_PPV_POLICY_GAP_LE_1S = 1,
    INS_EKF_PPV_POLICY_ZERO = 2,
    INS_EKF_PPV_POLICY_COS_POS = 3,
    INS_EKF_PPV_POLICY_COS_TOT = 4,
    INS_EKF_PPV_POLICY_INNOV_H = 5, /**< Zero P_pv if horizontal pos innov ≥ threshold. */
};

/** NHC attitude H rows — correct vs pre-bf2bfbd sign bug (A/B E2E only). */
enum InsEkfNhcJacobianMode : uint8_t {
    INS_EKF_NHC_JACOBIAN_CORRECT = 0,
    INS_EKF_NHC_JACOBIAN_LEGACY_BUG = 1,
};

#ifndef NAVICORE_INS_EKF_PPV_GAP_THRESHOLD_S
#define NAVICORE_INS_EKF_PPV_GAP_THRESHOLD_S 1.0f
#endif

#ifndef NAVICORE_INS_EKF_PPV_INNOV_H_THRESHOLD_M
#define NAVICORE_INS_EKF_PPV_INNOV_H_THRESHOLD_M 50.0f
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

/* --- Integridad: fix presente pero físicamente inverosímil vs INS (spoof/consistency) --- */
#ifndef NAVICORE_INS_EKF_CONSISTENCY_CHECK_ENABLED
#define NAVICORE_INS_EKF_CONSISTENCY_CHECK_ENABLED 1
#endif

#ifndef NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S
/**
 * Solo sospecha spoof si el último GNSS aceptado/visto fue reciente.
 * Tras outage largo (túnel), |innov| grande es reaquisição legítima → NIS, no reason=3.
 */
#define NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S 2.0f
#endif

#ifndef NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M
/** Tope duro de |innov_h| (m) en track continuo (gap corto). */
#define NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M 120.0f
#endif

#ifndef NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M
/** Margen añadido a |v|·Δt para multipath / timing (m). */
#define NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M 30.0f
#endif

#ifndef NAVICORE_INS_EKF_CONSISTENCY_SIGMA_K
/** Multiplicador de σ_pos horizontal de P (√(Pnn+Pee)). */
#define NAVICORE_INS_EKF_CONSISTENCY_SIGMA_K 6.0f
#endif

#ifndef NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS
/** |v_gps − v_ins| horizontal máximo aceptable (m/s) en track continuo. */
#define NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS 35.0f
#endif

/** gnss_last_reject_reason — aliases of MEAS_REJECT_* (generic integrity taxonomy). */
#include "meas_reject.hpp"
#define INS_EKF_GNSS_REJECT_NIS MEAS_REJECT_NIS
#define INS_EKF_GNSS_REJECT_S_SINGULAR MEAS_REJECT_S_SINGULAR
#define INS_EKF_GNSS_REJECT_INCONSISTENT MEAS_REJECT_INCONSISTENT

#ifndef NAVICORE_INS_EKF_NHC_ATT_Z_FORGET
/* H-ATT-b1/c: fracción de dx_att_z NHC rechazada en aplicación (0 = off). */
#define NAVICORE_INS_EKF_NHC_ATT_Z_FORGET 0.0f
#endif

#ifndef NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE
/* H-ATT-c: umbral Σ|dx_att_z|; <=0 = gate off (b1 ciego si λ>0). */
#define NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE 0.0f
#endif

#ifndef NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_TMAX_S
/* H-ATT-c: solo evaluar disparo para t <= tmax desde primer NHC. */
#define NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_TMAX_S 0.65f
#endif

#ifndef NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GRACE_TICKS
/* §13.22 E1: NHC ticks skipped before cand1 accumulate/evaluate (0 = off). */
#define NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GRACE_TICKS 0U
#endif

#ifndef NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE_NORM
/* §13.22 E2: 1 ⇒ gate thr is κ for sumabs/P[ATT_Z,ATT_Z]. */
#define NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE_NORM 0
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
    /* Bloques no nulos de F en el ultimo predict (discretizacion sparse). */
    float f_dp_dv_dt_s;
    float f_va[3][3];
    float f_vba[3][3];
    /* Identidades internas (solo predict, ANTES de NHC/ZUPT/GNSS). */
    float pos_pre_m[3];
    float vel_pre_mps[3];
    float kinematic_pos_residual_m; /* |p+ − (p− + v+·Δt)| ; integración semi-implícita */
    float vel_body_mps[3];          /* Rᵀ·v_NED al final del predict */
    float body_ned_roundtrip_err_mps; /* |v_NED − R·v_body| */
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float euler_dcm_frob; /* ||R(euler(q)) − R(q)||_F — convenio export vs interno */
    /* Presupuesto Δv (v2; v1 deja ceros). a_lin = R·imu − R·bias − g (+ coriolis=0). */
    float term_R_imu_mps2[3];
    float term_R_neg_bias_mps2[3];
    float term_gravity_mps2[3];
    float term_coriolis_mps2[3];
    bool valid;
} InsEkfPredictAudit;

typedef struct {
    float p_pos_pos_frob;
    float p_vel_vel_frob;
    float p_vel_pos_frob;
    float p_vel_att_frob;
    float p_att_att_frob;
    /* Bloque cruzado actitud × bias_giro (3×3) — candidato H-ATT / P_att-bias_g. */
    float p_att_bias_g_frob;
    float p_att_bias_g_max_abs;
    float p_att_z_bias_gz; /* P[ATT_Z, BIAS_GZ] — eje de la fuga observada */
    /* Bloque P_att interno (ATT_X=roll, ATT_Y=pitch, ATT_Z=yaw). */
    float p_att_xx; /* P[ATT_X,ATT_X] */
    float p_att_yy; /* P[ATT_Y,ATT_Y] */
    float p_att_zz; /* P[ATT_Z,ATT_Z] */
    float p_att_xz; /* P[ATT_X,ATT_Z] roll–yaw */
    float p_att_yz; /* P[ATT_Y,ATT_Z] pitch–yaw */
    float p_att_xy; /* P[ATT_X,ATT_Y] roll–pitch */
    /* Bloque cruzado actitud × velocidad (filas ATT_Y/Z × VEL_NED). */
    float p_att_y_vn;
    float p_att_y_ve;
    float p_att_y_vd;
    float p_att_z_vn;
    float p_att_z_ve;
    float p_att_z_vd;
    float p_bias_g_frob;
    float p_pos_std_n_m;
    float p_pos_std_e_m;
    float p_pos_std_d_m;
    float p_vel_std_n_mps;
    float p_vel_std_e_mps;
    float p_vel_std_d_mps;
    float p_vel_pos_max_abs;
    float p_vv_var_n_m2;
    float p_vv_var_e_m2;
    float p_vv_var_d_m2;
    float p_vv_body_forward_m2;
    float p_vv_body_lateral_m2;
    float p_vv_body_vertical_m2;
    float vel_body_x_mps;
    float vel_body_y_mps;
    float vel_body_z_mps;
} InsEkfCovBlockMetrics;

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

typedef struct {
    bool valid;
    float vel_before_predict[3];
    float dv_predict[3];
    float vel_after_predict[3];
    float dv_nhc[3];
    float vel_after_nhc[3];
    bool nhc_applied;
    float dv_zupt[3];
    float vel_after_zupt[3];
    bool zupt_applied;
} InsEkfVelPipelineAudit;

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
    float gnss_vel_var_m2_h;
    InsEkfGnssObsMode gnss_obs_mode;
    InsEkfPpvPolicy ppv_policy;
    float nis_threshold;

    /* --- Diagnostico GNSS (esqueleto update) --- */
    float gnss_nis_last;
    float gnss_innovation_last[3];
    float gnss_innovation_cov_last[3][3];
    uint8_t gnss_last_n_meas;
    float gnss_innovation_full[5];
    float gnss_nis_contrib[5];
    float gnss_s_diag[5];
    uint32_t gnss_last_update_timestamp_ms;
    uint8_t gnss_last_accepted;
    uint8_t gnss_last_reject_reason;
    /* Integridad: último chequeo INS↔GNSS (spoof-suspect si reason=3). */
    uint8_t gnss_consistency_enabled;
    uint8_t gnss_consistency_last_suspect;
    float gnss_consistency_last_innov_h_m;
    float gnss_consistency_last_plausible_m;
    float gnss_consistency_last_vel_jump_mps;
    /* GNSS v2: pos/vel gates independientes (0 en path v1). */
    uint8_t gnss_v2_accepted_pos;
    uint8_t gnss_v2_accepted_vel;
    float gnss_v2_nis_pos;
    float gnss_v2_nis_vel;
    /** v2 polish: R adaptativo, vel coherente, NHC entre fixes. */
    uint8_t v2_polish;
    float gnss_last_k_pos_max;
    float gnss_last_k_vel_max;
    float gnss_last_k_att_max;
    float gnss_last_dx_pos_norm_m;
    float gnss_last_dx_vel_norm_mps;
    float gnss_last_dx_att_norm_rad;
    float gnss_last_dx_pos_n_m;
    float gnss_last_dx_pos_e_m;
    float gnss_last_dx_pos_d_m;
    float gnss_last_dx_vel_n_mps;
    float gnss_last_dx_vel_e_mps;
    float gnss_last_dx_vel_d_mps;
    float gnss_last_dx_att_x_rad;
    float gnss_last_dx_att_y_rad;
    float gnss_last_dx_att_z_rad;
    float gnss_last_p_vel_pos[3][3];
    float gnss_last_k_vel_pos[3][3];
    float gnss_last_k_pos_pos[3][3];
    float gnss_last_s_inv[3][3];
    float gnss_last_dx_bias_a_norm;
    float gnss_last_dx_bias_g_norm;
    uint8_t gnss_last_ppv_policy;
    uint8_t gnss_last_ppv_triggered;
    float gnss_last_ppv_effective_gap_s;
    float gnss_last_cos_dv_pos_err_pre;
    float gnss_last_cos_dv_tot_err_pre;
    float gnss_last_ppv_frob_pre;
    float gnss_last_ppv_frob_post;

    uint32_t last_imu_timestamp_ms;
    uint32_t last_gnss_accept_ms;
    uint32_t last_gnss_fix_ms;
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
    float nhc_last_hph_yy;
    float nhc_last_hph_yz;
    float nhc_last_hph_zz;
    float nhc_last_s_yy;
    float nhc_last_s_yz;
    float nhc_last_s_zz;
    float nhc_last_s_inv_yy;
    float nhc_last_s_inv_yz;
    float nhc_last_s_inv_zz;
    float nhc_last_k_vel_max;
    float nhc_last_k_pos_max;
    float nhc_last_k_att_max;
    float nhc_last_k_bias_max;
    float nhc_last_k_bias_gz; /* max |K[BIAS_GZ, :]| en el update NHC */
    /* Path split (frozen S): K = P (H_vel+H_att)^T S^{-1}; dx = K y.
     * Exact for NHC (H only vel+att). Used by K_bias R1/R2/R3 autopsy. */
    float nhc_last_dx_bias_gz_via_vel;
    float nhc_last_dx_bias_gz_via_att;
    float nhc_last_k_bias_gz_via_vel; /* max |K_via_vel[BIAS_GZ,:]| */
    float nhc_last_k_bias_gz_via_att; /* max |K_via_att[BIAS_GZ,:]| */
    /* Filas K actitud NHC (2 innov: lat=y, vert=z). Pre-λ; Joseph usa este K. */
    float nhc_last_k_att_y0; /* K[ATT_Y, innov_y] pitch */
    float nhc_last_k_att_y1; /* K[ATT_Y, innov_z] */
    float nhc_last_k_att_z0; /* K[ATT_Z, innov_y] yaw */
    float nhc_last_k_att_z1; /* K[ATT_Z, innov_z] */
    float nhc_last_dx_att_y_via_innov_y; /* K_y0 * y0 */
    float nhc_last_dx_att_y_via_innov_z; /* K_y1 * y1 */
    float nhc_last_dx_att_z_raw; /* pre-λ; audit dx_att_z es post-λ */
    float nhc_last_h_row0_vel[3];
    float nhc_last_h_row1_vel[3];
    float nhc_last_vel_after_n_mps;
    float nhc_last_vel_after_e_mps;
    float nhc_last_vel_after_d_mps;
    float nhc_last_v_body_after_x_mps;
    float nhc_last_v_body_after_y_mps;
    float nhc_last_v_body_after_z_mps;
    float nhc_last_nis_contrib_y;
    float nhc_last_nis_contrib_z;
    float nhc_last_dx_pos_n_m;
    float nhc_last_dx_pos_e_m;
    float nhc_last_dx_pos_d_m;
    float nhc_last_dx_bias_norm;
    float nhc_last_dx_bias_gx;
    float nhc_last_dx_bias_gy;
    float nhc_last_dx_bias_gz;
    InsEkfCovBlockMetrics nhc_last_cov_pre{};
    InsEkfCovBlockMetrics nhc_last_cov_post{};

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
    InsEkfNhcJacobianMode nhc_jacobian_mode;
    /* H-ATT-b1/c: λ ∈ [0,1]; δx[ATT_Z] *= (1-λ) tras K·y, antes de inyectar. */
    float nhc_att_z_forget;
    /* H-ATT-c: gate. thr<=0 ⇒ aplicar λ siempre (b1); thr>0 ⇒ solo tras latch. */
    float nhc_att_z_forget_gate_thr;
    float nhc_att_z_forget_tmax_s;
    float nhc_att_z_sumabs;
    uint32_t nhc_att_z_epoch_ms;
    uint32_t nhc_att_z_gate_nhc_count; /* NHC updates seen while gate_thr>0 */
    uint32_t nhc_att_z_forget_grace_ticks; /* §13.22 E1 */
    bool nhc_att_z_forget_gate_norm; /* §13.22 E2: thr=κ on sumabs/Pzz */
    float nhc_att_z_gate_scale_pzz; /* E2: Pzz congelado al salir de gracia (0=unset) */
    bool nhc_att_z_forget_latched;
    float nhc_att_z_forget_fire_t_s;
    /* H-ATT-d: tras latch, H[*][ATT_Z]=0 antes de S/K/Joseph (no truncar δx). */
    bool nhc_att_z_unobs;
    bool nhc_last_att_z_unobs_active;
    /* Experiment: H[*][ATT_X/Y/Z]=0 always (NHC velocity-only). */
    bool nhc_att_unobs;
    /**
     * Coherence gate: block NHC→attitude until |v|, GNSS-valid and
     * |course−yaw| hold for nhc_att_gate_hold_s; then latch open.
     */
    bool nhc_att_coherence_gate;
    float nhc_att_gate_vmin_mps;
    float nhc_att_gate_yaw_max_rad;
    float nhc_att_gate_hold_s;
    bool nhc_att_gate_gnss_valid;
    bool nhc_att_gate_open;
    float nhc_att_gate_ok_accum_s;
    uint32_t nhc_att_gate_last_imu_ms;
    float nhc_att_gate_open_t_s;
    uint32_t nhc_every_n_ticks;
    float nhc_lateral_var_m2;
    float nhc_vertical_var_m2;
    float zupt_vel_var_m2;

    InsEkfPredictAudit predict_audit_last_;
    InsEkfAttitudePropAudit attitude_prop_audit_last_;

    FILE *cov_step_audit_fp;
    double cov_step_audit_timestamp_s;
    uint64_t cov_step_audit_imu_seq;

    FILE *vel_source_audit_fp;
    double vel_source_audit_timestamp_s;
    uint64_t vel_source_audit_imu_seq;
    float vel_source_audit_gps_speed_mps;

    InsEkfVelPipelineAudit vel_pipeline_audit_last_;

    FILE *nhc_block_audit_fp;
    double nhc_block_audit_timestamp_s;
    uint64_t nhc_block_audit_imu_seq;
    float nhc_block_audit_gps_speed_mps;

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
void ins_ekf_set_gnss_obs_mode(InsEkfFilter *filter, InsEkfGnssObsMode mode);
void ins_ekf_set_consistency_check_enabled(InsEkfFilter *filter, bool enabled);
bool ins_ekf_gnss_consistency_last_suspect(const InsEkfFilter *filter);
void ins_ekf_set_p_pv_policy(InsEkfFilter *filter, InsEkfPpvPolicy policy);
void ins_ekf_set_gnss_vel_var_m2(InsEkfFilter *filter, float var_m2_h);
const char *ins_ekf_gnss_obs_mode_name(InsEkfGnssObsMode mode);
const char *ins_ekf_p_pv_policy_name(InsEkfPpvPolicy policy);
bool ins_ekf_parse_p_pv_policy(const char *text, InsEkfPpvPolicy *out_policy);
bool ins_ekf_update_nhc(InsEkfFilter *filter);
void ins_ekf_set_nhc_enabled(InsEkfFilter *filter, bool enabled);
void ins_ekf_set_nhc_every_n_ticks(InsEkfFilter *filter, uint32_t every_n_ticks);
bool ins_ekf_nhc_enabled(const InsEkfFilter *filter);
void ins_ekf_set_nhc_jacobian_mode(InsEkfFilter *filter, InsEkfNhcJacobianMode mode);
InsEkfNhcJacobianMode ins_ekf_nhc_jacobian_mode(const InsEkfFilter *filter);
const char *ins_ekf_nhc_jacobian_mode_name(InsEkfNhcJacobianMode mode);
bool ins_ekf_parse_nhc_jacobian_mode(const char *text, InsEkfNhcJacobianMode *out_mode);
void ins_ekf_set_default_nhc_jacobian_mode(InsEkfNhcJacobianMode mode);
InsEkfNhcJacobianMode ins_ekf_default_nhc_jacobian_mode(void);
void ins_ekf_set_nhc_att_z_forget(InsEkfFilter *filter, float lambda);
float ins_ekf_nhc_att_z_forget(const InsEkfFilter *filter);
void ins_ekf_set_default_nhc_att_z_forget(float lambda);
float ins_ekf_default_nhc_att_z_forget(void);
void ins_ekf_set_nhc_att_z_forget_gate(InsEkfFilter *filter, float thr_rad, float tmax_s);
void ins_ekf_set_default_nhc_att_z_forget_gate(float thr_rad, float tmax_s);
float ins_ekf_default_nhc_att_z_forget_gate_thr(void);
float ins_ekf_default_nhc_att_z_forget_tmax_s(void);
void ins_ekf_set_nhc_att_z_forget_grace_ticks(InsEkfFilter *filter, uint32_t grace_ticks);
void ins_ekf_set_default_nhc_att_z_forget_grace_ticks(uint32_t grace_ticks);
uint32_t ins_ekf_default_nhc_att_z_forget_grace_ticks(void);
void ins_ekf_set_nhc_att_z_forget_gate_norm(InsEkfFilter *filter, bool enabled);
void ins_ekf_set_default_nhc_att_z_forget_gate_norm(bool enabled);
bool ins_ekf_default_nhc_att_z_forget_gate_norm(void);
bool ins_ekf_nhc_att_z_forget_latched(const InsEkfFilter *filter);
float ins_ekf_nhc_att_z_forget_fire_t_s(const InsEkfFilter *filter);
float ins_ekf_nhc_att_z_sumabs(const InsEkfFilter *filter);
void ins_ekf_set_nhc_att_z_unobs(InsEkfFilter *filter, bool enabled);
bool ins_ekf_nhc_att_z_unobs(const InsEkfFilter *filter);
void ins_ekf_set_default_nhc_att_z_unobs(bool enabled);
bool ins_ekf_default_nhc_att_z_unobs(void);
void ins_ekf_set_nhc_att_unobs(InsEkfFilter *filter, bool enabled);
bool ins_ekf_nhc_att_unobs(const InsEkfFilter *filter);
void ins_ekf_set_nhc_att_coherence_gate(InsEkfFilter *filter, bool enabled);
bool ins_ekf_nhc_att_coherence_gate(const InsEkfFilter *filter);
void ins_ekf_configure_nhc_att_coherence_gate(
    InsEkfFilter *filter,
    float vmin_mps,
    float yaw_max_deg,
    float hold_s);
void ins_ekf_set_nhc_att_gate_gnss_valid(InsEkfFilter *filter, bool valid);
bool ins_ekf_nhc_att_gate_open(const InsEkfFilter *filter);
float ins_ekf_nhc_att_gate_open_t_s(const InsEkfFilter *filter);
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
bool ins_ekf_write_nhc_block_audit_header(FILE *audit_fp);
void ins_ekf_set_nhc_block_audit(InsEkfFilter *filter, FILE *audit_fp);
void ins_ekf_set_nhc_block_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq,
    float gps_speed_mps);
void ins_ekf_log_nhc_block_audit(InsEkfFilter *filter);

typedef struct {
    uint32_t timestamp_ms;
    uint8_t accepted;
    uint8_t reject_reason;
    uint8_t n_meas;
    float innov_n_m;
    float innov_e_m;
    float innov_d_m;
    float innov_vn_mps;
    float innov_ve_mps;
    float nis; /* gate NIS = gnss_nis_last */
    float nis_contrib_n;
    float nis_contrib_e;
    float nis_contrib_d;
    float nis_contrib_vn;
    float nis_contrib_ve;
    float s_nn;
    float s_ee;
    float s_dd;
    float s_vn;
    float s_ve;
    float k_pos_max;
    float k_vel_max;
    float k_att_max;
    float dx_pos_norm_m;
    float dx_vel_norm_mps;
    float dx_att_norm_rad;
    float dx_pos_n_m;
    float dx_pos_e_m;
    float dx_pos_d_m;
    float dx_vel_n_mps;
    float dx_vel_e_mps;
    float dx_vel_d_mps;
    float dx_att_x_rad;
    float dx_att_y_rad;
    float dx_att_z_rad;
    uint8_t ppv_policy;
    uint8_t ppv_triggered;
    float ppv_effective_gap_s;
    float cos_dv_pos_err_pre;
    float cos_dv_tot_err_pre;
    float ppv_frob_pre;
    float ppv_frob_post;
} InsEkfGnssUpdateDetail;

typedef struct {
    float p_vel_pos[3][3];
    float k_vel_pos[3][3];
    float k_pos_pos[3][3];
    float s_inv[3][3];
    float dx_bias_accel_norm;
    float dx_bias_gyro_norm;
} InsEkfGnssKBlockDetail;

bool ins_ekf_get_gnss_last_k_block_detail(
    const InsEkfFilter *filter,
    InsEkfGnssKBlockDetail *out_detail);

bool ins_ekf_get_gnss_last_update_detail(
    const InsEkfFilter *filter,
    InsEkfGnssUpdateDetail *out_detail);

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
bool ins_ekf_get_cov_block_metrics(
    const InsEkfFilter *filter,
    InsEkfCovBlockMetrics *out_metrics);
void ins_ekf_set_cov_step_audit(InsEkfFilter *filter, FILE *audit_fp);
void ins_ekf_set_cov_step_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq);
void ins_ekf_log_cov_step_audit(
    InsEkfFilter *filter,
    const char *update_type,
    const char *phase);
void ins_ekf_set_vel_source_audit(InsEkfFilter *filter, FILE *audit_fp);
void ins_ekf_set_vel_source_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq,
    float gps_speed_mps);
void ins_ekf_log_vel_modification(
    InsEkfFilter *filter,
    const char *source,
    const float dv_ned[3],
    const float h_nhc_row0_vel[3],
    const float h_nhc_row1_vel[3],
    bool log_nhc_h);
bool ins_ekf_get_vel_pipeline_audit(
    const InsEkfFilter *filter,
    InsEkfVelPipelineAudit *out_audit);
void ins_ekf_reset_vel_pipeline_audit(InsEkfFilter *filter);

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
