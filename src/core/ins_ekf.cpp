#include "ins_ekf.hpp"

#include "geodesy.hpp"
#include "ins_ekf_math.hpp"
#include "math_utils.hpp"
#include "nav_mode_policy.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef NAVICORE_INS_EKF_PI_F
#define NAVICORE_INS_EKF_PI_F 3.14159265358979323846f
#endif

namespace {

constexpr uint8_t kDim = INS_EKF_STATE_DIM;
constexpr float kPiF = NAVICORE_INS_EKF_PI_F;
constexpr float kDegToRadF = kPiF / 180.0f;
constexpr float kRadToDegF = 180.0f / kPiF;
constexpr float kGravityNed[3] = {0.0f, 0.0f, NAVICORE_INS_EKF_GRAVITY_MPS2};

float deg_to_rad(float deg)
{
    return deg * kDegToRadF;
}

float rad_to_deg(float rad)
{
    return rad * kRadToDegF;
}

float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void mat_zero(InsEkfMat15 m)
{
    memset(m, 0, sizeof(float) * kDim * kDim);
}

void mat_copy(const InsEkfMat15 src, InsEkfMat15 dst)
{
    memcpy(dst, src, sizeof(float) * kDim * kDim);
}

void mat_symmetrize(InsEkfMat15 m)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r + 1U; c < kDim; ++c) {
            const float avg = 0.5f * (m[r][c] + m[c][r]);
            m[r][c] = avg;
            m[c][r] = avg;
        }
    }
}

void mat15_transpose(const InsEkfMat15 in, InsEkfMat15 out)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            out[r][c] = in[c][r];
        }
    }
}

void mat15_transpose_inplace(InsEkfMat15 m)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r + 1U; c < kDim; ++c) {
            const float tmp = m[r][c];
            m[r][c] = m[c][r];
            m[c][r] = tmp;
        }
    }
}

void mat15_mul(const InsEkfMat15 a, const InsEkfMat15 b, InsEkfMat15 out)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                sum += a[r][k] * b[k][c];
            }
            out[r][c] = sum;
        }
    }
}

/*
 * Joseph: P = (I - KH) P (I - KH)^T + K R K^T
 * scratch_a / scratch_b: buffers 15x15 del filtro (zero extra stack).
 */
void ins_ekf_covariance_joseph_update(
    InsEkfMat15 p_in,
    const float k_gain[INS_EKF_STATE_DIM][3],
    float meas_var_m2,
    InsEkfMat15 p_out,
    InsEkfMat15 scratch_a,
    InsEkfMat15 scratch_b)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            scratch_a[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            scratch_a[r][INS_ERR_POS_N + i] -= k_gain[r][i];
        }
    }

    mat15_mul(scratch_a, p_in, scratch_b);
    mat15_transpose_inplace(scratch_a);
    mat15_mul(scratch_b, scratch_a, p_out);

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r; c < kDim; ++c) {
            float krk_t = 0.0f;
            for (uint8_t i = 0U; i < 3U; ++i) {
                krk_t += k_gain[r][i] * meas_var_m2 * k_gain[c][i];
            }
            p_out[r][c] += krk_t;
            if (r != c) {
                p_out[c][r] = p_out[r][c];
            }
        }
    }

    mat_symmetrize(p_out);
}

void ins_ekf_covariance_joseph_update2(
    InsEkfMat15 p_in,
    const float k_gain[INS_EKF_STATE_DIM][2],
    const float h_rows[2][INS_EKF_STATE_DIM],
    const float meas_var[2],
    InsEkfMat15 p_out,
    InsEkfMat15 scratch_a,
    InsEkfMat15 scratch_b)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            scratch_a[r][c] = (r == c) ? 1.0f : 0.0f;
            for (uint8_t i = 0U; i < 2U; ++i) {
                scratch_a[r][c] -= k_gain[r][i] * h_rows[i][c];
            }
        }
    }

    mat15_mul(scratch_a, p_in, scratch_b);
    mat15_transpose_inplace(scratch_a);
    mat15_mul(scratch_b, scratch_a, p_out);

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r; c < kDim; ++c) {
            float krk_t = 0.0f;
            for (uint8_t i = 0U; i < 2U; ++i) {
                krk_t += k_gain[r][i] * meas_var[i] * k_gain[c][i];
            }
            p_out[r][c] += krk_t;
            if (r != c) {
                p_out[c][r] = p_out[r][c];
            }
        }
    }

    mat_symmetrize(p_out);
}

bool ins_ekf_invert2x2(const float s[2][2], float inv_out[2][2])
{
    return navicore_mat_invert2x2(s, inv_out);
}

void ins_ekf_reset_error_state(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    memset(filter->delta_x_, 0, sizeof(filter->delta_x_));
}

void vec3_sub(const float a[3], const float b[3], float out[3])
{
    out[0] = a[0] - b[0];
    out[1] = a[1] - b[1];
    out[2] = a[2] - b[2];
}

void quat_normalize(float q[4])
{
    navicore_quat_normalize(q);
}

void yaw_rad_to_quat(float yaw_rad, float q[4])
{
    const float half_yaw = 0.5f * yaw_rad;
    q[0] = cosf(half_yaw);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(half_yaw);
}

void euler321_to_quat(float roll_rad, float pitch_rad, float yaw_rad, float q[4])
{
    const float cr = cosf(roll_rad * 0.5f);
    const float sr = sinf(roll_rad * 0.5f);
    const float cp = cosf(pitch_rad * 0.5f);
    const float sp = sinf(pitch_rad * 0.5f);
    const float cy = cosf(yaw_rad * 0.5f);
    const float sy = sinf(yaw_rad * 0.5f);

    q[0] = (cr * cp * cy) + (sr * sp * sy);
    q[1] = (sr * cp * cy) - (cr * sp * sy);
    q[2] = (cr * sp * cy) + (sr * cp * sy);
    q[3] = (cr * cp * sy) - (sr * sp * cy);
    quat_normalize(q);
}

void quat_integrate_first_order(float q[4], const float w_corr[3], float dt_s)
{
    const float half_dt = 0.5f * dt_s;
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];
    const float wx = w_corr[0];
    const float wy = w_corr[1];
    const float wz = w_corr[2];

    q[0] += half_dt * ((-qx * wx) - (qy * wy) - (qz * wz));
    q[1] += half_dt * ((qw * wx) + (qy * wz) - (qz * wy));
    q[2] += half_dt * ((qw * wy) - (qx * wz) + (qz * wx));
    q[3] += half_dt * ((qw * wz) + (qx * wy) - (qy * wx));

    quat_normalize(q);
}

void quat_to_dcm_bn(const float q[4], InsEkfMat3 dcm)
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float qw2 = qw * qw;
    const float qx2 = qx * qx;
    const float qy2 = qy * qy;
    const float qz2 = qz * qz;

    dcm[0][0] = qw2 + qx2 - qy2 - qz2;
    dcm[0][1] = 2.0f * ((qx * qy) - (qw * qz));
    dcm[0][2] = 2.0f * ((qx * qz) + (qw * qy));

    dcm[1][0] = 2.0f * ((qx * qy) + (qw * qz));
    dcm[1][1] = qw2 - qx2 + qy2 - qz2;
    dcm[1][2] = 2.0f * ((qy * qz) - (qw * qx));

    dcm[2][0] = 2.0f * ((qx * qz) - (qw * qy));
    dcm[2][1] = 2.0f * ((qy * qz) + (qw * qx));
    dcm[2][2] = qw2 - qx2 - qy2 + qz2;
}

void quat_to_euler321(const float q[4], float *roll_rad, float *pitch_rad, float *yaw_rad)
{
    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float sin_pitch = 2.0f * ((qw * qy) - (qz * qx));
    float pitch = 0.0f;
    if (sin_pitch >= 1.0f) {
        pitch = kPiF * 0.5f;
    } else if (sin_pitch <= -1.0f) {
        pitch = -kPiF * 0.5f;
    } else {
        pitch = asinf(sin_pitch);
    }

    const float roll = atan2f(
        2.0f * ((qw * qx) + (qy * qz)),
        1.0f - (2.0f * ((qx * qx) + (qy * qy))));
    const float yaw = atan2f(
        2.0f * ((qw * qz) + (qx * qy)),
        1.0f - (2.0f * ((qy * qy) + (qz * qz))));

    if (roll_rad != NULL) {
        *roll_rad = roll;
    }
    if (pitch_rad != NULL) {
        *pitch_rad = pitch;
    }
    if (yaw_rad != NULL) {
        *yaw_rad = yaw;
    }
}

void body_to_ned(const InsEkfMat3 dcm_bn, const float body[3], float ned[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        ned[i] = (dcm_bn[i][0] * body[0])
            + (dcm_bn[i][1] * body[1])
            + (dcm_bn[i][2] * body[2]);
    }
}

void ned_to_body(const InsEkfMat3 dcm_bn, const float ned[3], float body[3])
{
    for (uint8_t i = 0U; i < 3U; ++i) {
        body[i] = (dcm_bn[0][i] * ned[0])
            + (dcm_bn[1][i] * ned[1])
            + (dcm_bn[2][i] * ned[2]);
    }
}

void skew_symmetric(const float v[3], InsEkfMat3 m)
{
    m[0][0] = 0.0f;
    m[0][1] = -v[2];
    m[0][2] = v[1];
    m[1][0] = v[2];
    m[1][1] = 0.0f;
    m[1][2] = -v[0];
    m[2][0] = -v[1];
    m[2][1] = v[0];
    m[2][2] = 0.0f;
}

/*
 * Perturbacion derecha (q' = q * dq): v_body(true) = v_body + [v_body]x * dtheta.
 * NHC observa y = [-v_y, -v_z] => filas de H respecto a delta_theta son -[v_body]x.
 * LEGACY_BUG: signo opuesto (pre-bf2bfbd) — solo para A/B E2E controlado.
 */
void fill_nhc_attitude_coupling_rows(
    const float v_body[3],
    float h_rows_att[2][3],
    InsEkfNhcJacobianMode mode)
{
    if (mode == INS_EKF_NHC_JACOBIAN_LEGACY_BUG) {
        h_rows_att[0][0] = v_body[2];
        h_rows_att[0][1] = 0.0f;
        h_rows_att[0][2] = -v_body[0];

        h_rows_att[1][0] = -v_body[1];
        h_rows_att[1][1] = v_body[0];
        h_rows_att[1][2] = 0.0f;
        return;
    }

    h_rows_att[0][0] = -v_body[2];
    h_rows_att[0][1] = 0.0f;
    h_rows_att[0][2] = v_body[0];

    h_rows_att[1][0] = v_body[1];
    h_rows_att[1][1] = -v_body[0];
    h_rows_att[1][2] = 0.0f;
}

void mat3_mul(const InsEkfMat3 a, const InsEkfMat3 b, InsEkfMat3 out)
{
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < 3U; ++k) {
                sum += a[r][k] * b[k][c];
            }
            out[r][c] = sum;
        }
    }
}

void mat3_scale(const InsEkfMat3 in, float scale, InsEkfMat3 out)
{
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            out[r][c] = in[r][c] * scale;
        }
    }
}

void mat3_add(const InsEkfMat3 a, const InsEkfMat3 b, InsEkfMat3 out)
{
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            out[r][c] = a[r][c] + b[r][c];
        }
    }
}

void mat3_identity(InsEkfMat3 m)
{
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            m[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
}

void quat_apply_small_angle_error(float q[4], const float dtheta[3])
{
    const float dq[4] = {
        1.0f,
        0.5f * dtheta[0],
        0.5f * dtheta[1],
        0.5f * dtheta[2],
    };

    const float qw = q[0];
    const float qx = q[1];
    const float qy = q[2];
    const float qz = q[3];

    const float out[4] = {
        (qw * dq[0]) - (qx * dq[1]) - (qy * dq[2]) - (qz * dq[3]),
        (qw * dq[1]) + (qx * dq[0]) + (qy * dq[3]) - (qz * dq[2]),
        (qw * dq[2]) - (qx * dq[3]) + (qy * dq[0]) + (qz * dq[1]),
        (qw * dq[3]) + (qx * dq[2]) - (qy * dq[1]) + (qz * dq[0]),
    };

    q[0] = out[0];
    q[1] = out[1];
    q[2] = out[2];
    q[3] = out[3];
    quat_normalize(q);
}

void ins_ekf_inject_error_into_nominal(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    float dtheta[3] = {
        filter->delta_x_[INS_ERR_ATT_X],
        filter->delta_x_[INS_ERR_ATT_Y],
        filter->delta_x_[INS_ERR_ATT_Z],
    };

    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->pos_[i] += filter->delta_x_[INS_ERR_POS_N + i];
        filter->vel_[i] += filter->delta_x_[INS_ERR_VEL_N + i];
        filter->bias_a_[i] += filter->delta_x_[INS_ERR_BIAS_AX + i];
        filter->bias_g_[i] += filter->delta_x_[INS_ERR_BIAS_GX + i];
    }

    quat_apply_small_angle_error(filter->q_att_, dtheta);
    ins_ekf_reset_error_state(filter);
}

float f_state_jacobian_entry(
    uint8_t k,
    uint8_t c,
    float dt_s,
    const InsEkfMat3 f_va_t,
    const InsEkfMat3 f_vba_t,
    const InsEkfMat3 f_aa_t,
    const InsEkfMat3 f_bg_t)
{
    if (k == c) {
        return 1.0f;
    }

    if (k >= INS_ERR_VEL_N && k < INS_ERR_VEL_N + 3U && c < INS_ERR_POS_N + 3U) {
        if (k == (INS_ERR_VEL_N + (c - INS_ERR_POS_N))) {
            return dt_s;
        }
    }

    if (k >= INS_ERR_ATT_X && k < INS_ERR_ATT_X + 3U && c >= INS_ERR_VEL_N && c < INS_ERR_VEL_N + 3U) {
        return f_va_t[k - INS_ERR_ATT_X][c - INS_ERR_VEL_N];
    }

    if (k >= INS_ERR_BIAS_AX && k < INS_ERR_BIAS_AX + 3U && c >= INS_ERR_VEL_N && c < INS_ERR_VEL_N + 3U) {
        return f_vba_t[k - INS_ERR_BIAS_AX][c - INS_ERR_VEL_N];
    }

    if (k >= INS_ERR_ATT_X && k < INS_ERR_ATT_X + 3U && c >= INS_ERR_ATT_X && c < INS_ERR_ATT_X + 3U) {
        return f_aa_t[k - INS_ERR_ATT_X][c - INS_ERR_ATT_X];
    }

    if (k >= INS_ERR_BIAS_GX && k < INS_ERR_BIAS_GX + 3U && c >= INS_ERR_ATT_X && c < INS_ERR_ATT_X + 3U) {
        return f_bg_t[k - INS_ERR_BIAS_GX][c - INS_ERR_ATT_X];
    }

    return 0.0f;
}

float ins_ekf_predict_dt_s(const InsEkfFilter *filter, uint32_t timestamp_ms)
{
    if (filter == NULL || filter->last_imu_timestamp_ms == 0U) {
        return NAVICORE_INS_EKF_DT_S;
    }

    if (timestamp_ms <= filter->last_imu_timestamp_ms) {
        return NAVICORE_INS_EKF_DT_S;
    }

    const float dt_s = static_cast<float>(timestamp_ms - filter->last_imu_timestamp_ms) * 0.001f;
    return clampf(dt_s, 0.001f, 0.05f);
}

void ins_ekf_build_process_noise(const InsEkfFilter *filter, float dt_s, InsEkfMat15 q)
{
    mat_zero(q);

    const float pos_q = filter->accel_noise_var * dt_s * dt_s;
    const float vel_q = filter->accel_noise_var * dt_s;
    const float att_q = filter->gyro_noise_var * dt_s;
    /* Bias random walk: Q_b = PSD_rw * dt (Velocity/Angle RW del bias). */
    const float ba_q = filter->bias_accel_rw_var * dt_s;
    const float bg_q = filter->bias_gyro_rw_var * dt_s;

    for (uint8_t i = 0U; i < 3U; ++i) {
        q[INS_ERR_POS_N + i][INS_ERR_POS_N + i] = pos_q;
        q[INS_ERR_VEL_N + i][INS_ERR_VEL_N + i] = vel_q;
        q[INS_ERR_ATT_X + i][INS_ERR_ATT_X + i] = att_q;
        q[INS_ERR_BIAS_AX + i][INS_ERR_BIAS_AX + i] = ba_q;
        q[INS_ERR_BIAS_GX + i][INS_ERR_BIAS_GX + i] = bg_q;
    }
}

void ins_ekf_propagate_covariance_sparse(
    InsEkfMat15 p_in,
    const InsEkfMat3 f_va,
    const InsEkfMat3 f_vba,
    const InsEkfMat3 f_aa,
    const InsEkfMat3 f_bg,
    float dt_s,
    const InsEkfMat15 q,
    InsEkfMat15 p_out,
    InsEkfMat15 temp_scratch)
{
    InsEkfMat3 f_va_t{};
    InsEkfMat3 f_vba_t{};
    InsEkfMat3 f_aa_t{};
    InsEkfMat3 f_bg_t{};

    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            f_va_t[r][c] = f_va[c][r];
            f_vba_t[r][c] = f_vba[c][r];
            f_aa_t[r][c] = f_aa[c][r];
            f_bg_t[r][c] = f_bg[c][r];
        }
    }

    for (uint8_t c = 0U; c < kDim; ++c) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            const uint8_t pos_i = static_cast<uint8_t>(INS_ERR_POS_N + i);
            const uint8_t vel_i = static_cast<uint8_t>(INS_ERR_VEL_N + i);
            const uint8_t att_i = static_cast<uint8_t>(INS_ERR_ATT_X + i);
            const uint8_t ba_i = static_cast<uint8_t>(INS_ERR_BIAS_AX + i);
            const uint8_t bg_i = static_cast<uint8_t>(INS_ERR_BIAS_GX + i);

            temp_scratch[pos_i][c] = p_in[pos_i][c] + (dt_s * p_in[vel_i][c]);

            float vel_row = p_in[vel_i][c];
            for (uint8_t j = 0U; j < 3U; ++j) {
                vel_row += f_va[i][j] * p_in[INS_ERR_ATT_X + j][c];
                vel_row += f_vba[i][j] * p_in[INS_ERR_BIAS_AX + j][c];
            }
            temp_scratch[vel_i][c] = vel_row;

            float att_row = 0.0f;
            for (uint8_t j = 0U; j < 3U; ++j) {
                att_row += f_aa[i][j] * p_in[INS_ERR_ATT_X + j][c];
                att_row += f_bg[i][j] * p_in[INS_ERR_BIAS_GX + j][c];
            }
            temp_scratch[att_i][c] = att_row;

            temp_scratch[ba_i][c] = p_in[ba_i][c];
            temp_scratch[bg_i][c] = p_in[bg_i][c];
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                const float f_kc = f_state_jacobian_entry(k, c, dt_s, f_va_t, f_vba_t, f_aa_t, f_bg_t);
                sum += temp_scratch[r][k] * f_kc;
            }
            p_out[r][c] = sum + q[r][c];
        }
    }

    mat_symmetrize(p_out);
}

bool ins_ekf_invert3x3(const InsEkfMat3 s, InsEkfMat3 inv_out)
{
    return navicore_mat_invert3x3(s, inv_out);
}

} /* namespace */

void InsEkfFilter::predict(const ImuSample &imu_sample, float dt_s)
{
    if (!initialized || !imu_sample.valid || dt_s <= 0.0f) {
        return;
    }

    ins_ekf_log_cov_step_audit(this, "predict", "pre");

    predict_audit_last_.valid = false;
    predict_audit_last_.dt_s = dt_s;
    predict_audit_last_.imu_body_mps2[0] = imu_sample.accel_mps2[0];
    predict_audit_last_.imu_body_mps2[1] = imu_sample.accel_mps2[1];
    predict_audit_last_.imu_body_mps2[2] = imu_sample.accel_mps2[2];
    predict_audit_last_.pos_pre_m[0] = pos_[0];
    predict_audit_last_.pos_pre_m[1] = pos_[1];
    predict_audit_last_.pos_pre_m[2] = pos_[2];
    predict_audit_last_.vel_pre_mps[0] = vel_[0];
    predict_audit_last_.vel_pre_mps[1] = vel_[1];
    predict_audit_last_.vel_pre_mps[2] = vel_[2];
    predict_audit_last_.kinematic_pos_residual_m = 0.0f;
    predict_audit_last_.body_ned_roundtrip_err_mps = 0.0f;
    predict_audit_last_.euler_dcm_frob = 0.0f;
    predict_audit_last_.roll_rad = 0.0f;
    predict_audit_last_.pitch_rad = 0.0f;
    predict_audit_last_.yaw_rad = 0.0f;
    for (uint8_t i = 0U; i < 3U; ++i) {
        predict_audit_last_.vel_body_mps[i] = 0.0f;
    }

    float w_corr[3]{};
    float a_corr[3]{};
    vec3_sub(imu_sample.gyro_radps, bias_g_, w_corr);
    vec3_sub(imu_sample.accel_mps2, bias_a_, a_corr);
    predict_audit_last_.w_corr_radps[0] = w_corr[0];
    predict_audit_last_.w_corr_radps[1] = w_corr[1];
    predict_audit_last_.w_corr_radps[2] = w_corr[2];
    predict_audit_last_.a_corr_mps2[0] = a_corr[0];
    predict_audit_last_.a_corr_mps2[1] = a_corr[1];
    predict_audit_last_.a_corr_mps2[2] = a_corr[2];
    predict_audit_last_.bias_a_mps2[0] = bias_a_[0];
    predict_audit_last_.bias_a_mps2[1] = bias_a_[1];
    predict_audit_last_.bias_a_mps2[2] = bias_a_[2];
    predict_audit_last_.bias_g_radps[0] = bias_g_[0];
    predict_audit_last_.bias_g_radps[1] = bias_g_[1];
    predict_audit_last_.bias_g_radps[2] = bias_g_[2];

    attitude_prop_audit_last_.valid = false;
    attitude_prop_audit_last_.dt_s = dt_s;
    attitude_prop_audit_last_.gyro_raw_radps[0] = imu_sample.gyro_radps[0];
    attitude_prop_audit_last_.gyro_raw_radps[1] = imu_sample.gyro_radps[1];
    attitude_prop_audit_last_.gyro_raw_radps[2] = imu_sample.gyro_radps[2];
    attitude_prop_audit_last_.gyro_bias_radps[0] = bias_g_[0];
    attitude_prop_audit_last_.gyro_bias_radps[1] = bias_g_[1];
    attitude_prop_audit_last_.gyro_bias_radps[2] = bias_g_[2];
    attitude_prop_audit_last_.gyro_corr_radps[0] = w_corr[0];
    attitude_prop_audit_last_.gyro_corr_radps[1] = w_corr[1];
    attitude_prop_audit_last_.gyro_corr_radps[2] = w_corr[2];
    attitude_prop_audit_last_.delta_theta_integrated_rad[0] = w_corr[0] * dt_s;
    attitude_prop_audit_last_.delta_theta_integrated_rad[1] = w_corr[1] * dt_s;
    attitude_prop_audit_last_.delta_theta_integrated_rad[2] = w_corr[2] * dt_s;
    attitude_prop_audit_last_.delta_theta_integrated_mag_rad = std::sqrt(
        (attitude_prop_audit_last_.delta_theta_integrated_rad[0]
         * attitude_prop_audit_last_.delta_theta_integrated_rad[0])
        + (attitude_prop_audit_last_.delta_theta_integrated_rad[1]
           * attitude_prop_audit_last_.delta_theta_integrated_rad[1])
        + (attitude_prop_audit_last_.delta_theta_integrated_rad[2]
           * attitude_prop_audit_last_.delta_theta_integrated_rad[2]));

    attitude_prop_audit_last_.q_before[0] = q_att_[0];
    attitude_prop_audit_last_.q_before[1] = q_att_[1];
    attitude_prop_audit_last_.q_before[2] = q_att_[2];
    attitude_prop_audit_last_.q_before[3] = q_att_[3];
    quat_to_euler321(
        attitude_prop_audit_last_.q_before,
        &attitude_prop_audit_last_.roll_before_rad,
        &attitude_prop_audit_last_.pitch_before_rad,
        &attitude_prop_audit_last_.yaw_before_rad);

    quat_integrate_first_order(q_att_, w_corr, dt_s);

    attitude_prop_audit_last_.q_after[0] = q_att_[0];
    attitude_prop_audit_last_.q_after[1] = q_att_[1];
    attitude_prop_audit_last_.q_after[2] = q_att_[2];
    attitude_prop_audit_last_.q_after[3] = q_att_[3];
    quat_to_euler321(
        attitude_prop_audit_last_.q_after,
        &attitude_prop_audit_last_.roll_after_rad,
        &attitude_prop_audit_last_.pitch_after_rad,
        &attitude_prop_audit_last_.yaw_after_rad);
    attitude_prop_audit_last_.valid = true;

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(q_att_, dcm_bn);
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            predict_audit_last_.dcm_bn[r][c] = dcm_bn[r][c];
        }
    }

    float a_n[3]{};
    body_to_ned(dcm_bn, a_corr, a_n);
    predict_audit_last_.a_nav_mps2[0] = a_n[0];
    predict_audit_last_.a_nav_mps2[1] = a_n[1];
    predict_audit_last_.a_nav_mps2[2] = a_n[2];
    a_n[0] -= kGravityNed[0];
    a_n[1] -= kGravityNed[1];
    a_n[2] -= kGravityNed[2];
    predict_audit_last_.a_lin_mps2[0] = a_n[0];
    predict_audit_last_.a_lin_mps2[1] = a_n[1];
    predict_audit_last_.a_lin_mps2[2] = a_n[2];

    for (uint8_t i = 0U; i < 3U; ++i) {
        const float dv = a_n[i] * dt_s;
        vel_[i] += dv;
        pos_[i] += vel_[i] * dt_s;
    }
    {
        const float dv_log[3] = {
            a_n[0] * dt_s,
            a_n[1] * dt_s,
            a_n[2] * dt_s,
        };
        ins_ekf_log_vel_modification(this, "predict", dv_log, NULL, NULL, false);
    }
    predict_audit_last_.vel_ned_mps[0] = vel_[0];
    predict_audit_last_.vel_ned_mps[1] = vel_[1];
    predict_audit_last_.vel_ned_mps[2] = vel_[2];
    predict_audit_last_.pos_ned_m[0] = pos_[0];
    predict_audit_last_.pos_ned_m[1] = pos_[1];
    predict_audit_last_.pos_ned_m[2] = pos_[2];

    /* I1: p+ = p− + v+·Δt (misma regla que el integrador; residual debe ser ~0). */
    {
        float kin_sq = 0.0f;
        for (uint8_t i = 0U; i < 3U; ++i) {
            const float expected =
                predict_audit_last_.pos_pre_m[i] + (vel_[i] * dt_s);
            const float e = pos_[i] - expected;
            kin_sq += e * e;
        }
        predict_audit_last_.kinematic_pos_residual_m = sqrtf(kin_sq);
    }

    /* I2: v_body = Rᵀ·v_NED y round-trip R·v_body ≈ v_NED. */
    {
        float v_body[3]{};
        ned_to_body(dcm_bn, vel_, v_body);
        predict_audit_last_.vel_body_mps[0] = v_body[0];
        predict_audit_last_.vel_body_mps[1] = v_body[1];
        predict_audit_last_.vel_body_mps[2] = v_body[2];
        float v_back[3]{};
        body_to_ned(dcm_bn, v_body, v_back);
        float rt_sq = 0.0f;
        for (uint8_t i = 0U; i < 3U; ++i) {
            const float e = vel_[i] - v_back[i];
            rt_sq += e * e;
        }
        predict_audit_last_.body_ned_roundtrip_err_mps = sqrtf(rt_sq);
    }

    /* I3: R(euler(q)) vs R(q) — el yaw/roll/pitch exportable representa el mismo DCM. */
    {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        quat_to_euler321(q_att_, &roll, &pitch, &yaw);
        predict_audit_last_.roll_rad = roll;
        predict_audit_last_.pitch_rad = pitch;
        predict_audit_last_.yaw_rad = yaw;
        float q_from_euler[4]{};
        euler321_to_quat(roll, pitch, yaw, q_from_euler);
        InsEkfMat3 dcm_from_euler{};
        quat_to_dcm_bn(q_from_euler, dcm_from_euler);
        float frob_sq = 0.0f;
        for (uint8_t r = 0U; r < 3U; ++r) {
            for (uint8_t c = 0U; c < 3U; ++c) {
                const float e = dcm_bn[r][c] - dcm_from_euler[r][c];
                frob_sq += e * e;
            }
        }
        predict_audit_last_.euler_dcm_frob = sqrtf(frob_sq);
    }

    predict_audit_last_.valid = true;

    InsEkfMat3 accel_skew{};
    skew_symmetric(a_corr, accel_skew);

    InsEkfMat3 f_va_raw{};
    mat3_mul(dcm_bn, accel_skew, f_va_raw);
    InsEkfMat3 f_va{};
    mat3_scale(f_va_raw, -dt_s, f_va);

    InsEkfMat3 f_vba{};
    mat3_scale(dcm_bn, -dt_s, f_vba);

    InsEkfMat3 w_skew{};
    skew_symmetric(w_corr, w_skew);
    InsEkfMat3 f_aa{};
    mat3_identity(f_aa);
    InsEkfMat3 w_dt{};
    mat3_scale(w_skew, -dt_s, w_dt);
    mat3_add(f_aa, w_dt, f_aa);

    InsEkfMat3 f_bg{};
    mat3_identity(f_bg);
    for (uint8_t i = 0U; i < 3U; ++i) {
        f_bg[i][i] = -dt_s;
    }

    ins_ekf_build_process_noise(this, dt_s, scratch_a_);
    ins_ekf_propagate_covariance_sparse(
        cov.P,
        f_va,
        f_vba,
        f_aa,
        f_bg,
        dt_s,
        scratch_a_,
        cov.P,
        scratch_b_);

    predict_audit_last_.f_dp_dv_dt_s = dt_s;
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            predict_audit_last_.f_va[r][c] = f_va[r][c];
            predict_audit_last_.f_vba[r][c] = f_vba[r][c];
        }
    }

    ins_ekf_log_cov_step_audit(this, "predict", "post");

    last_imu_timestamp_ms = imu_sample.timestamp_ms;
}

static void gnss_store_delta_x_audit(InsEkfFilter *filter, const float delta_x[INS_EKF_STATE_DIM])
{
    if (filter == NULL || delta_x == NULL) {
        return;
    }

    float dx_pos_sq = 0.0f;
    float dx_vel_sq = 0.0f;
    float dx_att_sq = 0.0f;
    float dx_ba_sq = 0.0f;
    float dx_bg_sq = 0.0f;
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float dp = delta_x[INS_ERR_POS_N + i];
        const float dv = delta_x[INS_ERR_VEL_N + i];
        const float da = delta_x[INS_ERR_ATT_X + i];
        const float dba = delta_x[INS_ERR_BIAS_AX + i];
        const float dbg = delta_x[INS_ERR_BIAS_GX + i];
        dx_pos_sq += dp * dp;
        dx_vel_sq += dv * dv;
        dx_att_sq += da * da;
        dx_ba_sq += dba * dba;
        dx_bg_sq += dbg * dbg;
        if (i == 0U) {
            filter->gnss_last_dx_pos_n_m = dp;
            filter->gnss_last_dx_vel_n_mps = dv;
            filter->gnss_last_dx_att_x_rad = da;
        } else if (i == 1U) {
            filter->gnss_last_dx_pos_e_m = dp;
            filter->gnss_last_dx_vel_e_mps = dv;
            filter->gnss_last_dx_att_y_rad = da;
        } else {
            filter->gnss_last_dx_pos_d_m = dp;
            filter->gnss_last_dx_vel_d_mps = dv;
            filter->gnss_last_dx_att_z_rad = da;
        }
    }

    filter->gnss_last_dx_pos_norm_m = std::sqrt(dx_pos_sq);
    filter->gnss_last_dx_vel_norm_mps = std::sqrt(dx_vel_sq);
    filter->gnss_last_dx_att_norm_rad = std::sqrt(dx_att_sq);
    filter->gnss_last_dx_bias_a_norm = std::sqrt(dx_ba_sq);
    filter->gnss_last_dx_bias_g_norm = std::sqrt(dx_bg_sq);
}

static void gnss_store_k_audit_n(
    InsEkfFilter *filter,
    const float k_gain[INS_EKF_STATE_DIM][5],
    uint8_t n_meas)
{
    if (filter == NULL || k_gain == NULL || n_meas == 0U) {
        return;
    }

    float k_pos = 0.0f;
    float k_vel = 0.0f;
    float k_att = 0.0f;
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < n_meas; ++c) {
            const float k_abs = std::fabs(k_gain[r][c]);
            if (r < INS_ERR_POS_N + 3U) {
                k_pos = fmaxf(k_pos, k_abs);
            } else if (r >= INS_ERR_VEL_N && r < INS_ERR_VEL_N + 3U) {
                k_vel = fmaxf(k_vel, k_abs);
            } else if (r >= INS_ERR_ATT_X && r < INS_ERR_ATT_X + 3U) {
                k_att = fmaxf(k_att, k_abs);
            }
        }
    }

    filter->gnss_last_k_pos_max = k_pos;
    filter->gnss_last_k_vel_max = k_vel;
    filter->gnss_last_k_att_max = k_att;
}

static bool invert_matrix_n(
    const float a[5][5],
    float inv_out[5][5],
    uint8_t n)
{
    if (n == 0U || n > 5U) {
        return false;
    }

    float aug[5][10]{};
    for (uint8_t i = 0U; i < n; ++i) {
        for (uint8_t j = 0U; j < n; ++j) {
            aug[i][j] = a[i][j];
        }
        aug[i][n + i] = 1.0f;
    }

    for (uint8_t col = 0U; col < n; ++col) {
        uint8_t pivot = col;
        float max_abs = std::fabs(aug[col][col]);
        for (uint8_t r = col + 1U; r < n; ++r) {
            const float v = std::fabs(aug[r][col]);
            if (v > max_abs) {
                max_abs = v;
                pivot = r;
            }
        }
        if (max_abs <= 1.0e-12f) {
            return false;
        }
        if (pivot != col) {
            for (uint8_t c = 0U; c < 2U * n; ++c) {
                const float tmp = aug[col][c];
                aug[col][c] = aug[pivot][c];
                aug[pivot][c] = tmp;
            }
        }

        const float diag = aug[col][col];
        for (uint8_t c = 0U; c < 2U * n; ++c) {
            aug[col][c] /= diag;
        }

        for (uint8_t r = 0U; r < n; ++r) {
            if (r == col) {
                continue;
            }
            const float factor = aug[r][col];
            if (factor == 0.0f) {
                continue;
            }
            for (uint8_t c = 0U; c < 2U * n; ++c) {
                aug[r][c] -= factor * aug[col][c];
            }
        }
    }

    for (uint8_t i = 0U; i < n; ++i) {
        for (uint8_t j = 0U; j < n; ++j) {
            inv_out[i][j] = aug[i][n + j];
        }
    }
    return true;
}

static void gnss_build_s_matrix(
    const InsEkfFilter *filter,
    const float h_rows[5][INS_EKF_STATE_DIM],
    const float meas_var[5],
    uint8_t n_meas,
    float s_out[5][5])
{
    for (uint8_t i = 0U; i < n_meas; ++i) {
        for (uint8_t j = 0U; j < n_meas; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                for (uint8_t l = 0U; l < kDim; ++l) {
                    sum += h_rows[i][k] * filter->cov.P[k][l] * h_rows[j][l];
                }
            }
            s_out[i][j] = sum;
        }
        s_out[i][i] += meas_var[i];
    }
}

static void gnss_joseph_update_n(
    InsEkfMat15 p_in,
    const float k_gain[INS_EKF_STATE_DIM][5],
    const float h_rows[5][INS_EKF_STATE_DIM],
    const float meas_var[5],
    uint8_t n_meas,
    InsEkfMat15 p_out,
    InsEkfMat15 scratch_a,
    InsEkfMat15 scratch_b)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            scratch_a[r][c] = (r == c) ? 1.0f : 0.0f;
            for (uint8_t i = 0U; i < n_meas; ++i) {
                scratch_a[r][c] -= k_gain[r][i] * h_rows[i][c];
            }
        }
    }

    mat15_mul(scratch_a, p_in, scratch_b);
    mat15_transpose_inplace(scratch_a);
    mat15_mul(scratch_b, scratch_a, p_out);

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r; c < kDim; ++c) {
            float krk_t = 0.0f;
            for (uint8_t i = 0U; i < n_meas; ++i) {
                krk_t += k_gain[r][i] * meas_var[i] * k_gain[c][i];
            }
            p_out[r][c] += krk_t;
            if (r != c) {
                p_out[c][r] = p_out[r][c];
            }
        }
    }

    mat_symmetrize(p_out);
}

static void gnss_store_k_audit(
    InsEkfFilter *filter,
    const float k_gain[INS_EKF_STATE_DIM][3])
{
    float k5[INS_EKF_STATE_DIM][5]{};
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            k5[r][c] = k_gain[r][c];
        }
    }
    gnss_store_k_audit_n(filter, k5, 3U);
}

static void gnss_clear_correction_audit(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->gnss_last_k_pos_max = 0.0f;
    filter->gnss_last_k_vel_max = 0.0f;
    filter->gnss_last_k_att_max = 0.0f;
    const float zeros[INS_EKF_STATE_DIM]{};
    gnss_store_delta_x_audit(filter, zeros);
}

static void ins_ekf_zero_pv_cross(InsEkfCovariance *cov)
{
    if (cov == NULL) {
        return;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            cov->P[INS_ERR_VEL_N + i][INS_ERR_POS_N + j] = 0.0f;
            cov->P[INS_ERR_POS_N + j][INS_ERR_VEL_N + i] = 0.0f;
        }
    }
}

static float ins_ekf_effective_gnss_gap_s(const InsEkfFilter *filter, uint32_t timestamp_ms)
{
    if (filter == NULL) {
        return -1.0f;
    }

    float gap_accept_s = -1.0f;
    if (filter->last_gnss_accept_ms > 0U && timestamp_ms >= filter->last_gnss_accept_ms) {
        gap_accept_s =
            static_cast<float>(timestamp_ms - filter->last_gnss_accept_ms) * 0.001f;
    }

    if (gap_accept_s >= 0.0f) {
        return gap_accept_s;
    }

    if (filter->last_gnss_fix_ms > 0U && timestamp_ms >= filter->last_gnss_fix_ms) {
        return static_cast<float>(timestamp_ms - filter->last_gnss_fix_ms) * 0.001f;
    }

    return -1.0f;
}

static float ins_ekf_cos_alignment_2d(
    float dv_n,
    float dv_e,
    float err_n,
    float err_e)
{
    const float dn = std::hypot(dv_n, dv_e);
    const float en = std::hypot(err_n, err_e);
    if (dn < 1e-9f || en < 1e-9f) {
        return 0.0f;
    }
    return (dv_n * err_n + dv_e * err_e) / (dn * en);
}

static bool gnss_compute_kalman_update(
    InsEkfFilter *filter,
    const float h_rows[5U][INS_EKF_STATE_DIM],
    const float y[5U],
    const float meas_var[5U],
    uint8_t n_meas,
    float s_mat[5U][5U],
    float s_inv[5U][5U],
    float k_gain[INS_EKF_STATE_DIM][5U],
    float delta_x[INS_EKF_STATE_DIM])
{
    if (filter == NULL || n_meas == 0U) {
        return false;
    }

    gnss_build_s_matrix(filter, h_rows, meas_var, n_meas, s_mat);
    if (!invert_matrix_n(s_mat, s_inv, n_meas)) {
        return false;
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        float ph_t[5U]{};
        for (uint8_t c = 0U; c < n_meas; ++c) {
            for (uint8_t k = 0U; k < kDim; ++k) {
                ph_t[c] += filter->cov.P[r][k] * h_rows[c][k];
            }
        }
        for (uint8_t c = 0U; c < n_meas; ++c) {
            float k_rc = 0.0f;
            for (uint8_t j = 0U; j < n_meas; ++j) {
                k_rc += ph_t[j] * s_inv[j][c];
            }
            k_gain[r][c] = k_rc;
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        float correction = 0.0f;
        for (uint8_t c = 0U; c < n_meas; ++c) {
            correction += k_gain[r][c] * y[c];
        }
        delta_x[r] = correction;
    }

    return true;
}

static void ins_ekf_capture_ppv_frob_pre(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    InsEkfCovBlockMetrics metrics{};
    if (ins_ekf_get_cov_block_metrics(filter, &metrics)) {
        filter->gnss_last_ppv_frob_pre = metrics.p_vel_pos_frob;
    }
}

static void ins_ekf_capture_ppv_frob_post(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    InsEkfCovBlockMetrics metrics{};
    if (ins_ekf_get_cov_block_metrics(filter, &metrics)) {
        filter->gnss_last_ppv_frob_post = metrics.p_vel_pos_frob;
    }
}

static bool ins_ekf_ppv_policy_triggered(
    InsEkfFilter *filter,
    const float k_gain[INS_EKF_STATE_DIM][5U],
    const float y[5U],
    uint8_t n_meas,
    float err_pre_n,
    float err_pre_e,
    float effective_gap_s,
    const float delta_x[INS_EKF_STATE_DIM],
    bool zero_before_k)
{
    if (filter == NULL) {
        return false;
    }

    filter->gnss_last_ppv_policy = static_cast<uint8_t>(filter->ppv_policy);
    filter->gnss_last_ppv_effective_gap_s = effective_gap_s;
    filter->gnss_last_cos_dv_pos_err_pre = 0.0f;
    filter->gnss_last_cos_dv_tot_err_pre = 0.0f;

    if (n_meas >= 5U) {
        float dv_pos_n = 0.0f;
        float dv_pos_e = 0.0f;
        for (uint8_t j = 0U; j < 3U; ++j) {
            dv_pos_n += k_gain[INS_ERR_VEL_N][j] * y[j];
            dv_pos_e += k_gain[INS_ERR_VEL_E][j] * y[j];
        }
        filter->gnss_last_cos_dv_pos_err_pre = ins_ekf_cos_alignment_2d(
            dv_pos_n,
            dv_pos_e,
            err_pre_n,
            err_pre_e);
        filter->gnss_last_cos_dv_tot_err_pre = ins_ekf_cos_alignment_2d(
            delta_x[INS_ERR_VEL_N],
            delta_x[INS_ERR_VEL_E],
            err_pre_n,
            err_pre_e);
    }

    if (zero_before_k) {
        return false;
    }

    bool triggered = false;
    switch (filter->ppv_policy) {
    case INS_EKF_PPV_POLICY_NONE:
        triggered = false;
        break;
    case INS_EKF_PPV_POLICY_GAP_LE_1S:
        triggered = effective_gap_s >= 0.0f
            && effective_gap_s <= NAVICORE_INS_EKF_PPV_GAP_THRESHOLD_S;
        break;
    case INS_EKF_PPV_POLICY_ZERO:
        triggered = true;
        break;
    case INS_EKF_PPV_POLICY_COS_POS:
        triggered = filter->gnss_last_cos_dv_pos_err_pre > 0.0f;
        break;
    case INS_EKF_PPV_POLICY_COS_TOT:
        triggered = filter->gnss_last_cos_dv_tot_err_pre > 0.0f;
        break;
    case INS_EKF_PPV_POLICY_INNOV_H:
        /* Handled as zero_before_k in update_gnss (innov known pre-K). */
        triggered = false;
        break;
    default:
        break;
    }

    filter->gnss_last_ppv_triggered = triggered ? 1U : 0U;
    return triggered;
}

/**
 * Physical consistency: fix may still be valid/present but incompatible with INS.
 * Reject reason 3. Only on short GNSS gaps (continuous track); long outages → reacquire via NIS.
 * Validate spoof only via SW injection — never RF without licence.
 */
static bool ins_ekf_gnss_fails_physical_consistency(
    InsEkfFilter *filter,
    const float y_pos_ned[3],
    bool has_pos,
    float v_gps_n,
    float v_gps_e,
    bool has_vel,
    float effective_gap_s)
{
    if (filter == NULL || filter->gnss_consistency_enabled == 0U) {
        return false;
    }

    filter->gnss_consistency_last_suspect = 0U;
    filter->gnss_consistency_last_innov_h_m = 0.0f;
    filter->gnss_consistency_last_plausible_m = 0.0f;
    filter->gnss_consistency_last_vel_jump_mps = 0.0f;

    /* Known long gap: allow large innov (tunnel exit). Unknown gap (-1): treat as short. */
    if (effective_gap_s > NAVICORE_INS_EKF_CONSISTENCY_MAX_GAP_S) {
        return false;
    }

    const float gap_s =
        (effective_gap_s > 0.0f) ? effective_gap_s : NAVICORE_INS_EKF_DT_S;
    const float v_h =
        sqrtf((filter->vel_[0] * filter->vel_[0]) + (filter->vel_[1] * filter->vel_[1]));
    const float p_nn = filter->cov.P[INS_ERR_POS_N][INS_ERR_POS_N];
    const float p_ee = filter->cov.P[INS_ERR_POS_E][INS_ERR_POS_E];
    const float sigma_h = sqrtf(fmaxf(0.0f, p_nn) + fmaxf(0.0f, p_ee));

    if (has_pos && y_pos_ned != NULL) {
        const float innov_h =
            sqrtf((y_pos_ned[0] * y_pos_ned[0]) + (y_pos_ned[1] * y_pos_ned[1]));
        const float plausible =
            (v_h * gap_s)
            + NAVICORE_INS_EKF_CONSISTENCY_POS_MARGIN_M
            + (NAVICORE_INS_EKF_CONSISTENCY_SIGMA_K * sigma_h);
        const float gate = fminf(
            NAVICORE_INS_EKF_CONSISTENCY_MAX_POS_JUMP_M,
            fmaxf(plausible, 40.0f));
        filter->gnss_consistency_last_innov_h_m = innov_h;
        filter->gnss_consistency_last_plausible_m = gate;
        if (innov_h > gate) {
            filter->gnss_consistency_last_suspect = 1U;
            return true;
        }
    }

    if (has_vel) {
        const float dv_n = v_gps_n - filter->vel_[0];
        const float dv_e = v_gps_e - filter->vel_[1];
        const float vel_jump = sqrtf((dv_n * dv_n) + (dv_e * dv_e));
        filter->gnss_consistency_last_vel_jump_mps = vel_jump;
        if (vel_jump > NAVICORE_INS_EKF_CONSISTENCY_MAX_VEL_JUMP_MPS) {
            filter->gnss_consistency_last_suspect = 1U;
            return true;
        }
    }

    return false;
}

bool InsEkfFilter::update_gnss(const GpsSample &gps_sample, float *out_nis)
{
    if (!initialized || !gps_sample.fix_valid) {
        return false;
    }

    const uint32_t gnss_fix_timestamp_ms = gps_sample.timestamp_ms;
    struct GnssFixTimestampScope {
        InsEkfFilter *filter;
        uint32_t ts;
        ~GnssFixTimestampScope()
        {
            if (filter != NULL) {
                filter->last_gnss_fix_ms = ts;
            }
        }
    } gnss_fix_ts_scope{this, gnss_fix_timestamp_ms};

    ins_ekf_log_cov_step_audit(this, "gnss", "pre");
    ins_ekf_capture_ppv_frob_pre(this);

    constexpr uint8_t kMaxMeas = 5U;
    float y[kMaxMeas]{};
    float h_rows[kMaxMeas][INS_EKF_STATE_DIM]{};
    float meas_var[kMaxMeas]{};
    uint8_t n_meas = 0U;

    const float course_rad = gps_sample.course_deg * kDegToRadF;
    const float v_gps_n = gps_sample.speed_mps * std::cos(course_rad);
    const float v_gps_e = gps_sample.speed_mps * std::sin(course_rad);
    const bool has_vel_meas = gps_sample.speed_mps > 0.0f;

    if (gnss_obs_mode == INS_EKF_GNSS_OBS_VEL_ONLY) {
        if (!has_vel_meas) {
            return false;
        }
        n_meas = 2U;
        y[0] = v_gps_n - vel_[0];
        y[1] = v_gps_e - vel_[1];
        h_rows[0][INS_ERR_VEL_N] = 1.0f;
        h_rows[1][INS_ERR_VEL_E] = 1.0f;
        meas_var[0] = gnss_vel_var_m2_h;
        meas_var[1] = gnss_vel_var_m2_h;
        gnss_innovation_last[0] = y[0];
        gnss_innovation_last[1] = y[1];
        gnss_innovation_last[2] = 0.0f;
    } else {
        float z_n = 0.0f;
        float z_e = 0.0f;
        float z_d = 0.0f;
        geodesy::lla_to_ned(
            ref_lat_deg,
            ref_lon_deg,
            ref_alt_m,
            gps_sample.position.x,
            gps_sample.position.y,
            gps_sample.position.z,
            &z_n,
            &z_e,
            &z_d);

        y[0] = z_n - pos_[0];
        y[1] = z_e - pos_[1];
        y[2] = z_d - pos_[2];
        h_rows[0][INS_ERR_POS_N] = 1.0f;
        h_rows[1][INS_ERR_POS_E] = 1.0f;
        h_rows[2][INS_ERR_POS_D] = 1.0f;
        meas_var[0] = gnss_pos_var_m2;
        meas_var[1] = gnss_pos_var_m2;
        meas_var[2] = gnss_pos_var_m2;
        n_meas = 3U;

        gnss_innovation_last[0] = y[0];
        gnss_innovation_last[1] = y[1];
        gnss_innovation_last[2] = y[2];

        if (gnss_obs_mode == INS_EKF_GNSS_OBS_POS_VEL && has_vel_meas) {
            y[3] = v_gps_n - vel_[0];
            y[4] = v_gps_e - vel_[1];
            h_rows[3][INS_ERR_VEL_N] = 1.0f;
            h_rows[4][INS_ERR_VEL_E] = 1.0f;
            meas_var[3] = gnss_vel_var_m2_h;
            meas_var[4] = gnss_vel_var_m2_h;
            n_meas = 5U;
        }
    }

    const float effective_gap_s = ins_ekf_effective_gnss_gap_s(this, gps_sample.timestamp_ms);
    const float err_pre_n = vel_[0] - v_gps_n;
    const float err_pre_e = vel_[1] - v_gps_e;

    {
        const bool has_pos = (gnss_obs_mode != INS_EKF_GNSS_OBS_VEL_ONLY);
        const float y_pos[3] = {y[0], y[1], y[2]};
        if (ins_ekf_gnss_fails_physical_consistency(
                this,
                has_pos ? y_pos : NULL,
                has_pos,
                v_gps_n,
                v_gps_e,
                has_vel_meas,
                effective_gap_s)) {
            outlier_detected = true;
            gnss_nis_last = 0.0f;
            gnss_last_accepted = 0U;
            gnss_last_reject_reason = INS_EKF_GNSS_REJECT_INCONSISTENT;
            gnss_last_n_meas = n_meas;
            gnss_clear_correction_audit(this);
            if (out_nis != NULL) {
                *out_nis = 0.0f;
            }
            ins_ekf_log_cov_step_audit(this, "gnss", "post_reject");
            return false;
        }
    }

    bool zero_before_k = false;
    if (ppv_policy == INS_EKF_PPV_POLICY_ZERO) {
        zero_before_k = true;
    } else if (ppv_policy == INS_EKF_PPV_POLICY_GAP_LE_1S
        && effective_gap_s >= 0.0f
        && effective_gap_s <= NAVICORE_INS_EKF_PPV_GAP_THRESHOLD_S) {
        zero_before_k = true;
    } else if (ppv_policy == INS_EKF_PPV_POLICY_INNOV_H && n_meas >= 3U) {
        const float innov_h = sqrtf((y[0] * y[0]) + (y[1] * y[1]));
        if (innov_h >= NAVICORE_INS_EKF_PPV_INNOV_H_THRESHOLD_M) {
            zero_before_k = true;
        }
    }
    if (zero_before_k) {
        ins_ekf_zero_pv_cross(&cov);
        gnss_last_ppv_policy = static_cast<uint8_t>(ppv_policy);
        gnss_last_ppv_effective_gap_s = effective_gap_s;
        gnss_last_ppv_triggered = 1U;
    }

    gnss_last_update_timestamp_ms = last_imu_timestamp_ms;

    float s_mat[kMaxMeas][kMaxMeas]{};
    float s_inv[kMaxMeas][kMaxMeas]{};
    float k_gain[INS_EKF_STATE_DIM][kMaxMeas]{};
    float delta_x_candidate[INS_EKF_STATE_DIM]{};

    if (!gnss_compute_kalman_update(
            this,
            h_rows,
            y,
            meas_var,
            n_meas,
            s_mat,
            s_inv,
            k_gain,
            delta_x_candidate)) {
        outlier_detected = true;
        gnss_nis_last = 0.0f;
        gnss_last_accepted = 0U;
        gnss_last_reject_reason = 2U;
        gnss_clear_correction_audit(this);
        if (out_nis != NULL) {
            *out_nis = gnss_nis_last;
        }
        return false;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            gnss_innovation_cov_last[i][j] =
                (i < n_meas && j < n_meas) ? s_mat[i][j] : 0.0f;
        }
    }

    float nis = 0.0f;
    for (uint8_t i = 0U; i < n_meas; ++i) {
        for (uint8_t j = 0U; j < n_meas; ++j) {
            nis += y[i] * s_inv[i][j] * y[j];
        }
    }

    gnss_nis_last = nis;
    gnss_last_n_meas = n_meas;
    for (uint8_t i = 0U; i < kMaxMeas; ++i) {
        gnss_innovation_full[i] = (i < n_meas) ? y[i] : 0.0f;
        gnss_s_diag[i] = (i < n_meas) ? s_mat[i][i] : 0.0f;
        gnss_nis_contrib[i] = 0.0f;
        if (i < n_meas) {
            float s_inv_y = 0.0f;
            for (uint8_t j = 0U; j < n_meas; ++j) {
                s_inv_y += s_inv[i][j] * y[j];
            }
            gnss_nis_contrib[i] = y[i] * s_inv_y;
        }
    }
    if (out_nis != NULL) {
        *out_nis = nis;
    }

    const bool ppv_recompute = ins_ekf_ppv_policy_triggered(
            this,
            k_gain,
            y,
            n_meas,
            err_pre_n,
            err_pre_e,
            effective_gap_s,
            delta_x_candidate,
            zero_before_k);
    if (ppv_recompute) {
        ins_ekf_zero_pv_cross(&cov);
        if (!gnss_compute_kalman_update(
                this,
                h_rows,
                y,
                meas_var,
                n_meas,
                s_mat,
                s_inv,
                k_gain,
                delta_x_candidate)) {
            outlier_detected = true;
            gnss_nis_last = 0.0f;
            gnss_last_accepted = 0U;
            gnss_last_reject_reason = 2U;
            gnss_clear_correction_audit(this);
            if (out_nis != NULL) {
                *out_nis = gnss_nis_last;
            }
            return false;
        }
    }

    gnss_store_k_audit_n(this, k_gain, n_meas);
    gnss_store_delta_x_audit(this, delta_x_candidate);

    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            gnss_last_p_vel_pos[i][j] = cov.P[INS_ERR_VEL_N + i][INS_ERR_POS_N + j];
            gnss_last_s_inv[i][j] = (i < n_meas && j < n_meas) ? s_inv[i][j] : 0.0f;
            if (gnss_obs_mode == INS_EKF_GNSS_OBS_VEL_ONLY) {
                gnss_last_k_pos_pos[i][j] = 0.0f;
                gnss_last_k_vel_pos[i][j] = (j < 2U) ? k_gain[INS_ERR_VEL_N + i][j] : 0.0f;
            } else {
                gnss_last_k_pos_pos[i][j] = k_gain[INS_ERR_POS_N + i][j];
                gnss_last_k_vel_pos[i][j] = k_gain[INS_ERR_VEL_N + i][j];
            }
        }
    }

    /* Probe only: same NIS the gate uses vs what auditors may recompute. */
    {
        static uint32_t gnss_nis_decision_log_n = 0U;
        const bool reject_nis = nis > nis_threshold;
        if (gnss_nis_decision_log_n < 25U) {
            ++gnss_nis_decision_log_n;
            printf(
                "GNSS_NIS_DECISION t_ms=%u n_meas=%u "
                "NIS_used=%.6f thr=%.6f "
                "innov=[%.3f,%.3f,%.3f,%.3f,%.3f] "
                "contrib=[%.3f,%.3f,%.3f,%.3f,%.3f] "
                "S_diag=[%.3f,%.3f,%.3f,%.3f,%.3f] "
                "decision=%s reason=%u\n",
                static_cast<unsigned>(last_imu_timestamp_ms),
                static_cast<unsigned>(n_meas),
                static_cast<double>(nis),
                static_cast<double>(nis_threshold),
                static_cast<double>(gnss_innovation_full[0]),
                static_cast<double>(gnss_innovation_full[1]),
                static_cast<double>(gnss_innovation_full[2]),
                static_cast<double>(gnss_innovation_full[3]),
                static_cast<double>(gnss_innovation_full[4]),
                static_cast<double>(gnss_nis_contrib[0]),
                static_cast<double>(gnss_nis_contrib[1]),
                static_cast<double>(gnss_nis_contrib[2]),
                static_cast<double>(gnss_nis_contrib[3]),
                static_cast<double>(gnss_nis_contrib[4]),
                static_cast<double>(gnss_s_diag[0]),
                static_cast<double>(gnss_s_diag[1]),
                static_cast<double>(gnss_s_diag[2]),
                static_cast<double>(gnss_s_diag[3]),
                static_cast<double>(gnss_s_diag[4]),
                reject_nis ? "reject" : "accept",
                reject_nis ? 1U : 0U);
        }
    }

    if (nis > nis_threshold) {
        outlier_detected = true;
        gnss_last_accepted = 0U;
        gnss_last_reject_reason = 1U;
        ins_ekf_log_cov_step_audit(this, "gnss", "post_reject");
        ins_ekf_capture_ppv_frob_post(this);
        return false;
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        delta_x_[r] = delta_x_candidate[r];
    }

    const float gnss_dv_log[3] = {
        delta_x_[INS_ERR_VEL_N],
        delta_x_[INS_ERR_VEL_E],
        delta_x_[INS_ERR_VEL_D],
    };
    ins_ekf_inject_error_into_nominal(this);
    ins_ekf_log_vel_modification(this, "gnss", gnss_dv_log, NULL, NULL, false);

    gnss_joseph_update_n(
        cov.P,
        k_gain,
        h_rows,
        meas_var,
        n_meas,
        cov.P,
        scratch_a_,
        scratch_b_);

    outlier_detected = false;
    gnss_last_accepted = 1U;
    gnss_last_reject_reason = 0U;
    ins_ekf_log_cov_step_audit(this, "gnss", "post_accept");
    ins_ekf_capture_ppv_frob_post(this);
    return true;
}

/* S, K, δx from current H rows (2-meas NHC). Returns false if S singular. */
static bool nhc_solve_gain(
    InsEkfFilter *filter,
    const float h_rows[2][INS_EKF_STATE_DIM],
    const float y[2],
    float k_gain[INS_EKF_STATE_DIM][2],
    float s_inv[2][2])
{
    float s_mat[2][2]{};
    for (uint8_t i = 0U; i < 2U; ++i) {
        for (uint8_t j = 0U; j < 2U; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                for (uint8_t l = 0U; l < kDim; ++l) {
                    sum += h_rows[i][k] * filter->cov.P[k][l] * h_rows[j][l];
                }
            }
            s_mat[i][j] = sum;
        }
    }
    s_mat[0][0] += filter->nhc_lateral_var_m2;
    s_mat[1][1] += filter->nhc_vertical_var_m2;

    filter->nhc_last_hph_yy = s_mat[0][0] - filter->nhc_lateral_var_m2;
    filter->nhc_last_hph_yz = s_mat[0][1];
    filter->nhc_last_hph_zz = s_mat[1][1] - filter->nhc_vertical_var_m2;
    filter->nhc_last_s_yy = s_mat[0][0];
    filter->nhc_last_s_yz = s_mat[0][1];
    filter->nhc_last_s_zz = s_mat[1][1];

    if (!ins_ekf_invert2x2(s_mat, s_inv)) {
        return false;
    }

    filter->nhc_last_s_inv_yy = s_inv[0][0];
    filter->nhc_last_s_inv_yz = s_inv[0][1];
    filter->nhc_last_s_inv_zz = s_inv[1][1];

    for (uint8_t r = 0U; r < kDim; ++r) {
        float ph_t0 = 0.0f;
        float ph_t1 = 0.0f;
        for (uint8_t k = 0U; k < kDim; ++k) {
            ph_t0 += filter->cov.P[r][k] * h_rows[0][k];
            ph_t1 += filter->cov.P[r][k] * h_rows[1][k];
        }
        k_gain[r][0] = (ph_t0 * s_inv[0][0]) + (ph_t1 * s_inv[0][1]);
        k_gain[r][1] = (ph_t0 * s_inv[1][0]) + (ph_t1 * s_inv[1][1]);
        filter->delta_x_[r] = (k_gain[r][0] * y[0]) + (k_gain[r][1] * y[1]);
    }
    return true;
}

bool InsEkfFilter::update_nhc()
{
    if (!initialized || !nhc_enabled) {
        return false;
    }

    ins_ekf_log_cov_step_audit(this, "nhc", "pre");

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(q_att_, dcm_bn);

    float v_body[3]{};
    ned_to_body(dcm_bn, vel_, v_body);

    float yaw_rad = 0.0f;
    quat_to_euler321(q_att_, NULL, NULL, &yaw_rad);

    /* Pseudo-medicion: v_lateral (Y) y v_vertical (Z) cuerpo ≈ 0 m/s. */
    const float y[2] = {
        -v_body[1],
        -v_body[2],
    };

    nhc_innovation_last[0] = y[0];
    nhc_innovation_last[1] = y[1];
    const float abs_lateral = fabsf(y[0]);
    const float abs_vertical = fabsf(y[1]);
    const float innov_norm = sqrtf((y[0] * y[0]) + (y[1] * y[1]));
    if (abs_lateral > nhc_innovation_max_lateral_mps) {
        nhc_innovation_max_lateral_mps = abs_lateral;
    }
    if (abs_vertical > nhc_innovation_max_vertical_mps) {
        nhc_innovation_max_vertical_mps = abs_vertical;
    }
    if (innov_norm > nhc_innovation_max_norm_mps) {
        nhc_innovation_max_norm_mps = innov_norm;
    }

    float h_rows[2][INS_EKF_STATE_DIM]{};
    h_rows[0][INS_ERR_VEL_N + 0] = dcm_bn[0][1];
    h_rows[0][INS_ERR_VEL_N + 1] = dcm_bn[1][1];
    h_rows[0][INS_ERR_VEL_N + 2] = dcm_bn[2][1];
    h_rows[1][INS_ERR_VEL_N + 0] = dcm_bn[0][2];
    h_rows[1][INS_ERR_VEL_N + 1] = dcm_bn[1][2];
    h_rows[1][INS_ERR_VEL_N + 2] = dcm_bn[2][2];

    /* Acoplamiento actitud: ver fill_nhc_attitude_coupling_rows / ins_ekf_fill_nhc_attitude_coupling. */
    float h_att[2][3]{};
    fill_nhc_attitude_coupling_rows(v_body, h_att, nhc_jacobian_mode);
    h_rows[0][INS_ERR_ATT_X] = h_att[0][0];
    h_rows[0][INS_ERR_ATT_Y] = h_att[0][1];
    h_rows[0][INS_ERR_ATT_Z] = h_att[0][2];
    h_rows[1][INS_ERR_ATT_X] = h_att[1][0];
    h_rows[1][INS_ERR_ATT_Y] = h_att[1][1];
    h_rows[1][INS_ERR_ATT_Z] = h_att[1][2];

    /* Coherence gate: accumulate kinematic OK time; latch open once hold_s met. */
    if (nhc_att_coherence_gate && !nhc_att_gate_open) {
        const float speed_h = sqrtf((vel_[0] * vel_[0]) + (vel_[1] * vel_[1]));
        float course_yaw_abs_deg = 180.0f;
        bool kin_ok = false;
        if (speed_h >= nhc_att_gate_vmin_mps) {
            const float course_rad = atan2f(vel_[1], vel_[0]);
            float d = course_rad - yaw_rad;
            while (d > static_cast<float>(M_PI)) {
                d -= 2.0f * static_cast<float>(M_PI);
            }
            while (d < -static_cast<float>(M_PI)) {
                d += 2.0f * static_cast<float>(M_PI);
            }
            course_yaw_abs_deg = fabsf(d) * (180.0f / static_cast<float>(M_PI));
            kin_ok = course_yaw_abs_deg <= (nhc_att_gate_yaw_max_rad * (180.0f / static_cast<float>(M_PI)));
        }
        const bool sample_ok = kin_ok && nhc_att_gate_gnss_valid;

        float dt_gate_s = 0.0f;
        if (nhc_att_gate_last_imu_ms != 0U
            && last_imu_timestamp_ms >= nhc_att_gate_last_imu_ms) {
            dt_gate_s = static_cast<float>(last_imu_timestamp_ms - nhc_att_gate_last_imu_ms) * 0.001f;
        }
        nhc_att_gate_last_imu_ms = last_imu_timestamp_ms;

        if (sample_ok) {
            if (dt_gate_s > 0.0f && dt_gate_s < 1.0f) {
                nhc_att_gate_ok_accum_s += dt_gate_s;
            }
            if (nhc_att_gate_ok_accum_s >= nhc_att_gate_hold_s) {
                nhc_att_gate_open = true;
                nhc_att_gate_open_t_s = static_cast<float>(last_imu_timestamp_ms) * 0.001f;
                printf(
                    "NHC_ATT_GATE_OPEN t_s=%.3f speed=%.2f |course-yaw|=%.1f deg hold=%.2f s\n",
                    static_cast<double>(nhc_att_gate_open_t_s),
                    static_cast<double>(speed_h),
                    static_cast<double>(course_yaw_abs_deg),
                    static_cast<double>(nhc_att_gate_hold_s));
            }
        } else {
            nhc_att_gate_ok_accum_s = 0.0f;
        }
    }

    const bool block_att =
        nhc_att_unobs || (nhc_att_coherence_gate && !nhc_att_gate_open);
    if (block_att) {
        h_rows[0][INS_ERR_ATT_X] = 0.0f;
        h_rows[0][INS_ERR_ATT_Y] = 0.0f;
        h_rows[0][INS_ERR_ATT_Z] = 0.0f;
        h_rows[1][INS_ERR_ATT_X] = 0.0f;
        h_rows[1][INS_ERR_ATT_Y] = 0.0f;
        h_rows[1][INS_ERR_ATT_Z] = 0.0f;
    }

    /* H-ATT-d: si ya latched, unobs desde el primer solve (Joseph = K aplicado). */
    const bool unobs_pre = !block_att && nhc_att_z_unobs && nhc_att_z_forget_latched;
    if (unobs_pre) {
        h_rows[0][INS_ERR_ATT_Z] = 0.0f;
        h_rows[1][INS_ERR_ATT_Z] = 0.0f;
    }

    float k_gain[INS_EKF_STATE_DIM][2]{};
    float s_inv[2][2]{};
    if (!nhc_solve_gain(this, h_rows, y, k_gain, s_inv)) {
        return false;
    }

    /* Cand1: acumular |dx_att_z| del solve actual (full H si aún no unobs). */
    const float dx_z_gate = delta_x_[INS_ERR_ATT_Z];
    nhc_last_dx_att_z_raw = dx_z_gate;

    float forget = nhc_att_z_forget;
    if (forget < 0.0f) {
        forget = 0.0f;
    } else if (forget > 1.0f) {
        forget = 1.0f;
    }

    const float gate_thr = nhc_att_z_forget_gate_thr;
    bool fired_this_tick = false;
    if (gate_thr > 0.0f) {
        if (nhc_att_z_epoch_ms == 0U) {
            nhc_att_z_epoch_ms = last_imu_timestamp_ms;
        }
        ++nhc_att_z_gate_nhc_count;
        /* §13.22 E1: gracia — no acumular ni evaluar durante los primeros N NHC. */
        const bool in_grace = nhc_att_z_gate_nhc_count <= nhc_att_z_forget_grace_ticks;
        if (!in_grace) {
            nhc_att_z_sumabs += fabsf(dx_z_gate);
        }
        const float t_s =
            static_cast<float>(last_imu_timestamp_ms - nhc_att_z_epoch_ms) * 0.001f;
        float score = nhc_att_z_sumabs;
        if (nhc_att_z_forget_gate_norm && !in_grace) {
            /* Congelar Pzz al primer tick post-gracia (proxy de arranque). */
            if (nhc_att_z_gate_scale_pzz <= 0.0f) {
                const float pzz = cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z];
                nhc_att_z_gate_scale_pzz = (pzz > 1.0e-30f) ? pzz : 1.0e-30f;
            }
            score = nhc_att_z_sumabs / nhc_att_z_gate_scale_pzz;
        }
        if (!in_grace && !nhc_att_z_forget_latched && t_s <= nhc_att_z_forget_tmax_s
            && score >= gate_thr) {
            nhc_att_z_forget_latched = true;
            nhc_att_z_forget_fire_t_s = t_s;
            fired_this_tick = true;
            if (nhc_att_z_unobs) {
                printf(
                    "HATT_D_FIRE t_s=%.6f sumabs=%.9e score=%.9e thr=%.9e grace=%u norm=%d unobs=1\n",
                    static_cast<double>(t_s),
                    static_cast<double>(nhc_att_z_sumabs),
                    static_cast<double>(score),
                    static_cast<double>(gate_thr),
                    static_cast<unsigned>(nhc_att_z_forget_grace_ticks),
                    nhc_att_z_forget_gate_norm ? 1 : 0);
            } else {
                printf(
                    "HATT_C_FIRE t_s=%.6f sumabs=%.9e score=%.9e thr=%.9e lambda=%.3f grace=%u norm=%d\n",
                    static_cast<double>(t_s),
                    static_cast<double>(nhc_att_z_sumabs),
                    static_cast<double>(score),
                    static_cast<double>(gate_thr),
                    static_cast<double>(forget),
                    static_cast<unsigned>(nhc_att_z_forget_grace_ticks),
                    nhc_att_z_forget_gate_norm ? 1 : 0);
            }
        }
    } else {
        nhc_att_z_sumabs += fabsf(dx_z_gate);
    }

    nhc_last_att_z_unobs_active = false;
    if (nhc_att_z_unobs && nhc_att_z_forget_latched) {
        /* H-ATT-d: H[*][ATT_Z]=0, recompute S/K/δx; Joseph usará este K. Sin truncar δx. */
        if (!unobs_pre || fired_this_tick) {
            h_rows[0][INS_ERR_ATT_Z] = 0.0f;
            h_rows[1][INS_ERR_ATT_Z] = 0.0f;
            if (!nhc_solve_gain(this, h_rows, y, k_gain, s_inv)) {
                return false;
            }
        }
        nhc_last_att_z_unobs_active = true;
    } else {
        /* H-ATT-b1/c: truncar δx_z post-hoc; K/Joseph del solve full (incoherente a propósito). */
        bool apply_forget = false;
        if (gate_thr > 0.0f) {
            apply_forget = nhc_att_z_forget_latched && (forget > 0.0f);
        } else {
            apply_forget = forget > 0.0f;
        }
        if (apply_forget) {
            delta_x_[INS_ERR_ATT_Z] = dx_z_gate * (1.0f - forget);
        }
    }

    /* Filas K pitch/yaw + descomposición dx_y (K aplicado = Joseph). */
    nhc_last_k_att_y0 = k_gain[INS_ERR_ATT_Y][0];
    nhc_last_k_att_y1 = k_gain[INS_ERR_ATT_Y][1];
    nhc_last_k_att_z0 = k_gain[INS_ERR_ATT_Z][0];
    nhc_last_k_att_z1 = k_gain[INS_ERR_ATT_Z][1];
    nhc_last_dx_att_y_via_innov_y = k_gain[INS_ERR_ATT_Y][0] * y[0];
    nhc_last_dx_att_y_via_innov_z = k_gain[INS_ERR_ATT_Y][1] * y[1];

    float k_max = 0.0f;
    float k_y = 0.0f;
    float k_z = 0.0f;
    float k_pos = 0.0f;
    float k_vel = 0.0f;
    float k_att = 0.0f;
    float k_bias = 0.0f;
    for (uint8_t r = 0U; r < kDim; ++r) {
        k_y = fmaxf(k_y, fabsf(k_gain[r][0]));
        k_z = fmaxf(k_z, fabsf(k_gain[r][1]));
        for (uint8_t c = 0U; c < 2U; ++c) {
            k_max = fmaxf(k_max, fabsf(k_gain[r][c]));
        }
        if (r <= INS_ERR_POS_D) {
            k_pos = fmaxf(k_pos, fmaxf(fabsf(k_gain[r][0]), fabsf(k_gain[r][1])));
        } else if (r >= INS_ERR_VEL_N && r <= INS_ERR_VEL_D) {
            k_vel = fmaxf(k_vel, fmaxf(fabsf(k_gain[r][0]), fabsf(k_gain[r][1])));
        } else if (r >= INS_ERR_ATT_X && r <= INS_ERR_ATT_Z) {
            k_att = fmaxf(k_att, fmaxf(fabsf(k_gain[r][0]), fabsf(k_gain[r][1])));
        } else if (r >= INS_ERR_BIAS_AX && r <= INS_ERR_BIAS_GZ) {
            k_bias = fmaxf(k_bias, fmaxf(fabsf(k_gain[r][0]), fabsf(k_gain[r][1])));
        }
    }
    nhc_last_k_vel_max = k_vel;
    nhc_last_k_pos_max = k_pos;
    nhc_last_k_att_max = k_att;
    nhc_last_k_bias_max = k_bias;
    nhc_last_k_bias_gz = fmaxf(
        fabsf(k_gain[INS_ERR_BIAS_GZ][0]),
        fabsf(k_gain[INS_ERR_BIAS_GZ][1]));

    /* Path decompose K[BIAS_GZ] via H_vel vs H_att (same S as full update). */
    {
        const uint8_t bg = INS_ERR_BIAS_GZ;
        float ph_vel0 = 0.0f;
        float ph_vel1 = 0.0f;
        float ph_att0 = 0.0f;
        float ph_att1 = 0.0f;
        for (uint8_t i = 0U; i < 3U; ++i) {
            const uint8_t vk = static_cast<uint8_t>(INS_ERR_VEL_N + i);
            const uint8_t ak = static_cast<uint8_t>(INS_ERR_ATT_X + i);
            ph_vel0 += cov.P[bg][vk] * h_rows[0][vk];
            ph_vel1 += cov.P[bg][vk] * h_rows[1][vk];
            ph_att0 += cov.P[bg][ak] * h_rows[0][ak];
            ph_att1 += cov.P[bg][ak] * h_rows[1][ak];
        }
        const float k_vel_y = (ph_vel0 * s_inv[0][0]) + (ph_vel1 * s_inv[0][1]);
        const float k_vel_z = (ph_vel0 * s_inv[1][0]) + (ph_vel1 * s_inv[1][1]);
        const float k_att_y = (ph_att0 * s_inv[0][0]) + (ph_att1 * s_inv[0][1]);
        const float k_att_z = (ph_att0 * s_inv[1][0]) + (ph_att1 * s_inv[1][1]);
        nhc_last_dx_bias_gz_via_vel = (k_vel_y * y[0]) + (k_vel_z * y[1]);
        nhc_last_dx_bias_gz_via_att = (k_att_y * y[0]) + (k_att_z * y[1]);
        nhc_last_k_bias_gz_via_vel = fmaxf(fabsf(k_vel_y), fabsf(k_vel_z));
        nhc_last_k_bias_gz_via_att = fmaxf(fabsf(k_att_y), fabsf(k_att_z));
    }

    float nis = 0.0f;
    for (uint8_t i = 0U; i < 2U; ++i) {
        for (uint8_t j = 0U; j < 2U; ++j) {
            nis += y[i] * s_inv[i][j] * y[j];
        }
    }
    const float sy0 = (s_inv[0][0] * y[0]) + (s_inv[0][1] * y[1]);
    const float sy1 = (s_inv[1][0] * y[0]) + (s_inv[1][1] * y[1]);
    nhc_last_nis_contrib_y = y[0] * sy0;
    nhc_last_nis_contrib_z = y[1] * sy1;

    float dx_vel_sq = 0.0f;
    float dx_att_sq = 0.0f;
    float dx_pos_sq = 0.0f;
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float dv = delta_x_[INS_ERR_VEL_N + i];
        const float da = delta_x_[INS_ERR_ATT_X + i];
        const float dp = delta_x_[INS_ERR_POS_N + i];
        dx_vel_sq += dv * dv;
        dx_att_sq += da * da;
        dx_pos_sq += dp * dp;
    }

    nhc_last_dx_vel_n_mps = delta_x_[INS_ERR_VEL_N];
    nhc_last_dx_vel_e_mps = delta_x_[INS_ERR_VEL_E];
    nhc_last_dx_vel_d_mps = delta_x_[INS_ERR_VEL_D];
    nhc_last_dx_att_x_rad = delta_x_[INS_ERR_ATT_X];
    nhc_last_dx_att_y_rad = delta_x_[INS_ERR_ATT_Y];
    nhc_last_dx_att_z_rad = delta_x_[INS_ERR_ATT_Z];
    nhc_last_dx_pos_n_m = delta_x_[INS_ERR_POS_N];
    nhc_last_dx_pos_e_m = delta_x_[INS_ERR_POS_E];
    nhc_last_dx_pos_d_m = delta_x_[INS_ERR_POS_D];

    float dx_bias_sq = 0.0f;
    for (uint8_t r = INS_ERR_BIAS_AX; r <= INS_ERR_BIAS_GZ; ++r) {
        const float db = delta_x_[r];
        dx_bias_sq += db * db;
    }
    nhc_last_dx_bias_norm = sqrtf(dx_bias_sq);
    nhc_last_dx_bias_gx = delta_x_[INS_ERR_BIAS_GX];
    nhc_last_dx_bias_gy = delta_x_[INS_ERR_BIAS_GY];
    nhc_last_dx_bias_gz = delta_x_[INS_ERR_BIAS_GZ];

    nhc_last_update_timestamp_ms = last_imu_timestamp_ms;
    nhc_last_innov_y_mps = y[0];
    nhc_last_innov_z_mps = y[1];
    nhc_last_innov_norm_mps = innov_norm;
    nhc_last_k_max = k_max;
    nhc_last_k_y = k_y;
    nhc_last_k_z = k_z;
    nhc_last_nis = nis;
    nhc_last_dx_vel_norm_mps = sqrtf(dx_vel_sq);
    nhc_last_dx_att_norm_rad = sqrtf(dx_att_sq);
    nhc_last_dx_pos_norm_m = sqrtf(dx_pos_sq);
    nhc_last_v_body_x_mps = v_body[0];
    nhc_last_v_body_y_mps = v_body[1];
    nhc_last_v_body_z_mps = v_body[2];
    nhc_last_vel_n_mps = vel_[0];
    nhc_last_vel_e_mps = vel_[1];
    nhc_last_vel_d_mps = vel_[2];
    nhc_last_yaw_rad = yaw_rad;

    nhc_stat_sum_innov_y += static_cast<double>(y[0]);
    nhc_stat_sum_innov_z += static_cast<double>(y[1]);
    nhc_stat_sum_innov_y_sq += static_cast<double>(y[0]) * static_cast<double>(y[0]);
    nhc_stat_sum_innov_z_sq += static_cast<double>(y[1]) * static_cast<double>(y[1]);
    nhc_stat_sum_k_y += static_cast<double>(k_y);
    nhc_stat_sum_k_z += static_cast<double>(k_z);
    nhc_stat_sum_nis += static_cast<double>(nis);
    nhc_stat_sum_v_body_y += static_cast<double>(v_body[1]);
    nhc_stat_sum_v_body_z += static_cast<double>(v_body[2]);
    if (nis > nhc_stat_max_nis) {
        nhc_stat_max_nis = nis;
    }
    if ((y[0] * delta_x_[INS_ERR_ATT_Y]) > 0.0f) {
        ++nhc_stat_same_sign_count;
    }

    nhc_last_h_row0_vel[0] = h_rows[0][INS_ERR_VEL_N + 0];
    nhc_last_h_row0_vel[1] = h_rows[0][INS_ERR_VEL_N + 1];
    nhc_last_h_row0_vel[2] = h_rows[0][INS_ERR_VEL_N + 2];
    nhc_last_h_row1_vel[0] = h_rows[1][INS_ERR_VEL_N + 0];
    nhc_last_h_row1_vel[1] = h_rows[1][INS_ERR_VEL_N + 1];
    nhc_last_h_row1_vel[2] = h_rows[1][INS_ERR_VEL_N + 2];

    InsEkfCovBlockMetrics cov_pre{};
    (void)ins_ekf_get_cov_block_metrics(this, &cov_pre);
    nhc_last_cov_pre = cov_pre;

    const float nhc_dv_log[3] = {
        delta_x_[INS_ERR_VEL_N],
        delta_x_[INS_ERR_VEL_E],
        delta_x_[INS_ERR_VEL_D],
    };
    const float h0[3] = {
        h_rows[0][INS_ERR_VEL_N],
        h_rows[0][INS_ERR_VEL_E],
        h_rows[0][INS_ERR_VEL_D],
    };
    const float h1[3] = {
        h_rows[1][INS_ERR_VEL_N],
        h_rows[1][INS_ERR_VEL_E],
        h_rows[1][INS_ERR_VEL_D],
    };
    ins_ekf_inject_error_into_nominal(this);
    ins_ekf_log_vel_modification(this, "nhc", nhc_dv_log, h0, h1, true);

    float v_body_after[3]{};
    ned_to_body(dcm_bn, vel_, v_body_after);
    nhc_last_v_body_after_x_mps = v_body_after[0];
    nhc_last_v_body_after_y_mps = v_body_after[1];
    nhc_last_v_body_after_z_mps = v_body_after[2];
    nhc_last_vel_after_n_mps = vel_[0];
    nhc_last_vel_after_e_mps = vel_[1];
    nhc_last_vel_after_d_mps = vel_[2];

    const float meas_var[2] = {nhc_lateral_var_m2, nhc_vertical_var_m2};
    ins_ekf_covariance_joseph_update2(
        cov.P,
        k_gain,
        h_rows,
        meas_var,
        cov.P,
        scratch_a_,
        scratch_b_);

    InsEkfCovBlockMetrics cov_post{};
    (void)ins_ekf_get_cov_block_metrics(this, &cov_post);
    nhc_last_cov_post = cov_post;

    ++nhc_update_count;
    ins_ekf_log_cov_step_audit(this, "nhc", "post");
    ins_ekf_log_nhc_block_audit(this);
    return true;
}

void ins_ekf_covariance_joseph_update3(
    InsEkfMat15 p_in,
    const float k_gain[INS_EKF_STATE_DIM][3],
    const float h_rows[3][INS_EKF_STATE_DIM],
    const float meas_var[3],
    InsEkfMat15 p_out,
    InsEkfMat15 scratch_a,
    InsEkfMat15 scratch_b)
{
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            scratch_a[r][c] = (r == c) ? 1.0f : 0.0f;
            for (uint8_t i = 0U; i < 3U; ++i) {
                scratch_a[r][c] -= k_gain[r][i] * h_rows[i][c];
            }
        }
    }

    mat15_mul(scratch_a, p_in, scratch_b);
    mat15_transpose_inplace(scratch_a);
    mat15_mul(scratch_b, scratch_a, p_out);

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = r; c < kDim; ++c) {
            float krk_t = 0.0f;
            for (uint8_t i = 0U; i < 3U; ++i) {
                krk_t += k_gain[r][i] * meas_var[i] * k_gain[c][i];
            }
            p_out[r][c] += krk_t;
            if (r != c) {
                p_out[c][r] = p_out[r][c];
            }
        }
    }

    mat_symmetrize(p_out);
}

bool InsEkfFilter::update_zupt()
{
    if (!initialized) {
        return false;
    }

    ins_ekf_log_cov_step_audit(this, "zupt", "pre");

    /* Pseudo-medicion: velocidad NED = 0 m/s. */
    const float y[3] = {
        -vel_[0],
        -vel_[1],
        -vel_[2],
    };

    float h_rows[3][INS_EKF_STATE_DIM]{};
    h_rows[0][INS_ERR_VEL_N] = 1.0f;
    h_rows[1][INS_ERR_VEL_E] = 1.0f;
    h_rows[2][INS_ERR_VEL_D] = 1.0f;

    float s_mat[3][3]{};
    float zupt_r_used[3] = {
        zupt_vel_var_m2,
        zupt_vel_var_m2,
        zupt_vel_var_m2,
    };
    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                for (uint8_t l = 0U; l < kDim; ++l) {
                    sum += h_rows[i][k] * cov.P[k][l] * h_rows[j][l];
                }
            }
            s_mat[i][j] = sum;
        }
        const float p_ii = s_mat[i][i];
        const float g_max = NAVICORE_INS_EKF_ZUPT_MAX_GAIN;
        const float r_min = p_ii * (1.0f - g_max) / g_max;
        if (zupt_r_used[i] < r_min) {
            zupt_r_used[i] = r_min;
        }
        s_mat[i][i] += zupt_r_used[i];
    }

    InsEkfMat3 s_inv{};
    if (!ins_ekf_invert3x3(s_mat, s_inv)) {
        return false;
    }

    float k_gain[INS_EKF_STATE_DIM][3]{};
    for (uint8_t r = 0U; r < kDim; ++r) {
        float ph_t0 = 0.0f;
        float ph_t1 = 0.0f;
        float ph_t2 = 0.0f;
        for (uint8_t k = 0U; k < kDim; ++k) {
            ph_t0 += cov.P[r][k] * h_rows[0][k];
            ph_t1 += cov.P[r][k] * h_rows[1][k];
            ph_t2 += cov.P[r][k] * h_rows[2][k];
        }
        k_gain[r][0] = (ph_t0 * s_inv[0][0]) + (ph_t1 * s_inv[0][1]) + (ph_t2 * s_inv[0][2]);
        k_gain[r][1] = (ph_t0 * s_inv[1][0]) + (ph_t1 * s_inv[1][1]) + (ph_t2 * s_inv[1][2]);
        k_gain[r][2] = (ph_t0 * s_inv[2][0]) + (ph_t1 * s_inv[2][1]) + (ph_t2 * s_inv[2][2]);
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        delta_x_[r] = (k_gain[r][0] * y[0]) + (k_gain[r][1] * y[1]) + (k_gain[r][2] * y[2]);
    }

    const float zupt_dv_log[3] = {
        delta_x_[INS_ERR_VEL_N],
        delta_x_[INS_ERR_VEL_E],
        delta_x_[INS_ERR_VEL_D],
    };
    ins_ekf_inject_error_into_nominal(this);
    ins_ekf_log_vel_modification(this, "zupt", zupt_dv_log, NULL, NULL, false);

    const float meas_var[3] = {zupt_r_used[0], zupt_r_used[1], zupt_r_used[2]};
    ins_ekf_covariance_joseph_update3(
        cov.P,
        k_gain,
        h_rows,
        meas_var,
        cov.P,
        scratch_a_,
        scratch_b_);

    ++zupt_update_count;
    ins_ekf_log_cov_step_audit(this, "zupt", "post");
    return true;
}

void ins_ekf_init(
    InsEkfFilter *filter,
    Vector3D initial_position,
    float initial_yaw_rad,
    NavDomain domain)
{
    if (filter == NULL) {
        return;
    }

    memset(filter, 0, sizeof(InsEkfFilter));

    filter->ref_lat_deg = initial_position.x;
    filter->ref_lon_deg = initial_position.y;
    filter->ref_alt_m = initial_position.z;
    filter->domain = domain;

    yaw_rad_to_quat(initial_yaw_rad, filter->q_att_);
    quat_normalize(filter->q_att_);

    mat_zero(filter->cov.P);
    filter->cov.P[INS_ERR_POS_N][INS_ERR_POS_N] = 4.0f;
    filter->cov.P[INS_ERR_POS_E][INS_ERR_POS_E] = 4.0f;
    filter->cov.P[INS_ERR_POS_D][INS_ERR_POS_D] = 9.0f;
    filter->cov.P[INS_ERR_VEL_N][INS_ERR_VEL_N] = 1.0f;
    filter->cov.P[INS_ERR_VEL_E][INS_ERR_VEL_E] = 1.0f;
    filter->cov.P[INS_ERR_VEL_D][INS_ERR_VEL_D] = 1.0f;
    filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_X] = deg_to_rad(5.0f) * deg_to_rad(5.0f);
    filter->cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Y] = deg_to_rad(5.0f) * deg_to_rad(5.0f);
    filter->cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z] = deg_to_rad(10.0f) * deg_to_rad(10.0f);
    filter->cov.P[INS_ERR_BIAS_AX][INS_ERR_BIAS_AX] = 0.25f;
    filter->cov.P[INS_ERR_BIAS_AY][INS_ERR_BIAS_AY] = 0.25f;
    filter->cov.P[INS_ERR_BIAS_AZ][INS_ERR_BIAS_AZ] = 0.25f;
    filter->cov.P[INS_ERR_BIAS_GX][INS_ERR_BIAS_GX] = 0.01f;
    filter->cov.P[INS_ERR_BIAS_GY][INS_ERR_BIAS_GY] = 0.01f;
    filter->cov.P[INS_ERR_BIAS_GZ][INS_ERR_BIAS_GZ] = 0.01f;

    filter->accel_noise_var = NAVICORE_INS_EKF_ACCEL_NOISE_VAR;
    filter->gyro_noise_var = NAVICORE_INS_EKF_GYRO_NOISE_VAR;
    filter->bias_accel_rw_var = NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR;
    filter->bias_gyro_rw_var = NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR;
    filter->gnss_pos_var_m2 = NAVICORE_INS_EKF_GNSS_POS_VAR_M2;
    filter->gnss_vel_var_m2_h = NAVICORE_INS_EKF_GNSS_VEL_VAR_M2;
    filter->gnss_obs_mode = INS_EKF_GNSS_OBS_POS;
    filter->ppv_policy = INS_EKF_PPV_POLICY_NONE;
    filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD;
    filter->gnss_consistency_enabled =
        (NAVICORE_INS_EKF_CONSISTENCY_CHECK_ENABLED != 0) ? 1U : 0U;
    filter->gnss_consistency_last_suspect = 0U;
    filter->gnss_consistency_last_innov_h_m = 0.0f;
    filter->gnss_consistency_last_plausible_m = 0.0f;
    filter->gnss_consistency_last_vel_jump_mps = 0.0f;
    filter->nhc_enabled = false;
    filter->nhc_jacobian_mode = ins_ekf_default_nhc_jacobian_mode();
    filter->nhc_att_z_forget = ins_ekf_default_nhc_att_z_forget();
    filter->nhc_att_z_forget_gate_thr = ins_ekf_default_nhc_att_z_forget_gate_thr();
    filter->nhc_att_z_forget_tmax_s = ins_ekf_default_nhc_att_z_forget_tmax_s();
    filter->nhc_att_z_sumabs = 0.0f;
    filter->nhc_att_z_epoch_ms = 0U;
    filter->nhc_att_z_gate_nhc_count = 0U;
    filter->nhc_att_z_forget_grace_ticks = ins_ekf_default_nhc_att_z_forget_grace_ticks();
    filter->nhc_att_z_forget_gate_norm = ins_ekf_default_nhc_att_z_forget_gate_norm();
    filter->nhc_att_z_gate_scale_pzz = 0.0f;
    filter->nhc_att_z_forget_latched = false;
    filter->nhc_att_z_forget_fire_t_s = -1.0f;
    filter->nhc_att_z_unobs = ins_ekf_default_nhc_att_z_unobs();
    filter->nhc_last_att_z_unobs_active = false;
    filter->nhc_att_unobs = false;
    filter->nhc_att_coherence_gate = false;
    filter->nhc_att_gate_vmin_mps = 5.0f;
    filter->nhc_att_gate_yaw_max_rad = 15.0f * (static_cast<float>(M_PI) / 180.0f);
    filter->nhc_att_gate_hold_s = 2.5f;
    filter->nhc_att_gate_gnss_valid = false;
    filter->nhc_att_gate_open = false;
    filter->nhc_att_gate_ok_accum_s = 0.0f;
    filter->nhc_att_gate_last_imu_ms = 0U;
    filter->nhc_att_gate_open_t_s = -1.0f;
    filter->nhc_every_n_ticks = NAVICORE_INS_EKF_NHC_EVERY_N_TICKS;
    filter->nhc_lateral_var_m2 =
        NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS * NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS;
    filter->nhc_vertical_var_m2 =
        NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS * NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS;
    filter->nhc_update_count = 0U;
    filter->nhc_tick_counter = 0U;
    filter->nhc_innovation_last[0] = 0.0f;
    filter->nhc_innovation_last[1] = 0.0f;
    filter->nhc_innovation_max_lateral_mps = 0.0f;
    filter->nhc_innovation_max_vertical_mps = 0.0f;
    filter->nhc_innovation_max_norm_mps = 0.0f;
    filter->nhc_stat_sum_innov_y = 0.0;
    filter->nhc_stat_sum_innov_z = 0.0;
    filter->nhc_stat_sum_innov_y_sq = 0.0;
    filter->nhc_stat_sum_innov_z_sq = 0.0;
    filter->nhc_stat_sum_k_y = 0.0;
    filter->nhc_stat_sum_k_z = 0.0;
    filter->nhc_stat_sum_nis = 0.0;
    filter->nhc_stat_sum_v_body_y = 0.0;
    filter->nhc_stat_sum_v_body_z = 0.0;
    filter->nhc_stat_max_nis = 0.0f;
    filter->nhc_stat_same_sign_count = 0U;
    filter->zupt_update_count = 0U;
    filter->zupt_vel_var_m2 =
        NAVICORE_INS_EKF_ZUPT_VEL_STD_MPS * NAVICORE_INS_EKF_ZUPT_VEL_STD_MPS;
    filter->predict_audit_last_.valid = false;

    filter->initialized = true;
    filter->outlier_detected = false;
    ins_ekf_reset_error_state(filter);
}

bool ins_ekf_predict(InsEkfFilter *filter, const ImuSample *imu)
{
    if (filter == NULL || imu == NULL || !imu->valid || !filter->initialized) {
        return false;
    }

    /* Reject non-finite IMU — fail-closed (safety inject / sensor garbage). */
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!std::isfinite(imu->accel_mps2[i]) || !std::isfinite(imu->gyro_radps[i])) {
            return false;
        }
    }

    ins_ekf_reset_vel_pipeline_audit(filter);
    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->vel_pipeline_audit_last_.vel_before_predict[i] = filter->vel_[i];
    }

    const float dt_s = ins_ekf_predict_dt_s(filter, imu->timestamp_ms);
    filter->predict(*imu, dt_s);

    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->vel_pipeline_audit_last_.vel_after_predict[i] = filter->vel_[i];
        filter->vel_pipeline_audit_last_.dv_predict[i] =
            filter->vel_pipeline_audit_last_.vel_after_predict[i]
            - filter->vel_pipeline_audit_last_.vel_before_predict[i];
    }

    if (filter->nhc_enabled) {
        filter->nhc_tick_counter++;
        const uint32_t nhc_stride =
            (filter->nhc_every_n_ticks == 0U) ? 1U : filter->nhc_every_n_ticks;
        if ((filter->nhc_tick_counter % nhc_stride) == 0U) {
            const float vel_before_nhc[3] = {
                filter->vel_[0],
                filter->vel_[1],
                filter->vel_[2],
            };
            if (filter->update_nhc()) {
                filter->vel_pipeline_audit_last_.nhc_applied = true;
                for (uint8_t i = 0U; i < 3U; ++i) {
                    filter->vel_pipeline_audit_last_.vel_after_nhc[i] = filter->vel_[i];
                    filter->vel_pipeline_audit_last_.dv_nhc[i] =
                        filter->vel_[i] - vel_before_nhc[i];
                }
            }
        }
    }

    if (!filter->vel_pipeline_audit_last_.nhc_applied) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            filter->vel_pipeline_audit_last_.vel_after_nhc[i] =
                filter->vel_pipeline_audit_last_.vel_after_predict[i];
            filter->vel_pipeline_audit_last_.dv_nhc[i] = 0.0f;
        }
    }

    filter->vel_pipeline_audit_last_.valid = true;
    return true;
}

bool ins_ekf_update_nhc(InsEkfFilter *filter)
{
    if (filter == NULL || !filter->initialized || !filter->nhc_enabled) {
        return false;
    }

    return filter->update_nhc();
}

void ins_ekf_set_nhc_enabled(InsEkfFilter *filter, bool enabled)
{
    if (filter == NULL) {
        return;
    }

    filter->nhc_enabled = enabled;
}

void ins_ekf_set_nhc_every_n_ticks(InsEkfFilter *filter, uint32_t every_n_ticks)
{
    if (filter == NULL) {
        return;
    }

    filter->nhc_every_n_ticks = (every_n_ticks == 0U) ? 1U : every_n_ticks;
}

bool ins_ekf_nhc_enabled(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return false;
    }

    return filter->nhc_enabled;
}

uint32_t ins_ekf_nhc_update_count(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return 0U;
    }

    return filter->nhc_update_count;
}

void ins_ekf_get_nhc_innovation_last(
    const InsEkfFilter *filter,
    float *out_lateral_mps,
    float *out_vertical_mps)
{
    if (filter == NULL) {
        return;
    }

    if (out_lateral_mps != NULL) {
        *out_lateral_mps = filter->nhc_innovation_last[0];
    }
    if (out_vertical_mps != NULL) {
        *out_vertical_mps = filter->nhc_innovation_last[1];
    }
}

void ins_ekf_get_nhc_innovation_max(
    const InsEkfFilter *filter,
    float *out_lateral_mps,
    float *out_vertical_mps,
    float *out_norm_mps)
{
    if (filter == NULL) {
        return;
    }

    if (out_lateral_mps != NULL) {
        *out_lateral_mps = filter->nhc_innovation_max_lateral_mps;
    }
    if (out_vertical_mps != NULL) {
        *out_vertical_mps = filter->nhc_innovation_max_vertical_mps;
    }
    if (out_norm_mps != NULL) {
        *out_norm_mps = filter->nhc_innovation_max_norm_mps;
    }
}

bool ins_ekf_get_nhc_last_update_detail(
    const InsEkfFilter *filter,
    InsEkfNhcUpdateDetail *out_detail)
{
    if (filter == NULL || out_detail == NULL || filter->nhc_update_count == 0U) {
        return false;
    }

    out_detail->timestamp_ms = filter->nhc_last_update_timestamp_ms;
    out_detail->innov_y_mps = filter->nhc_last_innov_y_mps;
    out_detail->innov_z_mps = filter->nhc_last_innov_z_mps;
    out_detail->innov_norm_mps = filter->nhc_last_innov_norm_mps;
    out_detail->k_max = filter->nhc_last_k_max;
    out_detail->k_y = filter->nhc_last_k_y;
    out_detail->k_z = filter->nhc_last_k_z;
    out_detail->nis = filter->nhc_last_nis;
    out_detail->dx_vel_norm_mps = filter->nhc_last_dx_vel_norm_mps;
    out_detail->dx_att_norm_rad = filter->nhc_last_dx_att_norm_rad;
    out_detail->dx_pos_norm_m = filter->nhc_last_dx_pos_norm_m;
    out_detail->v_body_x_mps = filter->nhc_last_v_body_x_mps;
    out_detail->v_body_y_mps = filter->nhc_last_v_body_y_mps;
    out_detail->v_body_z_mps = filter->nhc_last_v_body_z_mps;
    out_detail->vel_n_mps = filter->nhc_last_vel_n_mps;
    out_detail->vel_e_mps = filter->nhc_last_vel_e_mps;
    out_detail->vel_d_mps = filter->nhc_last_vel_d_mps;
    out_detail->yaw_rad = filter->nhc_last_yaw_rad;
    out_detail->dx_vel_n_mps = filter->nhc_last_dx_vel_n_mps;
    out_detail->dx_vel_e_mps = filter->nhc_last_dx_vel_e_mps;
    out_detail->dx_vel_d_mps = filter->nhc_last_dx_vel_d_mps;
    out_detail->dx_att_x_rad = filter->nhc_last_dx_att_x_rad;
    out_detail->dx_att_y_rad = filter->nhc_last_dx_att_y_rad;
    out_detail->dx_att_z_rad = filter->nhc_last_dx_att_z_rad;
    return true;
}

bool ins_ekf_write_nhc_block_audit_header(FILE *audit_fp)
{
    if (audit_fp == NULL) {
        return false;
    }

    fprintf(
        audit_fp,
        "timestamp_s,imu_seq,nhc_update_index,accepted,"
        "v_body_x_before_mps,v_body_y_before_mps,v_body_z_before_mps,"
        "v_body_x_after_mps,v_body_y_after_mps,v_body_z_after_mps,"
        "dv_body_x_mps,dv_body_y_mps,dv_body_z_mps,"
        "innov_y_mps,innov_z_mps,innov_norm_mps,"
        "hph_yy,hph_yz,hph_zz,r_y_m2,r_z_m2,"
        "s_yy,s_yz,s_zz,s_inv_yy,s_inv_yz,s_inv_zz,"
        "s_eigmin,s_eigmax,s_cond,"
        "nis_total,nis_contrib_y,nis_contrib_z,"
        "k_y_max,k_z_max,k_vel_max,k_pos_max,k_att_max,k_bias_max,"
        "dx_pos_n_m,dx_pos_e_m,dx_pos_d_m,dx_pos_norm_m,"
        "dx_vel_n_mps,dx_vel_e_mps,dx_vel_d_mps,dx_vel_norm_mps,"
        "dx_att_x_rad,dx_att_y_rad,dx_att_z_rad,dx_att_norm_rad,dx_bias_norm,"
        "vel_n_mps,vel_e_mps,vel_d_mps,vel_h_mps,"
        "vel_after_n_mps,vel_after_e_mps,vel_after_h_mps,"
        "h_r0_vn,h_r0_ve,h_r0_vd,h_r1_vn,h_r1_ve,h_r1_vd,"
        "P_pre_pp_frob,P_pre_vv_frob,P_pre_pv_frob,P_pre_vp_frob,P_pre_aa_frob,P_pre_vel_pos_max_abs,"
        "P_pre_vv_body_fwd_m2,P_pre_vv_body_lat_m2,P_pre_vv_body_vert_m2,"
        "P_post_pp_frob,P_post_vv_frob,P_post_pv_frob,P_post_vp_frob,P_post_aa_frob,P_post_vel_pos_max_abs,"
        "P_post_vv_body_fwd_m2,P_post_vv_body_lat_m2,P_post_vv_body_vert_m2,"
        "delta_P_pp_frob,delta_P_vv_frob,delta_P_pv_frob,delta_P_aa_frob,delta_P_vel_pos_max_abs,"
        "delta_P_vv_body_fwd_m2,delta_P_vv_body_lat_m2,delta_P_vv_body_vert_m2,"
        "gps_speed_mps,"
        "P_pre_att_bias_g_frob,P_pre_att_bias_g_max_abs,P_pre_att_z_bias_gz,P_pre_bias_g_frob,"
        "P_post_att_bias_g_frob,P_post_att_bias_g_max_abs,P_post_att_z_bias_gz,P_post_bias_g_frob,"
        "k_bias_gz,dx_bias_gx,dx_bias_gy,dx_bias_gz,"
        "dx_bias_gz_via_vel,dx_bias_gz_via_att,k_bias_gz_via_vel,k_bias_gz_via_att,"
        "P_pre_att_xx,P_pre_att_yy,P_pre_att_zz,P_pre_att_xz,P_pre_att_yz,P_pre_att_xy,"
        "P_post_att_xx,P_post_att_yy,P_post_att_zz,P_post_att_xz,P_post_att_yz,P_post_att_xy,"
        "k_att_y0,k_att_y1,k_att_z0,k_att_z1,"
        "dx_att_y_via_innov_y,dx_att_y_via_innov_z,dx_att_z_raw,"
        "P_pre_att_y_vn,P_pre_att_y_ve,P_pre_att_y_vd,"
        "P_pre_att_z_vn,P_pre_att_z_ve,P_pre_att_z_vd,"
        "P_pre_vel_att_frob,"
        "P_post_att_y_vn,P_post_att_y_ve,P_post_att_y_vd,"
        "P_post_att_z_vn,P_post_att_z_ve,P_post_att_z_vd,"
        "P_post_vel_att_frob,"
        "f_va_vn_attx,f_va_vn_atty,f_va_vn_attz,"
        "f_va_ve_attx,f_va_ve_atty,f_va_ve_attz,"
        "f_va_vd_attx,f_va_vd_atty,f_va_vd_attz,"
        "a_nav_n,a_nav_e,a_nav_d\n");
    return true;
}

void ins_ekf_set_nhc_block_audit(InsEkfFilter *filter, FILE *audit_fp)
{
    if (filter == NULL) {
        return;
    }
    filter->nhc_block_audit_fp = audit_fp;
}

void ins_ekf_set_nhc_block_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq,
    float gps_speed_mps)
{
    if (filter == NULL) {
        return;
    }
    filter->nhc_block_audit_timestamp_s = timestamp_s;
    filter->nhc_block_audit_imu_seq = imu_seq;
    filter->nhc_block_audit_gps_speed_mps = gps_speed_mps;
}

static void nhc_symmetric2_eigen_extremes(
    float s00,
    float s01,
    float s11,
    float *out_min,
    float *out_max)
{
    const float trace = s00 + s11;
    const float det = (s00 * s11) - (s01 * s01);
    const float disc = fmaxf((trace * trace) - (4.0f * det), 0.0f);
    const float root = sqrtf(disc);
    const float e0 = 0.5f * (trace - root);
    const float e1 = 0.5f * (trace + root);
    if (out_min != NULL) {
        *out_min = fminf(e0, e1);
    }
    if (out_max != NULL) {
        *out_max = fmaxf(e0, e1);
    }
}

void ins_ekf_log_nhc_block_audit(InsEkfFilter *filter)
{
    if (filter == NULL || filter->nhc_block_audit_fp == NULL || !filter->initialized) {
        return;
    }

    const InsEkfCovBlockMetrics &pre = filter->nhc_last_cov_pre;
    const InsEkfCovBlockMetrics &post = filter->nhc_last_cov_post;
    const float vel_h_before = hypotf(filter->nhc_last_vel_n_mps, filter->nhc_last_vel_e_mps);
    const float vel_h_after = hypotf(
        filter->nhc_last_vel_after_n_mps,
        filter->nhc_last_vel_after_e_mps);
    const float dv_body_x =
        filter->nhc_last_v_body_after_x_mps - filter->nhc_last_v_body_x_mps;
    const float dv_body_y =
        filter->nhc_last_v_body_after_y_mps - filter->nhc_last_v_body_y_mps;
    const float dv_body_z =
        filter->nhc_last_v_body_after_z_mps - filter->nhc_last_v_body_z_mps;

    float s_eigmin = 0.0f;
    float s_eigmax = 0.0f;
    float s_cond = 0.0f;
    {
        const float yy = filter->nhc_last_s_yy;
        const float yz = filter->nhc_last_s_yz;
        const float zz = filter->nhc_last_s_zz;
        nhc_symmetric2_eigen_extremes(yy, yz, zz, &s_eigmin, &s_eigmax);
        s_cond = (s_eigmin > 1.0e-12f) ? (s_eigmax / s_eigmin) : 0.0f;
    }

    fprintf(
        filter->nhc_block_audit_fp,
        "%.9f,%llu,%u,1,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%.6f,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.6f,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e\n",
        filter->nhc_block_audit_timestamp_s,
        static_cast<unsigned long long>(filter->nhc_block_audit_imu_seq),
        filter->nhc_update_count,
        filter->nhc_last_v_body_x_mps,
        filter->nhc_last_v_body_y_mps,
        filter->nhc_last_v_body_z_mps,
        filter->nhc_last_v_body_after_x_mps,
        filter->nhc_last_v_body_after_y_mps,
        filter->nhc_last_v_body_after_z_mps,
        dv_body_x,
        dv_body_y,
        dv_body_z,
        filter->nhc_last_innov_y_mps,
        filter->nhc_last_innov_z_mps,
        filter->nhc_last_innov_norm_mps,
        filter->nhc_last_hph_yy,
        filter->nhc_last_hph_yz,
        filter->nhc_last_hph_zz,
        filter->nhc_lateral_var_m2,
        filter->nhc_vertical_var_m2,
        filter->nhc_last_s_yy,
        filter->nhc_last_s_yz,
        filter->nhc_last_s_zz,
        filter->nhc_last_s_inv_yy,
        filter->nhc_last_s_inv_yz,
        filter->nhc_last_s_inv_zz,
        s_eigmin,
        s_eigmax,
        s_cond,
        filter->nhc_last_nis,
        filter->nhc_last_nis_contrib_y,
        filter->nhc_last_nis_contrib_z,
        filter->nhc_last_k_y,
        filter->nhc_last_k_z,
        filter->nhc_last_k_vel_max,
        filter->nhc_last_k_pos_max,
        filter->nhc_last_k_att_max,
        filter->nhc_last_k_bias_max,
        filter->nhc_last_dx_pos_n_m,
        filter->nhc_last_dx_pos_e_m,
        filter->nhc_last_dx_pos_d_m,
        filter->nhc_last_dx_pos_norm_m,
        filter->nhc_last_dx_vel_n_mps,
        filter->nhc_last_dx_vel_e_mps,
        filter->nhc_last_dx_vel_d_mps,
        filter->nhc_last_dx_vel_norm_mps,
        filter->nhc_last_dx_att_x_rad,
        filter->nhc_last_dx_att_y_rad,
        filter->nhc_last_dx_att_z_rad,
        filter->nhc_last_dx_att_norm_rad,
        filter->nhc_last_dx_bias_norm,
        filter->nhc_last_vel_n_mps,
        filter->nhc_last_vel_e_mps,
        filter->nhc_last_vel_d_mps,
        vel_h_before,
        filter->nhc_last_vel_after_n_mps,
        filter->nhc_last_vel_after_e_mps,
        vel_h_after,
        filter->nhc_last_h_row0_vel[0],
        filter->nhc_last_h_row0_vel[1],
        filter->nhc_last_h_row0_vel[2],
        filter->nhc_last_h_row1_vel[0],
        filter->nhc_last_h_row1_vel[1],
        filter->nhc_last_h_row1_vel[2],
        pre.p_pos_pos_frob,
        pre.p_vel_vel_frob,
        pre.p_vel_pos_frob,
        pre.p_vel_pos_frob,
        pre.p_att_att_frob,
        pre.p_vel_pos_max_abs,
        pre.p_vv_body_forward_m2,
        pre.p_vv_body_lateral_m2,
        pre.p_vv_body_vertical_m2,
        post.p_pos_pos_frob,
        post.p_vel_vel_frob,
        post.p_vel_pos_frob,
        post.p_vel_pos_frob,
        post.p_att_att_frob,
        post.p_vel_pos_max_abs,
        post.p_vv_body_forward_m2,
        post.p_vv_body_lateral_m2,
        post.p_vv_body_vertical_m2,
        post.p_pos_pos_frob - pre.p_pos_pos_frob,
        post.p_vel_vel_frob - pre.p_vel_vel_frob,
        post.p_vel_pos_frob - pre.p_vel_pos_frob,
        post.p_att_att_frob - pre.p_att_att_frob,
        post.p_vel_pos_max_abs - pre.p_vel_pos_max_abs,
        post.p_vv_body_forward_m2 - pre.p_vv_body_forward_m2,
        post.p_vv_body_lateral_m2 - pre.p_vv_body_lateral_m2,
        post.p_vv_body_vertical_m2 - pre.p_vv_body_vertical_m2,
        filter->nhc_block_audit_gps_speed_mps,
        pre.p_att_bias_g_frob,
        pre.p_att_bias_g_max_abs,
        pre.p_att_z_bias_gz,
        pre.p_bias_g_frob,
        post.p_att_bias_g_frob,
        post.p_att_bias_g_max_abs,
        post.p_att_z_bias_gz,
        post.p_bias_g_frob,
        filter->nhc_last_k_bias_gz,
        filter->nhc_last_dx_bias_gx,
        filter->nhc_last_dx_bias_gy,
        filter->nhc_last_dx_bias_gz,
        filter->nhc_last_dx_bias_gz_via_vel,
        filter->nhc_last_dx_bias_gz_via_att,
        filter->nhc_last_k_bias_gz_via_vel,
        filter->nhc_last_k_bias_gz_via_att,
        pre.p_att_xx,
        pre.p_att_yy,
        pre.p_att_zz,
        pre.p_att_xz,
        pre.p_att_yz,
        pre.p_att_xy,
        post.p_att_xx,
        post.p_att_yy,
        post.p_att_zz,
        post.p_att_xz,
        post.p_att_yz,
        post.p_att_xy,
        filter->nhc_last_k_att_y0,
        filter->nhc_last_k_att_y1,
        filter->nhc_last_k_att_z0,
        filter->nhc_last_k_att_z1,
        filter->nhc_last_dx_att_y_via_innov_y,
        filter->nhc_last_dx_att_y_via_innov_z,
        filter->nhc_last_dx_att_z_raw,
        pre.p_att_y_vn,
        pre.p_att_y_ve,
        pre.p_att_y_vd,
        pre.p_att_z_vn,
        pre.p_att_z_ve,
        pre.p_att_z_vd,
        pre.p_vel_att_frob,
        post.p_att_y_vn,
        post.p_att_y_ve,
        post.p_att_y_vd,
        post.p_att_z_vn,
        post.p_att_z_ve,
        post.p_att_z_vd,
        post.p_vel_att_frob,
        filter->predict_audit_last_.f_va[0][0],
        filter->predict_audit_last_.f_va[0][1],
        filter->predict_audit_last_.f_va[0][2],
        filter->predict_audit_last_.f_va[1][0],
        filter->predict_audit_last_.f_va[1][1],
        filter->predict_audit_last_.f_va[1][2],
        filter->predict_audit_last_.f_va[2][0],
        filter->predict_audit_last_.f_va[2][1],
        filter->predict_audit_last_.f_va[2][2],
        filter->predict_audit_last_.a_nav_mps2[0],
        filter->predict_audit_last_.a_nav_mps2[1],
        filter->predict_audit_last_.a_nav_mps2[2]);
}

bool ins_ekf_get_nhc_run_summary(
    const InsEkfFilter *filter,
    InsEkfNhcRunSummary *out_summary)
{
    if (filter == NULL || out_summary == NULL || filter->nhc_update_count == 0U) {
        return false;
    }

    const uint32_t n = filter->nhc_update_count;
    const double inv_n = 1.0 / static_cast<double>(n);
    const double mean_y = filter->nhc_stat_sum_innov_y * inv_n;
    const double mean_z = filter->nhc_stat_sum_innov_z * inv_n;
    const double var_y = (filter->nhc_stat_sum_innov_y_sq * inv_n) - (mean_y * mean_y);
    const double var_z = (filter->nhc_stat_sum_innov_z_sq * inv_n) - (mean_z * mean_z);

    out_summary->sample_count = n;
    out_summary->mean_innov_y_mps = static_cast<float>(mean_y);
    out_summary->mean_innov_z_mps = static_cast<float>(mean_z);
    out_summary->std_innov_y_mps = static_cast<float>(sqrt(fmax(0.0, var_y)));
    out_summary->std_innov_z_mps = static_cast<float>(sqrt(fmax(0.0, var_z)));
    out_summary->mean_k_y = static_cast<float>(filter->nhc_stat_sum_k_y * inv_n);
    out_summary->mean_k_z = static_cast<float>(filter->nhc_stat_sum_k_z * inv_n);
    out_summary->frac_same_sign_corr =
        static_cast<float>(filter->nhc_stat_same_sign_count) * static_cast<float>(inv_n);
    out_summary->mean_nis = static_cast<float>(filter->nhc_stat_sum_nis * inv_n);
    out_summary->max_nis = filter->nhc_stat_max_nis;
    out_summary->mean_v_body_y_mps =
        static_cast<float>(filter->nhc_stat_sum_v_body_y * inv_n);
    out_summary->mean_v_body_z_mps =
        static_cast<float>(filter->nhc_stat_sum_v_body_z * inv_n);
    return true;
}

bool ins_ekf_update_zupt(InsEkfFilter *filter)
{
    if (filter == NULL || !filter->initialized) {
        return false;
    }

    return filter->update_zupt();
}

uint32_t ins_ekf_zupt_update_count(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return 0U;
    }

    return filter->zupt_update_count;
}

bool ins_ekf_update_gnss(InsEkfFilter *filter, const GpsSample *gps)
{
    if (filter == NULL || gps == NULL || !gps->fix_valid || !filter->initialized) {
        return false;
    }

    float nis = 0.0f;
    const bool computed = filter->update_gnss(*gps, &nis);

    if (!computed) {
        ++filter->gnss_reject_count;
        return false;
    }

    filter->outlier_detected = false;
    ++filter->gnss_accept_count;
    filter->last_gnss_accept_ms = gps->timestamp_ms;
    return true;
}

void ins_ekf_set_gnss_obs_mode(InsEkfFilter *filter, InsEkfGnssObsMode mode)
{
    if (filter == NULL) {
        return;
    }

    filter->gnss_obs_mode = mode;
    switch (mode) {
    case INS_EKF_GNSS_OBS_POS:
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD;
        break;
    case INS_EKF_GNSS_OBS_VEL_ONLY:
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF;
        break;
    case INS_EKF_GNSS_OBS_POS_VEL:
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD_POS_VEL_5DOF;
        break;
    default:
        filter->gnss_obs_mode = INS_EKF_GNSS_OBS_POS;
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD;
        break;
    }
}

void ins_ekf_set_consistency_check_enabled(InsEkfFilter *filter, bool enabled)
{
    if (filter == NULL) {
        return;
    }
    filter->gnss_consistency_enabled = enabled ? 1U : 0U;
}

bool ins_ekf_gnss_consistency_last_suspect(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return false;
    }
    return filter->gnss_consistency_last_suspect != 0U;
}

void ins_ekf_set_p_pv_policy(InsEkfFilter *filter, InsEkfPpvPolicy policy)
{
    if (filter == NULL) {
        return;
    }

    filter->ppv_policy = policy;
}

const char *ins_ekf_p_pv_policy_name(InsEkfPpvPolicy policy)
{
    switch (policy) {
    case INS_EKF_PPV_POLICY_NONE:
        return "none";
    case INS_EKF_PPV_POLICY_GAP_LE_1S:
        return "gap_le_1s";
    case INS_EKF_PPV_POLICY_ZERO:
        return "zero";
    case INS_EKF_PPV_POLICY_COS_POS:
        return "cos_pos";
    case INS_EKF_PPV_POLICY_COS_TOT:
        return "cos_tot";
    case INS_EKF_PPV_POLICY_INNOV_H:
        return "innov_h";
    default:
        return "unknown";
    }
}

bool ins_ekf_parse_p_pv_policy(const char *text, InsEkfPpvPolicy *out_policy)
{
    if (text == NULL || out_policy == NULL || text[0] == '\0') {
        return false;
    }

    if (strcmp(text, "none") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_NONE;
        return true;
    }
    if (strcmp(text, "gap_le_1s") == 0 || strcmp(text, "gap-le-1s") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_GAP_LE_1S;
        return true;
    }
    if (strcmp(text, "zero") == 0 || strcmp(text, "unconditional") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_ZERO;
        return true;
    }
    if (strcmp(text, "cos_pos") == 0 || strcmp(text, "cos-pos") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_COS_POS;
        return true;
    }
    if (strcmp(text, "cos_tot") == 0 || strcmp(text, "cos-tot") == 0
        || strcmp(text, "cos_total") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_COS_TOT;
        return true;
    }
    if (strcmp(text, "innov_h") == 0 || strcmp(text, "innov-h") == 0) {
        *out_policy = INS_EKF_PPV_POLICY_INNOV_H;
        return true;
    }
    return false;
}

void ins_ekf_set_gnss_vel_var_m2(InsEkfFilter *filter, float var_m2_h)
{
    if (filter == NULL || var_m2_h <= 0.0f) {
        return;
    }

    filter->gnss_vel_var_m2_h = var_m2_h;
}

const char *ins_ekf_gnss_obs_mode_name(InsEkfGnssObsMode mode)
{
    switch (mode) {
    case INS_EKF_GNSS_OBS_POS:
        return "pos";
    case INS_EKF_GNSS_OBS_POS_VEL:
        return "pos_vel";
    case INS_EKF_GNSS_OBS_VEL_ONLY:
        return "vel_only";
    default:
        return "unknown";
    }
}

bool ins_ekf_outlier_detected(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return false;
    }

    return filter->outlier_detected;
}

void ins_ekf_clear_outlier_flag(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }

    filter->outlier_detected = false;
}

float ins_ekf_last_nis(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return 0.0f;
    }

    return filter->gnss_nis_last;
}

void ins_ekf_get_gnss_innovation(const InsEkfFilter *filter, float out_ned[3])
{
    if (filter == NULL || out_ned == NULL) {
        return;
    }

    out_ned[0] = filter->gnss_innovation_last[0];
    out_ned[1] = filter->gnss_innovation_last[1];
    out_ned[2] = filter->gnss_innovation_last[2];
}

bool ins_ekf_get_gnss_last_update_detail(
    const InsEkfFilter *filter,
    InsEkfGnssUpdateDetail *out_detail)
{
    if (filter == NULL || out_detail == NULL || filter->gnss_last_update_timestamp_ms == 0U) {
        return false;
    }

    out_detail->timestamp_ms = filter->gnss_last_update_timestamp_ms;
    out_detail->accepted = filter->gnss_last_accepted;
    out_detail->reject_reason = filter->gnss_last_reject_reason;
    out_detail->n_meas = filter->gnss_last_n_meas;
    out_detail->innov_n_m = filter->gnss_innovation_last[0];
    out_detail->innov_e_m = filter->gnss_innovation_last[1];
    out_detail->innov_d_m = filter->gnss_innovation_last[2];
    out_detail->innov_vn_mps = filter->gnss_innovation_full[3];
    out_detail->innov_ve_mps = filter->gnss_innovation_full[4];
    out_detail->nis = filter->gnss_nis_last;
    out_detail->nis_contrib_n = filter->gnss_nis_contrib[0];
    out_detail->nis_contrib_e = filter->gnss_nis_contrib[1];
    out_detail->nis_contrib_d = filter->gnss_nis_contrib[2];
    out_detail->nis_contrib_vn = filter->gnss_nis_contrib[3];
    out_detail->nis_contrib_ve = filter->gnss_nis_contrib[4];
    out_detail->s_nn = filter->gnss_s_diag[0];
    out_detail->s_ee = filter->gnss_s_diag[1];
    out_detail->s_dd = filter->gnss_s_diag[2];
    out_detail->s_vn = filter->gnss_s_diag[3];
    out_detail->s_ve = filter->gnss_s_diag[4];
    out_detail->k_pos_max = filter->gnss_last_k_pos_max;
    out_detail->k_vel_max = filter->gnss_last_k_vel_max;
    out_detail->k_att_max = filter->gnss_last_k_att_max;
    out_detail->dx_pos_norm_m = filter->gnss_last_dx_pos_norm_m;
    out_detail->dx_vel_norm_mps = filter->gnss_last_dx_vel_norm_mps;
    out_detail->dx_att_norm_rad = filter->gnss_last_dx_att_norm_rad;
    out_detail->dx_pos_n_m = filter->gnss_last_dx_pos_n_m;
    out_detail->dx_pos_e_m = filter->gnss_last_dx_pos_e_m;
    out_detail->dx_pos_d_m = filter->gnss_last_dx_pos_d_m;
    out_detail->dx_vel_n_mps = filter->gnss_last_dx_vel_n_mps;
    out_detail->dx_vel_e_mps = filter->gnss_last_dx_vel_e_mps;
    out_detail->dx_vel_d_mps = filter->gnss_last_dx_vel_d_mps;
    out_detail->dx_att_x_rad = filter->gnss_last_dx_att_x_rad;
    out_detail->dx_att_y_rad = filter->gnss_last_dx_att_y_rad;
    out_detail->dx_att_z_rad = filter->gnss_last_dx_att_z_rad;
    out_detail->ppv_policy = filter->gnss_last_ppv_policy;
    out_detail->ppv_triggered = filter->gnss_last_ppv_triggered;
    out_detail->ppv_effective_gap_s = filter->gnss_last_ppv_effective_gap_s;
    out_detail->cos_dv_pos_err_pre = filter->gnss_last_cos_dv_pos_err_pre;
    out_detail->cos_dv_tot_err_pre = filter->gnss_last_cos_dv_tot_err_pre;
    out_detail->ppv_frob_pre = filter->gnss_last_ppv_frob_pre;
    out_detail->ppv_frob_post = filter->gnss_last_ppv_frob_post;
    return true;
}

bool ins_ekf_get_gnss_last_k_block_detail(
    const InsEkfFilter *filter,
    InsEkfGnssKBlockDetail *out_detail)
{
    if (filter == NULL || out_detail == NULL || filter->gnss_last_update_timestamp_ms == 0U) {
        return false;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            out_detail->p_vel_pos[i][j] = filter->gnss_last_p_vel_pos[i][j];
            out_detail->k_vel_pos[i][j] = filter->gnss_last_k_vel_pos[i][j];
            out_detail->k_pos_pos[i][j] = filter->gnss_last_k_pos_pos[i][j];
            out_detail->s_inv[i][j] = filter->gnss_last_s_inv[i][j];
        }
    }
    out_detail->dx_bias_accel_norm = filter->gnss_last_dx_bias_a_norm;
    out_detail->dx_bias_gyro_norm = filter->gnss_last_dx_bias_g_norm;
    return true;
}

void ins_ekf_get_bias(
    const InsEkfFilter *filter,
    float out_accel_bias[3],
    float out_gyro_bias[3])
{
    if (filter == NULL || out_accel_bias == NULL || out_gyro_bias == NULL) {
        return;
    }

    out_accel_bias[0] = filter->bias_a_[0];
    out_accel_bias[1] = filter->bias_a_[1];
    out_accel_bias[2] = filter->bias_a_[2];
    out_gyro_bias[0] = filter->bias_g_[0];
    out_gyro_bias[1] = filter->bias_g_[1];
    out_gyro_bias[2] = filter->bias_g_[2];
}

void ins_ekf_get_position_ned(const InsEkfFilter *filter, float out_ned[3])
{
    if (filter == NULL || out_ned == NULL || !filter->initialized) {
        return;
    }

    out_ned[0] = filter->pos_[0];
    out_ned[1] = filter->pos_[1];
    out_ned[2] = filter->pos_[2];
}

void ins_ekf_get_velocity_ned(const InsEkfFilter *filter, float out_ned[3])
{
    if (filter == NULL || out_ned == NULL || !filter->initialized) {
        return;
    }

    out_ned[0] = filter->vel_[0];
    out_ned[1] = filter->vel_[1];
    out_ned[2] = filter->vel_[2];
}

static float block3_frobenius(const float block[3][3])
{
    float sum_sq = 0.0f;
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            sum_sq += block[r][c] * block[r][c];
        }
    }
    return std::sqrt(sum_sq);
}

static float block3_max_abs(const float block[3][3])
{
    float max_abs = 0.0f;
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            const float abs_val = fabsf(block[r][c]);
            if (abs_val > max_abs) {
                max_abs = abs_val;
            }
        }
    }
    return max_abs;
}

bool ins_ekf_get_cov_block_metrics(
    const InsEkfFilter *filter,
    InsEkfCovBlockMetrics *out_metrics)
{
    if (filter == NULL || out_metrics == NULL || !filter->initialized) {
        return false;
    }

    float p_pos_pos[3][3]{};
    float p_vel_vel[3][3]{};
    float p_vel_pos[3][3]{};
    float p_vel_att[3][3]{};
    float p_att_att[3][3]{};
    float p_att_bias_g[3][3]{};
    float p_bias_g[3][3]{};

    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            const uint8_t pos_i = static_cast<uint8_t>(INS_ERR_POS_N + i);
            const uint8_t pos_j = static_cast<uint8_t>(INS_ERR_POS_N + j);
            const uint8_t vel_i = static_cast<uint8_t>(INS_ERR_VEL_N + i);
            const uint8_t vel_j = static_cast<uint8_t>(INS_ERR_VEL_N + j);
            const uint8_t att_i = static_cast<uint8_t>(INS_ERR_ATT_X + i);
            const uint8_t att_j = static_cast<uint8_t>(INS_ERR_ATT_X + j);
            const uint8_t bg_i = static_cast<uint8_t>(INS_ERR_BIAS_GX + i);
            const uint8_t bg_j = static_cast<uint8_t>(INS_ERR_BIAS_GX + j);
            p_pos_pos[i][j] = filter->cov.P[pos_i][pos_j];
            p_vel_vel[i][j] = filter->cov.P[vel_i][vel_j];
            p_vel_pos[i][j] = filter->cov.P[vel_i][pos_j];
            p_vel_att[i][j] = filter->cov.P[vel_i][att_j];
            p_att_att[i][j] = filter->cov.P[att_i][att_j];
            p_att_bias_g[i][j] = filter->cov.P[att_i][bg_j];
            p_bias_g[i][j] = filter->cov.P[bg_i][bg_j];
        }
    }

    out_metrics->p_pos_pos_frob = block3_frobenius(p_pos_pos);
    out_metrics->p_vel_vel_frob = block3_frobenius(p_vel_vel);
    out_metrics->p_vel_pos_frob = block3_frobenius(p_vel_pos);
    out_metrics->p_vel_att_frob = block3_frobenius(p_vel_att);
    out_metrics->p_att_att_frob = block3_frobenius(p_att_att);
    out_metrics->p_att_bias_g_frob = block3_frobenius(p_att_bias_g);
    out_metrics->p_att_bias_g_max_abs = block3_max_abs(p_att_bias_g);
    out_metrics->p_att_z_bias_gz =
        filter->cov.P[INS_ERR_ATT_Z][INS_ERR_BIAS_GZ];
    out_metrics->p_att_xx = filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_X];
    out_metrics->p_att_yy = filter->cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Y];
    out_metrics->p_att_zz = filter->cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z];
    out_metrics->p_att_xz = filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_Z];
    out_metrics->p_att_yz = filter->cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Z];
    out_metrics->p_att_xy = filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_Y];
    out_metrics->p_att_y_vn = filter->cov.P[INS_ERR_ATT_Y][INS_ERR_VEL_N];
    out_metrics->p_att_y_ve = filter->cov.P[INS_ERR_ATT_Y][INS_ERR_VEL_E];
    out_metrics->p_att_y_vd = filter->cov.P[INS_ERR_ATT_Y][INS_ERR_VEL_D];
    out_metrics->p_att_z_vn = filter->cov.P[INS_ERR_ATT_Z][INS_ERR_VEL_N];
    out_metrics->p_att_z_ve = filter->cov.P[INS_ERR_ATT_Z][INS_ERR_VEL_E];
    out_metrics->p_att_z_vd = filter->cov.P[INS_ERR_ATT_Z][INS_ERR_VEL_D];
    out_metrics->p_bias_g_frob = block3_frobenius(p_bias_g);
    out_metrics->p_pos_std_n_m =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_N][INS_ERR_POS_N], 0.0f));
    out_metrics->p_pos_std_e_m =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_E][INS_ERR_POS_E], 0.0f));
    out_metrics->p_pos_std_d_m =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_D][INS_ERR_POS_D], 0.0f));
    out_metrics->p_vel_std_n_mps =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_VEL_N][INS_ERR_VEL_N], 0.0f));
    out_metrics->p_vel_std_e_mps =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_VEL_E][INS_ERR_VEL_E], 0.0f));
    out_metrics->p_vel_std_d_mps =
        sqrtf(fmaxf(filter->cov.P[INS_ERR_VEL_D][INS_ERR_VEL_D], 0.0f));
    out_metrics->p_vel_pos_max_abs = block3_max_abs(p_vel_pos);
    out_metrics->p_vv_var_n_m2 = filter->cov.P[INS_ERR_VEL_N][INS_ERR_VEL_N];
    out_metrics->p_vv_var_e_m2 = filter->cov.P[INS_ERR_VEL_E][INS_ERR_VEL_E];
    out_metrics->p_vv_var_d_m2 = filter->cov.P[INS_ERR_VEL_D][INS_ERR_VEL_D];

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(filter->q_att_, dcm_bn);
    float v_body[3]{};
    ned_to_body(dcm_bn, filter->vel_, v_body);
    out_metrics->vel_body_x_mps = v_body[0];
    out_metrics->vel_body_y_mps = v_body[1];
    out_metrics->vel_body_z_mps = v_body[2];

    for (uint8_t axis = 0U; axis < 3U; ++axis) {
        float var_body = 0.0f;
        for (uint8_t j = 0U; j < 3U; ++j) {
            float row_sum = 0.0f;
            for (uint8_t k = 0U; k < 3U; ++k) {
                row_sum += dcm_bn[axis][j] * p_vel_vel[j][k];
            }
            var_body += row_sum * dcm_bn[axis][j];
        }
        if (axis == 0U) {
            out_metrics->p_vv_body_forward_m2 = fmaxf(var_body, 0.0f);
        } else if (axis == 1U) {
            out_metrics->p_vv_body_lateral_m2 = fmaxf(var_body, 0.0f);
        } else {
            out_metrics->p_vv_body_vertical_m2 = fmaxf(var_body, 0.0f);
        }
    }
    return true;
}

void ins_ekf_set_cov_step_audit(InsEkfFilter *filter, FILE *audit_fp)
{
    if (filter == NULL) {
        return;
    }
    filter->cov_step_audit_fp = audit_fp;
}

void ins_ekf_set_cov_step_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq)
{
    if (filter == NULL) {
        return;
    }
    filter->cov_step_audit_timestamp_s = timestamp_s;
    filter->cov_step_audit_imu_seq = imu_seq;
}

void ins_ekf_log_cov_step_audit(
    InsEkfFilter *filter,
    const char *update_type,
    const char *phase)
{
    if (filter == NULL || filter->cov_step_audit_fp == NULL || update_type == NULL
        || phase == NULL || !filter->initialized) {
        return;
    }

    InsEkfCovBlockMetrics metrics{};
    if (!ins_ekf_get_cov_block_metrics(filter, &metrics)) {
        return;
    }

    const float vel_h = hypotf(filter->vel_[0], filter->vel_[1]);
    fprintf(
        filter->cov_step_audit_fp,
        "%.9f,%llu,%s,%s,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f\n",
        filter->cov_step_audit_timestamp_s,
        static_cast<unsigned long long>(filter->cov_step_audit_imu_seq),
        update_type,
        phase,
        metrics.p_pos_pos_frob,
        metrics.p_vel_vel_frob,
        metrics.p_vel_pos_frob,
        metrics.p_att_att_frob,
        metrics.p_vv_var_n_m2,
        metrics.p_vv_var_e_m2,
        metrics.p_vv_var_d_m2,
        metrics.p_vv_body_forward_m2,
        metrics.p_vv_body_lateral_m2,
        metrics.p_vv_body_vertical_m2,
        metrics.p_vel_std_n_mps,
        metrics.p_vel_std_e_mps,
        metrics.p_vel_std_d_mps,
        vel_h,
        metrics.vel_body_x_mps,
        metrics.vel_body_y_mps,
        metrics.vel_body_z_mps);
}

void ins_ekf_reset_vel_pipeline_audit(InsEkfFilter *filter)
{
    if (filter == NULL) {
        return;
    }
    memset(&filter->vel_pipeline_audit_last_, 0, sizeof(filter->vel_pipeline_audit_last_));
}

bool ins_ekf_get_vel_pipeline_audit(
    const InsEkfFilter *filter,
    InsEkfVelPipelineAudit *out_audit)
{
    if (filter == NULL || out_audit == NULL || !filter->initialized) {
        return false;
    }
    *out_audit = filter->vel_pipeline_audit_last_;
    return out_audit->valid;
}

void ins_ekf_set_vel_source_audit(InsEkfFilter *filter, FILE *audit_fp)
{
    if (filter == NULL) {
        return;
    }
    filter->vel_source_audit_fp = audit_fp;
}

void ins_ekf_set_vel_source_audit_context(
    InsEkfFilter *filter,
    double timestamp_s,
    uint64_t imu_seq,
    float gps_speed_mps)
{
    if (filter == NULL) {
        return;
    }
    filter->vel_source_audit_timestamp_s = timestamp_s;
    filter->vel_source_audit_imu_seq = imu_seq;
    filter->vel_source_audit_gps_speed_mps = gps_speed_mps;
}

void ins_ekf_log_vel_modification(
    InsEkfFilter *filter,
    const char *source,
    const float dv_ned[3],
    const float h_nhc_row0_vel[3],
    const float h_nhc_row1_vel[3],
    bool log_nhc_h)
{
    if (filter == NULL || filter->vel_source_audit_fp == NULL || source == NULL
        || dv_ned == NULL || !filter->initialized) {
        return;
    }

    const float dv_norm = sqrtf(
        (dv_ned[0] * dv_ned[0]) + (dv_ned[1] * dv_ned[1]) + (dv_ned[2] * dv_ned[2]));
    const float vel_h = hypotf(filter->vel_[0], filter->vel_[1]);

    const float h00 = (log_nhc_h && h_nhc_row0_vel != NULL) ? h_nhc_row0_vel[0] : 0.0f;
    const float h01 = (log_nhc_h && h_nhc_row0_vel != NULL) ? h_nhc_row0_vel[1] : 0.0f;
    const float h02 = (log_nhc_h && h_nhc_row0_vel != NULL) ? h_nhc_row0_vel[2] : 0.0f;
    const float h10 = (log_nhc_h && h_nhc_row1_vel != NULL) ? h_nhc_row1_vel[0] : 0.0f;
    const float h11 = (log_nhc_h && h_nhc_row1_vel != NULL) ? h_nhc_row1_vel[1] : 0.0f;
    const float h12 = (log_nhc_h && h_nhc_row1_vel != NULL) ? h_nhc_row1_vel[2] : 0.0f;

    fprintf(
        filter->vel_source_audit_fp,
        "%.9f,%llu,%s,%.9e,%.9e,%.9e,%.9e,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
        filter->vel_source_audit_timestamp_s,
        static_cast<unsigned long long>(filter->vel_source_audit_imu_seq),
        source,
        dv_ned[0],
        dv_ned[1],
        dv_ned[2],
        dv_norm,
        filter->vel_[0],
        filter->vel_[1],
        filter->vel_[2],
        vel_h,
        filter->vel_source_audit_gps_speed_mps,
        h00,
        h01,
        h02,
        h10,
        h11,
        h12);
}

float ins_ekf_get_covariance_flat(const InsEkfFilter *filter, uint16_t linear_idx)
{
    if (filter == NULL || !filter->initialized) {
        return 0.0f;
    }

    constexpr uint16_t kCovElements =
        static_cast<uint16_t>(INS_EKF_STATE_DIM * INS_EKF_STATE_DIM);
    if (linear_idx >= kCovElements) {
        return 0.0f;
    }

    const float *cov_flat = &filter->cov.P[0][0];
    return cov_flat[linear_idx];
}

void ins_ekf_get_attitude_rad(
    const InsEkfFilter *filter,
    float *roll_rad,
    float *pitch_rad,
    float *yaw_rad)
{
    if (filter == NULL) {
        return;
    }

    quat_to_euler321(filter->q_att_, roll_rad, pitch_rad, yaw_rad);
}

bool ins_ekf_compute_roll_pitch_from_gravity_body(
    const float accel_body_mps2[3],
    float *roll_rad,
    float *pitch_rad)
{
    if (accel_body_mps2 == NULL) {
        return false;
    }

    const float ax = accel_body_mps2[0];
    const float ay = accel_body_mps2[1];
    const float az = accel_body_mps2[2];
    const float horiz_sq = (ay * ay) + (az * az);
    if (horiz_sq < 1.0e-6f) {
        return false;
    }

    const float roll = atan2f(ay, az);
    const float pitch = atan2f(-ax, sqrtf(horiz_sq));
    if (roll_rad != NULL) {
        *roll_rad = roll;
    }
    if (pitch_rad != NULL) {
        *pitch_rad = pitch;
    }
    return true;
}

bool ins_ekf_apply_gravity_tilt_init(
    InsEkfFilter *filter,
    const float mean_accel_body_mps2[3],
    const float mean_gyro_body_radps[3])
{
    if (filter == NULL || mean_accel_body_mps2 == NULL || !filter->initialized) {
        return false;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    if (!ins_ekf_compute_roll_pitch_from_gravity_body(
            mean_accel_body_mps2,
            &roll_rad,
            &pitch_rad)) {
        return false;
    }

    float yaw_rad = 0.0f;
    quat_to_euler321(filter->q_att_, NULL, NULL, &yaw_rad);
    euler321_to_quat(roll_rad, pitch_rad, yaw_rad, filter->q_att_);
    quat_normalize(filter->q_att_);

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(filter->q_att_, dcm_bn);
    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_body[3] = {0.0f, 0.0f, 0.0f};
    ned_to_body(dcm_bn, g_ned, g_body);
    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->bias_a_[i] = mean_accel_body_mps2[i] - g_body[i];
        if (mean_gyro_body_radps != NULL) {
            filter->bias_g_[i] = mean_gyro_body_radps[i];
        }
    }

    filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_X] = NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2;
    filter->cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Y] = NAVICORE_INS_EKF_INIT_ATT_ROLL_PITCH_VAR_RAD2;
    return true;
}

bool ins_ekf_get_last_predict_audit(
    const InsEkfFilter *filter,
    InsEkfPredictAudit *out_audit)
{
    if (filter == NULL || out_audit == NULL || !filter->predict_audit_last_.valid) {
        return false;
    }

    *out_audit = filter->predict_audit_last_;
    return true;
}

bool ins_ekf_get_last_attitude_prop_audit(
    const InsEkfFilter *filter,
    InsEkfAttitudePropAudit *out_audit)
{
    if (filter == NULL || out_audit == NULL || !filter->attitude_prop_audit_last_.valid) {
        return false;
    }

    *out_audit = filter->attitude_prop_audit_last_;
    return true;
}

uint32_t ins_ekf_gnss_accept_count(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return 0U;
    }

    return filter->gnss_accept_count;
}

uint32_t ins_ekf_gnss_reject_count(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return 0U;
    }

    return filter->gnss_reject_count;
}

/*
 * Product façade export: NED→LLA + nav_mode_select.
 * Estimate-engine facts stay in the filter; NavState is the navigation ABI
 * (see docs/ESTIMATE_ENGINE_VS_NAV_VOCAB.md).
 */
void ins_ekf_export_nav_state(
    const InsEkfFilter *filter,
    NavState *out_state,
    uint32_t timestamp_ms,
    const GpsSample *last_gps)
{
    if (filter == NULL || out_state == NULL || !filter->initialized) {
        return;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    float alt_m = 0.0f;

    geodesy::ned_to_lla(
        filter->ref_lat_deg,
        filter->ref_lon_deg,
        filter->ref_alt_m,
        filter->pos_[0],
        filter->pos_[1],
        filter->pos_[2],
        &lat_deg,
        &lon_deg,
        &alt_m);

    float yaw_rad = 0.0f;
    quat_to_euler321(filter->q_att_, NULL, NULL, &yaw_rad);
    const float heading_deg = navstate_normalize_heading(rad_to_deg(yaw_rad));

    out_state->position = vector3d_make(lat_deg, lon_deg, alt_m);
    out_state->velocity = vector3d_make(
        filter->vel_[0],
        filter->vel_[1],
        -filter->vel_[2]);
    out_state->heading_deg = heading_deg;
    out_state->domain = filter->domain;
    out_state->timestamp_ms = timestamp_ms;

    const bool gps_fix_now = (last_gps != NULL) && last_gps->fix_valid;
    const bool gnss_recent = filter->last_gnss_accept_ms != 0U
        && ((timestamp_ms - filter->last_gnss_accept_ms) <= NAV_MODE_GNSS_RECENT_MS);
    const uint32_t fix_age_ms = (filter->last_gnss_accept_ms == 0U)
        ? timestamp_ms
        : (timestamp_ms - filter->last_gnss_accept_ms);

    NavModeSelectInput sel_in{};
    sel_in.initialized = true;
    sel_in.gps_fix_valid = gps_fix_now;
    sel_in.gnss_accepted_recent = gnss_recent;
    sel_in.gnss_outlier = filter->outlier_detected;
    sel_in.fix_age_ms = fix_age_ms;
    sel_in.satellites = (last_gps != NULL) ? last_gps->satellites : 0U;
    sel_in.imu_cross_check_fail = false; /* applied in BSP after export if needed */

    const NavModeSelectResult sel = nav_mode_select(&sel_in);
    out_state->mode = sel.mode;
    out_state->confidence = sel.confidence;
}

bool ins_ekf_pack_navigation_state(
    const InsEkfFilter *filter,
    uint32_t timestamp_ms,
    uint32_t health_flags,
    NavigationState *out_state)
{
    if (filter == NULL || out_state == NULL || !filter->initialized) {
        return false;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    float alt_m = 0.0f;
    geodesy::ned_to_lla(
        filter->ref_lat_deg,
        filter->ref_lon_deg,
        filter->ref_alt_m,
        filter->pos_[0],
        filter->pos_[1],
        filter->pos_[2],
        &lat_deg,
        &lon_deg,
        &alt_m);

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    quat_to_euler321(filter->q_att_, &roll_rad, &pitch_rad, &yaw_rad);

    const float pos_std_n = sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_N][INS_ERR_POS_N], 0.0f));
    const float pos_std_e = sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_E][INS_ERR_POS_E], 0.0f));
    const float pos_std_d = sqrtf(fmaxf(filter->cov.P[INS_ERR_POS_D][INS_ERR_POS_D], 0.0f));
    const float att_std_roll = sqrtf(fmaxf(filter->cov.P[INS_ERR_ATT_X][INS_ERR_ATT_X], 0.0f));
    const float att_std_pitch = sqrtf(fmaxf(filter->cov.P[INS_ERR_ATT_Y][INS_ERR_ATT_Y], 0.0f));
    const float att_std_yaw = sqrtf(fmaxf(filter->cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z], 0.0f));

    NavigationState packed{};
    packed.timestamp_us = static_cast<uint64_t>(timestamp_ms) * 1000ULL;
    packed.lat_rad = static_cast<double>(deg_to_rad(lat_deg));
    packed.lon_rad = static_cast<double>(deg_to_rad(lon_deg));
    packed.alt_m = alt_m;
    packed.vn_mps = filter->vel_[0];
    packed.ve_mps = filter->vel_[1];
    packed.vd_mps = filter->vel_[2];
    packed.roll_rad = roll_rad;
    packed.pitch_rad = pitch_rad;
    packed.yaw_rad = yaw_rad;
    packed.health_flags = health_flags;
    packed.pos_uncertainty_m = sqrtf(
        (pos_std_n * pos_std_n) + (pos_std_e * pos_std_e) + (pos_std_d * pos_std_d));
    packed.att_uncertainty_rad = sqrtf(
        (att_std_roll * att_std_roll)
        + (att_std_pitch * att_std_pitch)
        + (att_std_yaw * att_std_yaw));

    *out_state = packed;
    return true;
}

void ins_ekf_fill_nhc_attitude_coupling(const float v_body[3], float h_rows_att[2][3])
{
    if (v_body == NULL || h_rows_att == NULL) {
        return;
    }
    /* FD / unit tests always probe the mathematically correct rows. */
    fill_nhc_attitude_coupling_rows(v_body, h_rows_att, INS_EKF_NHC_JACOBIAN_CORRECT);
}

static InsEkfNhcJacobianMode g_nhc_jacobian_default = INS_EKF_NHC_JACOBIAN_CORRECT;

void ins_ekf_set_default_nhc_jacobian_mode(InsEkfNhcJacobianMode mode)
{
    g_nhc_jacobian_default = mode;
}

InsEkfNhcJacobianMode ins_ekf_default_nhc_jacobian_mode(void)
{
    return g_nhc_jacobian_default;
}

static float g_nhc_att_z_forget_default = NAVICORE_INS_EKF_NHC_ATT_Z_FORGET;

static float clamp_nhc_att_z_forget(float lambda)
{
    if (lambda < 0.0f) {
        return 0.0f;
    }
    if (lambda > 1.0f) {
        return 1.0f;
    }
    return lambda;
}

void ins_ekf_set_default_nhc_att_z_forget(float lambda)
{
    g_nhc_att_z_forget_default = clamp_nhc_att_z_forget(lambda);
}

float ins_ekf_default_nhc_att_z_forget(void)
{
    return g_nhc_att_z_forget_default;
}

void ins_ekf_set_nhc_att_z_forget(InsEkfFilter *filter, float lambda)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_z_forget = clamp_nhc_att_z_forget(lambda);
}

float ins_ekf_nhc_att_z_forget(const InsEkfFilter *filter)
{
    if (filter == nullptr) {
        return 0.0f;
    }
    return filter->nhc_att_z_forget;
}

static float g_nhc_att_z_forget_gate_thr_default = NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE;
static float g_nhc_att_z_forget_tmax_s_default = NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_TMAX_S;

void ins_ekf_set_default_nhc_att_z_forget_gate(float thr_rad, float tmax_s)
{
    g_nhc_att_z_forget_gate_thr_default = (thr_rad < 0.0f) ? 0.0f : thr_rad;
    if (tmax_s < 0.0f) {
        g_nhc_att_z_forget_tmax_s_default = NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_TMAX_S;
    } else {
        g_nhc_att_z_forget_tmax_s_default = tmax_s;
    }
}

float ins_ekf_default_nhc_att_z_forget_gate_thr(void)
{
    return g_nhc_att_z_forget_gate_thr_default;
}

float ins_ekf_default_nhc_att_z_forget_tmax_s(void)
{
    return g_nhc_att_z_forget_tmax_s_default;
}

void ins_ekf_set_nhc_att_z_forget_gate(InsEkfFilter *filter, float thr_rad, float tmax_s)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_z_forget_gate_thr = (thr_rad < 0.0f) ? 0.0f : thr_rad;
    filter->nhc_att_z_forget_tmax_s =
        (tmax_s < 0.0f) ? NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_TMAX_S : tmax_s;
}

static uint32_t g_nhc_att_z_forget_grace_ticks_default =
    NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GRACE_TICKS;

void ins_ekf_set_default_nhc_att_z_forget_grace_ticks(uint32_t grace_ticks)
{
    g_nhc_att_z_forget_grace_ticks_default = grace_ticks;
}

uint32_t ins_ekf_default_nhc_att_z_forget_grace_ticks(void)
{
    return g_nhc_att_z_forget_grace_ticks_default;
}

void ins_ekf_set_nhc_att_z_forget_grace_ticks(InsEkfFilter *filter, uint32_t grace_ticks)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_z_forget_grace_ticks = grace_ticks;
}

static bool g_nhc_att_z_forget_gate_norm_default =
#if NAVICORE_INS_EKF_NHC_ATT_Z_FORGET_GATE_NORM
    true
#else
    false
#endif
    ;

void ins_ekf_set_default_nhc_att_z_forget_gate_norm(bool enabled)
{
    g_nhc_att_z_forget_gate_norm_default = enabled;
}

bool ins_ekf_default_nhc_att_z_forget_gate_norm(void)
{
    return g_nhc_att_z_forget_gate_norm_default;
}

void ins_ekf_set_nhc_att_z_forget_gate_norm(InsEkfFilter *filter, bool enabled)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_z_forget_gate_norm = enabled;
}

bool ins_ekf_nhc_att_z_forget_latched(const InsEkfFilter *filter)
{
    return filter != nullptr && filter->nhc_att_z_forget_latched;
}

float ins_ekf_nhc_att_z_forget_fire_t_s(const InsEkfFilter *filter)
{
    if (filter == nullptr) {
        return -1.0f;
    }
    return filter->nhc_att_z_forget_fire_t_s;
}

float ins_ekf_nhc_att_z_sumabs(const InsEkfFilter *filter)
{
    if (filter == nullptr) {
        return 0.0f;
    }
    return filter->nhc_att_z_sumabs;
}

static bool g_nhc_att_z_unobs_default = false;

void ins_ekf_set_default_nhc_att_z_unobs(bool enabled)
{
    g_nhc_att_z_unobs_default = enabled;
}

bool ins_ekf_default_nhc_att_z_unobs(void)
{
    return g_nhc_att_z_unobs_default;
}

void ins_ekf_set_nhc_att_z_unobs(InsEkfFilter *filter, bool enabled)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_z_unobs = enabled;
}

bool ins_ekf_nhc_att_z_unobs(const InsEkfFilter *filter)
{
    return filter != nullptr && filter->nhc_att_z_unobs;
}

void ins_ekf_set_nhc_att_unobs(InsEkfFilter *filter, bool enabled)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_unobs = enabled;
}

bool ins_ekf_nhc_att_unobs(const InsEkfFilter *filter)
{
    return filter != nullptr && filter->nhc_att_unobs;
}

void ins_ekf_set_nhc_att_coherence_gate(InsEkfFilter *filter, bool enabled)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_coherence_gate = enabled;
    if (enabled) {
        filter->nhc_att_gate_open = false;
        filter->nhc_att_gate_ok_accum_s = 0.0f;
        filter->nhc_att_gate_last_imu_ms = 0U;
        filter->nhc_att_gate_open_t_s = -1.0f;
    }
}

bool ins_ekf_nhc_att_coherence_gate(const InsEkfFilter *filter)
{
    return filter != nullptr && filter->nhc_att_coherence_gate;
}

void ins_ekf_configure_nhc_att_coherence_gate(
    InsEkfFilter *filter,
    float vmin_mps,
    float yaw_max_deg,
    float hold_s)
{
    if (filter == nullptr) {
        return;
    }
    if (vmin_mps > 0.0f) {
        filter->nhc_att_gate_vmin_mps = vmin_mps;
    }
    if (yaw_max_deg > 0.0f) {
        filter->nhc_att_gate_yaw_max_rad =
            yaw_max_deg * (static_cast<float>(M_PI) / 180.0f);
    }
    if (hold_s > 0.0f) {
        filter->nhc_att_gate_hold_s = hold_s;
    }
}

void ins_ekf_set_nhc_att_gate_gnss_valid(InsEkfFilter *filter, bool valid)
{
    if (filter == nullptr) {
        return;
    }
    filter->nhc_att_gate_gnss_valid = valid;
}

bool ins_ekf_nhc_att_gate_open(const InsEkfFilter *filter)
{
    return filter != nullptr && filter->nhc_att_gate_open;
}

float ins_ekf_nhc_att_gate_open_t_s(const InsEkfFilter *filter)
{
    if (filter == nullptr) {
        return -1.0f;
    }
    return filter->nhc_att_gate_open_t_s;
}

void ins_ekf_set_nhc_jacobian_mode(InsEkfFilter *filter, InsEkfNhcJacobianMode mode)
{
    if (filter == NULL) {
        return;
    }
    filter->nhc_jacobian_mode = mode;
}

InsEkfNhcJacobianMode ins_ekf_nhc_jacobian_mode(const InsEkfFilter *filter)
{
    if (filter == NULL) {
        return INS_EKF_NHC_JACOBIAN_CORRECT;
    }
    return filter->nhc_jacobian_mode;
}

const char *ins_ekf_nhc_jacobian_mode_name(InsEkfNhcJacobianMode mode)
{
    switch (mode) {
    case INS_EKF_NHC_JACOBIAN_LEGACY_BUG:
        return "legacy_bug";
    case INS_EKF_NHC_JACOBIAN_CORRECT:
    default:
        return "correct";
    }
}

bool ins_ekf_parse_nhc_jacobian_mode(const char *text, InsEkfNhcJacobianMode *out_mode)
{
    if (text == NULL || out_mode == NULL || text[0] == '\0') {
        return false;
    }
    if (strcmp(text, "correct") == 0 || strcmp(text, "fixed") == 0) {
        *out_mode = INS_EKF_NHC_JACOBIAN_CORRECT;
        return true;
    }
    if (strcmp(text, "legacy") == 0 || strcmp(text, "legacy_bug") == 0
        || strcmp(text, "buggy") == 0) {
        *out_mode = INS_EKF_NHC_JACOBIAN_LEGACY_BUG;
        return true;
    }
    return false;
}

void ins_ekf_kinematics_quat_to_dcm_bn(const float q[4], float dcm[3][3])
{
    if (q == NULL || dcm == NULL) {
        return;
    }
    InsEkfMat3 dcm_local{};
    quat_to_dcm_bn(q, dcm_local);
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            dcm[r][c] = dcm_local[r][c];
        }
    }
}

void ins_ekf_kinematics_ned_to_body(const float dcm[3][3], const float ned[3], float body[3])
{
    if (dcm == NULL || ned == NULL || body == NULL) {
        return;
    }
    InsEkfMat3 dcm_local{};
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            dcm_local[r][c] = dcm[r][c];
        }
    }
    ned_to_body(dcm_local, ned, body);
}

void ins_ekf_kinematics_quat_apply_small_angle_error(float q[4], const float dtheta[3])
{
    if (q == NULL || dtheta == NULL) {
        return;
    }
    quat_apply_small_angle_error(q, dtheta);
}

/* =============================================================================
 * EKF v2 — núcleos reescritos (predict / GNSS / seed / NHC). v1 intacto arriba.
 * ============================================================================= */

#include "ins_ekf_v2.hpp"

const char *ins_ekf_core_version_name(InsEkfCoreVersion version)
{
    switch (version) {
    case INS_EKF_CORE_V1:
        return "v1";
    case INS_EKF_CORE_V2:
        return "v2";
    default:
        return "unknown";
    }
}

bool ins_ekf_core_version_parse(const char *text, InsEkfCoreVersion *out_version)
{
    if (text == NULL || out_version == NULL) {
        return false;
    }
    if (strcmp(text, "v1") == 0 || strcmp(text, "V1") == 0) {
        *out_version = INS_EKF_CORE_V1;
        return true;
    }
    if (strcmp(text, "v2") == 0 || strcmp(text, "V2") == 0) {
        *out_version = INS_EKF_CORE_V2;
        return true;
    }
    return false;
}

static void ins_ekf_v2_predict_nominal(InsEkfFilter *filter, const ImuSample &imu_sample, float dt_s)
{
    if (filter == NULL || !filter->initialized || !imu_sample.valid || dt_s <= 0.0f) {
        return;
    }

    InsEkfFilter &self = *filter;
    ins_ekf_log_cov_step_audit(&self, "predict", "pre");

    self.predict_audit_last_.valid = false;
    self.predict_audit_last_.dt_s = dt_s;
    self.predict_audit_last_.imu_body_mps2[0] = imu_sample.accel_mps2[0];
    self.predict_audit_last_.imu_body_mps2[1] = imu_sample.accel_mps2[1];
    self.predict_audit_last_.imu_body_mps2[2] = imu_sample.accel_mps2[2];
    self.predict_audit_last_.pos_pre_m[0] = self.pos_[0];
    self.predict_audit_last_.pos_pre_m[1] = self.pos_[1];
    self.predict_audit_last_.pos_pre_m[2] = self.pos_[2];
    self.predict_audit_last_.vel_pre_mps[0] = self.vel_[0];
    self.predict_audit_last_.vel_pre_mps[1] = self.vel_[1];
    self.predict_audit_last_.vel_pre_mps[2] = self.vel_[2];
    self.predict_audit_last_.kinematic_pos_residual_m = 0.0f;
    self.predict_audit_last_.body_ned_roundtrip_err_mps = 0.0f;
    self.predict_audit_last_.euler_dcm_frob = 0.0f;
    self.predict_audit_last_.roll_rad = 0.0f;
    self.predict_audit_last_.pitch_rad = 0.0f;
    self.predict_audit_last_.yaw_rad = 0.0f;
    for (uint8_t i = 0U; i < 3U; ++i) {
        self.predict_audit_last_.vel_body_mps[i] = 0.0f;
        self.predict_audit_last_.term_R_imu_mps2[i] = 0.0f;
        self.predict_audit_last_.term_R_neg_bias_mps2[i] = 0.0f;
        self.predict_audit_last_.term_gravity_mps2[i] = 0.0f;
        self.predict_audit_last_.term_coriolis_mps2[i] = 0.0f;
    }

    float w_corr[3]{};
    float a_corr[3]{};
    vec3_sub(imu_sample.gyro_radps, self.bias_g_, w_corr);
    vec3_sub(imu_sample.accel_mps2, self.bias_a_, a_corr);
    self.predict_audit_last_.w_corr_radps[0] = w_corr[0];
    self.predict_audit_last_.w_corr_radps[1] = w_corr[1];
    self.predict_audit_last_.w_corr_radps[2] = w_corr[2];
    self.predict_audit_last_.a_corr_mps2[0] = a_corr[0];
    self.predict_audit_last_.a_corr_mps2[1] = a_corr[1];
    self.predict_audit_last_.a_corr_mps2[2] = a_corr[2];
    self.predict_audit_last_.bias_a_mps2[0] = self.bias_a_[0];
    self.predict_audit_last_.bias_a_mps2[1] = self.bias_a_[1];
    self.predict_audit_last_.bias_a_mps2[2] = self.bias_a_[2];
    self.predict_audit_last_.bias_g_radps[0] = self.bias_g_[0];
    self.predict_audit_last_.bias_g_radps[1] = self.bias_g_[1];
    self.predict_audit_last_.bias_g_radps[2] = self.bias_g_[2];

    self.attitude_prop_audit_last_.valid = false;
    self.attitude_prop_audit_last_.dt_s = dt_s;
    self.attitude_prop_audit_last_.gyro_raw_radps[0] = imu_sample.gyro_radps[0];
    self.attitude_prop_audit_last_.gyro_raw_radps[1] = imu_sample.gyro_radps[1];
    self.attitude_prop_audit_last_.gyro_raw_radps[2] = imu_sample.gyro_radps[2];
    self.attitude_prop_audit_last_.gyro_bias_radps[0] = self.bias_g_[0];
    self.attitude_prop_audit_last_.gyro_bias_radps[1] = self.bias_g_[1];
    self.attitude_prop_audit_last_.gyro_bias_radps[2] = self.bias_g_[2];
    self.attitude_prop_audit_last_.gyro_corr_radps[0] = w_corr[0];
    self.attitude_prop_audit_last_.gyro_corr_radps[1] = w_corr[1];
    self.attitude_prop_audit_last_.gyro_corr_radps[2] = w_corr[2];
    self.attitude_prop_audit_last_.delta_theta_integrated_rad[0] = w_corr[0] * dt_s;
    self.attitude_prop_audit_last_.delta_theta_integrated_rad[1] = w_corr[1] * dt_s;
    self.attitude_prop_audit_last_.delta_theta_integrated_rad[2] = w_corr[2] * dt_s;
    self.attitude_prop_audit_last_.delta_theta_integrated_mag_rad = std::sqrt(
        (self.attitude_prop_audit_last_.delta_theta_integrated_rad[0]
         * self.attitude_prop_audit_last_.delta_theta_integrated_rad[0])
        + (self.attitude_prop_audit_last_.delta_theta_integrated_rad[1]
           * self.attitude_prop_audit_last_.delta_theta_integrated_rad[1])
        + (self.attitude_prop_audit_last_.delta_theta_integrated_rad[2]
           * self.attitude_prop_audit_last_.delta_theta_integrated_rad[2]));
    self.attitude_prop_audit_last_.q_before[0] = self.q_att_[0];
    self.attitude_prop_audit_last_.q_before[1] = self.q_att_[1];
    self.attitude_prop_audit_last_.q_before[2] = self.q_att_[2];
    self.attitude_prop_audit_last_.q_before[3] = self.q_att_[3];
    quat_to_euler321(
        self.attitude_prop_audit_last_.q_before,
        &self.attitude_prop_audit_last_.roll_before_rad,
        &self.attitude_prop_audit_last_.pitch_before_rad,
        &self.attitude_prop_audit_last_.yaw_before_rad);

    /* Actitud v2: misma integración 1er orden + renormalización explícita. */
    quat_integrate_first_order(self.q_att_, w_corr, dt_s);
    quat_normalize(self.q_att_);

    self.attitude_prop_audit_last_.q_after[0] = self.q_att_[0];
    self.attitude_prop_audit_last_.q_after[1] = self.q_att_[1];
    self.attitude_prop_audit_last_.q_after[2] = self.q_att_[2];
    self.attitude_prop_audit_last_.q_after[3] = self.q_att_[3];
    quat_to_euler321(
        self.attitude_prop_audit_last_.q_after,
        &self.attitude_prop_audit_last_.roll_after_rad,
        &self.attitude_prop_audit_last_.pitch_after_rad,
        &self.attitude_prop_audit_last_.yaw_after_rad);
    self.attitude_prop_audit_last_.valid = true;

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(self.q_att_, dcm_bn);
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            self.predict_audit_last_.dcm_bn[r][c] = dcm_bn[r][c];
        }
    }

    /* Presupuesto: a_lin = R·imu − R·bias − g  (Coriolis ≡ 0). */
    float term_R_imu[3]{};
    float term_R_bias[3]{};
    body_to_ned(dcm_bn, imu_sample.accel_mps2, term_R_imu);
    body_to_ned(dcm_bn, self.bias_a_, term_R_bias);

    float a_n[3]{};
    for (uint8_t i = 0U; i < 3U; ++i) {
        self.predict_audit_last_.term_R_imu_mps2[i] = term_R_imu[i];
        self.predict_audit_last_.term_R_neg_bias_mps2[i] = -term_R_bias[i];
        self.predict_audit_last_.term_gravity_mps2[i] = -kGravityNed[i];
        self.predict_audit_last_.term_coriolis_mps2[i] = 0.0f;
        a_n[i] = term_R_imu[i] - term_R_bias[i] - kGravityNed[i];
    }

    self.predict_audit_last_.a_nav_mps2[0] = term_R_imu[0] - term_R_bias[0];
    self.predict_audit_last_.a_nav_mps2[1] = term_R_imu[1] - term_R_bias[1];
    self.predict_audit_last_.a_nav_mps2[2] = term_R_imu[2] - term_R_bias[2];
    self.predict_audit_last_.a_lin_mps2[0] = a_n[0];
    self.predict_audit_last_.a_lin_mps2[1] = a_n[1];
    self.predict_audit_last_.a_lin_mps2[2] = a_n[2];

    /* Integración trapezoidal: v+ = v− + a·Δt; p+ = p− + 0.5·(v−+v+)·Δt. */
    const float vel_pre[3] = {self.vel_[0], self.vel_[1], self.vel_[2]};
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float dv = a_n[i] * dt_s;
        self.vel_[i] += dv;
        self.pos_[i] += 0.5f * (vel_pre[i] + self.vel_[i]) * dt_s;
    }
    {
        const float dv_log[3] = {a_n[0] * dt_s, a_n[1] * dt_s, a_n[2] * dt_s};
        ins_ekf_log_vel_modification(&self, "predict", dv_log, NULL, NULL, false);
    }

    self.predict_audit_last_.vel_ned_mps[0] = self.vel_[0];
    self.predict_audit_last_.vel_ned_mps[1] = self.vel_[1];
    self.predict_audit_last_.vel_ned_mps[2] = self.vel_[2];
    self.predict_audit_last_.pos_ned_m[0] = self.pos_[0];
    self.predict_audit_last_.pos_ned_m[1] = self.pos_[1];
    self.predict_audit_last_.pos_ned_m[2] = self.pos_[2];

    /* I1 (trapezoidal): |p+ − (p− + 0.5·(v−+v+)·Δt)| ~ 0. */
    {
        float kin_sq = 0.0f;
        for (uint8_t i = 0U; i < 3U; ++i) {
            const float expected =
                self.predict_audit_last_.pos_pre_m[i]
                + (0.5f * (vel_pre[i] + self.vel_[i]) * dt_s);
            const float e = self.pos_[i] - expected;
            kin_sq += e * e;
        }
        self.predict_audit_last_.kinematic_pos_residual_m = sqrtf(kin_sq);
    }

    {
        float v_body[3]{};
        ned_to_body(dcm_bn, self.vel_, v_body);
        self.predict_audit_last_.vel_body_mps[0] = v_body[0];
        self.predict_audit_last_.vel_body_mps[1] = v_body[1];
        self.predict_audit_last_.vel_body_mps[2] = v_body[2];
        float v_back[3]{};
        body_to_ned(dcm_bn, v_body, v_back);
        float rt_sq = 0.0f;
        for (uint8_t i = 0U; i < 3U; ++i) {
            const float e = self.vel_[i] - v_back[i];
            rt_sq += e * e;
        }
        self.predict_audit_last_.body_ned_roundtrip_err_mps = sqrtf(rt_sq);
    }

    {
        float roll = 0.0f;
        float pitch = 0.0f;
        float yaw = 0.0f;
        quat_to_euler321(self.q_att_, &roll, &pitch, &yaw);
        self.predict_audit_last_.roll_rad = roll;
        self.predict_audit_last_.pitch_rad = pitch;
        self.predict_audit_last_.yaw_rad = yaw;
        float q_from_euler[4]{};
        euler321_to_quat(roll, pitch, yaw, q_from_euler);
        InsEkfMat3 dcm_from_euler{};
        quat_to_dcm_bn(q_from_euler, dcm_from_euler);
        float frob_sq = 0.0f;
        for (uint8_t r = 0U; r < 3U; ++r) {
            for (uint8_t c = 0U; c < 3U; ++c) {
                const float e = dcm_bn[r][c] - dcm_from_euler[r][c];
                frob_sq += e * e;
            }
        }
        self.predict_audit_last_.euler_dcm_frob = sqrtf(frob_sq);
    }

    self.predict_audit_last_.valid = true;

    InsEkfMat3 accel_skew{};
    skew_symmetric(a_corr, accel_skew);
    InsEkfMat3 f_va_raw{};
    mat3_mul(dcm_bn, accel_skew, f_va_raw);
    InsEkfMat3 f_va{};
    mat3_scale(f_va_raw, -dt_s, f_va);
    InsEkfMat3 f_vba{};
    mat3_scale(dcm_bn, -dt_s, f_vba);
    InsEkfMat3 w_skew{};
    skew_symmetric(w_corr, w_skew);
    InsEkfMat3 f_aa{};
    mat3_identity(f_aa);
    InsEkfMat3 w_dt{};
    mat3_scale(w_skew, -dt_s, w_dt);
    mat3_add(f_aa, w_dt, f_aa);
    InsEkfMat3 f_bg{};
    mat3_identity(f_bg);
    for (uint8_t i = 0U; i < 3U; ++i) {
        f_bg[i][i] = -dt_s;
    }

    ins_ekf_build_process_noise(&self, dt_s, self.scratch_a_);
    ins_ekf_propagate_covariance_sparse(
        self.cov.P,
        f_va,
        f_vba,
        f_aa,
        f_bg,
        dt_s,
        self.scratch_a_,
        self.cov.P,
        self.scratch_b_);

    self.predict_audit_last_.f_dp_dv_dt_s = dt_s;
    for (uint8_t r = 0U; r < 3U; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            self.predict_audit_last_.f_va[r][c] = f_va[r][c];
            self.predict_audit_last_.f_vba[r][c] = f_vba[r][c];
        }
    }

    ins_ekf_log_cov_step_audit(&self, "predict", "post");
    self.last_imu_timestamp_ms = imu_sample.timestamp_ms;
}

bool ins_ekf_v2_predict(InsEkfFilter *filter, const ImuSample *imu)
{
    if (filter == NULL || imu == NULL || !imu->valid || !filter->initialized) {
        return false;
    }

    ins_ekf_reset_vel_pipeline_audit(filter);
    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->vel_pipeline_audit_last_.vel_before_predict[i] = filter->vel_[i];
    }

    const float dt_s = ins_ekf_predict_dt_s(filter, imu->timestamp_ms);
    ins_ekf_v2_predict_nominal(filter, *imu, dt_s);

    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->vel_pipeline_audit_last_.vel_after_predict[i] = filter->vel_[i];
        filter->vel_pipeline_audit_last_.dv_predict[i] =
            filter->vel_pipeline_audit_last_.vel_after_predict[i]
            - filter->vel_pipeline_audit_last_.vel_before_predict[i];
        filter->vel_pipeline_audit_last_.vel_after_nhc[i] =
            filter->vel_pipeline_audit_last_.vel_after_predict[i];
        filter->vel_pipeline_audit_last_.dv_nhc[i] = 0.0f;
    }
    filter->vel_pipeline_audit_last_.nhc_applied = false;
    filter->vel_pipeline_audit_last_.valid = true;
    return true;
}

bool ins_ekf_v2_update_gnss(InsEkfFilter *filter, const GpsSample *gps)
{
    if (filter == NULL || gps == NULL || !gps->fix_valid || !filter->initialized) {
        return false;
    }

    const InsEkfGnssObsMode requested = filter->gnss_obs_mode;
    const bool polish = filter->v2_polish != 0U;
    filter->gnss_v2_accepted_pos = 0U;
    filter->gnss_v2_accepted_vel = 0U;
    filter->gnss_v2_nis_pos = 0.0f;
    filter->gnss_v2_nis_vel = 0.0f;

    bool accepted_pos = false;
    bool accepted_vel = false;
    float nis_pos = 0.0f;
    float nis_vel = 0.0f;

    float innov_pos[3] = {0.0f, 0.0f, 0.0f};
    float innov_vel[2] = {0.0f, 0.0f};
    float nis_contrib_pos[3] = {0.0f, 0.0f, 0.0f};
    float nis_contrib_vel[2] = {0.0f, 0.0f};
    float s_diag_pos[3] = {0.0f, 0.0f, 0.0f};
    float s_diag_vel[2] = {0.0f, 0.0f};
    uint8_t reject_pos = 0U;
    uint8_t reject_vel = 0U;

    if (requested == INS_EKF_GNSS_OBS_VEL_ONLY) {
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF;
        accepted_vel = filter->update_gnss(*gps, &nis_vel);
        filter->gnss_v2_nis_vel = nis_vel;
        filter->gnss_v2_accepted_vel = accepted_vel ? 1U : 0U;
        if (accepted_vel) {
            ++filter->gnss_accept_count;
            filter->last_gnss_accept_ms = gps->timestamp_ms;
            filter->outlier_detected = false;
            return true;
        }
        ++filter->gnss_reject_count;
        return false;
    }

    /* --- Posición 3-DoF --- */
    const float saved_pos_var = filter->gnss_pos_var_m2;
    if (polish) {
        /* R adaptativo: innov grande → confiar menos en GPS, pero no abandonar. */
        float z_n = 0.0f;
        float z_e = 0.0f;
        float z_d = 0.0f;
        geodesy::lla_to_ned(
            filter->ref_lat_deg,
            filter->ref_lon_deg,
            filter->ref_alt_m,
            gps->position.x,
            gps->position.y,
            gps->position.z,
            &z_n,
            &z_e,
            &z_d);
        const float innov_h = sqrtf(
            ((z_n - filter->pos_[0]) * (z_n - filter->pos_[0]))
            + ((z_e - filter->pos_[1]) * (z_e - filter->pos_[1])));
        if (innov_h > NAVICORE_INS_EKF_V2_POLISH_POS_INNOV_SOFT_M) {
            const float scale =
                innov_h / NAVICORE_INS_EKF_V2_POLISH_POS_INNOV_SOFT_M;
            filter->gnss_pos_var_m2 = saved_pos_var * scale * scale;
        }
    }

    filter->gnss_obs_mode = INS_EKF_GNSS_OBS_POS;
    /* Baseline: bypass. Polish: umbral laxo (sigue anclando; R ya suaviza tirones). */
    filter->nis_threshold =
        polish ? NAVICORE_INS_EKF_V2_NIS_POS_THRESHOLD : 1.0e6f;
    accepted_pos = filter->update_gnss(*gps, &nis_pos);
    if (polish && !accepted_pos) {
        /* Nunca abandonar: reintentar con bypass tras R ya inflado. */
        filter->nis_threshold = 1.0e6f;
        accepted_pos = filter->update_gnss(*gps, &nis_pos);
    }
    filter->gnss_pos_var_m2 = saved_pos_var;

    nis_pos = filter->gnss_nis_last;
    reject_pos = filter->gnss_last_reject_reason;
    innov_pos[0] = filter->gnss_innovation_last[0];
    innov_pos[1] = filter->gnss_innovation_last[1];
    innov_pos[2] = filter->gnss_innovation_last[2];
    nis_contrib_pos[0] = filter->gnss_nis_contrib[0];
    nis_contrib_pos[1] = filter->gnss_nis_contrib[1];
    nis_contrib_pos[2] = filter->gnss_nis_contrib[2];
    s_diag_pos[0] = filter->gnss_s_diag[0];
    s_diag_pos[1] = filter->gnss_s_diag[1];
    s_diag_pos[2] = filter->gnss_s_diag[2];
    filter->gnss_v2_nis_pos = nis_pos;
    filter->gnss_v2_accepted_pos = accepted_pos ? 1U : 0U;

    /* --- Velocidad 2-DoF (coherente en polish) --- */
    bool try_vel =
        (requested == INS_EKF_GNSS_OBS_POS_VEL) && (gps->speed_mps > 0.0f);
    if (try_vel && polish) {
        const bool speed_ok = gps->speed_mps >= NAVICORE_INS_EKF_V2_POLISH_VEL_MIN_MPS;
        float yaw_rad = 0.0f;
        quat_to_euler321(filter->q_att_, NULL, NULL, &yaw_rad);
        const float course_rad = gps->course_deg * kDegToRadF;
        float d = course_rad - yaw_rad;
        while (d > kPiF) {
            d -= 2.0f * kPiF;
        }
        while (d < -kPiF) {
            d += 2.0f * kPiF;
        }
        const float course_yaw_abs_deg = fabsf(d) * kRadToDegF;
        const bool heading_ok =
            course_yaw_abs_deg <= NAVICORE_INS_EKF_V2_POLISH_COURSE_YAW_MAX_DEG;
        try_vel = speed_ok && heading_ok && std::isfinite(gps->course_deg);
    }

    const float saved_vel_var = filter->gnss_vel_var_m2_h;
    if (try_vel) {
        if (polish) {
            filter->gnss_vel_var_m2_h =
                saved_vel_var * NAVICORE_INS_EKF_V2_POLISH_VEL_R_SCALE;
        }
        filter->gnss_obs_mode = INS_EKF_GNSS_OBS_VEL_ONLY;
        filter->nis_threshold = polish
            ? (NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF * 1.5f)
            : NAVICORE_INS_EKF_NIS_THRESHOLD_VEL_2DOF;
        accepted_vel = filter->update_gnss(*gps, &nis_vel);
        filter->gnss_vel_var_m2_h = saved_vel_var;
        nis_vel = filter->gnss_nis_last;
        reject_vel = filter->gnss_last_reject_reason;
        innov_vel[0] = filter->gnss_innovation_full[0];
        innov_vel[1] = filter->gnss_innovation_full[1];
        nis_contrib_vel[0] = filter->gnss_nis_contrib[0];
        nis_contrib_vel[1] = filter->gnss_nis_contrib[1];
        s_diag_vel[0] = filter->gnss_s_diag[0];
        s_diag_vel[1] = filter->gnss_s_diag[1];
        filter->gnss_v2_nis_vel = nis_vel;
        filter->gnss_v2_accepted_vel = accepted_vel ? 1U : 0U;

        /* Polish: si vel aceptada y speed alta, alinear yaw suave hacia course. */
        if (polish && accepted_vel && gps->speed_mps >= NAVICORE_INS_EKF_V2_POLISH_VEL_MIN_MPS) {
            float roll = 0.0f;
            float pitch = 0.0f;
            float yaw = 0.0f;
            quat_to_euler321(filter->q_att_, &roll, &pitch, &yaw);
            const float course_rad = gps->course_deg * kDegToRadF;
            float d = course_rad - yaw;
            while (d > kPiF) {
                d -= 2.0f * kPiF;
            }
            while (d < -kPiF) {
                d += 2.0f * kPiF;
            }
            const float alpha = 0.15f; /* blend suave, no hard set */
            euler321_to_quat(roll, pitch, yaw + alpha * d, filter->q_att_);
            quat_normalize(filter->q_att_);
        }
    }

    filter->gnss_obs_mode = requested;
    if (requested == INS_EKF_GNSS_OBS_POS_VEL) {
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD_POS_VEL_5DOF;
    } else {
        filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD;
    }

    filter->gnss_innovation_last[0] = innov_pos[0];
    filter->gnss_innovation_last[1] = innov_pos[1];
    filter->gnss_innovation_last[2] = innov_pos[2];
    filter->gnss_innovation_full[0] = innov_pos[0];
    filter->gnss_innovation_full[1] = innov_pos[1];
    filter->gnss_innovation_full[2] = innov_pos[2];
    filter->gnss_innovation_full[3] = try_vel ? innov_vel[0] : 0.0f;
    filter->gnss_innovation_full[4] = try_vel ? innov_vel[1] : 0.0f;
    filter->gnss_nis_contrib[0] = nis_contrib_pos[0];
    filter->gnss_nis_contrib[1] = nis_contrib_pos[1];
    filter->gnss_nis_contrib[2] = nis_contrib_pos[2];
    filter->gnss_nis_contrib[3] = try_vel ? nis_contrib_vel[0] : 0.0f;
    filter->gnss_nis_contrib[4] = try_vel ? nis_contrib_vel[1] : 0.0f;
    filter->gnss_s_diag[0] = s_diag_pos[0];
    filter->gnss_s_diag[1] = s_diag_pos[1];
    filter->gnss_s_diag[2] = s_diag_pos[2];
    filter->gnss_s_diag[3] = try_vel ? s_diag_vel[0] : 0.0f;
    filter->gnss_s_diag[4] = try_vel ? s_diag_vel[1] : 0.0f;
    filter->gnss_nis_last = nis_pos;
    filter->gnss_last_n_meas = try_vel ? 5U : 3U;

    if (accepted_pos) {
        filter->gnss_last_accepted = 1U;
        filter->gnss_last_reject_reason = 0U;
        filter->outlier_detected = false;
        ++filter->gnss_accept_count;
        filter->last_gnss_accept_ms = gps->timestamp_ms;
        return true;
    }

    filter->gnss_last_accepted = 0U;
    filter->gnss_last_reject_reason = (reject_pos != 0U) ? reject_pos : 1U;
    filter->outlier_detected = true;
    ++filter->gnss_reject_count;
    (void)reject_vel;
    (void)accepted_vel;
    return false;
}

void ins_ekf_v2_set_polish(InsEkfFilter *filter, bool enabled)
{
    if (filter == NULL) {
        return;
    }
    filter->v2_polish = enabled ? 1U : 0U;
}

bool ins_ekf_v2_polish_enabled(const InsEkfFilter *filter)
{
    return filter != NULL && filter->v2_polish != 0U;
}

bool ins_ekf_v2_seed_from_gnss(
    InsEkfFilter *filter,
    const GpsSample *gps,
    NavDomain domain)
{
    if (filter == NULL || gps == NULL || !gps->fix_valid) {
        return false;
    }

    const float yaw_rad = static_cast<float>(gps->course_deg * M_PI / 180.0);
    ins_ekf_init(filter, gps->position, yaw_rad, domain);
    ins_ekf_set_nhc_enabled(filter, false);

    /* Seed v explícito desde GNSS (H1/H2); nunca arrancar en v=0 si hay speed. */
    if (gps->speed_mps > 0.0f && std::isfinite(gps->speed_mps) && std::isfinite(gps->course_deg)) {
        const float course_rad = static_cast<float>(gps->course_deg * M_PI / 180.0);
        filter->vel_[0] = gps->speed_mps * std::cos(course_rad);
        filter->vel_[1] = gps->speed_mps * std::sin(course_rad);
        filter->vel_[2] = 0.0f;
        /* Yaw ya viene de course vía init; reforzar P_yaw moderada. */
        filter->cov.P[INS_ERR_ATT_Z][INS_ERR_ATT_Z] =
            NAVICORE_INS_EKF_INIT_ATT_YAW_VAR_RAD2 * 0.25f;
    } else {
        filter->vel_[0] = 0.0f;
        filter->vel_[1] = 0.0f;
        filter->vel_[2] = 0.0f;
    }

    filter->gnss_v2_accepted_pos = 0U;
    filter->gnss_v2_accepted_vel = 0U;
    filter->gnss_v2_nis_pos = 0.0f;
    filter->gnss_v2_nis_vel = 0.0f;
    return true;
}

bool ins_ekf_v2_maybe_update_nhc(InsEkfFilter *filter, uint32_t now_imu_ms)
{
    if (filter == NULL || !filter->initialized || !filter->nhc_enabled) {
        return false;
    }

    const bool polish = filter->v2_polish != 0U;
    const float gap_thr =
        polish ? NAVICORE_INS_EKF_V2_POLISH_NHC_GAP_S : NAVICORE_INS_EKF_V2_NHC_GNSS_GAP_S;

    /* Doc 17 / polish: NHC solo con gap GNSS (entre fixes), nunca EVERY tick. */
    if (filter->last_gnss_accept_ms > 0U && now_imu_ms >= filter->last_gnss_accept_ms) {
        const float gap_s =
            static_cast<float>(now_imu_ms - filter->last_gnss_accept_ms) * 0.001f;
        if (gap_s < gap_thr) {
            return false;
        }
    }

    /* Polish: solo si hay movimiento horizontal (no frenar en parado). */
    if (polish) {
        const float vh = sqrtf(
            (filter->vel_[0] * filter->vel_[0]) + (filter->vel_[1] * filter->vel_[1]));
        if (vh < NAVICORE_INS_EKF_V2_POLISH_VEL_MIN_MPS) {
            return false;
        }
    }

    filter->nhc_tick_counter++;
    const uint32_t nhc_stride =
        (filter->nhc_every_n_ticks == 0U) ? 1U : filter->nhc_every_n_ticks;
    if ((filter->nhc_tick_counter % nhc_stride) != 0U) {
        return false;
    }

    const float vel_before_nhc[3] = {
        filter->vel_[0],
        filter->vel_[1],
        filter->vel_[2],
    };
    if (!filter->update_nhc()) {
        return false;
    }

    filter->vel_pipeline_audit_last_.nhc_applied = true;
    for (uint8_t i = 0U; i < 3U; ++i) {
        filter->vel_pipeline_audit_last_.vel_after_nhc[i] = filter->vel_[i];
        filter->vel_pipeline_audit_last_.dv_nhc[i] = filter->vel_[i] - vel_before_nhc[i];
    }
    return true;
}
