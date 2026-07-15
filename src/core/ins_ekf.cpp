#include "ins_ekf.hpp"

#include "math_utils.hpp"

#include <math.h>
#include <string.h>

#ifndef NAVICORE_INS_EKF_PI_F
#define NAVICORE_INS_EKF_PI_F 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
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

void mat15_mul(const InsEkfMat15 a, const InsEkfMat15 b, InsEkfMat15 out)
{
    InsEkfMat15 acc{};

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                sum += a[r][k] * b[k][c];
            }
            acc[r][c] = sum;
        }
    }

    mat_copy(acc, out);
}

/*
 * Joseph: P = (I - KH) P (I - KH)^T + K R K^T
 * H observa solo posicion (3 primeros estados de error). R diagonal.
 */
void ins_ekf_covariance_joseph_update(
    InsEkfMat15 p_in,
    const float k_gain[INS_EKF_STATE_DIM][3],
    float meas_var_m2,
    InsEkfMat15 p_out)
{
    InsEkfMat15 a_mat{};
    InsEkfMat15 ap_mat{};
    InsEkfMat15 a_transpose{};
    InsEkfMat15 apa_mat{};

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            a_mat[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            a_mat[r][INS_ERR_POS_N + i] -= k_gain[r][i];
        }
    }

    mat15_mul(a_mat, p_in, ap_mat);
    mat15_transpose(a_mat, a_transpose);
    mat15_mul(ap_mat, a_transpose, apa_mat);
    mat_copy(apa_mat, p_out);

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
    InsEkfMat15 p_out)
{
    InsEkfMat15 a_mat{};
    InsEkfMat15 ap_mat{};
    InsEkfMat15 a_transpose{};
    InsEkfMat15 apa_mat{};

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            a_mat[r][c] = (r == c) ? 1.0f : 0.0f;
            for (uint8_t i = 0U; i < 2U; ++i) {
                a_mat[r][c] -= k_gain[r][i] * h_rows[i][c];
            }
        }
    }

    mat15_mul(a_mat, p_in, ap_mat);
    mat15_transpose(a_mat, a_transpose);
    mat15_mul(ap_mat, a_transpose, apa_mat);
    mat_copy(apa_mat, p_out);

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
    const float det = (s[0][0] * s[1][1]) - (s[0][1] * s[1][0]);
    if (fabsf(det) <= 1.0e-12f) {
        return false;
    }

    const float inv_det = 1.0f / det;
    inv_out[0][0] = s[1][1] * inv_det;
    inv_out[0][1] = -s[0][1] * inv_det;
    inv_out[1][0] = -s[1][0] * inv_det;
    inv_out[1][1] = s[0][0] * inv_det;
    return true;
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
    const float norm_sq =
        (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);

    if (norm_sq <= 1.0e-12f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    const float inv_norm = 1.0f / sqrtf(norm_sq);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

void yaw_rad_to_quat(float yaw_rad, float q[4])
{
    const float half_yaw = 0.5f * yaw_rad;
    q[0] = cosf(half_yaw);
    q[1] = 0.0f;
    q[2] = 0.0f;
    q[3] = sinf(half_yaw);
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

    if (k >= INS_ERR_VEL_N && k < INS_ERR_VEL_N + 3U && c >= INS_ERR_POS_N && c < INS_ERR_POS_N + 3U) {
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

void latlonalt_to_ned(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float lat_deg,
    float lon_deg,
    float alt_m,
    float *north_m,
    float *east_m,
    float *down_m)
{
    const float dlat_m = (lat_deg - ref_lat_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float lat_rad = deg_to_rad((ref_lat_deg + lat_deg) * 0.5f);
    const float dlon_m = (lon_deg - ref_lon_deg) * NAVICORE_METERS_PER_DEG_LAT * cosf(lat_rad);

    *north_m = dlat_m;
    *east_m = dlon_m;
    *down_m = ref_alt_m - alt_m;
}

void ned_to_latlonalt(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    float north_m,
    float east_m,
    float down_m,
    float *lat_deg,
    float *lon_deg,
    float *alt_m)
{
    const float lat_rad = deg_to_rad(ref_lat_deg);
    const float cos_lat = cosf(lat_rad);

    *lat_deg = ref_lat_deg + (north_m / NAVICORE_METERS_PER_DEG_LAT);
    if (fabsf(cos_lat) > 1.0e-6f) {
        *lon_deg = ref_lon_deg + (east_m / (NAVICORE_METERS_PER_DEG_LAT * cos_lat));
    } else {
        *lon_deg = ref_lon_deg;
    }
    *alt_m = ref_alt_m - down_m;
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
    InsEkfMat15 p_out)
{
    InsEkfMat15 temp{};
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

            temp[pos_i][c] = p_in[pos_i][c] + (dt_s * p_in[vel_i][c]);

            float vel_row = p_in[vel_i][c];
            for (uint8_t j = 0U; j < 3U; ++j) {
                vel_row += f_va[i][j] * p_in[INS_ERR_ATT_X + j][c];
                vel_row += f_vba[i][j] * p_in[INS_ERR_BIAS_AX + j][c];
            }
            temp[vel_i][c] = vel_row;

            float att_row = 0.0f;
            for (uint8_t j = 0U; j < 3U; ++j) {
                att_row += f_aa[i][j] * p_in[INS_ERR_ATT_X + j][c];
                att_row += f_bg[i][j] * p_in[INS_ERR_BIAS_GX + j][c];
            }
            temp[att_i][c] = att_row;

            temp[ba_i][c] = p_in[ba_i][c];
            temp[bg_i][c] = p_in[bg_i][c];
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < kDim; ++c) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                const float f_kc = f_state_jacobian_entry(k, c, dt_s, f_va_t, f_vba_t, f_aa_t, f_bg_t);
                sum += temp[r][k] * f_kc;
            }
            p_out[r][c] = sum + q[r][c];
        }
    }

    mat_symmetrize(p_out);
}

bool ins_ekf_invert3x3(const InsEkfMat3 s, InsEkfMat3 inv_out)
{
    const float det =
        (s[0][0] * ((s[1][1] * s[2][2]) - (s[1][2] * s[2][1])))
        - (s[0][1] * ((s[1][0] * s[2][2]) - (s[1][2] * s[2][0])))
        + (s[0][2] * ((s[1][0] * s[2][1]) - (s[1][1] * s[2][0])));

    if (fabsf(det) <= 1.0e-12f) {
        return false;
    }

    const float inv_det = 1.0f / det;

    inv_out[0][0] = ((s[1][1] * s[2][2]) - (s[1][2] * s[2][1])) * inv_det;
    inv_out[0][1] = ((s[0][2] * s[2][1]) - (s[0][1] * s[2][2])) * inv_det;
    inv_out[0][2] = ((s[0][1] * s[1][2]) - (s[0][2] * s[1][1])) * inv_det;

    inv_out[1][0] = ((s[1][2] * s[2][0]) - (s[1][0] * s[2][2])) * inv_det;
    inv_out[1][1] = ((s[0][0] * s[2][2]) - (s[0][2] * s[2][0])) * inv_det;
    inv_out[1][2] = ((s[0][2] * s[1][0]) - (s[0][0] * s[1][2])) * inv_det;

    inv_out[2][0] = ((s[1][0] * s[2][1]) - (s[1][1] * s[2][0])) * inv_det;
    inv_out[2][1] = ((s[0][1] * s[2][0]) - (s[0][0] * s[2][1])) * inv_det;
    inv_out[2][2] = ((s[0][0] * s[1][1]) - (s[0][1] * s[1][0])) * inv_det;

    return true;
}

} /* namespace */

void InsEkfFilter::predict(const ImuSample &imu_sample, float dt_s)
{
    if (!initialized || !imu_sample.valid || dt_s <= 0.0f) {
        return;
    }

    float w_corr[3]{};
    float a_corr[3]{};
    vec3_sub(imu_sample.gyro_radps, bias_g_, w_corr);
    vec3_sub(imu_sample.accel_mps2, bias_a_, a_corr);

    quat_integrate_first_order(q_att_, w_corr, dt_s);

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(q_att_, dcm_bn);

    float a_n[3]{};
    body_to_ned(dcm_bn, a_corr, a_n);
    a_n[0] -= kGravityNed[0];
    a_n[1] -= kGravityNed[1];
    a_n[2] -= kGravityNed[2];

    for (uint8_t i = 0U; i < 3U; ++i) {
        vel_[i] += a_n[i] * dt_s;
        pos_[i] += vel_[i] * dt_s;
    }

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

    InsEkfMat15 q_mat{};
    ins_ekf_build_process_noise(this, dt_s, q_mat);

    InsEkfMat15 p_new{};
    ins_ekf_propagate_covariance_sparse(
        cov.P,
        f_va,
        f_vba,
        f_aa,
        f_bg,
        dt_s,
        q_mat,
        p_new);
    mat_copy(p_new, cov.P);

    last_imu_timestamp_ms = imu_sample.timestamp_ms;
}

bool InsEkfFilter::update_gnss(const GpsSample &gps_sample, float *out_nis)
{
    if (!initialized || !gps_sample.fix_valid) {
        return false;
    }

    float z_n = 0.0f;
    float z_e = 0.0f;
    float z_d = 0.0f;
    latlonalt_to_ned(
        ref_lat_deg,
        ref_lon_deg,
        ref_alt_m,
        gps_sample.position.x,
        gps_sample.position.y,
        gps_sample.position.z,
        &z_n,
        &z_e,
        &z_d);

    const float y[3] = {
        z_n - pos_[0],
        z_e - pos_[1],
        z_d - pos_[2],
    };

    gnss_innovation_last[0] = y[0];
    gnss_innovation_last[1] = y[1];
    gnss_innovation_last[2] = y[2];

    InsEkfMat3 s{};
    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            s[i][j] = cov.P[INS_ERR_POS_N + i][INS_ERR_POS_N + j];
            gnss_innovation_cov_last[i][j] = s[i][j];
        }
        s[i][i] += gnss_pos_var_m2;
        gnss_innovation_cov_last[i][i] = s[i][i];
    }

    InsEkfMat3 s_inv{};
    if (!ins_ekf_invert3x3(s, s_inv)) {
        outlier_detected = true;
        gnss_nis_last = 0.0f;
        if (out_nis != NULL) {
            *out_nis = gnss_nis_last;
        }
        return false;
    }

    float nis = 0.0f;
    for (uint8_t i = 0U; i < 3U; ++i) {
        for (uint8_t j = 0U; j < 3U; ++j) {
            nis += y[i] * s_inv[i][j] * y[j];
        }
    }

    gnss_nis_last = nis;
    if (out_nis != NULL) {
        *out_nis = nis;
    }

    if (nis > nis_threshold) {
        outlier_detected = true;
        return false;
    }

    float k_gain[INS_EKF_STATE_DIM][3]{};
    for (uint8_t r = 0U; r < kDim; ++r) {
        for (uint8_t c = 0U; c < 3U; ++c) {
            float sum = 0.0f;
            for (uint8_t j = 0U; j < 3U; ++j) {
                sum += cov.P[r][INS_ERR_POS_N + j] * s_inv[j][c];
            }
            k_gain[r][c] = sum;
        }
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        float correction = 0.0f;
        for (uint8_t c = 0U; c < 3U; ++c) {
            correction += k_gain[r][c] * y[c];
        }
        delta_x_[r] = correction;
    }

    ins_ekf_inject_error_into_nominal(this);

    InsEkfMat15 p_joseph{};
    ins_ekf_covariance_joseph_update(cov.P, k_gain, gnss_pos_var_m2, p_joseph);
    mat_copy(p_joseph, cov.P);

    outlier_detected = false;
    return true;
}

bool InsEkfFilter::update_nhc()
{
    if (!initialized || !nhc_enabled) {
        return false;
    }

    InsEkfMat3 dcm_bn{};
    quat_to_dcm_bn(q_att_, dcm_bn);

    float v_body[3]{};
    ned_to_body(dcm_bn, vel_, v_body);

    /* Pseudo-medicion: v_lateral (Y) y v_vertical (Z) cuerpo ≈ 0 m/s. */
    const float y[2] = {
        -v_body[1],
        -v_body[2],
    };

    float h_rows[2][INS_EKF_STATE_DIM]{};
    h_rows[0][INS_ERR_VEL_N + 0] = dcm_bn[0][1];
    h_rows[0][INS_ERR_VEL_N + 1] = dcm_bn[1][1];
    h_rows[0][INS_ERR_VEL_N + 2] = dcm_bn[2][1];
    h_rows[1][INS_ERR_VEL_N + 0] = dcm_bn[0][2];
    h_rows[1][INS_ERR_VEL_N + 1] = dcm_bn[1][2];
    h_rows[1][INS_ERR_VEL_N + 2] = dcm_bn[2][2];

    /* Acoplamiento actitud (perturbacion derecha q' = q * dq): d(v_b) = -[v_b]x dtheta. */
    h_rows[0][INS_ERR_ATT_X] = v_body[2];
    h_rows[0][INS_ERR_ATT_Z] = -v_body[0];
    h_rows[1][INS_ERR_ATT_X] = -v_body[1];
    h_rows[1][INS_ERR_ATT_Y] = v_body[0];

    float s_mat[2][2]{};
    for (uint8_t i = 0U; i < 2U; ++i) {
        for (uint8_t j = 0U; j < 2U; ++j) {
            float sum = 0.0f;
            for (uint8_t k = 0U; k < kDim; ++k) {
                for (uint8_t l = 0U; l < kDim; ++l) {
                    sum += h_rows[i][k] * cov.P[k][l] * h_rows[j][l];
                }
            }
            s_mat[i][j] = sum;
        }
    }
    s_mat[0][0] += nhc_lateral_var_m2;
    s_mat[1][1] += nhc_vertical_var_m2;

    float s_inv[2][2]{};
    if (!ins_ekf_invert2x2(s_mat, s_inv)) {
        return false;
    }

    float k_gain[INS_EKF_STATE_DIM][2]{};
    for (uint8_t r = 0U; r < kDim; ++r) {
        float ph_t0 = 0.0f;
        float ph_t1 = 0.0f;
        for (uint8_t k = 0U; k < kDim; ++k) {
            ph_t0 += cov.P[r][k] * h_rows[0][k];
            ph_t1 += cov.P[r][k] * h_rows[1][k];
        }
        k_gain[r][0] = (ph_t0 * s_inv[0][0]) + (ph_t1 * s_inv[0][1]);
        k_gain[r][1] = (ph_t0 * s_inv[1][0]) + (ph_t1 * s_inv[1][1]);
    }

    for (uint8_t r = 0U; r < kDim; ++r) {
        delta_x_[r] = (k_gain[r][0] * y[0]) + (k_gain[r][1] * y[1]);
    }

    ins_ekf_inject_error_into_nominal(this);

    const float meas_var[2] = {nhc_lateral_var_m2, nhc_vertical_var_m2};
    InsEkfMat15 p_joseph{};
    ins_ekf_covariance_joseph_update2(cov.P, k_gain, h_rows, meas_var, p_joseph);
    mat_copy(p_joseph, cov.P);

    ++nhc_update_count;
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
    filter->nis_threshold = NAVICORE_INS_EKF_NIS_THRESHOLD;
    filter->nhc_enabled = false;
    filter->nhc_lateral_var_m2 =
        NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS * NAVICORE_INS_EKF_NHC_LATERAL_STD_MPS;
    filter->nhc_vertical_var_m2 =
        NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS * NAVICORE_INS_EKF_NHC_VERTICAL_STD_MPS;
    filter->nhc_update_count = 0U;

    filter->initialized = true;
    filter->outlier_detected = false;
    ins_ekf_reset_error_state(filter);
}

bool ins_ekf_predict(InsEkfFilter *filter, const ImuSample *imu)
{
    if (filter == NULL || imu == NULL || !imu->valid || !filter->initialized) {
        return false;
    }

    const float dt_s = ins_ekf_predict_dt_s(filter, imu->timestamp_ms);
    filter->predict(*imu, dt_s);
    if (filter->nhc_enabled) {
        filter->update_nhc();
    }
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

    ned_to_latlonalt(
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
        && ((timestamp_ms - filter->last_gnss_accept_ms) <= 2000U);
    const bool gnss_outlier = filter->outlier_detected;

    if (gnss_outlier) {
        out_state->mode = NAV_MODE_DEAD_RECKONING;
        out_state->confidence = nav_confidence_make(false, 0U, timestamp_ms, 0.25f);
    } else if (gps_fix_now && gnss_recent) {
        const uint8_t sats = last_gps->satellites;
        const float quality = clampf(0.55f + ((float)sats * 0.03f), 0.55f, 0.95f);
        out_state->mode = NAV_MODE_HYBRID;
        out_state->confidence = nav_confidence_make(true, sats, 0U, quality);
    } else if (gps_fix_now) {
        out_state->mode = NAV_MODE_GPS;
        out_state->confidence = nav_confidence_make(
            true,
            last_gps->satellites,
            0U,
            0.65f);
    } else {
        const uint32_t fix_age_ms = (filter->last_gnss_accept_ms == 0U)
            ? timestamp_ms
            : (timestamp_ms - filter->last_gnss_accept_ms);
        const float age_s = (float)fix_age_ms * 0.001f;
        const float quality = clampf(0.75f - (age_s * 0.05f), 0.15f, 0.75f);
        out_state->mode = NAV_MODE_DEAD_RECKONING;
        out_state->confidence = nav_confidence_make(false, 0U, fix_age_ms, quality);
    }
}
