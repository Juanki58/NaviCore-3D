#include "real_run_replay.hpp"

#include "geodesy.hpp"
#include "ins_ekf.hpp"
#include "ins_ekf_15_state.hpp"
#include "interfaces/INaviFilter.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr size_t kMaxCsvLineBytes = 2048U;
constexpr double kDefaultDtS = static_cast<double>(NAVICORE_INS_EKF_DT_S);
constexpr double kMinDtS = 0.001;
constexpr double kMaxDtS = 0.05;
constexpr float kGnssMinHorizontalStdM = 5.0f;
constexpr float kGnssMinVerticalStdM = 8.0f;
constexpr float kRadToDegF = 180.0f / static_cast<float>(M_PI);
constexpr float kDegToRadF = static_cast<float>(M_PI) / 180.0f;
constexpr float kLegacyMountRollRad = -45.18f * static_cast<float>(M_PI) / 180.0f;
constexpr float kLegacyMountPitchRad = -51.52f * static_cast<float>(M_PI) / 180.0f;
constexpr float kLegacyMountYawRad = 110.40f * static_cast<float>(M_PI) / 180.0f;
constexpr float kDefaultYawInitMinSpeedMps = 3.0f;
constexpr uint32_t kDefaultYawInitMinSamples = 20U;
constexpr float kDefaultYawInitMaxHeadingStdDeg = 5.0f;
constexpr float kMinGnssHeadingDisplacementM = 0.3f;
constexpr uint32_t kDefaultH9aGravityInitMinSamples = 50U;
constexpr float kDefaultH9aGravityInitWindowS = 2.0f;
constexpr uint32_t kMaxYawHeadingWindow = 32U;
constexpr float kH3PositionCovResetM2 = 10000.0f;
constexpr float kH3GracePeriodS = 5.0f;
constexpr float kH3NisGateBypassThreshold = 1.0e12f;
constexpr float kH5SyncAuditMinSpeedMps = 2.0f;

enum class ReplayRowType {
    UNKNOWN = 0,
    IMU,
    GPS,
};

struct ParsedReplayRow {
    double timestamp_s;
    ReplayRowType row_type;
    float accel[3];
    float gyro[3];
    double pos_ned[3];
    float accuracy_horizontal;
    float accuracy_vertical;
    float speed;
    bool has_accel;
    bool has_gyro;
    bool has_pos;
    bool has_accuracy;
    bool has_speed;
};

void trim_ascii(char *text)
{
    if (text == nullptr) {
        return;
    }

    char *start = text;
    while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n') {
        ++start;
    }

    if (start != text) {
        std::memmove(text, start, std::strlen(start) + 1U);
    }

    const size_t len = std::strlen(text);
    size_t end = len;
    while (end > 0U) {
        const char ch = text[end - 1U];
        if (ch != ' ' && ch != '\t' && ch != '\r' && ch != '\n') {
            break;
        }
        text[end - 1U] = '\0';
        --end;
    }
}

bool parse_optional_float(const char *text, float *out_value)
{
    if (out_value == nullptr) {
        return false;
    }

    if (text == nullptr) {
        return false;
    }

    char buffer[64];
    std::strncpy(buffer, text, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    trim_ascii(buffer);

    if (buffer[0] == '\0') {
        return false;
    }

    char *end = nullptr;
    const double value = std::strtod(buffer, &end);
    if (end == buffer || !std::isfinite(value)) {
        return false;
    }

    *out_value = static_cast<float>(value);
    return true;
}

bool parse_optional_double(const char *text, double *out_value)
{
    float as_float = 0.0f;
    if (!parse_optional_float(text, &as_float)) {
        return false;
    }

    if (out_value != nullptr) {
        *out_value = static_cast<double>(as_float);
    }
    return true;
}

ReplayRowType parse_row_type(const char *text)
{
    if (text == nullptr) {
        return ReplayRowType::UNKNOWN;
    }

    char buffer[32];
    std::strncpy(buffer, text, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';
    trim_ascii(buffer);

    if (std::strcmp(buffer, "IMU") == 0) {
        return ReplayRowType::IMU;
    }
    if (std::strcmp(buffer, "GPS") == 0) {
        return ReplayRowType::GPS;
    }
    return ReplayRowType::UNKNOWN;
}

bool parse_replay_csv_line(const char *line, ParsedReplayRow *out_row)
{
    if (line == nullptr || out_row == nullptr) {
        return false;
    }

    char local_line[kMaxCsvLineBytes];
    std::strncpy(local_line, line, sizeof(local_line) - 1U);
    local_line[sizeof(local_line) - 1U] = '\0';

    char *fields[14] = {nullptr};
    size_t field_count = 0U;
    char *cursor = local_line;
    while (field_count < 14U) {
        fields[field_count++] = cursor;
        char *comma = std::strchr(cursor, ',');
        if (comma == nullptr) {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }

    if (field_count < 2U) {
        return false;
    }

    ParsedReplayRow row{};
    if (!parse_optional_double(fields[0], &row.timestamp_s)) {
        return false;
    }

    row.row_type = parse_row_type(fields[1]);
    if (row.row_type == ReplayRowType::UNKNOWN) {
        return false;
    }

    row.has_accel =
        parse_optional_float(fields[2], &row.accel[0])
        && parse_optional_float(fields[3], &row.accel[1])
        && parse_optional_float(fields[4], &row.accel[2]);
    row.has_gyro =
        parse_optional_float(fields[5], &row.gyro[0])
        && parse_optional_float(fields[6], &row.gyro[1])
        && parse_optional_float(fields[7], &row.gyro[2]);
    row.has_pos =
        parse_optional_double(fields[8], &row.pos_ned[0])
        && parse_optional_double(fields[9], &row.pos_ned[1])
        && parse_optional_double(fields[10], &row.pos_ned[2]);
    row.has_accuracy =
        parse_optional_float(fields[11], &row.accuracy_horizontal)
        && parse_optional_float(fields[12], &row.accuracy_vertical);
    row.has_speed = parse_optional_float(fields[13], &row.speed);

    *out_row = row;
    return true;
}

double compute_dt_s(double current_timestamp_s, double previous_timestamp_s)
{
    if (previous_timestamp_s < 0.0) {
        return kDefaultDtS;
    }

    const double dt_s = current_timestamp_s - previous_timestamp_s;
    if (dt_s <= 0.0) {
        return kDefaultDtS;
    }
    if (dt_s < kMinDtS) {
        return kMinDtS;
    }
    if (dt_s > kMaxDtS) {
        return kMaxDtS;
    }
    return dt_s;
}

void build_euler321_rotation_matrix(
    float matrix[3][3],
    float roll_rad,
    float pitch_rad,
    float yaw_rad)
{
    const float cr = std::cos(roll_rad);
    const float sr = std::sin(roll_rad);
    const float cp = std::cos(pitch_rad);
    const float sp = std::sin(pitch_rad);
    const float cy = std::cos(yaw_rad);
    const float sy = std::sin(yaw_rad);

    matrix[0][0] = cy * cp;
    matrix[0][1] = cy * sp * sr - sy * cr;
    matrix[0][2] = cy * sp * cr + sy * sr;

    matrix[1][0] = sy * cp;
    matrix[1][1] = sy * sp * sr + cy * cr;
    matrix[1][2] = sy * sp * cr - cy * sr;

    matrix[2][0] = -sp;
    matrix[2][1] = cp * sr;
    matrix[2][2] = cp * cr;
}

void mat3_vec3_mul(const float matrix[3][3], const float input[3], float output[3])
{
    for (int row = 0; row < 3; ++row) {
        output[row] =
            (matrix[row][0] * input[0])
            + (matrix[row][1] * input[1])
            + (matrix[row][2] * input[2]);
    }
}

void mat3_transpose(const float matrix[3][3], float transposed[3][3])
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            transposed[row][col] = matrix[col][row];
        }
    }
}

bool replay_constraints_use_nhc(
    double timestamp_s,
    float last_gps_speed_mps,
    float static_phase_end_s,
    float moving_speed_threshold_mps)
{
    return (timestamp_s > static_cast<double>(static_phase_end_s))
        && (last_gps_speed_mps > moving_speed_threshold_mps);
}

float horizontal_drift_m(const NaviState &state, const double ref_pos_ned[3])
{
    const float dn = static_cast<float>(state.pos_ned[0]) - static_cast<float>(ref_pos_ned[0]);
    const float de = static_cast<float>(state.pos_ned[1]) - static_cast<float>(ref_pos_ned[1]);
    return std::sqrt((dn * dn) + (de * de));
}

void gap3_compute_cycle_pred_accum(
    const float anchor_pos[3],
    const float anchor_vel[3],
    const float state_pos[3],
    const float state_vel[3],
    double last_obs_timestamp_s,
    double current_timestamp_s,
    double *out_pred_dpos_h,
    double *out_pred_dvel_h,
    double *out_pred_dt)
{
    if (out_pred_dpos_h != nullptr) {
        const float dn = state_pos[0] - anchor_pos[0];
        const float de = state_pos[1] - anchor_pos[1];
        *out_pred_dpos_h = std::sqrt(static_cast<double>((dn * dn) + (de * de)));
    }
    if (out_pred_dvel_h != nullptr) {
        const float dvn = state_vel[0] - anchor_vel[0];
        const float dve = state_vel[1] - anchor_vel[1];
        *out_pred_dvel_h = std::sqrt(static_cast<double>((dvn * dvn) + (dve * dve)));
    }
    if (out_pred_dt != nullptr) {
        *out_pred_dt = current_timestamp_s - last_obs_timestamp_s;
    }
}

void gap3_set_observation_anchor(
    float anchor_pos[3],
    float anchor_vel[3],
    const float state_pos[3],
    const float state_vel[3],
    double *last_obs_timestamp_s,
    double current_timestamp_s,
    bool *has_anchor)
{
    if (anchor_pos == nullptr || anchor_vel == nullptr || state_pos == nullptr
        || state_vel == nullptr || last_obs_timestamp_s == nullptr || has_anchor == nullptr) {
        return;
    }

    anchor_pos[0] = state_pos[0];
    anchor_pos[1] = state_pos[1];
    anchor_pos[2] = state_pos[2];
    anchor_vel[0] = state_vel[0];
    anchor_vel[1] = state_vel[1];
    anchor_vel[2] = state_vel[2];
    *last_obs_timestamp_s = current_timestamp_s;
    *has_anchor = true;
}

bool invert_matrix3(float s[3][3], float inv_out[3][3])
{
    const float det =
        (s[0][0] * ((s[1][1] * s[2][2]) - (s[1][2] * s[2][1])))
        - (s[0][1] * ((s[1][0] * s[2][2]) - (s[1][2] * s[2][0])))
        + (s[0][2] * ((s[1][0] * s[2][1]) - (s[1][1] * s[2][0])));
    if (std::fabs(det) < 1.0e-12f) {
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

bool invert_matrix2(float s[2][2], float inv_out[2][2])
{
    const float det = (s[0][0] * s[1][1]) - (s[0][1] * s[1][0]);
    if (std::fabs(det) < 1.0e-12f) {
        return false;
    }

    const float inv_det = 1.0f / det;
    inv_out[0][0] = s[1][1] * inv_det;
    inv_out[0][1] = -s[0][1] * inv_det;
    inv_out[1][0] = -s[1][0] * inv_det;
    inv_out[1][1] = s[0][0] * inv_det;
    return true;
}

float quadratic_form3(const float y[3], const float s_inv[3][3])
{
    float nis = 0.0f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            nis += y[i] * s_inv[i][j] * y[j];
        }
    }
    return nis;
}

void nis_component_contributions(
    const float y[3],
    const float s_inv[3][3],
    float out_contrib[3])
{
    float sy[3] = {0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            sy[i] += s_inv[i][j] * y[j];
        }
    }
    out_contrib[0] = y[0] * sy[0];
    out_contrib[1] = y[1] * sy[1];
    out_contrib[2] = y[2] * sy[2];
}

void symmetric2_eigen_extremes(
    float s00,
    float s01,
    float s11,
    float *out_min,
    float *out_max)
{
    const float trace = s00 + s11;
    const float det = (s00 * s11) - (s01 * s01);
    const float disc = fmaxf((trace * trace) - (4.0f * det), 0.0f);
    const float root = std::sqrt(disc);
    const float e0 = 0.5f * (trace - root);
    const float e1 = 0.5f * (trace + root);
    if (out_min != nullptr) {
        *out_min = fminf(e0, e1);
    }
    if (out_max != nullptr) {
        *out_max = fmaxf(e0, e1);
    }
}

void symmetric3_eigen_extremes(
    const float s[3][3],
    float *out_min,
    float *out_max)
{
    float a[3][3]{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            a[i][j] = s[i][j];
        }
    }

    for (int sweep = 0; sweep < 50; ++sweep) {
        float off_sum = 0.0f;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                off_sum += std::fabs(a[p][q]);
            }
        }
        if (off_sum < 1.0e-12f) {
            break;
        }

        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                const float apq = a[p][q];
                if (std::fabs(apq) < 1.0e-15f) {
                    continue;
                }
                const float app = a[p][p];
                const float aqq = a[q][q];
                const float tau = (aqq - app) / (2.0f * apq);
                const float t = (tau >= 0.0f ? 1.0f : -1.0f)
                    / (std::fabs(tau) + std::sqrt(1.0f + (tau * tau)));
                const float c = 1.0f / std::sqrt(1.0f + (t * t));
                const float s_rot = t * c;
                for (int k = 0; k < 3; ++k) {
                    const float akp = a[p][k];
                    const float akq = a[q][k];
                    a[p][k] = (c * akp) - (s_rot * akq);
                    a[q][k] = (s_rot * akp) + (c * akq);
                }
                for (int k = 0; k < 3; ++k) {
                    const float apk = a[k][p];
                    const float aqk = a[k][q];
                    a[k][p] = (c * apk) - (s_rot * aqk);
                    a[k][q] = (s_rot * apk) + (c * aqk);
                }
                a[p][p] = (c * c * app) - (2.0f * c * s_rot * apq) + (s_rot * s_rot * aqq);
                a[q][q] = (s_rot * s_rot * app) + (2.0f * c * s_rot * apq) + (c * c * aqq);
                a[p][q] = 0.0f;
                a[q][p] = 0.0f;
            }
        }
    }

    float eigmin = a[0][0];
    float eigmax = a[0][0];
    for (int i = 1; i < 3; ++i) {
        eigmin = fminf(eigmin, a[i][i]);
        eigmax = fmaxf(eigmax, a[i][i]);
    }
    if (out_min != nullptr) {
        *out_min = eigmin;
    }
    if (out_max != nullptr) {
        *out_max = eigmax;
    }
}

void matrix_s_condition(
    const float s[3][3],
    float *out_eigmin,
    float *out_eigmax,
    float *out_cond)
{
    float eigmin = 0.0f;
    float eigmax = 0.0f;
    symmetric3_eigen_extremes(s, &eigmin, &eigmax);
    if (out_eigmin != nullptr) {
        *out_eigmin = eigmin;
    }
    if (out_eigmax != nullptr) {
        *out_eigmax = eigmax;
    }
    if (out_cond != nullptr) {
        *out_cond = (eigmin > 1.0e-12f) ? (eigmax / eigmin) : 0.0f;
    }
}

bool write_output_header(FILE *output_fp)
{
    if (output_fp == nullptr) {
        return false;
    }

    std::fprintf(
        output_fp,
        "timestamp_s,pos_n_m,pos_e_m,pos_d_m,"
        "vel_body_x_mps,vel_body_y_mps,vel_body_z_mps,"
        "roll_deg,pitch_deg,yaw_deg,"
        "accel_bias_x,accel_bias_y,accel_bias_z,"
        "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
        "cov_pos_n,cov_pos_e,cov_pos_d,"
        "cov_att_roll,cov_att_pitch,cov_att_yaw,"
        "nis,row_type\n");
    return true;
}

bool write_output_state(FILE *output_fp, double timestamp_s, const NaviState &state, const char *row_type)
{
    if (output_fp == nullptr || row_type == nullptr) {
        return false;
    }

    std::fprintf(
        output_fp,
        "%.9f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%s\n",
        timestamp_s,
        state.pos_ned[0],
        state.pos_ned[1],
        state.pos_ned[2],
        state.vel_body[0],
        state.vel_body[1],
        state.vel_body[2],
        state.att_euler[0] * kRadToDegF,
        state.att_euler[1] * kRadToDegF,
        state.att_euler[2] * kRadToDegF,
        state.accel_bias[0],
        state.accel_bias[1],
        state.accel_bias[2],
        state.gyro_bias[0],
        state.gyro_bias[1],
        state.gyro_bias[2],
        state.cov_pos_diag[0],
        state.cov_pos_diag[1],
        state.cov_pos_diag[2],
        state.cov_att_diag[0],
        state.cov_att_diag[1],
        state.cov_att_diag[2],
        state.nis,
        row_type);
    return true;
}

bool validate_replay_header(const char *header_line)
{
    if (header_line == nullptr) {
        return false;
    }

    return (std::strstr(header_line, "timestamp_s") != nullptr)
        && (std::strstr(header_line, "type") != nullptr)
        && (std::strstr(header_line, "accel_x") != nullptr)
        && (std::strstr(header_line, "pos_n") != nullptr);
}

void set_identity_matrix(float matrix[3][3])
{
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            matrix[row][col] = (row == col) ? 1.0f : 0.0f;
        }
    }
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

float wrap_angle_deg(float angle_deg)
{
    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void euler321_to_quat(float roll_rad, float pitch_rad, float yaw_rad, float q[4])
{
    const float cr = std::cos(roll_rad * 0.5f);
    const float sr = std::sin(roll_rad * 0.5f);
    const float cp = std::cos(pitch_rad * 0.5f);
    const float sp = std::sin(pitch_rad * 0.5f);
    const float cy = std::cos(yaw_rad * 0.5f);
    const float sy = std::sin(yaw_rad * 0.5f);

    q[0] = (cr * cp * cy) + (sr * sp * sy);
    q[1] = (sr * cp * cy) - (cr * sp * sy);
    q[2] = (cr * sp * cy) + (sr * cp * sy);
    q[3] = (cr * cp * sy) - (sr * sp * cy);
}

void quat_normalize_inplace(float q[4])
{
    const float norm_sq =
        (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);
    if (norm_sq <= 0.0f) {
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
        return;
    }

    const float inv_norm = 1.0f / std::sqrt(norm_sq);
    q[0] *= inv_norm;
    q[1] *= inv_norm;
    q[2] *= inv_norm;
    q[3] *= inv_norm;
}

bool set_ekf_yaw_preserve_roll_pitch(InsEkfFilter *ekf, float yaw_rad)
{
    if (ekf == nullptr || !ekf->initialized) {
        return false;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float current_yaw_rad = 0.0f;
    ins_ekf_get_attitude_rad(ekf, &roll_rad, &pitch_rad, &current_yaw_rad);

    const float new_yaw_rad = wrap_angle_rad(yaw_rad);
    euler321_to_quat(roll_rad, pitch_rad, new_yaw_rad, ekf->q_att_);
    quat_normalize_inplace(ekf->q_att_);
    return true;
}

void reset_position_covariance_diagonal(InsEkfFilter *ekf, float variance_m2)
{
    if (ekf == nullptr) {
        return;
    }

    ekf->cov.P[INS_ERR_POS_N][INS_ERR_POS_N] = variance_m2;
    ekf->cov.P[INS_ERR_POS_E][INS_ERR_POS_E] = variance_m2;
    ekf->cov.P[INS_ERR_POS_D][INS_ERR_POS_D] = variance_m2;
}

void scale_covariance_matrix(InsEkfFilter *ekf, float scale)
{
    if (ekf == nullptr || scale <= 0.0f || std::fabs(scale - 1.0f) < 1.0e-6f) {
        return;
    }

    for (uint8_t i = 0U; i < INS_EKF_STATE_DIM; ++i) {
        for (uint8_t j = 0U; j < INS_EKF_STATE_DIM; ++j) {
            ekf->cov.P[i][j] *= scale;
        }
    }
}

bool invert_2x2_matrix(const float matrix[2][2], float inverse[2][2])
{
    const float det =
        (matrix[0][0] * matrix[1][1]) - (matrix[0][1] * matrix[1][0]);
    if (std::fabs(det) < 1.0e-12f) {
        return false;
    }

    const float inv_det = 1.0f / det;
    inverse[0][0] = matrix[1][1] * inv_det;
    inverse[0][1] = -matrix[0][1] * inv_det;
    inverse[1][0] = -matrix[1][0] * inv_det;
    inverse[1][1] = matrix[0][0] * inv_det;
    return true;
}

void scale_imu_process_noise(InsEkfFilter *ekf, float scale)
{
    if (ekf == nullptr || scale <= 0.0f || std::fabs(scale - 1.0f) < 1.0e-6f) {
        return;
    }

    ekf->accel_noise_var *= scale;
    ekf->gyro_noise_var *= scale;
}

float compute_nees_pos_2d(float error_n, float error_e, const float p_pos[3][3])
{
    const float p_ne[2][2] = {
        {p_pos[0][0], p_pos[0][1]},
        {p_pos[1][0], p_pos[1][1]},
    };
    float p_ne_inv[2][2]{};
    if (!invert_2x2_matrix(p_ne, p_ne_inv)) {
        return 0.0f;
    }

    const float weighted_n =
        (p_ne_inv[0][0] * error_n) + (p_ne_inv[0][1] * error_e);
    const float weighted_e =
        (p_ne_inv[1][0] * error_n) + (p_ne_inv[1][1] * error_e);
    return (error_n * weighted_n) + (error_e * weighted_e);
}

float covariance_trace(const InsEkfFilter &ekf)
{
    float trace = 0.0f;
    for (uint8_t i = 0U; i < INS_EKF_STATE_DIM; ++i) {
        trace += ekf.cov.P[i][i];
    }
    return trace;
}

bool covariance_determinant(const InsEkfFilter &ekf, float *out_det)
{
    if (out_det == nullptr) {
        return false;
    }

    float matrix[INS_EKF_STATE_DIM][INS_EKF_STATE_DIM]{};
    for (uint8_t row = 0U; row < INS_EKF_STATE_DIM; ++row) {
        for (uint8_t col = 0U; col < INS_EKF_STATE_DIM; ++col) {
            matrix[row][col] = ekf.cov.P[row][col];
        }
    }

    float det = 1.0f;
    for (uint8_t col = 0U; col < INS_EKF_STATE_DIM; ++col) {
        uint8_t pivot_row = col;
        float pivot_abs_max = std::fabs(matrix[col][col]);
        for (uint8_t row = col + 1U; row < INS_EKF_STATE_DIM; ++row) {
            const float candidate = std::fabs(matrix[row][col]);
            if (candidate > pivot_abs_max) {
                pivot_abs_max = candidate;
                pivot_row = row;
            }
        }

        if (pivot_abs_max < 1.0e-12f) {
            *out_det = 0.0f;
            return true;
        }

        if (pivot_row != col) {
            for (uint8_t k = 0U; k < INS_EKF_STATE_DIM; ++k) {
                const float tmp = matrix[col][k];
                matrix[col][k] = matrix[pivot_row][k];
                matrix[pivot_row][k] = tmp;
            }
            det = -det;
        }

        const float pivot = matrix[col][col];
        det *= pivot;

        for (uint8_t row = col + 1U; row < INS_EKF_STATE_DIM; ++row) {
            const float factor = matrix[row][col] / pivot;
            for (uint8_t k = col; k < INS_EKF_STATE_DIM; ++k) {
                matrix[row][k] -= factor * matrix[col][k];
            }
        }
    }

    *out_det = det;
    return true;
}

bool write_consistency_header(FILE *consistency_fp)
{
    if (consistency_fp == nullptr) {
        return false;
    }

    std::fprintf(
        consistency_fp,
        "timestamp_s,error_n_m,error_e_m,nees_pos,ratio_n,ratio_e,"
        "trace_P,det_P,innovation_norm_m,nis,gnss_accepted\n");
    return true;
}

bool write_consistency_row(
    FILE *consistency_fp,
    double timestamp_s,
    float error_n,
    float error_e,
    float nees_pos,
    float ratio_n,
    float ratio_e,
    float trace_p,
    float det_p,
    float innovation_norm,
    float nis,
    bool gnss_accepted)
{
    if (consistency_fp == nullptr) {
        return false;
    }

    std::fprintf(
        consistency_fp,
        "%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6e,%.6f,%.6f,%d\n",
        timestamp_s,
        error_n,
        error_e,
        nees_pos,
        ratio_n,
        ratio_e,
        trace_p,
        det_p,
        innovation_norm,
        nis,
        gnss_accepted ? 1 : 0);
    return true;
}

bool yaw_init_mode_uses_stable_heading(RealRunYawInitMode mode)
{
    return mode == RealRunYawInitMode::H2_GNSS_STABLE_HEADING
        || mode == RealRunYawInitMode::H3_COV_RESET_GRACE;
}

bool write_h3_diagnostics_header(FILE *diagnostics_fp)
{
    if (diagnostics_fp == nullptr) {
        return false;
    }

    std::fprintf(
        diagnostics_fp,
        "timestamp_s,yaw_gnss_deg,yaw_ekf_deg,delta_yaw_deg,"
        "innovation_n_m,innovation_e_m,innovation_d_m,"
        "nis,gnss_accepted,gnss_reject_total,"
        "h3_applied,grace_active,nis_gate_bypassed,"
        "P_pos_n,P_pos_e,P_pos_d\n");
    return true;
}

bool write_h3_diagnostics_row(
    FILE *diagnostics_fp,
    double timestamp_s,
    bool has_yaw_gnss,
    float yaw_gnss_deg,
    float yaw_ekf_deg,
    const float innovation_ned[3],
    float nis,
    bool gnss_accepted,
    uint32_t gnss_reject_total,
    bool h3_applied,
    bool grace_active,
    bool nis_gate_bypassed,
    const InsEkfFilter &ekf)
{
    if (diagnostics_fp == nullptr) {
        return false;
    }

    const float delta_yaw_deg = has_yaw_gnss
        ? wrap_angle_deg(yaw_gnss_deg - yaw_ekf_deg)
        : 0.0f;
    const float p_pos_n = ekf.cov.P[INS_ERR_POS_N][INS_ERR_POS_N];
    const float p_pos_e = ekf.cov.P[INS_ERR_POS_E][INS_ERR_POS_E];
    const float p_pos_d = ekf.cov.P[INS_ERR_POS_D][INS_ERR_POS_D];

    if (has_yaw_gnss) {
        std::fprintf(
            diagnostics_fp,
            "%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%u,%d,%d,%d,%.6f,%.6f,%.6f\n",
            timestamp_s,
            yaw_gnss_deg,
            yaw_ekf_deg,
            delta_yaw_deg,
            innovation_ned[0],
            innovation_ned[1],
            innovation_ned[2],
            nis,
            gnss_accepted ? 1 : 0,
            gnss_reject_total,
            h3_applied ? 1 : 0,
            grace_active ? 1 : 0,
            nis_gate_bypassed ? 1 : 0,
            p_pos_n,
            p_pos_e,
            p_pos_d);
    } else {
        std::fprintf(
            diagnostics_fp,
            "%.9f,,%.6f,,%.6f,%.6f,%.6f,%.6f,%d,%u,%d,%d,%d,%.6f,%.6f,%.6f\n",
            timestamp_s,
            yaw_ekf_deg,
            innovation_ned[0],
            innovation_ned[1],
            innovation_ned[2],
            nis,
            gnss_accepted ? 1 : 0,
            gnss_reject_total,
            h3_applied ? 1 : 0,
            grace_active ? 1 : 0,
            nis_gate_bypassed ? 1 : 0,
            p_pos_n,
            p_pos_e,
            p_pos_d);
    }
    return true;
}

float compute_gnss_course_rad(
    const double prev_pos_ned[3],
    const double curr_pos_ned[3],
    float *out_displacement_m)
{
    const double dn = curr_pos_ned[0] - prev_pos_ned[0];
    const double de = curr_pos_ned[1] - prev_pos_ned[1];
    const float displacement_m = static_cast<float>(std::sqrt((dn * dn) + (de * de)));
    if (out_displacement_m != nullptr) {
        *out_displacement_m = displacement_m;
    }
    return static_cast<float>(std::atan2(de, dn));
}

struct YawHeadingWindow {
    float headings_rad[kMaxYawHeadingWindow]{};
    uint32_t count;
    uint32_t capacity;
};

void yaw_heading_window_reset(YawHeadingWindow *window, uint32_t capacity)
{
    if (window == nullptr) {
        return;
    }
    window->count = 0U;
    window->capacity = capacity;
}

bool yaw_heading_window_push(YawHeadingWindow *window, float heading_rad)
{
    if (window == nullptr || window->capacity == 0U) {
        return false;
    }

    if (window->count < window->capacity) {
        window->headings_rad[window->count] = heading_rad;
        ++window->count;
        return true;
    }

    for (uint32_t i = 1U; i < window->capacity; ++i) {
        window->headings_rad[i - 1U] = window->headings_rad[i];
    }
    window->headings_rad[window->capacity - 1U] = heading_rad;
    return true;
}

bool yaw_heading_window_stats(
    const YawHeadingWindow *window,
    float *out_mean_rad,
    float *out_std_deg)
{
    if (window == nullptr || window->count == 0U) {
        return false;
    }

    float sum_sin = 0.0f;
    float sum_cos = 0.0f;
    for (uint32_t i = 0U; i < window->count; ++i) {
        sum_sin += std::sin(window->headings_rad[i]);
        sum_cos += std::cos(window->headings_rad[i]);
    }

    const float inv_n = 1.0f / static_cast<float>(window->count);
    sum_sin *= inv_n;
    sum_cos *= inv_n;

    const float mean_rad = std::atan2(sum_sin, sum_cos);
    const float resultant = std::sqrt((sum_sin * sum_sin) + (sum_cos * sum_cos));
    float std_rad = 0.0f;
    if (resultant > 1.0e-6f && resultant < 1.0f) {
        std_rad = std::sqrt(-2.0f * std::log(resultant));
    }

    if (out_mean_rad != nullptr) {
        *out_mean_rad = mean_rad;
    }
    if (out_std_deg != nullptr) {
        *out_std_deg = std_rad * kRadToDegF;
    }
    return true;
}

bool write_instrumentation_header(FILE *instrumentation_fp)
{
    if (instrumentation_fp == nullptr) {
        return false;
    }

    std::fprintf(
        instrumentation_fp,
        "timestamp_s,yaw_gnss_deg,yaw_ekf_deg,delta_yaw_deg,"
        "innovation_n_m,innovation_e_m,innovation_d_m,"
        "nis,gnss_accepted,gnss_reject_total\n");
    return true;
}

bool write_instrumentation_row(
    FILE *instrumentation_fp,
    double timestamp_s,
    bool has_yaw_gnss,
    float yaw_gnss_deg,
    float yaw_ekf_deg,
    const float innovation_ned[3],
    float nis,
    bool gnss_accepted,
    uint32_t gnss_reject_total)
{
    if (instrumentation_fp == nullptr) {
        return false;
    }

    const float delta_yaw_deg = has_yaw_gnss
        ? wrap_angle_deg(yaw_gnss_deg - yaw_ekf_deg)
        : 0.0f;

    if (has_yaw_gnss) {
        std::fprintf(
            instrumentation_fp,
            "%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%u\n",
            timestamp_s,
            yaw_gnss_deg,
            yaw_ekf_deg,
            delta_yaw_deg,
            innovation_ned[0],
            innovation_ned[1],
            innovation_ned[2],
            nis,
            gnss_accepted ? 1 : 0,
            gnss_reject_total);
    } else {
        std::fprintf(
            instrumentation_fp,
            "%.9f,,%.6f,,%.6f,%.6f,%.6f,%.6f,%d,%u\n",
            timestamp_s,
            yaw_ekf_deg,
            innovation_ned[0],
            innovation_ned[1],
            innovation_ned[2],
            nis,
            gnss_accepted ? 1 : 0,
            gnss_reject_total);
    }
    return true;
}

void snapshot_position_covariance(const InsEkfFilter &ekf, float out_hph[3][3])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            out_hph[i][j] = ekf.cov.P[INS_ERR_POS_N + static_cast<uint8_t>(i)]
                                      [INS_ERR_POS_N + static_cast<uint8_t>(j)];
        }
    }
}

void simulate_adapter_measurement_z(
    float ref_lat_deg,
    float ref_lon_deg,
    float ref_alt_m,
    const double pos_ned[3],
    float z_ned[3])
{
    const geodesy::LLA ref = geodesy::lla(ref_lat_deg, ref_lon_deg, ref_alt_m);
    const geodesy::NED pos{
        static_cast<float>(pos_ned[0]),
        static_cast<float>(pos_ned[1]),
        static_cast<float>(pos_ned[2]),
    };
    const geodesy::LLA point_lla = geodesy::ned_to_lla(pos, ref);
    const geodesy::NED measurement = geodesy::lla_to_ned(point_lla, ref);
    z_ned[0] = measurement.north_m;
    z_ned[1] = measurement.east_m;
    z_ned[2] = measurement.down_m;
}

struct LocationFixLla {
    float lat_deg;
    float lon_deg;
    float alt_m;
};

struct GeodesyDatumValidation {
    bool ready;
    geodesy::LLA ref;
    geodesy::LLA last_fix;
    geodesy::NED ned_wgs84;
    geodesy::NED ned_flat_legacy;
    bool has_replay_last;
    double replay_last_ned[3];
};

bool is_valid_location_lla(float lat_deg, float lon_deg, float alt_m)
{
    if (!std::isfinite(lat_deg) || !std::isfinite(lon_deg) || !std::isfinite(alt_m)) {
        return false;
    }
    if (std::fabs(lat_deg) < 1.0e-6f && std::fabs(lon_deg) < 1.0e-6f) {
        return false;
    }
    if (std::fabs(lat_deg) > 90.0f || std::fabs(lon_deg) > 180.0f) {
        return false;
    }
    return true;
}

bool append_location_fix_from_line(
    const char *line,
    std::vector<LocationFixLla> *fixes)
{
    if (line == nullptr || fixes == nullptr) {
        return false;
    }

    char buffer[kMaxCsvLineBytes];
    std::strncpy(buffer, line, sizeof(buffer) - 1U);
    buffer[sizeof(buffer) - 1U] = '\0';

    const char *fields[16]{};
    int field_count = 0;
    char *token = std::strtok(buffer, ",\r\n");
    while (token != nullptr && field_count < 16) {
        fields[field_count++] = token;
        token = std::strtok(nullptr, ",\r\n");
    }
    if (field_count <= 10) {
        return false;
    }

    const float alt_m = static_cast<float>(std::atof(fields[8]));
    const float lon_deg = static_cast<float>(std::atof(fields[9]));
    const float lat_deg = static_cast<float>(std::atof(fields[10]));
    if (!is_valid_location_lla(lat_deg, lon_deg, alt_m)) {
        return false;
    }

    fixes->push_back(LocationFixLla{lat_deg, lon_deg, alt_m});
    return true;
}

bool load_location_fixes(const char *path, std::vector<LocationFixLla> *fixes)
{
    if (path == nullptr || fixes == nullptr) {
        return false;
    }

    FILE *location_fp = std::fopen(path, "r");
    if (location_fp == nullptr) {
        return false;
    }

    char line[kMaxCsvLineBytes];
    if (std::fgets(line, static_cast<int>(sizeof(line)), location_fp) == nullptr) {
        std::fclose(location_fp);
        return false;
    }

    while (std::fgets(line, static_cast<int>(sizeof(line)), location_fp) != nullptr) {
        (void)append_location_fix_from_line(line, fixes);
    }

    std::fclose(location_fp);
    return !fixes->empty();
}

bool prescan_last_gps_replay_row(FILE *input_fp, double out_pos_ned[3])
{
    if (input_fp == nullptr || out_pos_ned == nullptr) {
        return false;
    }

    const long start_pos = std::ftell(input_fp);
    if (start_pos < 0L) {
        return false;
    }

    bool found = false;
    char line[kMaxCsvLineBytes];
    while (std::fgets(line, static_cast<int>(sizeof(line)), input_fp) != nullptr) {
        ParsedReplayRow row{};
        if (!parse_replay_csv_line(line, &row)) {
            continue;
        }
        if (row.row_type != ReplayRowType::GPS || !row.has_pos) {
            continue;
        }
        out_pos_ned[0] = row.pos_ned[0];
        out_pos_ned[1] = row.pos_ned[1];
        out_pos_ned[2] = row.pos_ned[2];
        found = true;
    }

    if (std::fseek(input_fp, start_pos, SEEK_SET) != 0) {
        return false;
    }
    return found;
}

bool prepare_geodesy_datum_validation(
    const char *replay_csv_path,
    const char *location_csv_path,
    FILE *replay_fp,
    GeodesyDatumValidation *out_validation)
{
    if (out_validation == nullptr) {
        return false;
    }

    *out_validation = GeodesyDatumValidation{};
    std::vector<LocationFixLla> fixes{};
    if (!load_location_fixes(location_csv_path, &fixes)) {
        return false;
    }

    const LocationFixLla &first = fixes.front();
    const LocationFixLla &last = fixes.back();
    out_validation->ref = geodesy::lla(first.lat_deg, first.lon_deg, first.alt_m);
    out_validation->last_fix = geodesy::lla(last.lat_deg, last.lon_deg, last.alt_m);
    out_validation->ned_wgs84 = geodesy::lla_to_ned(out_validation->last_fix, out_validation->ref);
    out_validation->ned_flat_legacy =
        geodesy::lla_to_ned_flat_legacy(out_validation->last_fix, out_validation->ref);
    out_validation->has_replay_last =
        prescan_last_gps_replay_row(replay_fp, out_validation->replay_last_ned);
    out_validation->ready = true;

    (void)replay_csv_path;
    return true;
}

void print_geodesy_datum_validation(const GeodesyDatumValidation &validation)
{
    if (!validation.ready) {
        std::printf(
            "REAL_RUN_REPLAY: H8 geodesy validation omitida (Location.csv no disponible)\n");
        return;
    }

    const float delta_n = validation.ned_flat_legacy.north_m - validation.ned_wgs84.north_m;
    const float delta_e = validation.ned_flat_legacy.east_m - validation.ned_wgs84.east_m;
    const float delta_d = validation.ned_flat_legacy.down_m - validation.ned_wgs84.down_m;
    const float delta_h = std::sqrt((delta_n * delta_n) + (delta_e * delta_e));

    std::printf("----------------------------------------------------------------\n");
    std::printf(" REAL_RUN_REPLAY: H8 validacion datum LLA->NED (ultimo fix del trayecto)\n");
    std::printf(
        "  Origen ref LLA:  lat=%.7f lon=%.7f alt=%.1f m\n",
        validation.ref.lat_deg,
        validation.ref.lon_deg,
        validation.ref.alt_m);
    std::printf(
        "  Ultimo fix LLA:  lat=%.7f lon=%.7f alt=%.1f m\n",
        validation.last_fix.lat_deg,
        validation.last_fix.lon_deg,
        validation.last_fix.alt_m);
    std::printf(
        "  NED plano legacy: N=%.3f E=%.3f D=%.3f m\n",
        validation.ned_flat_legacy.north_m,
        validation.ned_flat_legacy.east_m,
        validation.ned_flat_legacy.down_m);
    std::printf(
        "  NED WGS84 ECEF:   N=%.3f E=%.3f D=%.3f m\n",
        validation.ned_wgs84.north_m,
        validation.ned_wgs84.east_m,
        validation.ned_wgs84.down_m);
    std::printf(
        "  Delta legacy-WGS84: dN=%.3f dE=%.3f dD=%.3f |H|=%.3f m\n",
        delta_n,
        delta_e,
        delta_d,
        delta_h);
    if (validation.has_replay_last) {
        const float replay_delta_n =
            static_cast<float>(validation.replay_last_ned[0]) - validation.ned_wgs84.north_m;
        const float replay_delta_e =
            static_cast<float>(validation.replay_last_ned[1]) - validation.ned_wgs84.east_m;
        const float replay_delta_h =
            std::sqrt((replay_delta_n * replay_delta_n) + (replay_delta_e * replay_delta_e));
        std::printf(
            "  Replay CSV ultimo: N=%.3f E=%.3f D=%.3f m  (delta vs WGS84 |H|=%.4f m)\n",
            validation.replay_last_ned[0],
            validation.replay_last_ned[1],
            validation.replay_last_ned[2],
            replay_delta_h);
    }
    std::printf(
        "  Pipeline activo: WGS84 ECEF->NED (geodesy.hpp). Error datum plano eliminado.\n");
    std::printf("----------------------------------------------------------------\n");
}

bool write_h8_propagation_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,dt_s,"
        "a_sens_x,a_sens_y,a_sens_z,"
        "a_body_x,a_body_y,a_body_z,"
        "a_corr_x,a_corr_y,a_corr_z,"
        "a_nav_x,a_nav_y,a_nav_z,"
        "a_lin_x,a_lin_y,a_lin_z,"
        "bias_ax,bias_ay,bias_az,"
        "vel_pre_n,vel_pre_e,vel_pre_d,"
        "vel_post_n,vel_post_e,vel_post_d,"
        "pos_n,pos_e,pos_d,"
        "roll_deg,pitch_deg,yaw_deg,"
        "gps_speed_mps,constraint_mode\n");
    return true;
}

bool write_h8_propagation_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    float dt_s,
    const float a_sensor[3],
    const float a_body[3],
    const InsEkfPredictAudit &audit,
    const float vel_post_ned[3],
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    float gps_speed_mps,
    int constraint_mode)
{
    if (audit_fp == nullptr || a_sensor == nullptr || a_body == nullptr || vel_post_ned == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "%.9f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.3f,%d\n",
        timestamp_s,
        dt_s,
        a_sensor[0],
        a_sensor[1],
        a_sensor[2],
        a_body[0],
        a_body[1],
        a_body[2],
        audit.a_corr_mps2[0],
        audit.a_corr_mps2[1],
        audit.a_corr_mps2[2],
        audit.a_nav_mps2[0],
        audit.a_nav_mps2[1],
        audit.a_nav_mps2[2],
        audit.a_lin_mps2[0],
        audit.a_lin_mps2[1],
        audit.a_lin_mps2[2],
        audit.bias_a_mps2[0],
        audit.bias_a_mps2[1],
        audit.bias_a_mps2[2],
        audit.vel_ned_mps[0],
        audit.vel_ned_mps[1],
        audit.vel_ned_mps[2],
        vel_post_ned[0],
        vel_post_ned[1],
        vel_post_ned[2],
        audit.pos_ned_m[0],
        audit.pos_ned_m[1],
        audit.pos_ned_m[2],
        roll_deg,
        pitch_deg,
        yaw_deg,
        gps_speed_mps,
        constraint_mode);
    return true;
}

bool write_h9_tilt_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,"
        "roll_ekf_deg,pitch_ekf_deg,yaw_ekf_deg,"
        "g_body_x,g_body_y,g_body_z,"
        "g_nav_n,g_nav_e,g_nav_d,"
        "a_nav_n,a_nav_e,a_nav_d,"
        "a_lin_n,a_lin_e,a_lin_d,a_lin_h,"
        "constraint_mode\n");
    return true;
}

bool write_h9_tilt_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    const InsEkfPredictAudit &audit,
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    int constraint_mode)
{
    if (audit_fp == nullptr) {
        return false;
    }

    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_body[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_kinematics_ned_to_body(audit.dcm_bn, g_ned, g_body);

    const float a_lin_h = std::sqrt(
        (audit.a_lin_mps2[0] * audit.a_lin_mps2[0])
        + (audit.a_lin_mps2[1] * audit.a_lin_mps2[1]));

    std::fprintf(
        audit_fp,
        "%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%d\n",
        timestamp_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        g_body[0],
        g_body[1],
        g_body[2],
        g_ned[0],
        g_ned[1],
        g_ned[2],
        audit.a_nav_mps2[0],
        audit.a_nav_mps2[1],
        audit.a_nav_mps2[2],
        audit.a_lin_mps2[0],
        audit.a_lin_mps2[1],
        audit.a_lin_mps2[2],
        a_lin_h,
        constraint_mode);
    return true;
}

struct GravityTiltInitAccumulator {
    bool applied;
    bool started;
    float accel_sum[3];
    float gyro_sum[3];
    uint32_t sample_count;
    double first_timestamp_s;
};

bool accumulate_gravity_tilt_sample(
    GravityTiltInitAccumulator &acc,
    double timestamp_s,
    const float aligned_accel[3],
    const float aligned_gyro[3])
{
    if (aligned_accel == nullptr || aligned_gyro == nullptr) {
        return false;
    }

    if (!acc.started) {
        acc.started = true;
        acc.first_timestamp_s = timestamp_s;
    }

    acc.accel_sum[0] += aligned_accel[0];
    acc.accel_sum[1] += aligned_accel[1];
    acc.accel_sum[2] += aligned_accel[2];
    acc.gyro_sum[0] += aligned_gyro[0];
    acc.gyro_sum[1] += aligned_gyro[1];
    acc.gyro_sum[2] += aligned_gyro[2];
    ++acc.sample_count;
    return true;
}

bool gravity_tilt_init_ready(
    const GravityTiltInitAccumulator &acc,
    double timestamp_s,
    uint32_t min_samples,
    float max_window_s)
{
    if (!acc.started || acc.sample_count == 0U) {
        return false;
    }
    if (acc.sample_count >= min_samples) {
        return true;
    }
    return (timestamp_s - acc.first_timestamp_s) >= static_cast<double>(max_window_s);
}

bool finalize_gravity_tilt_init(
    GravityTiltInitAccumulator &acc,
    InsEkfFilter *ekf,
    double timestamp_s,
    float *out_roll_deg,
    float *out_pitch_deg)
{
    if (ekf == nullptr || acc.sample_count == 0U) {
        return false;
    }

    const float inv_count = 1.0f / static_cast<float>(acc.sample_count);
    const float mean_accel[3] = {
        acc.accel_sum[0] * inv_count,
        acc.accel_sum[1] * inv_count,
        acc.accel_sum[2] * inv_count,
    };
    const float mean_gyro[3] = {
        acc.gyro_sum[0] * inv_count,
        acc.gyro_sum[1] * inv_count,
        acc.gyro_sum[2] * inv_count,
    };

    if (!ins_ekf_apply_gravity_tilt_init(ekf, mean_accel, mean_gyro)) {
        return false;
    }

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    ins_ekf_get_attitude_rad(ekf, &roll_rad, &pitch_rad, nullptr);
    if (out_roll_deg != nullptr) {
        *out_roll_deg = roll_rad * kRadToDegF;
    }
    if (out_pitch_deg != nullptr) {
        *out_pitch_deg = pitch_rad * kRadToDegF;
    }

    acc.applied = true;
    std::printf(
        "REAL_RUN_REPLAY: H9a gravity tilt init @ t=%.3f s | samples=%u | "
        "mean_a_body=(%.3f, %.3f, %.3f) | mean_w_body=(%.5f, %.5f, %.5f) | "
        "roll=%.2f pitch=%.2f deg\n",
        timestamp_s,
        acc.sample_count,
        mean_accel[0],
        mean_accel[1],
        mean_accel[2],
        mean_gyro[0],
        mean_gyro[1],
        mean_gyro[2],
        roll_rad * kRadToDegF,
        pitch_rad * kRadToDegF);
    return true;
}

float vector_angle_deg(const float a[3], const float b[3])
{
    if (a == nullptr || b == nullptr) {
        return 0.0f;
    }

    const float a_norm = std::sqrt((a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]));
    const float b_norm = std::sqrt((b[0] * b[0]) + (b[1] * b[1]) + (b[2] * b[2]));
    if (a_norm < 1.0e-6f || b_norm < 1.0e-6f) {
        return 0.0f;
    }

    float dot = ((a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2])) / (a_norm * b_norm);
    dot = std::fmax(-1.0f, std::fmin(1.0f, dot));
    return std::acos(dot) * kRadToDegF;
}

float vector3_norm(const float v[3])
{
    if (v == nullptr) {
        return 0.0f;
    }
    return std::sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

float vector3_horizontal_norm_ned(const float v[3])
{
    if (v == nullptr) {
        return 0.0f;
    }
    return std::sqrt((v[0] * v[0]) + (v[1] * v[1]));
}

bool normalize_vector3(const float in[3], float out[3])
{
    if (in == nullptr || out == nullptr) {
        return false;
    }

    const float norm = std::sqrt((in[0] * in[0]) + (in[1] * in[1]) + (in[2] * in[2]));
    if (norm < 1.0e-6f) {
        return false;
    }

    const float inv = 1.0f / norm;
    out[0] = in[0] * inv;
    out[1] = in[1] * inv;
    out[2] = in[2] * inv;
    return true;
}

bool write_h9a_gravity_alignment_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,"
        "roll_ekf_deg,pitch_ekf_deg,yaw_ekf_deg,"
        "g_meas_x,g_meas_y,g_meas_z,"
        "g_pred_x,g_pred_y,g_pred_z,"
        "g_corr_x,g_corr_y,g_corr_z,"
        "gravity_alignment_error_deg,"
        "gravity_alignment_corr_error_deg,"
        "predicted_a_lin_h_from_angle_mps2,"
        "a_lin_h_mps2,"
        "implied_tilt_from_a_lin_deg,"
        "h9a_applied,"
        "constraint_mode\n");
    return true;
}

bool write_h9a_gravity_alignment_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    const float aligned_accel[3],
    const InsEkfPredictAudit &audit,
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    bool h9a_applied,
    int constraint_mode)
{
    if (audit_fp == nullptr || aligned_accel == nullptr) {
        return false;
    }

    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_pred[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_kinematics_ned_to_body(audit.dcm_bn, g_ned, g_pred);

    float g_meas_unit[3] = {0.0f, 0.0f, 0.0f};
    float g_pred_unit[3] = {0.0f, 0.0f, 0.0f};
    float g_corr_unit[3] = {0.0f, 0.0f, 0.0f};
    if (!normalize_vector3(aligned_accel, g_meas_unit)
        || !normalize_vector3(g_pred, g_pred_unit)
        || !normalize_vector3(audit.a_corr_mps2, g_corr_unit)) {
        return false;
    }

    const float alignment_error_deg = vector_angle_deg(g_meas_unit, g_pred_unit);
    const float alignment_corr_error_deg = vector_angle_deg(g_corr_unit, g_pred_unit);
    const float alignment_error_rad = alignment_error_deg * static_cast<float>(M_PI) / 180.0f;
    const float predicted_a_lin_h =
        NAVICORE_INS_EKF_GRAVITY_MPS2 * std::sin(alignment_error_rad);

    const float a_lin_h = std::sqrt(
        (audit.a_lin_mps2[0] * audit.a_lin_mps2[0])
        + (audit.a_lin_mps2[1] * audit.a_lin_mps2[1]));
    const float implied_tilt_deg = std::asin(
        std::fmax(-1.0f, std::fmin(1.0f, a_lin_h / NAVICORE_INS_EKF_GRAVITY_MPS2)))
        * kRadToDegF;

    std::fprintf(
        audit_fp,
        "%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,"
        "%d,%d\n",
        timestamp_s,
        roll_deg,
        pitch_deg,
        yaw_deg,
        g_meas_unit[0],
        g_meas_unit[1],
        g_meas_unit[2],
        g_pred_unit[0],
        g_pred_unit[1],
        g_pred_unit[2],
        g_corr_unit[0],
        g_corr_unit[1],
        g_corr_unit[2],
        alignment_error_deg,
        alignment_corr_error_deg,
        predicted_a_lin_h,
        a_lin_h,
        implied_tilt_deg,
        h9a_applied ? 1 : 0,
        constraint_mode);
    return true;
}

float h9a_gravity_alignment_audit_end_s(const RealRunReplayConfig &config)
{
    if (config.replay_end_s > 0.0f) {
        return config.replay_end_s;
    }
    return 60.0f;
}

float h9b_attitude_propagation_audit_end_s(const RealRunReplayConfig &config)
{
    return h9a_gravity_alignment_audit_end_s(config);
}

bool rotation_vector_align_unit(
    const float from_unit[3],
    const float to_unit[3],
    float out_rv_rad[3])
{
    if (from_unit == nullptr || to_unit == nullptr || out_rv_rad == nullptr) {
        return false;
    }

    const float cross[3] = {
        (from_unit[1] * to_unit[2]) - (from_unit[2] * to_unit[1]),
        (from_unit[2] * to_unit[0]) - (from_unit[0] * to_unit[2]),
        (from_unit[0] * to_unit[1]) - (from_unit[1] * to_unit[0]),
    };
    const float cross_norm = std::sqrt(
        (cross[0] * cross[0]) + (cross[1] * cross[1]) + (cross[2] * cross[2]));
    float dot = (from_unit[0] * to_unit[0])
        + (from_unit[1] * to_unit[1])
        + (from_unit[2] * to_unit[2]);
    dot = std::fmax(-1.0f, std::fmin(1.0f, dot));

    if (cross_norm < 1.0e-8f) {
        if (dot > 0.0f) {
            out_rv_rad[0] = 0.0f;
            out_rv_rad[1] = 0.0f;
            out_rv_rad[2] = 0.0f;
            return true;
        }

        float axis[3] = {1.0f, 0.0f, 0.0f};
        if (std::fabs(from_unit[0]) > 0.9f) {
            axis[0] = 0.0f;
            axis[1] = 1.0f;
            axis[2] = 0.0f;
        }
        const float axis_dot = (axis[0] * from_unit[0])
            + (axis[1] * from_unit[1])
            + (axis[2] * from_unit[2]);
        axis[0] -= axis_dot * from_unit[0];
        axis[1] -= axis_dot * from_unit[1];
        axis[2] -= axis_dot * from_unit[2];
        if (!normalize_vector3(axis, axis)) {
            return false;
        }
        const float angle = static_cast<float>(M_PI);
        out_rv_rad[0] = axis[0] * angle;
        out_rv_rad[1] = axis[1] * angle;
        out_rv_rad[2] = axis[2] * angle;
        return true;
    }

    const float angle = std::atan2(cross_norm, dot);
    const float scale = angle / cross_norm;
    out_rv_rad[0] = cross[0] * scale;
    out_rv_rad[1] = cross[1] * scale;
    out_rv_rad[2] = cross[2] * scale;
    return true;
}

bool write_h9b_attitude_propagation_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,dt_s,"
        "gyro_raw_x,gyro_raw_y,gyro_raw_z,"
        "gyro_bias_x,gyro_bias_y,gyro_bias_z,"
        "gyro_corr_x,gyro_corr_y,gyro_corr_z,"
        "delta_theta_int_x,delta_theta_int_y,delta_theta_int_z,delta_theta_int_mag_deg,"
        "quat_before_w,quat_before_x,quat_before_y,quat_before_z,"
        "quat_after_w,quat_after_x,quat_after_y,quat_after_z,"
        "roll_before_deg,pitch_before_deg,yaw_before_deg,"
        "roll_after_deg,pitch_after_deg,yaw_after_deg,"
        "g_meas_x,g_meas_y,g_meas_z,"
        "g_pred_x,g_pred_y,g_pred_z,"
        "gravity_alignment_error_deg,"
        "delta_theta_gravity_x,delta_theta_gravity_y,delta_theta_gravity_z,delta_theta_gravity_mag_deg,"
        "delta_theta_gravity_step_x,delta_theta_gravity_step_y,delta_theta_gravity_step_z,delta_theta_gravity_step_mag_deg,"
        "delta_theta_int_vs_gravity_step_dot,"
        "a_lin_h_mps2,"
        "h9a_applied,"
        "constraint_mode\n");
    return true;
}

bool write_h9b_attitude_propagation_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    const float aligned_accel[3],
    const InsEkfPredictAudit &predict_audit,
    const InsEkfAttitudePropAudit &att_audit,
    bool h9a_applied,
    int constraint_mode,
    bool *has_prev_gravity_rv,
    float prev_gravity_rv_rad[3])
{
    if (audit_fp == nullptr || aligned_accel == nullptr || att_audit.valid == false) {
        return false;
    }

    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_pred[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_kinematics_ned_to_body(predict_audit.dcm_bn, g_ned, g_pred);

    float g_meas_unit[3] = {0.0f, 0.0f, 0.0f};
    float g_pred_unit[3] = {0.0f, 0.0f, 0.0f};
    if (!normalize_vector3(aligned_accel, g_meas_unit)
        || !normalize_vector3(g_pred, g_pred_unit)) {
        return false;
    }

    const float alignment_error_deg = vector_angle_deg(g_meas_unit, g_pred_unit);

    float delta_theta_gravity_rad[3] = {0.0f, 0.0f, 0.0f};
    if (!rotation_vector_align_unit(g_pred_unit, g_meas_unit, delta_theta_gravity_rad)) {
        return false;
    }
    const float delta_theta_gravity_mag_deg =
        std::sqrt(
            (delta_theta_gravity_rad[0] * delta_theta_gravity_rad[0])
            + (delta_theta_gravity_rad[1] * delta_theta_gravity_rad[1])
            + (delta_theta_gravity_rad[2] * delta_theta_gravity_rad[2]))
        * kRadToDegF;

    float delta_theta_gravity_step_rad[3] = {0.0f, 0.0f, 0.0f};
    float delta_theta_gravity_step_mag_deg = 0.0f;
    if (has_prev_gravity_rv != nullptr
        && prev_gravity_rv_rad != nullptr
        && *has_prev_gravity_rv) {
        delta_theta_gravity_step_rad[0] =
            delta_theta_gravity_rad[0] - prev_gravity_rv_rad[0];
        delta_theta_gravity_step_rad[1] =
            delta_theta_gravity_rad[1] - prev_gravity_rv_rad[1];
        delta_theta_gravity_step_rad[2] =
            delta_theta_gravity_rad[2] - prev_gravity_rv_rad[2];
        delta_theta_gravity_step_mag_deg =
            std::sqrt(
                (delta_theta_gravity_step_rad[0] * delta_theta_gravity_step_rad[0])
                + (delta_theta_gravity_step_rad[1] * delta_theta_gravity_step_rad[1])
                + (delta_theta_gravity_step_rad[2] * delta_theta_gravity_step_rad[2]))
            * kRadToDegF;
    }

    if (has_prev_gravity_rv != nullptr && prev_gravity_rv_rad != nullptr) {
        prev_gravity_rv_rad[0] = delta_theta_gravity_rad[0];
        prev_gravity_rv_rad[1] = delta_theta_gravity_rad[1];
        prev_gravity_rv_rad[2] = delta_theta_gravity_rad[2];
        *has_prev_gravity_rv = true;
    }

    const float delta_theta_int_mag_deg = att_audit.delta_theta_integrated_mag_rad * kRadToDegF;
    const float int_dot_gravity_step =
        (att_audit.delta_theta_integrated_rad[0] * delta_theta_gravity_step_rad[0])
        + (att_audit.delta_theta_integrated_rad[1] * delta_theta_gravity_step_rad[1])
        + (att_audit.delta_theta_integrated_rad[2] * delta_theta_gravity_step_rad[2]);

    const float a_lin_h = std::sqrt(
        (predict_audit.a_lin_mps2[0] * predict_audit.a_lin_mps2[0])
        + (predict_audit.a_lin_mps2[1] * predict_audit.a_lin_mps2[1]));

    std::fprintf(
        audit_fp,
        "%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,"
        "%.6f,"
        "%d,%d\n",
        timestamp_s,
        att_audit.dt_s,
        att_audit.gyro_raw_radps[0],
        att_audit.gyro_raw_radps[1],
        att_audit.gyro_raw_radps[2],
        att_audit.gyro_bias_radps[0],
        att_audit.gyro_bias_radps[1],
        att_audit.gyro_bias_radps[2],
        att_audit.gyro_corr_radps[0],
        att_audit.gyro_corr_radps[1],
        att_audit.gyro_corr_radps[2],
        att_audit.delta_theta_integrated_rad[0],
        att_audit.delta_theta_integrated_rad[1],
        att_audit.delta_theta_integrated_rad[2],
        delta_theta_int_mag_deg,
        att_audit.q_before[0],
        att_audit.q_before[1],
        att_audit.q_before[2],
        att_audit.q_before[3],
        att_audit.q_after[0],
        att_audit.q_after[1],
        att_audit.q_after[2],
        att_audit.q_after[3],
        att_audit.roll_before_rad * kRadToDegF,
        att_audit.pitch_before_rad * kRadToDegF,
        att_audit.yaw_before_rad * kRadToDegF,
        att_audit.roll_after_rad * kRadToDegF,
        att_audit.pitch_after_rad * kRadToDegF,
        att_audit.yaw_after_rad * kRadToDegF,
        g_meas_unit[0],
        g_meas_unit[1],
        g_meas_unit[2],
        g_pred_unit[0],
        g_pred_unit[1],
        g_pred_unit[2],
        alignment_error_deg,
        delta_theta_gravity_rad[0],
        delta_theta_gravity_rad[1],
        delta_theta_gravity_rad[2],
        delta_theta_gravity_mag_deg,
        delta_theta_gravity_step_rad[0],
        delta_theta_gravity_step_rad[1],
        delta_theta_gravity_step_rad[2],
        delta_theta_gravity_step_mag_deg,
        int_dot_gravity_step,
        a_lin_h,
        h9a_applied ? 1 : 0,
        constraint_mode);
    return true;
}

float h9d_gravity_subtraction_audit_end_s(const RealRunReplayConfig &config)
{
    return h9b_attitude_propagation_audit_end_s(config);
}

bool write_h9d_gravity_subtraction_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,dt_s,"
        "a_body_x,a_body_y,a_body_z,"
        "a_corr_x,a_corr_y,a_corr_z,"
        "bias_ax,bias_ay,bias_az,"
        "roll_deg,pitch_deg,yaw_deg,"
        "dcm_bn_00,dcm_bn_01,dcm_bn_02,"
        "dcm_bn_10,dcm_bn_11,dcm_bn_12,"
        "dcm_bn_20,dcm_bn_21,dcm_bn_22,"
        "a_nav_pre_g_n,a_nav_pre_g_e,a_nav_pre_g_d,a_nav_pre_g_h,"
        "g_ned_n,g_ned_e,g_ned_d,"
        "g_body_pred_x,g_body_pred_y,g_body_pred_z,"
        "a_lin_n,a_lin_e,a_lin_d,a_lin_h,"
        "residual_body_x,residual_body_y,residual_body_z,residual_body_h,"
        "angle_a_corr_vs_g_body_pred_deg,"
        "body_axis0_to_nav_h_mps2,body_axis1_to_nav_h_mps2,body_axis2_to_nav_h_mps2,"
        "gravity_alignment_error_deg,"
        "gps_speed_mps,"
        "h9a_applied,"
        "constraint_mode\n");
    return true;
}

bool write_h9d_gravity_subtraction_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    const float aligned_accel[3],
    const InsEkfPredictAudit &audit,
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    float gps_speed_mps,
    bool h9a_applied,
    int constraint_mode)
{
    if (audit_fp == nullptr || aligned_accel == nullptr || !audit.valid) {
        return false;
    }

    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_body_pred[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_kinematics_ned_to_body(audit.dcm_bn, g_ned, g_body_pred);

    const float a_nav_pre_g_h = std::sqrt(
        (audit.a_nav_mps2[0] * audit.a_nav_mps2[0])
        + (audit.a_nav_mps2[1] * audit.a_nav_mps2[1]));

    float body_axis_to_nav_h[3] = {0.0f, 0.0f, 0.0f};
    for (uint8_t body_axis = 0U; body_axis < 3U; ++body_axis) {
        const float comp_n = audit.dcm_bn[0][body_axis] * audit.a_corr_mps2[body_axis];
        const float comp_e = audit.dcm_bn[1][body_axis] * audit.a_corr_mps2[body_axis];
        body_axis_to_nav_h[body_axis] = std::sqrt((comp_n * comp_n) + (comp_e * comp_e));
    }

    float residual_body[3] = {
        audit.a_corr_mps2[0] - g_body_pred[0],
        audit.a_corr_mps2[1] - g_body_pred[1],
        audit.a_corr_mps2[2] - g_body_pred[2],
    };
    const float residual_body_h = std::sqrt(
        (residual_body[0] * residual_body[0]) + (residual_body[1] * residual_body[1]));

    float a_corr_unit[3] = {0.0f, 0.0f, 0.0f};
    float g_body_unit[3] = {0.0f, 0.0f, 0.0f};
    float alignment_error_deg = 0.0f;
    if (normalize_vector3(audit.a_corr_mps2, a_corr_unit)
        && normalize_vector3(g_body_pred, g_body_unit)) {
        alignment_error_deg = vector_angle_deg(a_corr_unit, g_body_unit);
    }

    const float a_lin_h = std::sqrt(
        (audit.a_lin_mps2[0] * audit.a_lin_mps2[0])
        + (audit.a_lin_mps2[1] * audit.a_lin_mps2[1]));

    std::fprintf(
        audit_fp,
        "%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.6f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,%.6f,"
        "%d,%d\n",
        timestamp_s,
        audit.dt_s,
        aligned_accel[0],
        aligned_accel[1],
        aligned_accel[2],
        audit.a_corr_mps2[0],
        audit.a_corr_mps2[1],
        audit.a_corr_mps2[2],
        audit.bias_a_mps2[0],
        audit.bias_a_mps2[1],
        audit.bias_a_mps2[2],
        roll_deg,
        pitch_deg,
        yaw_deg,
        audit.dcm_bn[0][0],
        audit.dcm_bn[0][1],
        audit.dcm_bn[0][2],
        audit.dcm_bn[1][0],
        audit.dcm_bn[1][1],
        audit.dcm_bn[1][2],
        audit.dcm_bn[2][0],
        audit.dcm_bn[2][1],
        audit.dcm_bn[2][2],
        audit.a_nav_mps2[0],
        audit.a_nav_mps2[1],
        audit.a_nav_mps2[2],
        a_nav_pre_g_h,
        g_ned[0],
        g_ned[1],
        g_ned[2],
        g_body_pred[0],
        g_body_pred[1],
        g_body_pred[2],
        audit.a_lin_mps2[0],
        audit.a_lin_mps2[1],
        audit.a_lin_mps2[2],
        a_lin_h,
        residual_body[0],
        residual_body[1],
        residual_body[2],
        residual_body_h,
        alignment_error_deg,
        body_axis_to_nav_h[0],
        body_axis_to_nav_h[1],
        body_axis_to_nav_h[2],
        alignment_error_deg,
        gps_speed_mps,
        h9a_applied ? 1 : 0,
        constraint_mode);
    return true;
}

float propagation_chain_audit_end_s(const RealRunReplayConfig &config)
{
    return h9d_gravity_subtraction_audit_end_s(config);
}

void mat3_vec3_mul_dcm_bn(const float dcm_bn[3][3], const float body[3], float ned[3])
{
    if (dcm_bn == nullptr || body == nullptr || ned == nullptr) {
        return;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        ned[i] = (dcm_bn[i][0] * body[0])
            + (dcm_bn[i][1] * body[1])
            + (dcm_bn[i][2] * body[2]);
    }
}

bool write_propagation_chain_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,dt_s,"
        "a_raw_x,a_raw_y,a_raw_z,a_raw_norm,"
        "a_body_x,a_body_y,a_body_z,a_body_norm,"
        "bias_x,bias_y,bias_z,bias_norm,"
        "a_corr_x,a_corr_y,a_corr_z,a_corr_norm,"
        "a_nav_body_x,a_nav_body_y,a_nav_body_z,a_nav_body_norm,a_nav_body_h,"
        "a_nav_corr_x,a_nav_corr_y,a_nav_corr_z,a_nav_corr_norm,a_nav_corr_h,"
        "r_bn_bias_x,r_bn_bias_y,r_bn_bias_z,r_bn_bias_norm,r_bn_bias_h,"
        "nav_body_minus_corr_x,nav_body_minus_corr_y,nav_body_minus_corr_z,nav_body_minus_corr_h,"
        "a_lin_x,a_lin_y,a_lin_z,a_lin_norm,a_lin_h,"
        "g_ned_n,g_ned_e,g_ned_d,"
        "g_body_pred_x,g_body_pred_y,g_body_pred_z,g_body_pred_norm,"
        "g_body_meas_x,g_body_meas_y,g_body_meas_z,"
        "gravity_angle_deg,"
        "proj_body_long_mps2,proj_body_lat_mps2,proj_body_vert_mps2,"
        "proj_ned_n_mps2,proj_ned_e_mps2,proj_ned_d_mps2,"
        "roll_deg,pitch_deg,yaw_deg,"
        "gps_speed_mps,"
        "h9a_applied,"
        "constraint_mode\n");
    return true;
}

bool write_propagation_chain_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    const float a_raw_sensor[3],
    const float a_body[3],
    const InsEkfPredictAudit &audit,
    float roll_deg,
    float pitch_deg,
    float yaw_deg,
    float gps_speed_mps,
    bool h9a_applied,
    int constraint_mode)
{
    if (audit_fp == nullptr || a_raw_sensor == nullptr || a_body == nullptr || !audit.valid) {
        return false;
    }

    const float a_raw_norm = vector3_norm(a_raw_sensor);
    const float a_body_norm = vector3_norm(a_body);
    const float bias_norm = vector3_norm(audit.bias_a_mps2);
    const float a_corr_norm = vector3_norm(audit.a_corr_mps2);

    float a_nav_body[3] = {0.0f, 0.0f, 0.0f};
    mat3_vec3_mul_dcm_bn(audit.dcm_bn, a_body, a_nav_body);
    const float a_nav_body_norm = vector3_norm(a_nav_body);
    const float a_nav_body_h = vector3_horizontal_norm_ned(a_nav_body);

    const float a_nav_corr_norm = vector3_norm(audit.a_nav_mps2);
    const float a_nav_corr_h = vector3_horizontal_norm_ned(audit.a_nav_mps2);

    float r_bn_bias[3] = {0.0f, 0.0f, 0.0f};
    mat3_vec3_mul_dcm_bn(audit.dcm_bn, audit.bias_a_mps2, r_bn_bias);
    const float r_bn_bias_norm = vector3_norm(r_bn_bias);
    const float r_bn_bias_h = vector3_horizontal_norm_ned(r_bn_bias);

    const float nav_body_minus_corr[3] = {
        a_nav_body[0] - audit.a_nav_mps2[0],
        a_nav_body[1] - audit.a_nav_mps2[1],
        a_nav_body[2] - audit.a_nav_mps2[2],
    };
    const float nav_body_minus_corr_h = vector3_horizontal_norm_ned(nav_body_minus_corr);

    const float a_lin_norm = vector3_norm(audit.a_lin_mps2);
    const float a_lin_h = vector3_horizontal_norm_ned(audit.a_lin_mps2);

    const float g_ned[3] = {
        0.0f,
        0.0f,
        NAVICORE_INS_EKF_GRAVITY_MPS2,
    };
    float g_body_pred[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_kinematics_ned_to_body(audit.dcm_bn, g_ned, g_body_pred);
    const float g_body_pred_norm = vector3_norm(g_body_pred);

    float g_body_meas[3] = {0.0f, 0.0f, 0.0f};
    float gravity_angle_deg = 0.0f;
    if (normalize_vector3(audit.a_corr_mps2, g_body_meas)) {
        float g_body_pred_unit[3] = {0.0f, 0.0f, 0.0f};
        if (normalize_vector3(g_body_pred, g_body_pred_unit)) {
            gravity_angle_deg = vector_angle_deg(g_body_meas, g_body_pred_unit);
        }
    }

    std::fprintf(
        audit_fp,
        "%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,"
        "%.9f,%.9f,%.9f,"
        "%.9f,%.9f,%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,"
        "%d,%d\n",
        timestamp_s,
        audit.dt_s,
        a_raw_sensor[0],
        a_raw_sensor[1],
        a_raw_sensor[2],
        a_raw_norm,
        a_body[0],
        a_body[1],
        a_body[2],
        a_body_norm,
        audit.bias_a_mps2[0],
        audit.bias_a_mps2[1],
        audit.bias_a_mps2[2],
        bias_norm,
        audit.a_corr_mps2[0],
        audit.a_corr_mps2[1],
        audit.a_corr_mps2[2],
        a_corr_norm,
        a_nav_body[0],
        a_nav_body[1],
        a_nav_body[2],
        a_nav_body_norm,
        a_nav_body_h,
        audit.a_nav_mps2[0],
        audit.a_nav_mps2[1],
        audit.a_nav_mps2[2],
        a_nav_corr_norm,
        a_nav_corr_h,
        r_bn_bias[0],
        r_bn_bias[1],
        r_bn_bias[2],
        r_bn_bias_norm,
        r_bn_bias_h,
        nav_body_minus_corr[0],
        nav_body_minus_corr[1],
        nav_body_minus_corr[2],
        nav_body_minus_corr_h,
        audit.a_lin_mps2[0],
        audit.a_lin_mps2[1],
        audit.a_lin_mps2[2],
        a_lin_norm,
        a_lin_h,
        g_ned[0],
        g_ned[1],
        g_ned[2],
        g_body_pred[0],
        g_body_pred[1],
        g_body_pred[2],
        g_body_pred_norm,
        g_body_meas[0],
        g_body_meas[1],
        g_body_meas[2],
        gravity_angle_deg,
        audit.a_corr_mps2[0],
        audit.a_corr_mps2[1],
        audit.a_corr_mps2[2],
        audit.a_lin_mps2[0],
        audit.a_lin_mps2[1],
        audit.a_lin_mps2[2],
        roll_deg,
        pitch_deg,
        yaw_deg,
        gps_speed_mps,
        h9a_applied ? 1 : 0,
        constraint_mode);
    return true;
}

float h9_tilt_audit_end_s(const RealRunReplayConfig &config)
{
    if (config.predict_only_mode && config.replay_end_s > 0.0f) {
        return config.replay_end_s;
    }
    return config.static_phase_end_s;
}

bool write_h7_update_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,gps_index,z_n,z_e,z_d,hx_n,hx_e,hx_d,"
        "innov_n,innov_e,innov_d,S_nn,S_ee,S_dd,nis,nis_threshold,"
        "gnss_accepted,innov_h_m,gps_pos_n,gps_pos_e,gps_pos_d\n");
    return true;
}

bool write_h7_update_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    uint32_t gps_index,
    const float z_ned[3],
    const float hx_ned[3],
    const float innov_ned[3],
    const float s_matrix[3][3],
    float nis,
    float nis_threshold,
    bool gnss_accepted,
    const double gps_pos_ned[3])
{
    if (audit_fp == nullptr || z_ned == nullptr || hx_ned == nullptr || innov_ned == nullptr) {
        return false;
    }

    const float innov_h = std::sqrt(
        (innov_ned[0] * innov_ned[0]) + (innov_ned[1] * innov_ned[1]));

    std::fprintf(
        audit_fp,
        "%.9f,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%d,%.6f,%.6f,%.6f,%.6f\n",
        timestamp_s,
        gps_index,
        z_ned[0],
        z_ned[1],
        z_ned[2],
        hx_ned[0],
        hx_ned[1],
        hx_ned[2],
        innov_ned[0],
        innov_ned[1],
        innov_ned[2],
        s_matrix[0][0],
        s_matrix[1][1],
        s_matrix[2][2],
        nis,
        nis_threshold,
        gnss_accepted ? 1 : 0,
        innov_h,
        gps_pos_ned[0],
        gps_pos_ned[1],
        gps_pos_ned[2]);
    return true;
}

bool write_gap3_observation_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,update_type,accepted,reject_reason,"
        "pred_accum_dpos_h_m,pred_accum_dvel_h_mps,pred_accum_dt_s,"
        "innov_n,innov_e,innov_d,innov_h_m,innov_v_lateral,innov_v_vertical,"
        "nis,k_pos_max,k_vel_max,k_att_max,"
        "dx_pos_norm_m,dx_vel_norm_mps,dx_att_norm_rad,"
        "dx_pos_n_m,dx_pos_e_m,dx_vel_n_mps,dx_vel_e_mps,"
        "corr_pos_h_m,corr_vel_h_mps,corr_att_norm_rad,"
        "hypo_corr_pos_h_m,hypo_corr_vel_h_mps,"
        "pred_over_corr_dpos_ratio,pred_over_corr_dvel_ratio,"
        "state_pos_n_m,state_pos_e_m,state_vel_h_mps\n");
    return true;
}

bool write_gap3_observation_audit_row(
    FILE *audit_fp,
    const char *update_type,
    double timestamp_s,
    bool accepted,
    int reject_reason,
    double pred_accum_dpos_h,
    double pred_accum_dvel_h,
    double pred_accum_dt,
    const InsEkfGnssUpdateDetail *gnss_detail,
    const InsEkfNhcUpdateDetail *nhc_detail,
    const float pos_before[3],
    const float pos_after[3],
    const float vel_before[3],
    const float vel_after[3])
{
    if (audit_fp == nullptr || update_type == nullptr || pos_before == nullptr ||
        pos_after == nullptr || vel_before == nullptr || vel_after == nullptr) {
        return false;
    }

    float innov_n = 0.0f;
    float innov_e = 0.0f;
    float innov_d = 0.0f;
    float innov_v_lateral = 0.0f;
    float innov_v_vertical = 0.0f;
    float nis = 0.0f;
    float k_pos_max = 0.0f;
    float k_vel_max = 0.0f;
    float k_att_max = 0.0f;
    float dx_pos_norm = 0.0f;
    float dx_vel_norm = 0.0f;
    float dx_att_norm = 0.0f;
    float dx_pos_n = 0.0f;
    float dx_pos_e = 0.0f;
    float dx_vel_n = 0.0f;
    float dx_vel_e = 0.0f;

    if (gnss_detail != nullptr) {
        innov_n = gnss_detail->innov_n_m;
        innov_e = gnss_detail->innov_e_m;
        innov_d = gnss_detail->innov_d_m;
        nis = gnss_detail->nis;
        k_pos_max = gnss_detail->k_pos_max;
        k_vel_max = gnss_detail->k_vel_max;
        k_att_max = gnss_detail->k_att_max;
        dx_pos_norm = gnss_detail->dx_pos_norm_m;
        dx_vel_norm = gnss_detail->dx_vel_norm_mps;
        dx_att_norm = gnss_detail->dx_att_norm_rad;
        dx_pos_n = gnss_detail->dx_pos_n_m;
        dx_pos_e = gnss_detail->dx_pos_e_m;
        dx_vel_n = gnss_detail->dx_vel_n_mps;
        dx_vel_e = gnss_detail->dx_vel_e_mps;
    } else if (nhc_detail != nullptr) {
        innov_v_lateral = nhc_detail->innov_y_mps;
        innov_v_vertical = nhc_detail->innov_z_mps;
        nis = nhc_detail->nis;
        k_vel_max = nhc_detail->k_max;
        dx_vel_norm = nhc_detail->dx_vel_norm_mps;
        dx_vel_n = nhc_detail->dx_vel_n_mps;
        dx_vel_e = nhc_detail->dx_vel_e_mps;
        dx_att_norm = nhc_detail->dx_att_norm_rad;
    }

    const float innov_h = std::sqrt((innov_n * innov_n) + (innov_e * innov_e));
    const float corr_pos_h = std::sqrt(
        ((pos_after[0] - pos_before[0]) * (pos_after[0] - pos_before[0])) +
        ((pos_after[1] - pos_before[1]) * (pos_after[1] - pos_before[1])));
    const float corr_vel_h = std::sqrt(
        ((vel_after[0] - vel_before[0]) * (vel_after[0] - vel_before[0])) +
        ((vel_after[1] - vel_before[1]) * (vel_after[1] - vel_before[1])));
    const float corr_att_norm = (gnss_detail != nullptr) ? gnss_detail->dx_att_norm_rad : 0.0f;

    float hypo_corr_pos_h = corr_pos_h;
    float hypo_corr_vel_h = corr_vel_h;
    if (gnss_detail != nullptr) {
        hypo_corr_pos_h = std::sqrt(
            (gnss_detail->dx_pos_n_m * gnss_detail->dx_pos_n_m) +
            (gnss_detail->dx_pos_e_m * gnss_detail->dx_pos_e_m));
        hypo_corr_vel_h = std::sqrt(
            (gnss_detail->dx_vel_n_mps * gnss_detail->dx_vel_n_mps) +
            (gnss_detail->dx_vel_e_mps * gnss_detail->dx_vel_e_mps));
    } else if (nhc_detail != nullptr) {
        hypo_corr_vel_h = nhc_detail->dx_vel_norm_mps;
    }

    const double pred_over_corr_dpos =
        (corr_pos_h > 1.0e-6f) ? (pred_accum_dpos_h / static_cast<double>(corr_pos_h)) : -1.0;
    const double pred_over_corr_dvel =
        (corr_vel_h > 1.0e-6f) ? (pred_accum_dvel_h / static_cast<double>(corr_vel_h)) : -1.0;
    const float state_vel_h = std::sqrt(
        (vel_after[0] * vel_after[0]) + (vel_after[1] * vel_after[1]));

    std::fprintf(
        audit_fp,
        "%.9f,%s,%d,%d,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,"
        "%.6f,%.6f,"
        "%.6f,%.6f,%.6f\n",
        timestamp_s,
        update_type,
        accepted ? 1 : 0,
        reject_reason,
        pred_accum_dpos_h,
        pred_accum_dvel_h,
        pred_accum_dt,
        innov_n,
        innov_e,
        innov_d,
        innov_h,
        innov_v_lateral,
        innov_v_vertical,
        nis,
        k_pos_max,
        k_vel_max,
        k_att_max,
        dx_pos_norm,
        dx_vel_norm,
        dx_att_norm,
        dx_pos_n,
        dx_pos_e,
        dx_vel_n,
        dx_vel_e,
        corr_pos_h,
        corr_vel_h,
        corr_att_norm,
        hypo_corr_pos_h,
        hypo_corr_vel_h,
        pred_over_corr_dpos,
        pred_over_corr_dvel,
        pos_after[0],
        pos_after[1],
        state_vel_h);
    return true;
}

bool write_gap3_gnss_nis_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,gps_index,"
        "z_n_m,z_e_m,z_d_m,hx_n_m,hx_e_m,hx_d_m,"
        "innov_n_m,innov_e_m,innov_d_m,innov_h_m,innov_d_abs_m,pred_error_3d_m,"
        "vel_pred_n_mps,vel_pred_e_mps,vel_pred_d_mps,vel_pred_h_mps,"
        "gps_speed_mps,gps_course_deg,has_gps_speed,"
        "pseudo_innov_v_n_mps,pseudo_innov_v_e_mps,pseudo_innov_v_d_mps,pseudo_innov_v_h_mps,"
        "hph_nn,hph_ee,hph_dd,r_m2,"
        "s_nn,s_ee,s_dd,s_ne,"
        "nis_full,nis_horizontal_2d,nis_d_marginal,"
        "nis_contrib_n,nis_contrib_e,nis_contrib_d,"
        "nis_threshold,accepted,reject_reason,"
        "s_eigmin,s_eigmax,s_cond,"
        "k_pos_max,k_vel_max,k_att_max,"
        "dx_pos_n_m,dx_pos_e_m,dx_pos_d_m,dx_vel_n_mps,dx_vel_e_mps,dx_vel_d_mps,"
        "corr_pos_h_m,corr_vel_h_mps,"
        "vel_after_n_mps,vel_after_e_mps,vel_after_h_mps,"
        "dt_since_prev_gnss_s,dt_since_prev_accept_s,"
        "ppv_policy,ppv_triggered,ppv_effective_gap_s,"
        "cos_dv_pos_err_pre,cos_dv_tot_err_pre,"
        "ppv_frob_pre,ppv_frob_post\n");
    return true;
}

bool write_gap3_gnss_nis_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    uint32_t gps_index,
    const float z_ned[3],
    const float hx_ned[3],
    const float vel_pred[3],
    bool has_gps_speed,
    float gps_speed_mps,
    float gps_course_deg,
    const float hph[3][3],
    float r_m2,
    float nis_threshold,
    bool accepted,
    int reject_reason,
    double dt_since_prev_gnss_s,
    double dt_since_prev_accept_s,
    const InsEkfGnssUpdateDetail *gnss_detail,
    const float pos_after[3],
    const float vel_after[3])
{
    if (audit_fp == nullptr || z_ned == nullptr || hx_ned == nullptr || vel_pred == nullptr
        || pos_after == nullptr || vel_after == nullptr) {
        return false;
    }

    const float innov[3] = {
        z_ned[0] - hx_ned[0],
        z_ned[1] - hx_ned[1],
        z_ned[2] - hx_ned[2],
    };
    const float innov_h = std::sqrt((innov[0] * innov[0]) + (innov[1] * innov[1]));
    const float pred_error_3d = std::sqrt(
        (innov[0] * innov[0]) + (innov[1] * innov[1]) + (innov[2] * innov[2]));
    const float vel_pred_h = std::sqrt(
        (vel_pred[0] * vel_pred[0]) + (vel_pred[1] * vel_pred[1]));

    float v_gps_n = 0.0f;
    float v_gps_e = 0.0f;
    if (has_gps_speed && gps_speed_mps > 0.0f) {
        const float course_rad = gps_course_deg * kDegToRadF;
        v_gps_n = gps_speed_mps * std::cos(course_rad);
        v_gps_e = gps_speed_mps * std::sin(course_rad);
    }
    const float pseudo_innov_v[3] = {
        vel_pred[0] - v_gps_n,
        vel_pred[1] - v_gps_e,
        vel_pred[2],
    };
    const float pseudo_innov_v_h = std::sqrt(
        (pseudo_innov_v[0] * pseudo_innov_v[0]) + (pseudo_innov_v[1] * pseudo_innov_v[1]));

    float s_matrix[3][3]{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            s_matrix[i][j] = hph[i][j];
        }
        s_matrix[i][i] += r_m2;
    }

    float s_inv[3][3]{};
    float nis_full = 0.0f;
    float nis_horizontal_2d = 0.0f;
    float nis_d_marginal = 0.0f;
    float nis_contrib[3] = {0.0f, 0.0f, 0.0f};
    if (invert_matrix3(s_matrix, s_inv)) {
        nis_full = quadratic_form3(innov, s_inv);
        nis_component_contributions(innov, s_inv, nis_contrib);
        if (s_matrix[2][2] > 1.0e-12f) {
            nis_d_marginal = (innov[2] * innov[2]) / s_matrix[2][2];
        }

        float s_h[2][2] = {
            {s_matrix[0][0], s_matrix[0][1]},
            {s_matrix[1][0], s_matrix[1][1]},
        };
        float s_h_inv[2][2]{};
        const float y_h[2] = {innov[0], innov[1]};
        if (invert_matrix2(s_h, s_h_inv)) {
            nis_horizontal_2d = (y_h[0] * ((s_h_inv[0][0] * y_h[0]) + (s_h_inv[0][1] * y_h[1])))
                + (y_h[1] * ((s_h_inv[1][0] * y_h[0]) + (s_h_inv[1][1] * y_h[1])));
        }
    }

    float s_eigmin = 0.0f;
    float s_eigmax = 0.0f;
    float s_cond = 0.0f;
    matrix_s_condition(s_matrix, &s_eigmin, &s_eigmax, &s_cond);

    float k_pos_max = 0.0f;
    float k_vel_max = 0.0f;
    float k_att_max = 0.0f;
    float dx_pos[3] = {0.0f, 0.0f, 0.0f};
    float dx_vel[3] = {0.0f, 0.0f, 0.0f};
    float ppv_effective_gap_s = 0.0f;
    float cos_dv_pos_err_pre = 0.0f;
    float cos_dv_tot_err_pre = 0.0f;
    float ppv_frob_pre = 0.0f;
    float ppv_frob_post = 0.0f;
    const char *ppv_policy_name = "none";
    int ppv_triggered = 0;
    if (gnss_detail != nullptr) {
        k_pos_max = gnss_detail->k_pos_max;
        k_vel_max = gnss_detail->k_vel_max;
        k_att_max = gnss_detail->k_att_max;
        dx_pos[0] = gnss_detail->dx_pos_n_m;
        dx_pos[1] = gnss_detail->dx_pos_e_m;
        dx_pos[2] = gnss_detail->dx_pos_d_m;
        dx_vel[0] = gnss_detail->dx_vel_n_mps;
        dx_vel[1] = gnss_detail->dx_vel_e_mps;
        dx_vel[2] = gnss_detail->dx_vel_d_mps;
        ppv_effective_gap_s = gnss_detail->ppv_effective_gap_s;
        cos_dv_pos_err_pre = gnss_detail->cos_dv_pos_err_pre;
        cos_dv_tot_err_pre = gnss_detail->cos_dv_tot_err_pre;
        ppv_frob_pre = gnss_detail->ppv_frob_pre;
        ppv_frob_post = gnss_detail->ppv_frob_post;
        ppv_triggered = static_cast<int>(gnss_detail->ppv_triggered);
        ppv_policy_name = ins_ekf_p_pv_policy_name(
            static_cast<InsEkfPpvPolicy>(gnss_detail->ppv_policy));
        if (ppv_policy_name == nullptr) {
            ppv_policy_name = "unknown";
        }
    }

    const float corr_pos_h = std::sqrt(
        ((pos_after[0] - hx_ned[0]) * (pos_after[0] - hx_ned[0]))
        + ((pos_after[1] - hx_ned[1]) * (pos_after[1] - hx_ned[1])));
    const float corr_vel_h = std::sqrt(
        ((vel_after[0] - vel_pred[0]) * (vel_after[0] - vel_pred[0]))
        + ((vel_after[1] - vel_pred[1]) * (vel_after[1] - vel_pred[1])));
    const float vel_after_h = std::sqrt(
        (vel_after[0] * vel_after[0]) + (vel_after[1] * vel_after[1]));

    std::fprintf(
        audit_fp,
        "%.9f,%u,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%d,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%d,%d,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f",
        timestamp_s,
        gps_index,
        z_ned[0],
        z_ned[1],
        z_ned[2],
        hx_ned[0],
        hx_ned[1],
        hx_ned[2],
        innov[0],
        innov[1],
        innov[2],
        innov_h,
        std::fabs(innov[2]),
        pred_error_3d,
        vel_pred[0],
        vel_pred[1],
        vel_pred[2],
        vel_pred_h,
        gps_speed_mps,
        gps_course_deg,
        has_gps_speed ? 1 : 0,
        pseudo_innov_v[0],
        pseudo_innov_v[1],
        pseudo_innov_v[2],
        pseudo_innov_v_h,
        hph[0][0],
        hph[1][1],
        hph[2][2],
        r_m2,
        s_matrix[0][0],
        s_matrix[1][1],
        s_matrix[2][2],
        s_matrix[0][1],
        nis_full,
        nis_horizontal_2d,
        nis_d_marginal,
        nis_contrib[0],
        nis_contrib[1],
        nis_contrib[2],
        nis_threshold,
        accepted ? 1 : 0,
        reject_reason,
        s_eigmin,
        s_eigmax,
        s_cond,
        k_pos_max,
        k_vel_max,
        k_att_max,
        dx_pos[0],
        dx_pos[1],
        dx_pos[2],
        dx_vel[0],
        dx_vel[1],
        dx_vel[2],
        corr_pos_h,
        corr_vel_h,
        vel_after[0],
        vel_after[1],
        vel_after_h,
        dt_since_prev_gnss_s,
        dt_since_prev_accept_s);
    std::fprintf(
        audit_fp,
        ",%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f",
        ppv_policy_name,
        ppv_triggered,
        ppv_effective_gap_s,
        cos_dv_pos_err_pre,
        cos_dv_tot_err_pre,
        ppv_frob_pre,
        ppv_frob_post);
    std::fputc('\n', audit_fp);
    return true;
}

void write_matrix3_json_value(FILE *fp, const float m[3][3])
{
    if (fp == nullptr) {
        return;
    }

    std::fprintf(fp, "[\n");
    for (int i = 0; i < 3; ++i) {
        std::fprintf(
            fp,
            "      [%.9f, %.9f, %.9f]%s\n",
            m[i][0],
            m[i][1],
            m[i][2],
            (i < 2) ? "," : "");
    }
    std::fprintf(fp, "    ]");
}

bool write_gap3_gnss_k_block_audit_json(
    FILE *json_fp,
    double timestamp_s,
    uint32_t gps_index,
    bool accepted,
    const float z_ned[3],
    const float hx_ned[3],
    const float vel_prior[3],
    const float vel_post[3],
    const float pos_post[3],
    float gps_speed_mps,
    bool has_gps_speed,
    const float hph[3][3],
    float r_m2,
    const float s_matrix[3][3],
    const InsEkfGnssUpdateDetail *gnss_detail,
    const InsEkfGnssKBlockDetail *k_block)
{
    if (json_fp == nullptr || z_ned == nullptr || hx_ned == nullptr || vel_prior == nullptr
        || vel_post == nullptr || pos_post == nullptr || gnss_detail == nullptr
        || k_block == nullptr) {
        return false;
    }

    const float innov[3] = {
        z_ned[0] - hx_ned[0],
        z_ned[1] - hx_ned[1],
        z_ned[2] - hx_ned[2],
    };

    std::fprintf(json_fp, "{\n");
    std::fprintf(json_fp, "  \"experiment\": \"GAP-3 GNSS K-block single-fix audit\",\n");
    std::fprintf(json_fp, "  \"timestamp_s\": %.9f,\n", timestamp_s);
    std::fprintf(json_fp, "  \"gps_index\": %u,\n", gps_index);
    std::fprintf(json_fp, "  \"accepted\": %s,\n", accepted ? "true" : "false");
    std::fprintf(json_fp, "  \"measurement_model\": {\n");
    std::fprintf(json_fp, "    \"z_observed\": \"position_only [pN,pE,pD]\",\n");
    std::fprintf(json_fp, "    \"z_velocity_in_ekf\": false,\n");
    std::fprintf(json_fp, "    \"gps_speed_available_in_log\": %s,\n", has_gps_speed ? "true" : "false");
    std::fprintf(json_fp, "    \"gps_speed_mps\": %.6f,\n", gps_speed_mps);
    std::fprintf(json_fp, "    \"H_on_error_state\": \"implicit I3 on pos errors; zero on vel/att/bias\"\n");
    std::fprintf(json_fp, "  },\n");

    std::fprintf(json_fp, "  \"z_ned_m\": [%.6f, %.6f, %.6f],\n", z_ned[0], z_ned[1], z_ned[2]);
    std::fprintf(json_fp, "  \"hx_ned_m\": [%.6f, %.6f, %.6f],\n", hx_ned[0], hx_ned[1], hx_ned[2]);
    std::fprintf(json_fp, "  \"innovation_ned_m\": [%.6f, %.6f, %.6f],\n", innov[0], innov[1], innov[2]);
    std::fprintf(
        json_fp,
        "  \"x_prior\": {\"pos_ned_m\": [%.6f, %.6f, %.6f], \"vel_ned_mps\": [%.6f, %.6f, %.6f]},\n",
        hx_ned[0],
        hx_ned[1],
        hx_ned[2],
        vel_prior[0],
        vel_prior[1],
        vel_prior[2]);
    std::fprintf(
        json_fp,
        "  \"x_post\": {\"pos_ned_m\": [%.6f, %.6f, %.6f], \"vel_ned_mps\": [%.6f, %.6f, %.6f]},\n",
        pos_post[0],
        pos_post[1],
        pos_post[2],
        vel_post[0],
        vel_post[1],
        vel_post[2]);

    std::fprintf(json_fp, "  \"S_m2\": ");
    write_matrix3_json_value(json_fp, s_matrix);
    std::fprintf(json_fp, ",\n");
    std::fprintf(json_fp, "  \"HPH_m2\": ");
    write_matrix3_json_value(json_fp, hph);
    std::fprintf(json_fp, ",\n");
    std::fprintf(json_fp, "  \"R_m2\": %.6f,\n", r_m2);
    std::fprintf(json_fp, "  \"S_inv\": ");
    write_matrix3_json_value(json_fp, k_block->s_inv);
    std::fprintf(json_fp, ",\n");
    std::fprintf(json_fp, "  \"P_vel_pos_cross_m2\": ");
    write_matrix3_json_value(json_fp, k_block->p_vel_pos);
    std::fprintf(json_fp, ",\n");
    std::fprintf(json_fp, "  \"K_pos_pos\": ");
    write_matrix3_json_value(json_fp, k_block->k_pos_pos);
    std::fprintf(json_fp, ",\n");
    std::fprintf(json_fp, "  \"K_vel_pos\": ");
    write_matrix3_json_value(json_fp, k_block->k_vel_pos);
    std::fprintf(json_fp, ",\n");

    std::fprintf(json_fp, "  \"delta_x\": {\n");
    std::fprintf(
        json_fp,
        "    \"pos_ned_m\": [%.9f, %.9f, %.9f],\n",
        gnss_detail->dx_pos_n_m,
        gnss_detail->dx_pos_e_m,
        gnss_detail->dx_pos_d_m);
    std::fprintf(
        json_fp,
        "    \"vel_ned_mps\": [%.9f, %.9f, %.9f],\n",
        gnss_detail->dx_vel_n_mps,
        gnss_detail->dx_vel_e_mps,
        gnss_detail->dx_vel_d_mps);
    std::fprintf(
        json_fp,
        "    \"att_rad\": [%.9f, %.9f, %.9f],\n",
        gnss_detail->dx_att_x_rad,
        gnss_detail->dx_att_y_rad,
        gnss_detail->dx_att_z_rad);
    std::fprintf(
        json_fp,
        "    \"bias_accel_norm\": %.9f,\n",
        k_block->dx_bias_accel_norm);
    std::fprintf(
        json_fp,
        "    \"bias_gyro_norm\": %.9f,\n",
        k_block->dx_bias_gyro_norm);
    std::fprintf(json_fp, "    \"pos_norm_m\": %.9f,\n", gnss_detail->dx_pos_norm_m);
    std::fprintf(json_fp, "    \"vel_norm_mps\": %.9f,\n", gnss_detail->dx_vel_norm_mps);
    std::fprintf(json_fp, "    \"att_norm_rad\": %.9f\n", gnss_detail->dx_att_norm_rad);
    std::fprintf(json_fp, "  },\n");
    std::fprintf(json_fp, "  \"nis\": %.6f,\n", gnss_detail->nis);
    std::fprintf(json_fp, "  \"interpretation\": \"K_vel_pos = P_vel_pos * S_inv; velocity correction only via cross-covariance, not H\"\n");
    std::fprintf(json_fp, "}\n");
    return true;
}

static float matrix3_frobenius(const float m[3][3])
{
    float sum_sq = 0.0f;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            sum_sq += m[r][c] * m[r][c];
        }
    }
    return std::sqrt(sum_sq);
}

static float matrix3_max_abs(const float m[3][3])
{
    float max_abs = 0.0f;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const float abs_val = std::fabs(m[r][c]);
            if (abs_val > max_abs) {
                max_abs = abs_val;
            }
        }
    }
    return max_abs;
}

bool write_gap3_cov_propagation_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,event,"
        "P_pos_pos_frob,P_vel_vel_frob,P_vel_pos_frob,P_vel_att_frob,"
        "P_pos_std_n,P_pos_std_e,P_pos_std_d,"
        "P_vel_std_n,P_vel_std_e,P_vel_std_d,"
        "P_vel_pos_max,K_vel_pos_max,"
        "F_dt,F_va_frob,F_vba_frob,F_va_max,"
        "vel_h_mps,gps_speed_mps,gnss_accepted\n");
    return true;
}

bool write_gap3_cov_propagation_audit_row(
    FILE *audit_fp,
    const char *event,
    double timestamp_s,
    const InsEkfFilter &ekf,
    float vel_h_mps,
    float gps_speed_mps,
    int gnss_accepted,
    float k_vel_pos_max)
{
    if (audit_fp == nullptr || event == nullptr) {
        return false;
    }

    InsEkfCovBlockMetrics cov_metrics{};
    if (!ins_ekf_get_cov_block_metrics(&ekf, &cov_metrics)) {
        return false;
    }

    InsEkfPredictAudit predict_audit{};
    const bool has_predict_audit = ins_ekf_get_last_predict_audit(&ekf, &predict_audit);
    const float f_dt = has_predict_audit ? predict_audit.f_dp_dv_dt_s : 0.0f;
    const float f_va_frob =
        has_predict_audit ? matrix3_frobenius(predict_audit.f_va) : 0.0f;
    const float f_vba_frob =
        has_predict_audit ? matrix3_frobenius(predict_audit.f_vba) : 0.0f;
    const float f_va_max =
        has_predict_audit ? matrix3_max_abs(predict_audit.f_va) : 0.0f;

    std::fprintf(
        audit_fp,
        "%.9f,%s,"
        "%.9e,%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.9e,%.9e,"
        "%.6f,%.9e,%.9e,%.9e,"
        "%.6f,%.6f,%d\n",
        timestamp_s,
        event,
        cov_metrics.p_pos_pos_frob,
        cov_metrics.p_vel_vel_frob,
        cov_metrics.p_vel_pos_frob,
        cov_metrics.p_vel_att_frob,
        cov_metrics.p_pos_std_n_m,
        cov_metrics.p_pos_std_e_m,
        cov_metrics.p_pos_std_d_m,
        cov_metrics.p_vel_std_n_mps,
        cov_metrics.p_vel_std_e_mps,
        cov_metrics.p_vel_std_d_mps,
        cov_metrics.p_vel_pos_max_abs,
        k_vel_pos_max,
        f_dt,
        f_va_frob,
        f_vba_frob,
        f_va_max,
        vel_h_mps,
        gps_speed_mps,
        gnss_accepted);
    return true;
}

bool write_gap3_vel_source_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,imu_seq,source,dv_n,dv_e,dv_d,dv_norm,"
        "vel_n,vel_e,vel_d,vel_h_mps,gps_speed_mps,"
        "h_nhc_r0_vn,h_nhc_r0_ve,h_nhc_r0_vd,h_nhc_r1_vn,h_nhc_r1_ve,h_nhc_r1_vd\n");
    return true;
}

bool write_gap3_imu_constraint_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    uint64_t imu_seq,
    bool nhc_mode_selected,
    bool zupt_armed,
    bool zupt_applied,
    bool nhc_applied,
    float gps_speed_mps,
    float static_phase_end_s,
    float moving_speed_threshold_mps,
    float accel_norm_mps2,
    float gyro_norm_radps,
    float vel_h_mps,
    float bias_ax,
    float bias_ay,
    float bias_az,
    float a_body_x,
    float a_body_y,
    float a_body_z,
    const char *constraint_policy,
    bool nhc_policy_enabled)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "%.9f,%llu,%d,%d,%d,%d,%.6f,%.3f,%.3f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%s,%d\n",
        timestamp_s,
        static_cast<unsigned long long>(imu_seq),
        nhc_mode_selected ? 1 : 0,
        zupt_armed ? 1 : 0,
        zupt_applied ? 1 : 0,
        nhc_applied ? 1 : 0,
        gps_speed_mps,
        static_phase_end_s,
        moving_speed_threshold_mps,
        accel_norm_mps2,
        gyro_norm_radps,
        vel_h_mps,
        bias_ax,
        bias_ay,
        bias_az,
        a_body_x,
        a_body_y,
        a_body_z,
        constraint_policy != nullptr ? constraint_policy : "",
        nhc_policy_enabled ? 1 : 0);
    return true;
}

bool write_gap3_imu_constraint_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,imu_seq,nhc_mode_selected,zupt_armed,zupt_applied,nhc_applied,"
        "gps_speed_mps,static_phase_end_s,moving_speed_threshold_mps,"
        "accel_norm_mps2,gyro_norm_radps,vel_h_mps,"
        "bias_ax,bias_ay,bias_az,a_body_x,a_body_y,a_body_z,"
        "constraint_policy,nhc_policy_enabled\n");
    return true;
}

bool write_gap3_constraint_pipeline_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,imu_seq,constraint_policy,zupt_armed,nhc_armed,"
        "vel_before_n,vel_before_e,vel_before_d,"
        "dv_pred_n,dv_pred_e,dv_pred_d,"
        "vel_after_pred_n,vel_after_pred_e,vel_after_pred_d,"
        "dv_nhc_n,dv_nhc_e,dv_nhc_d,nhc_applied,"
        "vel_after_nhc_n,vel_after_nhc_e,vel_after_nhc_d,"
        "dv_zupt_n,dv_zupt_e,dv_zupt_d,zupt_applied,"
        "vel_after_zupt_n,vel_after_zupt_e,vel_after_zupt_d,"
        "vel_h_mps,gps_speed_mps\n");
    return true;
}

bool write_gap3_constraint_pipeline_audit_row(
    FILE *audit_fp,
    double timestamp_s,
    uint64_t imu_seq,
    const char *constraint_policy,
    bool zupt_armed,
    bool nhc_armed,
    const float vel_before[3],
    const InsEkfVelPipelineAudit &pipeline,
    float gps_speed_mps)
{
    if (audit_fp == nullptr || vel_before == nullptr || !pipeline.valid) {
        return false;
    }

    const float vel_h = std::hypot(pipeline.vel_after_zupt[0], pipeline.vel_after_zupt[1]);
    std::fprintf(
        audit_fp,
        "%.9f,%llu,%s,%d,%d,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%d,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%d,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f\n",
        timestamp_s,
        static_cast<unsigned long long>(imu_seq),
        constraint_policy != nullptr ? constraint_policy : "",
        zupt_armed ? 1 : 0,
        nhc_armed ? 1 : 0,
        vel_before[0],
        vel_before[1],
        vel_before[2],
        pipeline.dv_predict[0],
        pipeline.dv_predict[1],
        pipeline.dv_predict[2],
        pipeline.vel_after_predict[0],
        pipeline.vel_after_predict[1],
        pipeline.vel_after_predict[2],
        pipeline.dv_nhc[0],
        pipeline.dv_nhc[1],
        pipeline.dv_nhc[2],
        pipeline.nhc_applied ? 1 : 0,
        pipeline.vel_after_nhc[0],
        pipeline.vel_after_nhc[1],
        pipeline.vel_after_nhc[2],
        pipeline.dv_zupt[0],
        pipeline.dv_zupt[1],
        pipeline.dv_zupt[2],
        pipeline.zupt_applied ? 1 : 0,
        pipeline.vel_after_zupt[0],
        pipeline.vel_after_zupt[1],
        pipeline.vel_after_zupt[2],
        vel_h,
        gps_speed_mps);
    return true;
}

bool write_gap3_cov_step_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,imu_seq,update_type,phase,"
        "P_pp_frob,P_vv_frob,P_pv_frob,P_aa_frob,"
        "P_vv_n_m2,P_vv_e_m2,P_vv_d_m2,"
        "P_vv_body_fwd_m2,P_vv_body_lat_m2,P_vv_body_vert_m2,"
        "P_vv_std_n,P_vv_std_e,P_vv_std_d,"
        "vel_h_mps,v_body_x,v_body_y,v_body_z\n");
    return true;
}

bool write_h5_sync_audit_header(FILE *sync_fp)
{
    if (sync_fp == nullptr) {
        return false;
    }

    std::fprintf(
        sync_fp,
        "t_ekf,t_gps_raw,t_imu_last,innov_n,innov_e,v_n,v_e,"
        "ratio_n,ratio_e,dt_predict_update,gps_accepted\n");
    return true;
}

bool write_h5_sync_audit_row(
    FILE *sync_fp,
    double t_ekf_s,
    double t_gps_raw_s,
    double t_imu_last_s,
    float innov_n,
    float innov_e,
    float v_n,
    float v_e,
    bool gps_accepted)
{
    if (sync_fp == nullptr) {
        return false;
    }

    const float ratio_n = (std::fabs(v_n) > 0.5f) ? (innov_n / v_n) : 0.0f;
    const float ratio_e = (std::fabs(v_e) > 0.5f) ? (innov_e / v_e) : 0.0f;
    const double dt_predict_update_s = t_gps_raw_s - t_imu_last_s;

    std::fprintf(
        sync_fp,
        "%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.9f,%d\n",
        t_ekf_s,
        t_gps_raw_s,
        t_imu_last_s,
        innov_n,
        innov_e,
        v_n,
        v_e,
        ratio_n,
        ratio_e,
        dt_predict_update_s,
        gps_accepted ? 1 : 0);
    return true;
}

bool write_gnss_audit_header(FILE *audit_fp)
{
    if (audit_fp == nullptr) {
        return false;
    }

    std::fprintf(
        audit_fp,
        "timestamp_s,imu_timestamp_s,gps_timestamp_s,latency_imu_gps_s,"
        "ekf_pos_n_m,ekf_pos_e_m,ekf_pos_d_m,"
        "gps_pos_n_m,gps_pos_e_m,gps_pos_d_m,"
        "innovation_n_m,innovation_e_m,innovation_d_m,"
        "S_nn,S_ee,S_dd,S_ne,S_nd,S_ed,"
        "HPH_nn,HPH_ee,HPH_dd,HPH_ne,HPH_nd,HPH_ed,"
        "R_m2,nis,"
        "mahalanobis_n,mahalanobis_e,mahalanobis_d,"
        "gnss_accepted,gnss_rejected_total\n");
    return true;
}

bool write_gnss_audit_row(
    FILE *audit_fp,
    double gps_timestamp_s,
    double imu_timestamp_s,
    const float ekf_pos_before[3],
    const double gps_pos[3],
    const float innovation_ned[3],
    const float s_matrix[3][3],
    const float hph_matrix[3][3],
    float r_m2,
    float nis,
    bool gnss_accepted,
    uint32_t gnss_rejected_total)
{
    if (audit_fp == nullptr) {
        return false;
    }

    const double latency_s = gps_timestamp_s - imu_timestamp_s;

    float mahal_n = 0.0f;
    float mahal_e = 0.0f;
    float mahal_d = 0.0f;
    if (s_matrix[0][0] > 1.0e-9f) {
        mahal_n = innovation_ned[0] / std::sqrt(s_matrix[0][0]);
    }
    if (s_matrix[1][1] > 1.0e-9f) {
        mahal_e = innovation_ned[1] / std::sqrt(s_matrix[1][1]);
    }
    if (s_matrix[2][2] > 1.0e-9f) {
        mahal_d = innovation_ned[2] / std::sqrt(s_matrix[2][2]);
    }

    std::fprintf(
        audit_fp,
        "%.9f,%.9f,%.9f,%.9f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,"
        "%.6f,%.6f,"
        "%.6f,%.6f,%.6f,"
        "%d,%u\n",
        gps_timestamp_s,
        imu_timestamp_s,
        gps_timestamp_s,
        latency_s,
        ekf_pos_before[0],
        ekf_pos_before[1],
        ekf_pos_before[2],
        gps_pos[0],
        gps_pos[1],
        gps_pos[2],
        innovation_ned[0],
        innovation_ned[1],
        innovation_ned[2],
        s_matrix[0][0],
        s_matrix[1][1],
        s_matrix[2][2],
        s_matrix[0][1],
        s_matrix[0][2],
        s_matrix[1][2],
        hph_matrix[0][0],
        hph_matrix[1][1],
        hph_matrix[2][2],
        hph_matrix[0][1],
        hph_matrix[0][2],
        hph_matrix[1][2],
        r_m2,
        nis,
        mahal_n,
        mahal_e,
        mahal_d,
        gnss_accepted ? 1 : 0,
        gnss_rejected_total);
    return true;
}

bool parse_json_rotation_matrix(const char *path, float out_matrix[3][3])
{
    if (path == nullptr || out_matrix == nullptr) {
        return false;
    }

    FILE *fp = std::fopen(path, "r");
    if (fp == nullptr) {
        return false;
    }

    char buffer[8192];
    const size_t read_bytes = std::fread(buffer, 1U, sizeof(buffer) - 1U, fp);
    std::fclose(fp);
    if (read_bytes == 0U) {
        return false;
    }
    buffer[read_bytes] = '\0';

    const char *marker = std::strstr(buffer, "rotation_matrix");
    if (marker == nullptr) {
        return false;
    }

    const char *cursor = marker;
    int parsed = 0;
    while (*cursor != '\0' && parsed < 9) {
        if ((*cursor >= '0' && *cursor <= '9') || *cursor == '-' || *cursor == '+') {
            char *end = nullptr;
            const double value = std::strtod(cursor, &end);
            if (end == cursor || !std::isfinite(value)) {
                return false;
            }
            out_matrix[parsed / 3][parsed % 3] = static_cast<float>(value);
            ++parsed;
            cursor = end;
            continue;
        }
        ++cursor;
    }

    return parsed == 9;
}

} /* namespace */

const char *replay_constraint_policy_name(ReplayConstraintPolicy policy)
{
    switch (policy) {
    case ReplayConstraintPolicy::AUTO:
        return "auto";
    case ReplayConstraintPolicy::FORCED_TIME:
        return "forced_time";
    case ReplayConstraintPolicy::GPS_STOP:
        return "gps_stop";
    case ReplayConstraintPolicy::IMU_STATIONARY:
        return "imu_stationary";
    case ReplayConstraintPolicy::DISABLED:
        return "disabled";
    default:
        return "unknown";
    }
}

bool replay_parse_constraint_policy(const char *text, ReplayConstraintPolicy *out_policy)
{
    if (text == nullptr || out_policy == nullptr || text[0] == '\0') {
        return false;
    }
    if (std::strcmp(text, "auto") == 0 || std::strcmp(text, "AUTO") == 0) {
        *out_policy = ReplayConstraintPolicy::AUTO;
        return true;
    }
    if (std::strcmp(text, "forced_time") == 0 || std::strcmp(text, "FORCED_TIME") == 0) {
        *out_policy = ReplayConstraintPolicy::FORCED_TIME;
        return true;
    }
    if (std::strcmp(text, "gps_stop") == 0 || std::strcmp(text, "GPS_STOP") == 0) {
        *out_policy = ReplayConstraintPolicy::GPS_STOP;
        return true;
    }
    if (std::strcmp(text, "imu_stationary") == 0 || std::strcmp(text, "IMU_STATIONARY") == 0) {
        *out_policy = ReplayConstraintPolicy::IMU_STATIONARY;
        return true;
    }
    if (std::strcmp(text, "disabled") == 0 || std::strcmp(text, "DISABLED") == 0) {
        *out_policy = ReplayConstraintPolicy::DISABLED;
        return true;
    }
    return false;
}

bool replay_parse_nhc_policy(const char *text, ReplayNhcPolicy *out_policy)
{
    if (text == nullptr || out_policy == nullptr || text[0] == '\0') {
        return false;
    }
    if (std::strcmp(text, "enabled") == 0 || std::strcmp(text, "on") == 0
        || std::strcmp(text, "ENABLED") == 0 || std::strcmp(text, "ON") == 0) {
        *out_policy = ReplayNhcPolicy::ENABLED;
        return true;
    }
    if (std::strcmp(text, "disabled") == 0 || std::strcmp(text, "off") == 0
        || std::strcmp(text, "DISABLED") == 0 || std::strcmp(text, "OFF") == 0) {
        *out_policy = ReplayNhcPolicy::DISABLED;
        return true;
    }
    return false;
}

const char *replay_gnss_obs_mode_name(ReplayGnssObsMode mode)
{
    switch (mode) {
    case ReplayGnssObsMode::POS:
        return "pos";
    case ReplayGnssObsMode::POS_VEL:
        return "pos_vel";
    case ReplayGnssObsMode::VEL_ONLY:
        return "vel_only";
    default:
        return "unknown";
    }
}

bool replay_parse_gnss_obs_mode(const char *text, ReplayGnssObsMode *out_mode)
{
    if (text == nullptr || out_mode == nullptr || text[0] == '\0') {
        return false;
    }
    if (std::strcmp(text, "pos") == 0 || std::strcmp(text, "POS") == 0) {
        *out_mode = ReplayGnssObsMode::POS;
        return true;
    }
    if (std::strcmp(text, "pos_vel") == 0 || std::strcmp(text, "pos+vel") == 0
        || std::strcmp(text, "POS_VEL") == 0) {
        *out_mode = ReplayGnssObsMode::POS_VEL;
        return true;
    }
    if (std::strcmp(text, "vel_only") == 0 || std::strcmp(text, "vel-only") == 0
        || std::strcmp(text, "VEL_ONLY") == 0) {
        *out_mode = ReplayGnssObsMode::VEL_ONLY;
        return true;
    }
    return false;
}

const char *replay_p_pv_policy_name(ReplayPpvPolicy policy)
{
    switch (policy) {
    case ReplayPpvPolicy::NONE:
        return "none";
    case ReplayPpvPolicy::GAP_LE_1S:
        return "gap_le_1s";
    case ReplayPpvPolicy::ZERO:
        return "zero";
    case ReplayPpvPolicy::COS_POS:
        return "cos_pos";
    case ReplayPpvPolicy::COS_TOT:
        return "cos_tot";
    default:
        return "unknown";
    }
}

bool replay_parse_p_pv_policy(const char *text, ReplayPpvPolicy *out_policy)
{
    if (text == nullptr || out_policy == nullptr || text[0] == '\0') {
        return false;
    }
    InsEkfPpvPolicy ekf_policy = INS_EKF_PPV_POLICY_NONE;
    if (!ins_ekf_parse_p_pv_policy(text, &ekf_policy)) {
        return false;
    }
    switch (ekf_policy) {
    case INS_EKF_PPV_POLICY_GAP_LE_1S:
        *out_policy = ReplayPpvPolicy::GAP_LE_1S;
        return true;
    case INS_EKF_PPV_POLICY_ZERO:
        *out_policy = ReplayPpvPolicy::ZERO;
        return true;
    case INS_EKF_PPV_POLICY_COS_POS:
        *out_policy = ReplayPpvPolicy::COS_POS;
        return true;
    case INS_EKF_PPV_POLICY_COS_TOT:
        *out_policy = ReplayPpvPolicy::COS_TOT;
        return true;
    default:
        *out_policy = ReplayPpvPolicy::NONE;
        return true;
    }
}

ReplayConstraintDecision replay_evaluate_constraints(
    double timestamp_s,
    float last_gps_speed_mps,
    float accel_norm_mps2,
    float gyro_norm_radps,
    const RealRunReplayConfig &config)
{
    ReplayConstraintDecision decision{};
    decision.policy = config.constraint_policy;
    decision.nhc_armed = config.nhc_policy == ReplayNhcPolicy::ENABLED;

    if (config.predict_only_mode) {
        return decision;
    }

    ReplayConstraintPolicy effective_policy = config.constraint_policy;
    if (effective_policy == ReplayConstraintPolicy::AUTO) {
        effective_policy = ReplayConstraintPolicy::FORCED_TIME;
    }

    switch (effective_policy) {
    case ReplayConstraintPolicy::FORCED_TIME:
        decision.zupt_armed =
            (timestamp_s <= static_cast<double>(config.static_phase_end_s))
            || (last_gps_speed_mps <= config.moving_speed_threshold_mps);
        break;
    case ReplayConstraintPolicy::GPS_STOP:
        decision.zupt_armed = last_gps_speed_mps <= config.moving_speed_threshold_mps;
        break;
    case ReplayConstraintPolicy::IMU_STATIONARY: {
        const float gravity_mps2 =
            config.gravity_mps2 > 0.0f ? config.gravity_mps2 : 9.80665f;
        const float accel_dev = std::fabs(accel_norm_mps2 - gravity_mps2);
        decision.zupt_armed =
            (accel_dev <= config.imu_stationary_accel_dev_mps2)
            && (gyro_norm_radps <= config.imu_stationary_gyro_radps);
        break;
    }
    case ReplayConstraintPolicy::DISABLED:
        decision.zupt_armed = false;
        break;
    default:
        decision.zupt_armed = false;
        break;
    }

    return decision;
}

void apply_replay_constraints(
    INaviFilter *filter,
    double timestamp_s,
    float last_gps_speed_mps,
    float accel_norm_mps2,
    float gyro_norm_radps,
    const RealRunReplayConfig &config,
    ReplayConstraintDecision *out_decision)
{
    if (filter == nullptr || config.predict_only_mode) {
        if (out_decision != nullptr) {
            *out_decision = ReplayConstraintDecision{};
        }
        return;
    }

    const ReplayConstraintDecision decision = replay_evaluate_constraints(
        timestamp_s,
        last_gps_speed_mps,
        accel_norm_mps2,
        gyro_norm_radps,
        config);

    if (out_decision != nullptr) {
        *out_decision = decision;
    }

    if (decision.zupt_armed) {
        filter->apply_constraints(
            true,
            config.zupt_lateral_std_mps,
            config.zupt_vertical_std_mps);
        return;
    }

    if (decision.nhc_armed) {
        filter->apply_constraints(
            false,
            config.nhc_lateral_std_mps,
            config.nhc_vertical_std_mps);
        return;
    }

    filter->apply_constraints(false, 0.0f, 0.0f);
}

bool real_run_replay_load_mount_matrix(
    RealRunMountMode mode,
    const char *calibration_path,
    float out_matrix[3][3],
    char *out_label,
    size_t out_label_bytes)
{
    if (out_matrix == nullptr) {
        return false;
    }

    if (out_label != nullptr && out_label_bytes > 0U) {
        out_label[0] = '\0';
    }

    switch (mode) {
    case RealRunMountMode::NONE:
        set_identity_matrix(out_matrix);
        if (out_label != nullptr && out_label_bytes > 0U) {
            std::snprintf(out_label, out_label_bytes, "none");
        }
        return true;
    case RealRunMountMode::LEGACY_EULER_H0: {
        float euler_matrix[3][3]{};
        build_euler321_rotation_matrix(
            euler_matrix,
            kLegacyMountRollRad,
            kLegacyMountPitchRad,
            kLegacyMountYawRad);
        mat3_transpose(euler_matrix, out_matrix);
        if (out_label != nullptr && out_label_bytes > 0U) {
            std::snprintf(
                out_label,
                out_label_bytes,
                "legacy_euler_h0_RPY=(%.2f,%.2f,%.2f)deg^T",
                kLegacyMountRollRad * kRadToDegF,
                kLegacyMountPitchRad * kRadToDegF,
                kLegacyMountYawRad * kRadToDegF);
        }
        return true;
    }
    case RealRunMountMode::CALIBRATION_FILE:
        if (calibration_path == nullptr || calibration_path[0] == '\0') {
            return false;
        }
        if (!parse_json_rotation_matrix(calibration_path, out_matrix)) {
            return false;
        }
        if (out_label != nullptr && out_label_bytes > 0U) {
            std::snprintf(out_label, out_label_bytes, "calibration:%s", calibration_path);
        }
        return true;
    default:
        return false;
    }
}

bool real_run_replay_execute(const RealRunReplayConfig &config, RealRunReplayResult *out_result)
{
    RealRunReplayResult result{};

    if (config.input_csv_path == nullptr || config.output_csv_path == nullptr) {
        std::printf("REAL_RUN_REPLAY: rutas de entrada/salida no configuradas\n");
        return false;
    }

    FILE *input_fp = std::fopen(config.input_csv_path, "r");
    if (input_fp == nullptr) {
        std::printf("REAL_RUN_REPLAY: no se pudo abrir entrada: %s\n", config.input_csv_path);
        return false;
    }

    FILE *output_fp = std::fopen(config.output_csv_path, "w");
    if (output_fp == nullptr) {
        std::fclose(input_fp);
        std::printf("REAL_RUN_REPLAY: no se pudo abrir salida: %s\n", config.output_csv_path);
        return false;
    }

    FILE *instrumentation_fp = nullptr;
    if (config.instrumentation_csv_path != nullptr && config.instrumentation_csv_path[0] != '\0') {
        instrumentation_fp = std::fopen(config.instrumentation_csv_path, "w");
        if (instrumentation_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir instrumentacion: %s\n",
                config.instrumentation_csv_path);
            return false;
        }
        if (!write_instrumentation_header(instrumentation_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            std::fclose(instrumentation_fp);
            return false;
        }
    }

    FILE *gnss_audit_fp = nullptr;
    if (config.gnss_audit_csv_path != nullptr && config.gnss_audit_csv_path[0] != '\0') {
        gnss_audit_fp = std::fopen(config.gnss_audit_csv_path, "w");
        if (gnss_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir auditoria GNSS: %s\n",
                config.gnss_audit_csv_path);
            return false;
        }
        if (!write_gnss_audit_header(gnss_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            std::fclose(gnss_audit_fp);
            return false;
        }
    }

    FILE *h3_diagnostics_fp = nullptr;
    if (config.h3_diagnostics_csv_path != nullptr && config.h3_diagnostics_csv_path[0] != '\0') {
        h3_diagnostics_fp = std::fopen(config.h3_diagnostics_csv_path, "w");
        if (h3_diagnostics_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H3 diagnostics: %s\n",
                config.h3_diagnostics_csv_path);
            return false;
        }
        if (!write_h3_diagnostics_header(h3_diagnostics_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            std::fclose(h3_diagnostics_fp);
            return false;
        }
    }

    FILE *consistency_fp = nullptr;
    if (config.consistency_csv_path != nullptr && config.consistency_csv_path[0] != '\0') {
        consistency_fp = std::fopen(config.consistency_csv_path, "w");
        if (consistency_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir consistency CSV: %s\n",
                config.consistency_csv_path);
            return false;
        }
        if (!write_consistency_header(consistency_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            std::fclose(consistency_fp);
            return false;
        }
    }

    FILE *sync_audit_fp = nullptr;
    if (config.sync_audit_csv_path != nullptr && config.sync_audit_csv_path[0] != '\0') {
        sync_audit_fp = std::fopen(config.sync_audit_csv_path, "w");
        if (sync_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir sync audit CSV: %s\n",
                config.sync_audit_csv_path);
            return false;
        }
        if (!write_h5_sync_audit_header(sync_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            std::fclose(sync_audit_fp);
            return false;
        }
    }

    FILE *h7_update_audit_fp = nullptr;
    if (config.h7_update_audit_csv_path != nullptr && config.h7_update_audit_csv_path[0] != '\0') {
        h7_update_audit_fp = std::fopen(config.h7_update_audit_csv_path, "w");
        if (h7_update_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H7 update audit CSV: %s\n",
                config.h7_update_audit_csv_path);
            return false;
        }
        if (!write_h7_update_audit_header(h7_update_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            std::fclose(h7_update_audit_fp);
            return false;
        }
    }

    FILE *h8_propagation_audit_fp = nullptr;
    if (config.h8_propagation_audit_csv_path != nullptr
        && config.h8_propagation_audit_csv_path[0] != '\0') {
        h8_propagation_audit_fp = std::fopen(config.h8_propagation_audit_csv_path, "w");
        if (h8_propagation_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H8 propagation audit CSV: %s\n",
                config.h8_propagation_audit_csv_path);
            return false;
        }
        if (!write_h8_propagation_audit_header(h8_propagation_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            std::fclose(h8_propagation_audit_fp);
            return false;
        }
    }

    FILE *h9_tilt_audit_fp = nullptr;
    if (config.h9_tilt_audit_csv_path != nullptr
        && config.h9_tilt_audit_csv_path[0] != '\0') {
        h9_tilt_audit_fp = std::fopen(config.h9_tilt_audit_csv_path, "w");
        if (h9_tilt_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H9 tilt audit CSV: %s\n",
                config.h9_tilt_audit_csv_path);
            return false;
        }
        if (!write_h9_tilt_audit_header(h9_tilt_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            std::fclose(h9_tilt_audit_fp);
            return false;
        }
    }

    FILE *h9a_gravity_alignment_audit_fp = nullptr;
    if (config.h9a_gravity_alignment_audit_csv_path != nullptr
        && config.h9a_gravity_alignment_audit_csv_path[0] != '\0') {
        h9a_gravity_alignment_audit_fp = std::fopen(config.h9a_gravity_alignment_audit_csv_path, "w");
        if (h9a_gravity_alignment_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H9a gravity alignment audit CSV: %s\n",
                config.h9a_gravity_alignment_audit_csv_path);
            return false;
        }
        if (!write_h9a_gravity_alignment_audit_header(h9a_gravity_alignment_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            std::fclose(h9a_gravity_alignment_audit_fp);
            return false;
        }
    }

    FILE *h9b_attitude_propagation_audit_fp = nullptr;
    if (config.h9b_attitude_propagation_audit_csv_path != nullptr
        && config.h9b_attitude_propagation_audit_csv_path[0] != '\0') {
        h9b_attitude_propagation_audit_fp =
            std::fopen(config.h9b_attitude_propagation_audit_csv_path, "w");
        if (h9b_attitude_propagation_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            if (h9a_gravity_alignment_audit_fp != nullptr) {
                std::fclose(h9a_gravity_alignment_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H9b attitude propagation audit CSV: %s\n",
                config.h9b_attitude_propagation_audit_csv_path);
            return false;
        }
        if (!write_h9b_attitude_propagation_audit_header(h9b_attitude_propagation_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            if (h9a_gravity_alignment_audit_fp != nullptr) {
                std::fclose(h9a_gravity_alignment_audit_fp);
            }
            std::fclose(h9b_attitude_propagation_audit_fp);
            return false;
        }
    }

    FILE *h9d_gravity_subtraction_audit_fp = nullptr;
    if (config.h9d_gravity_subtraction_audit_csv_path != nullptr
        && config.h9d_gravity_subtraction_audit_csv_path[0] != '\0') {
        h9d_gravity_subtraction_audit_fp =
            std::fopen(config.h9d_gravity_subtraction_audit_csv_path, "w");
        if (h9d_gravity_subtraction_audit_fp == nullptr) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            if (h9a_gravity_alignment_audit_fp != nullptr) {
                std::fclose(h9a_gravity_alignment_audit_fp);
            }
            if (h9b_attitude_propagation_audit_fp != nullptr) {
                std::fclose(h9b_attitude_propagation_audit_fp);
            }
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir H9d gravity subtraction audit CSV: %s\n",
                config.h9d_gravity_subtraction_audit_csv_path);
            return false;
        }
        if (!write_h9d_gravity_subtraction_audit_header(h9d_gravity_subtraction_audit_fp)) {
            std::fclose(input_fp);
            std::fclose(output_fp);
            if (instrumentation_fp != nullptr) {
                std::fclose(instrumentation_fp);
            }
            if (gnss_audit_fp != nullptr) {
                std::fclose(gnss_audit_fp);
            }
            if (h3_diagnostics_fp != nullptr) {
                std::fclose(h3_diagnostics_fp);
            }
            if (consistency_fp != nullptr) {
                std::fclose(consistency_fp);
            }
            if (sync_audit_fp != nullptr) {
                std::fclose(sync_audit_fp);
            }
            if (h7_update_audit_fp != nullptr) {
                std::fclose(h7_update_audit_fp);
            }
            if (h8_propagation_audit_fp != nullptr) {
                std::fclose(h8_propagation_audit_fp);
            }
            if (h9_tilt_audit_fp != nullptr) {
                std::fclose(h9_tilt_audit_fp);
            }
            if (h9a_gravity_alignment_audit_fp != nullptr) {
                std::fclose(h9a_gravity_alignment_audit_fp);
            }
            if (h9b_attitude_propagation_audit_fp != nullptr) {
                std::fclose(h9b_attitude_propagation_audit_fp);
            }
            std::fclose(h9d_gravity_subtraction_audit_fp);
            return false;
        }
    }

    FILE *propagation_chain_audit_fp = nullptr;
    if (config.propagation_chain_audit_csv_path != nullptr
        && config.propagation_chain_audit_csv_path[0] != '\0') {
        propagation_chain_audit_fp = std::fopen(config.propagation_chain_audit_csv_path, "w");
        if (propagation_chain_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir propagation chain audit CSV: %s\n",
                config.propagation_chain_audit_csv_path);
            return false;
        }
        if (!write_propagation_chain_audit_header(propagation_chain_audit_fp)) {
            std::fclose(propagation_chain_audit_fp);
            return false;
        }
    }

    FILE *gap3_observation_audit_fp = nullptr;
    if (config.gap3_observation_audit_csv_path != nullptr
        && config.gap3_observation_audit_csv_path[0] != '\0') {
        gap3_observation_audit_fp = std::fopen(config.gap3_observation_audit_csv_path, "w");
        if (gap3_observation_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 observation audit CSV: %s\n",
                config.gap3_observation_audit_csv_path);
            return false;
        }
        if (!write_gap3_observation_audit_header(gap3_observation_audit_fp)) {
            std::fclose(gap3_observation_audit_fp);
            return false;
        }
    }

    FILE *gap3_gnss_nis_audit_fp = nullptr;
    if (config.gap3_gnss_nis_audit_csv_path != nullptr
        && config.gap3_gnss_nis_audit_csv_path[0] != '\0') {
        gap3_gnss_nis_audit_fp = std::fopen(config.gap3_gnss_nis_audit_csv_path, "w");
        if (gap3_gnss_nis_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 GNSS NIS audit CSV: %s\n",
                config.gap3_gnss_nis_audit_csv_path);
            return false;
        }
        if (!write_gap3_gnss_nis_audit_header(gap3_gnss_nis_audit_fp)) {
            std::fclose(gap3_gnss_nis_audit_fp);
            return false;
        }
    }

    FILE *gap3_nhc_block_audit_fp = nullptr;
    if (config.gap3_nhc_block_audit_csv_path != nullptr
        && config.gap3_nhc_block_audit_csv_path[0] != '\0') {
        gap3_nhc_block_audit_fp = std::fopen(config.gap3_nhc_block_audit_csv_path, "w");
        if (gap3_nhc_block_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 NHC block audit CSV: %s\n",
                config.gap3_nhc_block_audit_csv_path);
            return false;
        }
        if (!ins_ekf_write_nhc_block_audit_header(gap3_nhc_block_audit_fp)) {
            std::fclose(gap3_nhc_block_audit_fp);
            return false;
        }
    }

    FILE *gap3_gnss_k_block_json_fp = nullptr;
    if (config.gap3_gnss_k_block_audit_json_path != nullptr
        && config.gap3_gnss_k_block_audit_json_path[0] != '\0') {
        gap3_gnss_k_block_json_fp = std::fopen(config.gap3_gnss_k_block_audit_json_path, "w");
        if (gap3_gnss_k_block_json_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 GNSS K-block audit JSON: %s\n",
                config.gap3_gnss_k_block_audit_json_path);
            return false;
        }
    }

    FILE *gap3_cov_propagation_audit_fp = nullptr;
    if (config.gap3_cov_propagation_audit_csv_path != nullptr
        && config.gap3_cov_propagation_audit_csv_path[0] != '\0') {
        gap3_cov_propagation_audit_fp = std::fopen(config.gap3_cov_propagation_audit_csv_path, "w");
        if (gap3_cov_propagation_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 cov propagation audit CSV: %s\n",
                config.gap3_cov_propagation_audit_csv_path);
            return false;
        }
        if (!write_gap3_cov_propagation_audit_header(gap3_cov_propagation_audit_fp)) {
            std::fclose(gap3_cov_propagation_audit_fp);
            return false;
        }
    }

    FILE *gap3_cov_step_audit_fp = nullptr;
    if (config.gap3_cov_step_audit_csv_path != nullptr
        && config.gap3_cov_step_audit_csv_path[0] != '\0') {
        gap3_cov_step_audit_fp = std::fopen(config.gap3_cov_step_audit_csv_path, "w");
        if (gap3_cov_step_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 cov step audit CSV: %s\n",
                config.gap3_cov_step_audit_csv_path);
            return false;
        }
        if (!write_gap3_cov_step_audit_header(gap3_cov_step_audit_fp)) {
            std::fclose(gap3_cov_step_audit_fp);
            return false;
        }
    }

    FILE *gap3_vel_source_audit_fp = nullptr;
    if (config.gap3_vel_source_audit_csv_path != nullptr
        && config.gap3_vel_source_audit_csv_path[0] != '\0') {
        gap3_vel_source_audit_fp = std::fopen(config.gap3_vel_source_audit_csv_path, "w");
        if (gap3_vel_source_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 vel source audit CSV: %s\n",
                config.gap3_vel_source_audit_csv_path);
            return false;
        }
        if (!write_gap3_vel_source_audit_header(gap3_vel_source_audit_fp)) {
            std::fclose(gap3_vel_source_audit_fp);
            return false;
        }
    }

    FILE *gap3_imu_constraint_audit_fp = nullptr;
    if (config.gap3_imu_constraint_audit_csv_path != nullptr
        && config.gap3_imu_constraint_audit_csv_path[0] != '\0') {
        gap3_imu_constraint_audit_fp = std::fopen(config.gap3_imu_constraint_audit_csv_path, "w");
        if (gap3_imu_constraint_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 IMU constraint audit CSV: %s\n",
                config.gap3_imu_constraint_audit_csv_path);
            return false;
        }
        if (!write_gap3_imu_constraint_audit_header(gap3_imu_constraint_audit_fp)) {
            std::fclose(gap3_imu_constraint_audit_fp);
            return false;
        }
    }

    FILE *gap3_constraint_pipeline_audit_fp = nullptr;
    if (config.gap3_constraint_pipeline_audit_csv_path != nullptr
        && config.gap3_constraint_pipeline_audit_csv_path[0] != '\0') {
        gap3_constraint_pipeline_audit_fp =
            std::fopen(config.gap3_constraint_pipeline_audit_csv_path, "w");
        if (gap3_constraint_pipeline_audit_fp == nullptr) {
            std::printf(
                "REAL_RUN_REPLAY: no se pudo abrir GAP-3 constraint pipeline audit CSV: %s\n",
                config.gap3_constraint_pipeline_audit_csv_path);
            return false;
        }
        if (!write_gap3_constraint_pipeline_audit_header(gap3_constraint_pipeline_audit_fp)) {
            std::fclose(gap3_constraint_pipeline_audit_fp);
            return false;
        }
    }

    char header_line[kMaxCsvLineBytes];
    if (std::fgets(header_line, static_cast<int>(sizeof(header_line)), input_fp) == nullptr) {
        std::fclose(input_fp);
        std::fclose(output_fp);
        std::printf("REAL_RUN_REPLAY: CSV de entrada vacio\n");
        return false;
    }

    if (!validate_replay_header(header_line)) {
        std::fclose(input_fp);
        std::fclose(output_fp);
        std::printf("REAL_RUN_REPLAY: cabecera CSV no reconocida\n");
        return false;
    }

    if (!write_output_header(output_fp)) {
        std::fclose(input_fp);
        std::fclose(output_fp);
        return false;
    }

    GeodesyDatumValidation geodesy_validation{};
    bool geodesy_validation_printed = false;
    (void)prepare_geodesy_datum_validation(
        config.input_csv_path,
        "data/real_run/Location.csv",
        input_fp,
        &geodesy_validation);

    std::unique_ptr<INaviFilter> filter = create_default_navi_filter();
    InsEkf15State *filter_impl = dynamic_cast<InsEkf15State *>(filter.get());
    if (filter == nullptr || filter_impl == nullptr) {
        std::fclose(input_fp);
        std::fclose(output_fp);
        std::printf("REAL_RUN_REPLAY: no se pudo instanciar INaviFilter\n");
        return false;
    }

    bool filter_initialized = false;
    double previous_timestamp_s = -1.0;
    double last_imu_timestamp_s = 0.0;
    bool has_last_imu_timestamp = false;
    double last_predict_timestamp_s = 0.0;
    bool has_last_predict_timestamp = false;
    float last_gps_speed_mps = 0.0f;
    float last_imu_accel_norm_mps2 = 0.0f;
    float last_imu_gyro_norm_radps = 0.0f;
    double last_gps_pos_ned[3] = {0.0, 0.0, 0.0};
    bool has_last_gps_pos = false;
    double heading_ref_pos_ned[3] = {0.0, 0.0, 0.0};
    bool has_heading_ref_pos = false;
    bool yaw_init_applied = false;
    bool h3_applied = false;
    double h3_grace_period_end_s = -1.0;
    float latest_yaw_gnss_deg = 0.0f;
    bool has_latest_yaw_gnss = false;
    uint32_t gps_update_index = 0U;
    GravityTiltInitAccumulator gravity_tilt_init{};
    bool h9b_has_prev_gravity_rv = false;
    float h9b_prev_gravity_rv_rad[3] = {0.0f, 0.0f, 0.0f};
    float gap3_gnss_anchor_pos[3] = {0.0f, 0.0f, 0.0f};
    float gap3_gnss_anchor_vel[3] = {0.0f, 0.0f, 0.0f};
    bool gap3_gnss_has_anchor = false;
    double gap3_gnss_last_timestamp_s = 0.0;
    float gap3_nhc_anchor_pos[3] = {0.0f, 0.0f, 0.0f};
    float gap3_nhc_anchor_vel[3] = {0.0f, 0.0f, 0.0f};
    bool gap3_nhc_has_anchor = false;
    double gap3_nhc_last_timestamp_s = 0.0;
    float gap3_zupt_anchor_pos[3] = {0.0f, 0.0f, 0.0f};
    float gap3_zupt_anchor_vel[3] = {0.0f, 0.0f, 0.0f};
    bool gap3_zupt_has_anchor = false;
    double gap3_zupt_last_timestamp_s = 0.0;
    double gap3_last_gnss_timestamp_s = -1.0;
    double gap3_last_gnss_accept_timestamp_s = -1.0;
    double gap3_prev_gps_pos_ned[3] = {0.0, 0.0, 0.0};
    bool gap3_has_prev_gps_pos = false;
    double gap3_cov_last_predict_log_s = -1.0;
    uint64_t gap3_cov_step_imu_seq = 0U;

    YawHeadingWindow yaw_window{};
    uint32_t yaw_window_capacity = config.yaw_init_min_samples;
    if (yaw_window_capacity == 0U) {
        yaw_window_capacity = kDefaultYawInitMinSamples;
    }
    if (yaw_window_capacity > kMaxYawHeadingWindow) {
        yaw_window_capacity = kMaxYawHeadingWindow;
    }
    yaw_heading_window_reset(&yaw_window, yaw_window_capacity);

    const float yaw_min_speed_mps = config.yaw_init_min_speed_mps > 0.0f
        ? config.yaw_init_min_speed_mps
        : kDefaultYawInitMinSpeedMps;
    const float yaw_max_heading_std_deg = config.yaw_init_max_heading_std_deg > 0.0f
        ? config.yaw_init_max_heading_std_deg
        : kDefaultYawInitMaxHeadingStdDeg;

    float mount_matrix[3][3]{};
    char mount_label[256]{};
    if (!real_run_replay_load_mount_matrix(
            config.mount_mode,
            config.mount_calibration_path,
            mount_matrix,
            mount_label,
            sizeof(mount_label))) {
        std::fclose(input_fp);
        std::fclose(output_fp);
        std::printf("REAL_RUN_REPLAY: no se pudo cargar montaje IMU\n");
        return false;
    }

    std::printf("REAL_RUN_REPLAY: filtro=%s\n", filter->get_filter_name().c_str());
    std::printf("REAL_RUN_REPLAY: entrada=%s\n", config.input_csv_path);
    std::printf("REAL_RUN_REPLAY: salida=%s\n", config.output_csv_path);
    std::printf("REAL_RUN_REPLAY: montaje=%s\n", mount_label);
    if (config.yaw_init_mode == RealRunYawInitMode::H3_COV_RESET_GRACE) {
        std::printf(
            "REAL_RUN_REPLAY: yaw_init=H3_cov_reset_grace speed>=%.1f m/s samples=%u std<=%.1f deg\n",
            yaw_min_speed_mps,
            yaw_window_capacity,
            yaw_max_heading_std_deg);
        std::printf(
            "REAL_RUN_REPLAY: H3 P_reset=%.0f m^2 grace=%.1f s\n",
            kH3PositionCovResetM2,
            kH3GracePeriodS);
    } else if (config.yaw_init_mode == RealRunYawInitMode::H2_GNSS_STABLE_HEADING) {
        std::printf(
            "REAL_RUN_REPLAY: yaw_init=H2_gnss_stable speed>=%.1f m/s samples=%u std<=%.1f deg\n",
            yaw_min_speed_mps,
            yaw_window_capacity,
            yaw_max_heading_std_deg);
    } else {
        std::printf("REAL_RUN_REPLAY: yaw_init=H0_zero_yaw\n");
    }
    if (config.predict_only_mode) {
        std::printf(
            "REAL_RUN_REPLAY: PREDICT_ONLY activo (sin GNSS update, sin ZUPT/NHC)\n");
        if (config.replay_end_s > 0.0f) {
            std::printf(
                "REAL_RUN_REPLAY: predict_only_end=%.1f s\n",
                config.replay_end_s);
        }
    }
    if (config.h9a_gravity_tilt_init) {
        std::printf(
            "REAL_RUN_REPLAY: H9a gravity tilt init (min_samples=%u window=%.1f s)\n",
            config.h9a_gravity_init_min_samples,
            config.h9a_gravity_init_window_s);
    }
    std::printf(
        "REAL_RUN_REPLAY: constraint_policy=%s nhc_policy=%s nhc_every_n=%u static_end=%.1fs gps_thresh=%.2f m/s\n",
        replay_constraint_policy_name(config.constraint_policy),
        config.nhc_policy == ReplayNhcPolicy::ENABLED ? "enabled" : "disabled",
        config.nhc_every_n_ticks,
        config.static_phase_end_s,
        config.moving_speed_threshold_mps);
    if (instrumentation_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: instrumentacion=%s\n", config.instrumentation_csv_path);
    }
    if (gnss_audit_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: auditoria_gnss=%s\n", config.gnss_audit_csv_path);
    }
    if (h3_diagnostics_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: h3_diagnostics=%s\n", config.h3_diagnostics_csv_path);
    }
    if (config.p0_scale_factor > 0.0f && std::fabs(config.p0_scale_factor - 1.0f) > 1.0e-6f) {
        std::printf("REAL_RUN_REPLAY: P0_scale=%.1fx\n", config.p0_scale_factor);
    }
    if (config.q_scale_factor > 0.0f && std::fabs(config.q_scale_factor - 1.0f) > 1.0e-6f) {
        std::printf("REAL_RUN_REPLAY: Q_scale=%.1fx\n", config.q_scale_factor);
    }
    if (config.nhc_sigma_overridden) {
        std::printf(
            "REAL_RUN_REPLAY: NHC_sigma=%.3f m/s (lateral+vertical)\n",
            config.nhc_sigma_mps);
    } else {
        std::printf(
            "REAL_RUN_REPLAY: NHC_sigma lateral=%.3f vertical=%.3f m/s\n",
            config.nhc_lateral_std_mps,
            config.nhc_vertical_std_mps);
    }
    if (consistency_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: consistency=%s\n", config.consistency_csv_path);
    }
    if (sync_audit_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: h5_sync_audit=%s\n", config.sync_audit_csv_path);
    }
    if (h7_update_audit_fp != nullptr) {
        std::printf("REAL_RUN_REPLAY: h7_update_audit=%s\n", config.h7_update_audit_csv_path);
    }
    if (gap3_observation_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_observation_audit=%s\n",
            config.gap3_observation_audit_csv_path);
    }
    if (gap3_gnss_nis_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_gnss_nis_audit=%s\n",
            config.gap3_gnss_nis_audit_csv_path);
    }
    if (gap3_nhc_block_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_nhc_block_audit=%s\n",
            config.gap3_nhc_block_audit_csv_path);
    }
    if (gap3_gnss_k_block_json_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_gnss_k_block_audit=%s\n",
            config.gap3_gnss_k_block_audit_json_path);
    }
    if (gap3_cov_propagation_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_cov_propagation_audit=%s\n",
            config.gap3_cov_propagation_audit_csv_path);
    }
    if (gap3_cov_step_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: gap3_cov_step_audit=%s\n",
            config.gap3_cov_step_audit_csv_path);
    }
    if (h8_propagation_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: h8_propagation_audit=%s\n",
            config.h8_propagation_audit_csv_path);
    }
    if (h9_tilt_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: h9_tilt_audit=%s (primeros %.1f s)\n",
            config.h9_tilt_audit_csv_path,
            config.static_phase_end_s);
    }
    if (h9a_gravity_alignment_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: h9a_gravity_alignment_audit=%s (0-%.1f s)\n",
            config.h9a_gravity_alignment_audit_csv_path,
            h9a_gravity_alignment_audit_end_s(config));
    }
    if (h9b_attitude_propagation_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: h9b_attitude_propagation_audit=%s (0-%.1f s)\n",
            config.h9b_attitude_propagation_audit_csv_path,
            h9b_attitude_propagation_audit_end_s(config));
    }
    if (h9d_gravity_subtraction_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: h9d_gravity_subtraction_audit=%s (0-%.1f s)\n",
            config.h9d_gravity_subtraction_audit_csv_path,
            h9d_gravity_subtraction_audit_end_s(config));
    }
    if (propagation_chain_audit_fp != nullptr) {
        std::printf(
            "REAL_RUN_REPLAY: propagation_chain_audit=%s (0-%.1f s)\n",
            config.propagation_chain_audit_csv_path,
            propagation_chain_audit_end_s(config));
    }

    char line[kMaxCsvLineBytes];
    while (std::fgets(line, static_cast<int>(sizeof(line)), input_fp) != nullptr) {
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '\0') {
            continue;
        }

        ParsedReplayRow row{};
        if (!parse_replay_csv_line(line, &row)) {
            continue;
        }

        ++result.rows_processed;
        const double dt_s = compute_dt_s(row.timestamp_s, previous_timestamp_s);
        previous_timestamp_s = row.timestamp_s;
        result.duration_s = static_cast<float>(row.timestamp_s);

        if (config.replay_end_s > 0.0f
            && row.timestamp_s > static_cast<double>(config.replay_end_s)) {
            break;
        }

        if (row.row_type == ReplayRowType::GPS) {
            ++result.gps_rows;
            if (!row.has_pos) {
                continue;
            }

            if (!filter_initialized) {
                bool seeded = false;
                if (config.gnss_ref_overridden) {
                    const double gnss_ref_lla_deg[3] = {
                        static_cast<double>(config.gnss_ref_lat_deg),
                        static_cast<double>(config.gnss_ref_lon_deg),
                        static_cast<double>(config.gnss_ref_alt_m),
                    };
                    seeded = filter_impl->seed_from_ned_fix(
                        row.pos_ned,
                        gnss_ref_lla_deg,
                        NAVICORE_DOMAIN_AIR);
                } else {
                    seeded = filter_impl->seed_from_ned_fix(row.pos_ned, NAVICORE_DOMAIN_AIR);
                }
                if (!seeded) {
                    continue;
                }
                if (config.p0_scale_factor > 0.0f) {
                    scale_covariance_matrix(
                        &filter_impl->native(),
                        config.p0_scale_factor);
                }
                if (config.q_scale_factor > 0.0f) {
                    scale_imu_process_noise(
                        &filter_impl->native(),
                        config.q_scale_factor);
                }
                if (config.nhc_sigma_overridden) {
                    filter_impl->set_nhc_measurement_stds(
                        config.nhc_sigma_mps,
                        config.nhc_sigma_mps);
                } else {
                    filter_impl->set_nhc_measurement_stds(
                        config.nhc_lateral_std_mps,
                        config.nhc_vertical_std_mps);
                }
                filter_impl->sync_simulation_clock_ms(
                    static_cast<uint32_t>(row.timestamp_s * 1000.0));
                filter_initialized = true;
                result.filter_initialized = true;
                if (gap3_cov_step_audit_fp != nullptr) {
                    InsEkfFilter &ekf_init = filter_impl->native();
                    ins_ekf_set_cov_step_audit(&ekf_init, gap3_cov_step_audit_fp);
                    ins_ekf_set_cov_step_audit_context(&ekf_init, row.timestamp_s, 0U);
                    ins_ekf_log_cov_step_audit(&ekf_init, "init", "post");
                }
                if (gap3_vel_source_audit_fp != nullptr) {
                    ins_ekf_set_vel_source_audit(
                        &filter_impl->native(),
                        gap3_vel_source_audit_fp);
                }
                if (gap3_nhc_block_audit_fp != nullptr) {
                    ins_ekf_set_nhc_block_audit(
                        &filter_impl->native(),
                        gap3_nhc_block_audit_fp);
                }
                ins_ekf_set_nhc_every_n_ticks(
                    &filter_impl->native(),
                    config.nhc_every_n_ticks);
                {
                    InsEkfGnssObsMode ekf_gnss_mode = INS_EKF_GNSS_OBS_POS;
                    switch (config.gnss_obs_mode) {
                    case ReplayGnssObsMode::POS_VEL:
                        ekf_gnss_mode = INS_EKF_GNSS_OBS_POS_VEL;
                        break;
                    case ReplayGnssObsMode::VEL_ONLY:
                        ekf_gnss_mode = INS_EKF_GNSS_OBS_VEL_ONLY;
                        break;
                    default:
                        ekf_gnss_mode = INS_EKF_GNSS_OBS_POS;
                        break;
                    }
                    ins_ekf_set_gnss_obs_mode(&filter_impl->native(), ekf_gnss_mode);
                    if (config.gnss_vel_std_mps > 0.0f) {
                        const float var = config.gnss_vel_std_mps * config.gnss_vel_std_mps;
                        ins_ekf_set_gnss_vel_var_m2(&filter_impl->native(), var);
                    }
                    {
                        InsEkfPpvPolicy ekf_ppv = INS_EKF_PPV_POLICY_NONE;
                        switch (config.ppv_policy) {
                        case ReplayPpvPolicy::GAP_LE_1S:
                            ekf_ppv = INS_EKF_PPV_POLICY_GAP_LE_1S;
                            break;
                        case ReplayPpvPolicy::ZERO:
                            ekf_ppv = INS_EKF_PPV_POLICY_ZERO;
                            break;
                        case ReplayPpvPolicy::COS_POS:
                            ekf_ppv = INS_EKF_PPV_POLICY_COS_POS;
                            break;
                        case ReplayPpvPolicy::COS_TOT:
                            ekf_ppv = INS_EKF_PPV_POLICY_COS_TOT;
                            break;
                        default:
                            ekf_ppv = INS_EKF_PPV_POLICY_NONE;
                            break;
                        }
                        ins_ekf_set_p_pv_policy(&filter_impl->native(), ekf_ppv);
                    }
                    std::printf(
                        "REAL_RUN_REPLAY: gnss_obs_mode=%s | sigma_vel=%.2f m/s | p_pv_policy=%s\n",
                        ins_ekf_gnss_obs_mode_name(ekf_gnss_mode),
                        std::sqrt(filter_impl->native().gnss_vel_var_m2_h),
                        ins_ekf_p_pv_policy_name(filter_impl->native().ppv_policy));
                }
                if (gap3_cov_propagation_audit_fp != nullptr) {
                    InsEkfFilter &ekf_init = filter_impl->native();
                    (void)write_gap3_cov_propagation_audit_row(
                        gap3_cov_propagation_audit_fp,
                        "init",
                        row.timestamp_s,
                        ekf_init,
                        0.0f,
                        row.has_speed ? row.speed : 0.0f,
                        -1,
                        0.0f);
                }
                if (!geodesy_validation_printed) {
                    print_geodesy_datum_validation(geodesy_validation);
                    geodesy_validation_printed = true;
                }
                if (config.gnss_ref_overridden) {
                    std::printf(
                        "REAL_RUN_REPLAY: inicializado @ t=%.3f s | ref LLA=(%.7f, %.7f, %.1f) | NED=(%.3f, %.3f, %.3f) m\n",
                        row.timestamp_s,
                        config.gnss_ref_lat_deg,
                        config.gnss_ref_lon_deg,
                        config.gnss_ref_alt_m,
                        row.pos_ned[0],
                        row.pos_ned[1],
                        row.pos_ned[2]);
                } else {
                    std::printf(
                        "REAL_RUN_REPLAY: inicializado @ t=%.3f s | ref NED=(%.3f, %.3f, %.3f) m\n",
                        row.timestamp_s,
                        row.pos_ned[0],
                        row.pos_ned[1],
                        row.pos_ned[2]);
                }
            }

            if (row.has_speed) {
                last_gps_speed_mps = row.speed;
            }

            if (has_heading_ref_pos
                && row.has_speed
                && row.speed >= yaw_min_speed_mps) {
                float displacement_m = 0.0f;
                const float course_rad = compute_gnss_course_rad(
                    heading_ref_pos_ned,
                    row.pos_ned,
                    &displacement_m);
                if (displacement_m >= kMinGnssHeadingDisplacementM) {
                    latest_yaw_gnss_deg = course_rad * kRadToDegF;
                    has_latest_yaw_gnss = true;
                    (void)yaw_heading_window_push(&yaw_window, course_rad);
                }
            }

            if (!yaw_init_applied
                && !config.predict_only_mode
                && yaw_init_mode_uses_stable_heading(config.yaw_init_mode)
                && yaw_window.count >= yaw_window_capacity) {
                float mean_heading_rad = 0.0f;
                float heading_std_deg = 0.0f;
                if (yaw_heading_window_stats(&yaw_window, &mean_heading_rad, &heading_std_deg)
                    && heading_std_deg <= yaw_max_heading_std_deg) {
                    InsEkfFilter &ekf_for_yaw = filter_impl->native();
                    if (set_ekf_yaw_preserve_roll_pitch(&ekf_for_yaw, mean_heading_rad)) {
                        yaw_init_applied = true;
                        result.yaw_init_applied = true;
                        result.yaw_init_applied_at_s = static_cast<float>(row.timestamp_s);
                        result.yaw_init_heading_deg = mean_heading_rad * kRadToDegF;
                        latest_yaw_gnss_deg = mean_heading_rad * kRadToDegF;
                        has_latest_yaw_gnss = true;

                        if (config.yaw_init_mode == RealRunYawInitMode::H3_COV_RESET_GRACE) {
                            reset_position_covariance_diagonal(
                                &ekf_for_yaw,
                                kH3PositionCovResetM2);
                            h3_applied = true;
                            result.h3_applied = true;
                            h3_grace_period_end_s = row.timestamp_s + static_cast<double>(kH3GracePeriodS);
                            result.h3_grace_period_end_s = static_cast<float>(h3_grace_period_end_s);
                            std::printf(
                                "[H3 EXPERIMENT] P covariance reset applied & Grace period active until t = %.1f s\n",
                                h3_grace_period_end_s);
                            std::printf(
                                "REAL_RUN_REPLAY: H3 yaw inicializado @ t=%.3f s | heading=%.2f deg | std=%.2f deg\n",
                                row.timestamp_s,
                                result.yaw_init_heading_deg,
                                heading_std_deg);
                        } else {
                            std::printf(
                                "REAL_RUN_REPLAY: H2 yaw inicializado @ t=%.3f s | heading=%.2f deg | std=%.2f deg\n",
                                row.timestamp_s,
                                result.yaw_init_heading_deg,
                                heading_std_deg);
                        }
                    }
                }
            }

            if (config.predict_only_mode) {
                last_gps_pos_ned[0] = row.pos_ned[0];
                last_gps_pos_ned[1] = row.pos_ned[1];
                last_gps_pos_ned[2] = row.pos_ned[2];
                has_last_gps_pos = true;
                continue;
            }

            apply_replay_constraints(
                filter.get(),
                row.timestamp_s,
                row.has_speed ? row.speed : last_gps_speed_mps,
                last_imu_accel_norm_mps2,
                last_imu_gyro_norm_radps,
                config,
                nullptr);

            float std_dev[3] = {kGnssMinHorizontalStdM, kGnssMinHorizontalStdM, kGnssMinVerticalStdM};
            if (row.has_accuracy) {
                std_dev[0] = std::fmax(row.accuracy_horizontal, kGnssMinHorizontalStdM);
                std_dev[1] = std_dev[0];
                std_dev[2] = std::fmax(row.accuracy_vertical, kGnssMinVerticalStdM);
            }

            const uint32_t accepts_before = ins_ekf_gnss_accept_count(&filter_impl->native());
            InsEkfFilter &ekf = filter_impl->native();

            float trace_p = covariance_trace(ekf);
            float det_p = 0.0f;
            (void)covariance_determinant(ekf, &det_p);
            const float sqrt_pnn = std::sqrt(std::fmax(ekf.cov.P[INS_ERR_POS_N][INS_ERR_POS_N], 0.0f));
            const float sqrt_pee = std::sqrt(std::fmax(ekf.cov.P[INS_ERR_POS_E][INS_ERR_POS_E], 0.0f));
            const float sqrt_pdd = std::sqrt(std::fmax(ekf.cov.P[INS_ERR_POS_D][INS_ERR_POS_D], 0.0f));

            float ekf_pos_before[3] = {ekf.pos_[0], ekf.pos_[1], ekf.pos_[2]};
            const float vel_before[3] = {ekf.vel_[0], ekf.vel_[1], ekf.vel_[2]};
            const double ekf_timestamp_before_s =
                static_cast<double>(ekf.last_imu_timestamp_ms) * 0.001;
            const double imu_timestamp_s =
                has_last_imu_timestamp ? last_imu_timestamp_s : row.timestamp_s;
            float hph_before[3][3]{};
            snapshot_position_covariance(ekf, hph_before);
            const float r_m2_before = ekf.gnss_pos_var_m2;

            const bool grace_active =
                h3_applied
                && (row.timestamp_s <= h3_grace_period_end_s);
            const bool nis_gate_bypassed = grace_active;
            const float saved_nis_threshold = ekf.nis_threshold;
            if (nis_gate_bypassed) {
                ekf.nis_threshold = kH3NisGateBypassThreshold;
            }

            float z_ned[3] = {0.0f, 0.0f, 0.0f};
            simulate_adapter_measurement_z(
                ekf.ref_lat_deg,
                ekf.ref_lon_deg,
                ekf.ref_alt_m,
                row.pos_ned,
                z_ned);
            const float hx_ned[3] = {
                ekf_pos_before[0],
                ekf_pos_before[1],
                ekf_pos_before[2],
            };

            if (gap3_cov_propagation_audit_fp != nullptr) {
                const float vel_h_before = std::hypot(vel_before[0], vel_before[1]);
                (void)write_gap3_cov_propagation_audit_row(
                    gap3_cov_propagation_audit_fp,
                    "gnss_pre",
                    row.timestamp_s,
                    ekf,
                    vel_h_before,
                    row.has_speed ? row.speed : 0.0f,
                    -1,
                    0.0f);
            }
            if (gap3_cov_step_audit_fp != nullptr) {
                ins_ekf_set_cov_step_audit_context(&ekf, row.timestamp_s, gap3_cov_step_imu_seq);
            }

            float gps_course_deg = 0.0f;
            const bool has_gps_course = gap3_has_prev_gps_pos;
            if (has_gps_course) {
                gps_course_deg = compute_gnss_course_rad(
                    gap3_prev_gps_pos_ned,
                    row.pos_ned,
                    nullptr)
                    * kRadToDegF;
            }
            const bool has_vel_obs =
                row.has_speed && row.speed > 0.0f && has_gps_course;
            const bool vel_only_skip =
                config.gnss_obs_mode == ReplayGnssObsMode::VEL_ONLY && !has_vel_obs;

            if (!vel_only_skip) {
                if (config.gnss_obs_mode == ReplayGnssObsMode::POS) {
                    filter->update_gnss(row.pos_ned, std_dev);
                } else {
                    filter_impl->update_gnss_with_velocity(
                        row.pos_ned,
                        std_dev,
                        row.has_speed ? row.speed : 0.0f,
                        gps_course_deg,
                        has_vel_obs);
                }
            }

            if (nis_gate_bypassed) {
                ekf.nis_threshold = saved_nis_threshold;
            }

            const uint32_t accepts_after = ins_ekf_gnss_accept_count(&ekf);
            bool gnss_accepted = !vel_only_skip && accepts_after > accepts_before;
            if (nis_gate_bypassed) {
                gnss_accepted = true;
            }
            filter_impl->sync_simulation_clock_ms(static_cast<uint32_t>(row.timestamp_s * 1000.0));

            result.gnss_reject_count = ins_ekf_gnss_reject_count(&ekf);
            result.gnss_accept_count = accepts_after;

            float innovation_ned[3] = {0.0f, 0.0f, 0.0f};
            ins_ekf_get_gnss_innovation(&ekf, innovation_ned);
            const float nis_value = ins_ekf_last_nis(&ekf);

            if (gap3_cov_propagation_audit_fp != nullptr) {
                float vel_after_gnss[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_velocity_ned(&ekf, vel_after_gnss);
                const float vel_h_after = std::hypot(vel_after_gnss[0], vel_after_gnss[1]);
                (void)write_gap3_cov_propagation_audit_row(
                    gap3_cov_propagation_audit_fp,
                    gnss_accepted ? "gnss_post" : "gnss_reject",
                    row.timestamp_s,
                    ekf,
                    vel_h_after,
                    row.has_speed ? row.speed : 0.0f,
                    gnss_accepted ? 1 : 0,
                    ekf.gnss_last_k_vel_max);
            }

            float s_matrix[3][3]{};
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    s_matrix[i][j] = hph_before[i][j];
                }
                s_matrix[i][i] += r_m2_before;
            }

            if (gnss_audit_fp != nullptr) {
                const double imu_ts = has_last_imu_timestamp ? last_imu_timestamp_s : row.timestamp_s;
                (void)write_gnss_audit_row(
                    gnss_audit_fp,
                    row.timestamp_s,
                    imu_ts,
                    ekf_pos_before,
                    row.pos_ned,
                    innovation_ned,
                    s_matrix,
                    hph_before,
                    r_m2_before,
                    nis_value,
                    gnss_accepted,
                    ins_ekf_gnss_reject_count(&ekf));
            }

            if (h7_update_audit_fp != nullptr || gap3_gnss_nis_audit_fp != nullptr) {
                ++gps_update_index;
            }

            if (h7_update_audit_fp != nullptr) {
                (void)write_h7_update_audit_row(
                    h7_update_audit_fp,
                    row.timestamp_s,
                    gps_update_index,
                    z_ned,
                    hx_ned,
                    innovation_ned,
                    s_matrix,
                    nis_value,
                    nis_gate_bypassed ? kH3NisGateBypassThreshold : saved_nis_threshold,
                    gnss_accepted,
                    row.pos_ned);
            }

            if (gap3_observation_audit_fp != nullptr) {
                if (!gap3_gnss_has_anchor) {
                    gap3_set_observation_anchor(
                        gap3_gnss_anchor_pos,
                        gap3_gnss_anchor_vel,
                        ekf_pos_before,
                        vel_before,
                        &gap3_gnss_last_timestamp_s,
                        row.timestamp_s,
                        &gap3_gnss_has_anchor);
                }

                double pred_dpos_h = 0.0;
                double pred_dvel_h = 0.0;
                double pred_dt_s = 0.0;
                gap3_compute_cycle_pred_accum(
                    gap3_gnss_anchor_pos,
                    gap3_gnss_anchor_vel,
                    ekf_pos_before,
                    vel_before,
                    gap3_gnss_last_timestamp_s,
                    row.timestamp_s,
                    &pred_dpos_h,
                    &pred_dvel_h,
                    &pred_dt_s);

                float pos_after[3] = {0.0f, 0.0f, 0.0f};
                float vel_after[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_position_ned(&ekf, pos_after);
                ins_ekf_get_velocity_ned(&ekf, vel_after);

                InsEkfGnssUpdateDetail gnss_detail{};
                const InsEkfGnssUpdateDetail *gnss_detail_ptr = nullptr;
                if (ins_ekf_get_gnss_last_update_detail(&ekf, &gnss_detail)) {
                    gnss_detail_ptr = &gnss_detail;
                }

                const bool gnss_logged_accepted = gnss_accepted || nis_gate_bypassed;
                int reject_reason = gnss_detail_ptr != nullptr
                    ? static_cast<int>(gnss_detail.reject_reason)
                    : (gnss_logged_accepted ? 0 : 1);

                (void)write_gap3_observation_audit_row(
                    gap3_observation_audit_fp,
                    "GNSS",
                    row.timestamp_s,
                    gnss_logged_accepted,
                    reject_reason,
                    pred_dpos_h,
                    pred_dvel_h,
                    pred_dt_s,
                    gnss_detail_ptr,
                    nullptr,
                    ekf_pos_before,
                    pos_after,
                    vel_before,
                    vel_after);

                gap3_set_observation_anchor(
                    gap3_gnss_anchor_pos,
                    gap3_gnss_anchor_vel,
                    pos_after,
                    vel_after,
                    &gap3_gnss_last_timestamp_s,
                    row.timestamp_s,
                    &gap3_gnss_has_anchor);
            }

            if (gap3_gnss_nis_audit_fp != nullptr) {
                float pos_after_nis[3] = {0.0f, 0.0f, 0.0f};
                float vel_after_nis[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_position_ned(&ekf, pos_after_nis);
                ins_ekf_get_velocity_ned(&ekf, vel_after_nis);

                InsEkfGnssUpdateDetail gnss_nis_detail{};
                const InsEkfGnssUpdateDetail *gnss_nis_detail_ptr = nullptr;
                if (ins_ekf_get_gnss_last_update_detail(&ekf, &gnss_nis_detail)) {
                    gnss_nis_detail_ptr = &gnss_nis_detail;
                }

                const bool gnss_nis_accepted = gnss_accepted || nis_gate_bypassed;
                int gnss_nis_reject_reason = gnss_nis_detail_ptr != nullptr
                    ? static_cast<int>(gnss_nis_detail.reject_reason)
                    : (gnss_nis_accepted ? 0 : 1);

                float gps_course_deg = 0.0f;
                if (gap3_has_prev_gps_pos) {
                    gps_course_deg = compute_gnss_course_rad(
                        gap3_prev_gps_pos_ned,
                        row.pos_ned,
                        nullptr)
                        * kRadToDegF;
                }

                const double dt_since_prev_gnss = gap3_last_gnss_timestamp_s >= 0.0
                    ? (row.timestamp_s - gap3_last_gnss_timestamp_s)
                    : 0.0;
                const double dt_since_prev_accept = gap3_last_gnss_accept_timestamp_s >= 0.0
                    ? (row.timestamp_s - gap3_last_gnss_accept_timestamp_s)
                    : -1.0;

                (void)write_gap3_gnss_nis_audit_row(
                    gap3_gnss_nis_audit_fp,
                    row.timestamp_s,
                    gps_update_index,
                    z_ned,
                    hx_ned,
                    vel_before,
                    row.has_speed,
                    row.has_speed ? row.speed : 0.0f,
                    gps_course_deg,
                    hph_before,
                    r_m2_before,
                    nis_gate_bypassed ? kH3NisGateBypassThreshold : saved_nis_threshold,
                    gnss_nis_accepted,
                    gnss_nis_reject_reason,
                    dt_since_prev_gnss,
                    dt_since_prev_accept,
                    gnss_nis_detail_ptr,
                    pos_after_nis,
                    vel_after_nis);

                gap3_last_gnss_timestamp_s = row.timestamp_s;
                if (gnss_nis_accepted) {
                    gap3_last_gnss_accept_timestamp_s = row.timestamp_s;
                }
                gap3_prev_gps_pos_ned[0] = row.pos_ned[0];
                gap3_prev_gps_pos_ned[1] = row.pos_ned[1];
                gap3_prev_gps_pos_ned[2] = row.pos_ned[2];
                gap3_has_prev_gps_pos = true;
            }

            if (gap3_gnss_k_block_json_fp != nullptr && gnss_accepted) {
                InsEkfGnssKBlockDetail k_block{};
                InsEkfGnssUpdateDetail dump_detail{};
                if (ins_ekf_get_gnss_last_k_block_detail(&ekf, &k_block)
                    && ins_ekf_get_gnss_last_update_detail(&ekf, &dump_detail)) {
                    float pos_post_dump[3] = {0.0f, 0.0f, 0.0f};
                    float vel_post_dump[3] = {0.0f, 0.0f, 0.0f};
                    ins_ekf_get_position_ned(&ekf, pos_post_dump);
                    ins_ekf_get_velocity_ned(&ekf, vel_post_dump);
                    (void)write_gap3_gnss_k_block_audit_json(
                        gap3_gnss_k_block_json_fp,
                        row.timestamp_s,
                        gps_update_index,
                        true,
                        z_ned,
                        hx_ned,
                        vel_before,
                        vel_post_dump,
                        pos_post_dump,
                        row.has_speed ? row.speed : 0.0f,
                        row.has_speed,
                        hph_before,
                        r_m2_before,
                        s_matrix,
                        &dump_detail,
                        &k_block);
                    std::fprintf(gap3_gnss_k_block_json_fp, "\n");
                }
            }

            if (instrumentation_fp != nullptr) {
                float yaw_ekf_rad = 0.0f;
                ins_ekf_get_attitude_rad(&ekf, nullptr, nullptr, &yaw_ekf_rad);
                (void)write_instrumentation_row(
                    instrumentation_fp,
                    row.timestamp_s,
                    has_latest_yaw_gnss,
                    latest_yaw_gnss_deg,
                    yaw_ekf_rad * kRadToDegF,
                    innovation_ned,
                    nis_value,
                    gnss_accepted,
                    ins_ekf_gnss_reject_count(&ekf));
            }

            if (h3_diagnostics_fp != nullptr) {
                float yaw_ekf_rad = 0.0f;
                ins_ekf_get_attitude_rad(&ekf, nullptr, nullptr, &yaw_ekf_rad);
                (void)write_h3_diagnostics_row(
                    h3_diagnostics_fp,
                    row.timestamp_s,
                    has_latest_yaw_gnss,
                    latest_yaw_gnss_deg,
                    yaw_ekf_rad * kRadToDegF,
                    innovation_ned,
                    nis_value,
                    gnss_accepted,
                    ins_ekf_gnss_reject_count(&ekf),
                    h3_applied,
                    grace_active,
                    nis_gate_bypassed,
                    ekf);
            }

            if (consistency_fp != nullptr) {
                const float error_n =
                    static_cast<float>(row.pos_ned[0]) - ekf_pos_before[0];
                const float error_e =
                    static_cast<float>(row.pos_ned[1]) - ekf_pos_before[1];
                const float nees_pos = compute_nees_pos_2d(error_n, error_e, hph_before);
                const float ratio_n = (sqrt_pnn > 1.0e-6f)
                    ? (std::fabs(error_n) / sqrt_pnn)
                    : 0.0f;
                const float ratio_e = (sqrt_pee > 1.0e-6f)
                    ? (std::fabs(error_e) / sqrt_pee)
                    : 0.0f;
                const float innovation_norm = std::sqrt(
                    (innovation_ned[0] * innovation_ned[0])
                    + (innovation_ned[1] * innovation_ned[1])
                    + (innovation_ned[2] * innovation_ned[2]));

                (void)write_consistency_row(
                    consistency_fp,
                    row.timestamp_s,
                    error_n,
                    error_e,
                    nees_pos,
                    ratio_n,
                    ratio_e,
                    trace_p,
                    det_p,
                    innovation_norm,
                    nis_value,
                    gnss_accepted);
            }

            if (sync_audit_fp != nullptr) {
                const float ekf_speed_h = std::sqrt(
                    (vel_before[0] * vel_before[0]) + (vel_before[1] * vel_before[1]));
                const float gps_speed_mps = row.has_speed ? row.speed : ekf_speed_h;
                const float moving_speed_mps = std::fmax(gps_speed_mps, ekf_speed_h);
                if (moving_speed_mps > kH5SyncAuditMinSpeedMps) {
                    (void)write_h5_sync_audit_row(
                        sync_audit_fp,
                        ekf_timestamp_before_s,
                        row.timestamp_s,
                        imu_timestamp_s,
                        innovation_ned[0],
                        innovation_ned[1],
                        vel_before[0],
                        vel_before[1],
                        gnss_accepted);
                }
            }

            if (row.has_speed && row.speed >= yaw_min_speed_mps) {
                heading_ref_pos_ned[0] = row.pos_ned[0];
                heading_ref_pos_ned[1] = row.pos_ned[1];
                heading_ref_pos_ned[2] = row.pos_ned[2];
                has_heading_ref_pos = true;
            }

            last_gps_pos_ned[0] = row.pos_ned[0];
            last_gps_pos_ned[1] = row.pos_ned[1];
            last_gps_pos_ned[2] = row.pos_ned[2];
            has_last_gps_pos = true;

            const NaviState state = filter->get_state();
            (void)write_output_state(output_fp, row.timestamp_s, state, "GPS");
        } else if (row.row_type == ReplayRowType::IMU) {
            ++result.imu_rows;
            last_imu_timestamp_s = row.timestamp_s;
            has_last_imu_timestamp = true;
            if (!filter_initialized || !row.has_accel || !row.has_gyro) {
                continue;
            }

            float aligned_accel[3];
            float aligned_gyro[3];
            mat3_vec3_mul(mount_matrix, row.accel, aligned_accel);
            mat3_vec3_mul(mount_matrix, row.gyro, aligned_gyro);

            if (config.h9a_gravity_tilt_init && !gravity_tilt_init.applied) {
                (void)accumulate_gravity_tilt_sample(
                    gravity_tilt_init,
                    row.timestamp_s,
                    aligned_accel,
                    aligned_gyro);
                const uint32_t min_samples = config.h9a_gravity_init_min_samples > 0U
                    ? config.h9a_gravity_init_min_samples
                    : kDefaultH9aGravityInitMinSamples;
                const float init_window_s = config.h9a_gravity_init_window_s > 0.0f
                    ? config.h9a_gravity_init_window_s
                    : kDefaultH9aGravityInitWindowS;
                if (gravity_tilt_init_ready(
                        gravity_tilt_init,
                        row.timestamp_s,
                        min_samples,
                        init_window_s)) {
                    InsEkfFilter &ekf_gravity = filter_impl->native();
                    if (finalize_gravity_tilt_init(
                            gravity_tilt_init,
                            &ekf_gravity,
                            row.timestamp_s,
                            nullptr,
                            nullptr)) {
                        h9b_has_prev_gravity_rv = false;
                    }
                } else {
                    continue;
                }
            }

            float accel_norm = 0.0f;
            float gyro_norm = 0.0f;
            if (row.has_accel) {
                accel_norm = std::sqrt(
                    (row.accel[0] * row.accel[0])
                    + (row.accel[1] * row.accel[1])
                    + (row.accel[2] * row.accel[2]));
            }
            if (row.has_gyro) {
                gyro_norm = std::sqrt(
                    (row.gyro[0] * row.gyro[0])
                    + (row.gyro[1] * row.gyro[1])
                    + (row.gyro[2] * row.gyro[2]));
            }
            last_imu_accel_norm_mps2 = accel_norm;
            last_imu_gyro_norm_radps = gyro_norm;

            float vel_before_constraints[3] = {0.0f, 0.0f, 0.0f};
            ins_ekf_get_velocity_ned(&filter_impl->native(), vel_before_constraints);

            ReplayConstraintDecision constraint_decision{};
            apply_replay_constraints(
                filter.get(),
                row.timestamp_s,
                last_gps_speed_mps,
                accel_norm,
                gyro_norm,
                config,
                &constraint_decision);
            filter_impl->sync_simulation_clock_ms(static_cast<uint32_t>(row.timestamp_s * 1000.0));

            const bool nhc_mode_selected =
                constraint_decision.nhc_armed && !constraint_decision.zupt_armed;
            const bool zupt_armed = constraint_decision.zupt_armed;
            const char *constraint_policy_name =
                replay_constraint_policy_name(constraint_decision.policy);

            if (gap3_cov_step_audit_fp != nullptr || gap3_vel_source_audit_fp != nullptr
                || gap3_nhc_block_audit_fp != nullptr) {
                InsEkfFilter &ekf_step = filter_impl->native();
                ins_ekf_set_cov_step_audit_context(
                    &ekf_step,
                    row.timestamp_s,
                    ++gap3_cov_step_imu_seq);
                ins_ekf_set_vel_source_audit_context(
                    &ekf_step,
                    row.timestamp_s,
                    gap3_cov_step_imu_seq,
                    last_gps_speed_mps);
                ins_ekf_set_nhc_block_audit_context(
                    &ekf_step,
                    row.timestamp_s,
                    gap3_cov_step_imu_seq,
                    last_gps_speed_mps);
            }

            float pos_pre_predict[3] = {0.0f, 0.0f, 0.0f};
            float vel_pre_predict[3] = {0.0f, 0.0f, 0.0f};
            if (gap3_observation_audit_fp != nullptr) {
                ins_ekf_get_position_ned(&filter_impl->native(), pos_pre_predict);
                ins_ekf_get_velocity_ned(&filter_impl->native(), vel_pre_predict);
            }
            const uint32_t nhc_before_predict = ins_ekf_nhc_update_count(&filter_impl->native());
            const uint32_t zupt_before_predict = ins_ekf_zupt_update_count(&filter_impl->native());

            filter->predict(dt_s, aligned_accel, aligned_gyro);
            last_predict_timestamp_s = row.timestamp_s;
            has_last_predict_timestamp = true;

            InsEkfFilter &ekf_post = filter_impl->native();
            const uint32_t nhc_after_predict = ins_ekf_nhc_update_count(&ekf_post);
            const uint32_t zupt_after_predict = ins_ekf_zupt_update_count(&ekf_post);
            const bool nhc_applied = nhc_after_predict > nhc_before_predict;
            const bool zupt_applied = zupt_after_predict > zupt_before_predict;

            if (gap3_imu_constraint_audit_fp != nullptr) {
                float vel_post[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_velocity_ned(&ekf_post, vel_post);
                const float vel_h_post = std::hypot(vel_post[0], vel_post[1]);
                InsEkfPredictAudit predict_audit{};
                float a_body_x = 0.0f;
                float a_body_y = 0.0f;
                float a_body_z = 0.0f;
                if (ins_ekf_get_last_predict_audit(&ekf_post, &predict_audit) && predict_audit.valid) {
                    a_body_x = predict_audit.a_corr_mps2[0];
                    a_body_y = predict_audit.a_corr_mps2[1];
                    a_body_z = predict_audit.a_corr_mps2[2];
                }
                (void)write_gap3_imu_constraint_audit_row(
                    gap3_imu_constraint_audit_fp,
                    row.timestamp_s,
                    gap3_cov_step_imu_seq,
                    nhc_mode_selected,
                    zupt_armed,
                    zupt_applied,
                    nhc_applied,
                    last_gps_speed_mps,
                    config.static_phase_end_s,
                    config.moving_speed_threshold_mps,
                    accel_norm,
                    gyro_norm,
                    vel_h_post,
                    ekf_post.bias_a_[0],
                    ekf_post.bias_a_[1],
                    ekf_post.bias_a_[2],
                    a_body_x,
                    a_body_y,
                    a_body_z,
                    constraint_policy_name,
                    config.nhc_policy == ReplayNhcPolicy::ENABLED);
            }

            if (gap3_constraint_pipeline_audit_fp != nullptr) {
                InsEkfVelPipelineAudit pipeline{};
                if (ins_ekf_get_vel_pipeline_audit(&ekf_post, &pipeline)) {
                    (void)write_gap3_constraint_pipeline_audit_row(
                        gap3_constraint_pipeline_audit_fp,
                        row.timestamp_s,
                        gap3_cov_step_imu_seq,
                        constraint_policy_name,
                        zupt_armed,
                        constraint_decision.nhc_armed,
                        vel_before_constraints,
                        pipeline,
                        last_gps_speed_mps);
                }
            }

            if (gap3_cov_propagation_audit_fp != nullptr
                && (gap3_cov_last_predict_log_s < 0.0
                    || (row.timestamp_s - gap3_cov_last_predict_log_s) >= 1.0)) {
                InsEkfFilter &ekf_cov = filter_impl->native();
                float vel_predict[3] = {0.0f, 0.0f, 0.0f};
                ins_ekf_get_velocity_ned(&ekf_cov, vel_predict);
                const float vel_h_predict = std::hypot(vel_predict[0], vel_predict[1]);
                (void)write_gap3_cov_propagation_audit_row(
                    gap3_cov_propagation_audit_fp,
                    "predict_1hz",
                    row.timestamp_s,
                    ekf_cov,
                    vel_h_predict,
                    last_gps_speed_mps,
                    -1,
                    0.0f);
                gap3_cov_last_predict_log_s = row.timestamp_s;
            }

            if (gap3_observation_audit_fp != nullptr) {
                InsEkfFilter &ekf_gap3 = filter_impl->native();
                const uint32_t nhc_after_predict = ins_ekf_nhc_update_count(&ekf_gap3);
                const uint32_t zupt_after_predict = ins_ekf_zupt_update_count(&ekf_gap3);

                if (nhc_after_predict > nhc_before_predict || zupt_after_predict > zupt_before_predict) {
                    float pos_post_predict[3] = {0.0f, 0.0f, 0.0f};
                    float vel_post_predict[3] = {0.0f, 0.0f, 0.0f};
                    ins_ekf_get_position_ned(&ekf_gap3, pos_post_predict);
                    ins_ekf_get_velocity_ned(&ekf_gap3, vel_post_predict);

                    if (nhc_after_predict > nhc_before_predict) {
                        if (!gap3_nhc_has_anchor) {
                            gap3_set_observation_anchor(
                                gap3_nhc_anchor_pos,
                                gap3_nhc_anchor_vel,
                                pos_pre_predict,
                                vel_pre_predict,
                                &gap3_nhc_last_timestamp_s,
                                row.timestamp_s,
                                &gap3_nhc_has_anchor);
                        }

                        double pred_dpos_h = 0.0;
                        double pred_dvel_h = 0.0;
                        double pred_dt_s = 0.0;
                        gap3_compute_cycle_pred_accum(
                            gap3_nhc_anchor_pos,
                            gap3_nhc_anchor_vel,
                            pos_pre_predict,
                            vel_pre_predict,
                            gap3_nhc_last_timestamp_s,
                            row.timestamp_s,
                            &pred_dpos_h,
                            &pred_dvel_h,
                            &pred_dt_s);

                        InsEkfNhcUpdateDetail nhc_detail{};
                        const InsEkfNhcUpdateDetail *nhc_detail_ptr = nullptr;
                        if (ins_ekf_get_nhc_last_update_detail(&ekf_gap3, &nhc_detail)) {
                            nhc_detail_ptr = &nhc_detail;
                        }
                        (void)write_gap3_observation_audit_row(
                            gap3_observation_audit_fp,
                            "NHC",
                            row.timestamp_s,
                            true,
                            0,
                            pred_dpos_h,
                            pred_dvel_h,
                            pred_dt_s,
                            nullptr,
                            nhc_detail_ptr,
                            pos_pre_predict,
                            pos_post_predict,
                            vel_pre_predict,
                            vel_post_predict);

                        gap3_set_observation_anchor(
                            gap3_nhc_anchor_pos,
                            gap3_nhc_anchor_vel,
                            pos_post_predict,
                            vel_post_predict,
                            &gap3_nhc_last_timestamp_s,
                            row.timestamp_s,
                            &gap3_nhc_has_anchor);
                    }

                    if (zupt_after_predict > zupt_before_predict) {
                        if (!gap3_zupt_has_anchor) {
                            gap3_set_observation_anchor(
                                gap3_zupt_anchor_pos,
                                gap3_zupt_anchor_vel,
                                pos_pre_predict,
                                vel_pre_predict,
                                &gap3_zupt_last_timestamp_s,
                                row.timestamp_s,
                                &gap3_zupt_has_anchor);
                        }

                        double pred_dpos_h = 0.0;
                        double pred_dvel_h = 0.0;
                        double pred_dt_s = 0.0;
                        gap3_compute_cycle_pred_accum(
                            gap3_zupt_anchor_pos,
                            gap3_zupt_anchor_vel,
                            pos_pre_predict,
                            vel_pre_predict,
                            gap3_zupt_last_timestamp_s,
                            row.timestamp_s,
                            &pred_dpos_h,
                            &pred_dvel_h,
                            &pred_dt_s);

                        (void)write_gap3_observation_audit_row(
                            gap3_observation_audit_fp,
                            "ZUPT",
                            row.timestamp_s,
                            true,
                            0,
                            pred_dpos_h,
                            pred_dvel_h,
                            pred_dt_s,
                            nullptr,
                            nullptr,
                            pos_pre_predict,
                            pos_post_predict,
                            vel_pre_predict,
                            vel_post_predict);

                        gap3_set_observation_anchor(
                            gap3_zupt_anchor_pos,
                            gap3_zupt_anchor_vel,
                            pos_post_predict,
                            vel_post_predict,
                            &gap3_zupt_last_timestamp_s,
                            row.timestamp_s,
                            &gap3_zupt_has_anchor);
                    }
                }
            }

            if ((h8_propagation_audit_fp != nullptr
                    || h9_tilt_audit_fp != nullptr
                    || h9a_gravity_alignment_audit_fp != nullptr
                    || h9b_attitude_propagation_audit_fp != nullptr
                    || h9d_gravity_subtraction_audit_fp != nullptr
                    || propagation_chain_audit_fp != nullptr)
                && filter_impl != nullptr) {
                InsEkfPredictAudit audit{};
                InsEkfAttitudePropAudit att_audit{};
                InsEkfFilter &ekf_audit = filter_impl->native();
                const bool has_predict_audit = ins_ekf_get_last_predict_audit(&ekf_audit, &audit);
                const bool has_att_audit =
                    ins_ekf_get_last_attitude_prop_audit(&ekf_audit, &att_audit);
                if (has_predict_audit) {
                    float roll_rad = 0.0f;
                    float pitch_rad = 0.0f;
                    float yaw_rad = 0.0f;
                    ins_ekf_get_attitude_rad(&ekf_audit, &roll_rad, &pitch_rad, &yaw_rad);
                    const ReplayConstraintDecision audit_decision = replay_evaluate_constraints(
                        row.timestamp_s,
                        last_gps_speed_mps,
                        accel_norm,
                        gyro_norm,
                        config);
                    const int constraint_mode = config.predict_only_mode
                        ? -1
                        : (audit_decision.zupt_armed
                            ? 0
                            : (audit_decision.nhc_armed ? 1 : 2));
                    const float roll_deg = roll_rad * kRadToDegF;
                    const float pitch_deg = pitch_rad * kRadToDegF;
                    const float yaw_deg = yaw_rad * kRadToDegF;

                    if (h8_propagation_audit_fp != nullptr) {
                        float vel_post_ned[3] = {0.0f, 0.0f, 0.0f};
                        ins_ekf_get_velocity_ned(&ekf_audit, vel_post_ned);
                        (void)write_h8_propagation_audit_row(
                            h8_propagation_audit_fp,
                            row.timestamp_s,
                            audit.dt_s,
                            row.accel,
                            aligned_accel,
                            audit,
                            vel_post_ned,
                            roll_deg,
                            pitch_deg,
                            yaw_deg,
                            last_gps_speed_mps,
                            constraint_mode);
                    }

                    if (h9_tilt_audit_fp != nullptr
                        && row.timestamp_s <= static_cast<double>(h9_tilt_audit_end_s(config))) {
                        (void)write_h9_tilt_audit_row(
                            h9_tilt_audit_fp,
                            row.timestamp_s,
                            audit,
                            roll_deg,
                            pitch_deg,
                            yaw_deg,
                            constraint_mode);
                    }

                    if (h9a_gravity_alignment_audit_fp != nullptr
                        && row.timestamp_s <= static_cast<double>(
                            h9a_gravity_alignment_audit_end_s(config))) {
                        (void)write_h9a_gravity_alignment_audit_row(
                            h9a_gravity_alignment_audit_fp,
                            row.timestamp_s,
                            aligned_accel,
                            audit,
                            roll_deg,
                            pitch_deg,
                            yaw_deg,
                            gravity_tilt_init.applied,
                            constraint_mode);
                    }

                    if (h9b_attitude_propagation_audit_fp != nullptr
                        && has_att_audit
                        && row.timestamp_s <= static_cast<double>(
                            h9b_attitude_propagation_audit_end_s(config))) {
                        (void)write_h9b_attitude_propagation_audit_row(
                            h9b_attitude_propagation_audit_fp,
                            row.timestamp_s,
                            aligned_accel,
                            audit,
                            att_audit,
                            gravity_tilt_init.applied,
                            constraint_mode,
                            &h9b_has_prev_gravity_rv,
                            h9b_prev_gravity_rv_rad);
                    }

                    if (h9d_gravity_subtraction_audit_fp != nullptr
                        && row.timestamp_s <= static_cast<double>(
                            h9d_gravity_subtraction_audit_end_s(config))) {
                        (void)write_h9d_gravity_subtraction_audit_row(
                            h9d_gravity_subtraction_audit_fp,
                            row.timestamp_s,
                            aligned_accel,
                            audit,
                            roll_deg,
                            pitch_deg,
                            yaw_deg,
                            last_gps_speed_mps,
                            gravity_tilt_init.applied,
                            constraint_mode);
                    }

                    if (propagation_chain_audit_fp != nullptr
                        && row.timestamp_s <= static_cast<double>(
                            propagation_chain_audit_end_s(config))) {
                        (void)write_propagation_chain_audit_row(
                            propagation_chain_audit_fp,
                            row.timestamp_s,
                            row.accel,
                            aligned_accel,
                            audit,
                            roll_deg,
                            pitch_deg,
                            yaw_deg,
                            last_gps_speed_mps,
                            gravity_tilt_init.applied,
                            constraint_mode);
                    }
                }
            }

            const NaviState state = filter->get_state();
            (void)write_output_state(output_fp, row.timestamp_s, state, "IMU");
        }

        if (config.progress_interval_rows > 0U
            && (result.rows_processed % config.progress_interval_rows) == 0U) {
            const NaviState state = filter->get_state();
            const float drift_m = has_last_gps_pos
                ? horizontal_drift_m(state, last_gps_pos_ned)
                : 0.0f;
            std::printf(
                "[t=%6.1f s] rows=%u imu=%u gps=%u drift=%.2f m\n",
                row.timestamp_s,
                result.rows_processed,
                result.imu_rows,
                result.gps_rows,
                drift_m);
        }
    }

    std::fclose(input_fp);
    std::fclose(output_fp);
    if (instrumentation_fp != nullptr) {
        std::fclose(instrumentation_fp);
    }
    if (gnss_audit_fp != nullptr) {
        std::fclose(gnss_audit_fp);
    }
    if (h3_diagnostics_fp != nullptr) {
        std::fclose(h3_diagnostics_fp);
    }
    if (consistency_fp != nullptr) {
        std::fclose(consistency_fp);
    }
    if (sync_audit_fp != nullptr) {
        std::fclose(sync_audit_fp);
    }
    if (h7_update_audit_fp != nullptr) {
        std::fclose(h7_update_audit_fp);
    }
    if (gap3_observation_audit_fp != nullptr) {
        std::fclose(gap3_observation_audit_fp);
    }
    if (gap3_gnss_nis_audit_fp != nullptr) {
        std::fclose(gap3_gnss_nis_audit_fp);
    }
    if (gap3_nhc_block_audit_fp != nullptr) {
        std::fclose(gap3_nhc_block_audit_fp);
    }
    if (gap3_gnss_k_block_json_fp != nullptr) {
        std::fclose(gap3_gnss_k_block_json_fp);
    }
    if (gap3_cov_propagation_audit_fp != nullptr) {
        std::fclose(gap3_cov_propagation_audit_fp);
    }
    if (gap3_cov_step_audit_fp != nullptr) {
        std::fclose(gap3_cov_step_audit_fp);
    }
    if (gap3_vel_source_audit_fp != nullptr) {
        std::fclose(gap3_vel_source_audit_fp);
    }
    if (gap3_imu_constraint_audit_fp != nullptr) {
        std::fclose(gap3_imu_constraint_audit_fp);
    }
    if (gap3_constraint_pipeline_audit_fp != nullptr) {
        std::fclose(gap3_constraint_pipeline_audit_fp);
    }
    if (h8_propagation_audit_fp != nullptr) {
        std::fclose(h8_propagation_audit_fp);
    }
    if (h9_tilt_audit_fp != nullptr) {
        std::fclose(h9_tilt_audit_fp);
    }
    if (h9a_gravity_alignment_audit_fp != nullptr) {
        std::fclose(h9a_gravity_alignment_audit_fp);
    }
    if (h9b_attitude_propagation_audit_fp != nullptr) {
        std::fclose(h9b_attitude_propagation_audit_fp);
    }
    if (h9d_gravity_subtraction_audit_fp != nullptr) {
        std::fclose(h9d_gravity_subtraction_audit_fp);
    }
    if (propagation_chain_audit_fp != nullptr) {
        std::fclose(propagation_chain_audit_fp);
    }

    if (filter_initialized && has_last_gps_pos) {
        const NaviState final_state = filter->get_state();
        result.final_drift_m = horizontal_drift_m(final_state, last_gps_pos_ned);
        result.last_gps_pos_n_m = static_cast<float>(last_gps_pos_ned[0]);
        result.last_gps_pos_e_m = static_cast<float>(last_gps_pos_ned[1]);
        result.last_gps_pos_d_m = static_cast<float>(last_gps_pos_ned[2]);
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(" REAL_RUN_REPLAY resumen\n");
    std::printf("  Filas procesadas: %u (IMU=%u, GPS=%u)\n",
        result.rows_processed,
        result.imu_rows,
        result.gps_rows);
    std::printf("  Duracion:         %.2f s\n", result.duration_s);
    std::printf("  Filtro activo:    %s\n", filter_initialized ? "si" : "no");
    if (filter_initialized) {
        std::printf(
            "  GNSS aceptadas:   %u | rechazadas: %u (%.1f%% aceptadas)\n",
            result.gnss_accept_count,
            result.gnss_reject_count,
            (result.gnss_accept_count + result.gnss_reject_count) > 0U
                ? (100.0f * static_cast<float>(result.gnss_accept_count)
                    / static_cast<float>(result.gnss_accept_count + result.gnss_reject_count))
                : 0.0f);
        if (config.yaw_init_mode == RealRunYawInitMode::H3_COV_RESET_GRACE) {
            std::printf(
                "  H3 aplicado:      %s",
                result.h3_applied ? "si" : "no");
            if (result.h3_applied) {
                std::printf(" | gracia hasta t=%.1f s", result.h3_grace_period_end_s);
            }
            std::printf("\n");
        }
        if (yaw_init_mode_uses_stable_heading(config.yaw_init_mode)) {
            std::printf(
                "  Yaw H2 aplicado:  %s",
                result.yaw_init_applied ? "si" : "no");
            if (result.yaw_init_applied) {
                std::printf(
                    " @ t=%.2f s | heading=%.2f deg",
                    result.yaw_init_applied_at_s,
                    result.yaw_init_heading_deg);
            }
            std::printf("\n");
        }
    }
    if (filter_initialized && has_last_gps_pos) {
        std::printf(
            "  Ultimo GPS NED:   (%.3f, %.3f, %.3f) m\n",
            result.last_gps_pos_n_m,
            result.last_gps_pos_e_m,
            result.last_gps_pos_d_m);
        std::printf("  Deriva final H:   %.3f m\n", result.final_drift_m);
        const NaviState final_state = filter->get_state();
        std::printf(
            "  Estado final NED: (%.3f, %.3f, %.3f) m\n",
            final_state.pos_ned[0],
            final_state.pos_ned[1],
            final_state.pos_ned[2]);
    }
    std::printf("----------------------------------------------------------------\n");

    if (out_result != nullptr) {
        *out_result = result;
    }

    return filter_initialized;
}
