#include "inertial_replay.hpp"

#include <cctype>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr float kGravityMps2 = 9.80665f;
constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);
constexpr uint32_t kStaticCalibrationMs = 5000U;
constexpr size_t kMaxLineBytes = 8192U;
constexpr size_t kMaxReplayRows = 5000000U;

struct ReplayPaceState {
    std::chrono::steady_clock::time_point epoch;
    bool epoch_valid;
};

ReplayPaceState g_replay_pace{};

struct StaticCalibrationAccumulator {
    float accel_sum[3];
    float gyro_sum[3];
    size_t sample_count;
};

void static_calibration_reset(StaticCalibrationAccumulator *acc)
{
    if (acc == NULL) {
        return;
    }

    for (int axis = 0; axis < 3; ++axis) {
        acc->accel_sum[axis] = 0.0f;
        acc->gyro_sum[axis] = 0.0f;
    }
    acc->sample_count = 0U;
}

void static_calibration_accumulate(
    StaticCalibrationAccumulator *acc,
    const ImuSample &imu)
{
    if (acc == NULL) {
        return;
    }

    for (int axis = 0; axis < 3; ++axis) {
        acc->accel_sum[axis] += imu.accel_mps2[axis];
        acc->gyro_sum[axis] += imu.gyro_radps[axis];
    }
    ++acc->sample_count;
}

bool static_calibration_finalize(
    const StaticCalibrationAccumulator &acc,
    float out_accel_bias[3],
    float out_gyro_bias[3],
    float out_gravity_body[3])
{
    if (acc.sample_count == 0U || out_accel_bias == NULL || out_gyro_bias == NULL
        || out_gravity_body == NULL) {
        return false;
    }

    const float inv_count = 1.0f / static_cast<float>(acc.sample_count);
    float mean_accel[3] = {0.0f, 0.0f, 0.0f};

    for (int axis = 0; axis < 3; ++axis) {
        mean_accel[axis] = acc.accel_sum[axis] * inv_count;
        out_gyro_bias[axis] = acc.gyro_sum[axis] * inv_count;
    }

    const float gravity_mag = std::sqrt(
        (mean_accel[0] * mean_accel[0])
        + (mean_accel[1] * mean_accel[1])
        + (mean_accel[2] * mean_accel[2]));

    if (gravity_mag > 1.0e-3f) {
        const float scale = kGravityMps2 / gravity_mag;
        for (int axis = 0; axis < 3; ++axis) {
            out_gravity_body[axis] = mean_accel[axis] * scale;
        }
    } else {
        out_gravity_body[0] = 0.0f;
        out_gravity_body[1] = 0.0f;
        out_gravity_body[2] = kGravityMps2;
    }

    for (int axis = 0; axis < 3; ++axis) {
        out_accel_bias[axis] = mean_accel[axis] - out_gravity_body[axis];
    }

    return true;
}

void static_calibration_apply_to_log(
    InertialReplayLog *log,
    const std::vector<size_t> &post_calib_row_indices,
    const float accel_bias[3],
    const float gyro_bias[3])
{
    if (log == NULL || log->rows == NULL || accel_bias == NULL || gyro_bias == NULL) {
        return;
    }

    for (size_t idx = 0U; idx < post_calib_row_indices.size(); ++idx) {
        const size_t row_index = post_calib_row_indices[idx];
        if (row_index >= log->row_count) {
            continue;
        }

        InertialReplayRow *row = &log->rows[row_index];
        for (int axis = 0; axis < 3; ++axis) {
            row->imu.accel_mps2[axis] -= accel_bias[axis];
            row->imu.gyro_radps[axis] -= gyro_bias[axis];
        }
    }
}

enum ColumnId : int16_t {
    COL_UNKNOWN = -1,
    COL_TIME = 0,
    COL_ACC_X,
    COL_ACC_Y,
    COL_ACC_Z,
    COL_GYRO_X,
    COL_GYRO_Y,
    COL_GYRO_Z,
    COL_LAT,
    COL_LON,
    COL_ALT,
    COL_POS_X,
    COL_POS_Y,
    COL_POS_Z,
    COL_FIX_VALID,
    COL_SATELLITES,
    COL_SPEED,
    COL_COURSE,
    COL_COUNT,
};

struct ParsedHeader {
    int16_t indices[COL_COUNT];
    bool accel_in_g;
    bool gyro_in_degps;
    bool time_in_us;
    bool time_in_s;
};

bool is_nan_value(float v)
{
    return std::isnan(v) || std::isinf(v);
}

void trim_inplace(std::string *s)
{
    if (s == NULL) {
        return;
    }

    size_t start = 0U;
    while (start < s->size() && std::isspace(static_cast<unsigned char>((*s)[start])) != 0) {
        ++start;
    }

    size_t end = s->size();
    while (end > start && std::isspace(static_cast<unsigned char>((*s)[end - 1U])) != 0) {
        --end;
    }

    *s = s->substr(start, end - start);
}

std::string to_lower_ascii(const std::string &input)
{
    std::string out = input;
    for (size_t i = 0U; i < out.size(); ++i) {
        out[i] = static_cast<char>(std::tolower(static_cast<unsigned char>(out[i])));
    }
    return out;
}

bool header_token_contains(const std::string &token, const char *needle)
{
    return to_lower_ascii(token).find(needle) != std::string::npos;
}

ColumnId classify_column(const std::string &raw_token, ParsedHeader *header)
{
    const std::string token = to_lower_ascii(raw_token);

    if (token == "timestamp" || token == "time_us" || token == "timeus") {
        header->time_in_us = true;
        return COL_TIME;
    }
    if (token == "time_ms" || token == "t_ms" || token == "timestamp_ms" || token == "time") {
        return COL_TIME;
    }
    if (token == "time_s" || token == "t_s" || token == "t") {
        header->time_in_s = true;
        return COL_TIME;
    }

    if (token == "acc_x" || token == "accel_x" || token == "ax") {
        if (header_token_contains(raw_token, "_g") || header_token_contains(raw_token, "g")) {
            header->accel_in_g = true;
        }
        return COL_ACC_X;
    }
    if (token == "acc_y" || token == "accel_y" || token == "ay") {
        if (header_token_contains(raw_token, "_g") || header_token_contains(raw_token, "g")) {
            header->accel_in_g = true;
        }
        return COL_ACC_Y;
    }
    if (token == "acc_z" || token == "accel_z" || token == "az") {
        if (header_token_contains(raw_token, "_g") || header_token_contains(raw_token, "g")) {
            header->accel_in_g = true;
        }
        return COL_ACC_Z;
    }

    if (token == "gyro_x" || token == "gx" || token == "gyr_x") {
        if (header_token_contains(raw_token, "deg")) {
            header->gyro_in_degps = true;
        }
        return COL_GYRO_X;
    }
    if (token == "gyro_y" || token == "gy" || token == "gyr_y") {
        if (header_token_contains(raw_token, "deg")) {
            header->gyro_in_degps = true;
        }
        return COL_GYRO_Y;
    }
    if (token == "gyro_z" || token == "gz" || token == "gyr_z") {
        if (header_token_contains(raw_token, "deg")) {
            header->gyro_in_degps = true;
        }
        return COL_GYRO_Z;
    }

    if (token == "lat" || token == "latitude") {
        return COL_LAT;
    }
    if (token == "lon" || token == "longitude" || token == "lng") {
        return COL_LON;
    }
    if (token == "alt" || token == "altitude" || token == "height") {
        return COL_ALT;
    }
    if (token == "pos_x" || token == "latitude_deg") {
        return COL_POS_X;
    }
    if (token == "pos_y" || token == "longitude_deg") {
        return COL_POS_Y;
    }
    if (token == "pos_z" || token == "alt_m") {
        return COL_POS_Z;
    }

    if (token == "fix_valid" || token == "gps_fix" || token == "gnss_fix" || token == "fix") {
        return COL_FIX_VALID;
    }
    if (token == "satellites" || token == "sats" || token == "sat_count") {
        return COL_SATELLITES;
    }
    if (token == "speed_mps" || token == "speed" || token == "gps_speed") {
        return COL_SPEED;
    }
    if (token == "course_deg" || token == "course" || token == "heading_deg") {
        return COL_COURSE;
    }

    return COL_UNKNOWN;
}

void split_csv_line(const char *line, std::vector<std::string> *fields_out)
{
    if (line == NULL || fields_out == NULL) {
        return;
    }

    fields_out->clear();
    std::string field;
    for (const char *p = line; *p != '\0'; ++p) {
        if (*p == ',') {
            trim_inplace(&field);
            fields_out->push_back(field);
            field.clear();
        } else {
            field.push_back(*p);
        }
    }
    trim_inplace(&field);
    fields_out->push_back(field);
}

bool parse_header_line(const char *line, ParsedHeader *header)
{
    if (line == NULL || header == NULL) {
        return false;
    }

    for (int16_t i = 0; i < COL_COUNT; ++i) {
        header->indices[i] = COL_UNKNOWN;
    }
    header->accel_in_g = false;
    header->gyro_in_degps = false;
    header->time_in_us = false;
    header->time_in_s = false;

    std::vector<std::string> fields;
    split_csv_line(line, &fields);

    for (size_t i = 0U; i < fields.size(); ++i) {
        const ColumnId col = classify_column(fields[i], header);
        if (col != COL_UNKNOWN && col < COL_COUNT) {
            header->indices[col] = static_cast<int16_t>(i);
        }
    }

    const bool has_time = header->indices[COL_TIME] >= 0;
    const bool has_imu =
        header->indices[COL_ACC_X] >= 0
        && header->indices[COL_ACC_Y] >= 0
        && header->indices[COL_ACC_Z] >= 0
        && header->indices[COL_GYRO_X] >= 0
        && header->indices[COL_GYRO_Y] >= 0
        && header->indices[COL_GYRO_Z] >= 0;

    return has_time && has_imu;
}

bool field_is_empty(const std::string &field)
{
    if (field.empty()) {
        return true;
    }

    for (size_t i = 0U; i < field.size(); ++i) {
        const char c = field[i];
        if (c != ' ' && c != '\t' && c != '\r') {
            return false;
        }
    }
    return true;
}

float parse_float_field(const std::vector<std::string> &fields, int16_t index, bool *present)
{
    if (present != NULL) {
        *present = false;
    }

    if (index < 0 || static_cast<size_t>(index) >= fields.size()) {
        return 0.0f;
    }

    const std::string &field = fields[static_cast<size_t>(index)];
    if (field_is_empty(field)) {
        return 0.0f;
    }

    char *end = NULL;
    const float value = std::strtof(field.c_str(), &end);
    if (end == field.c_str()) {
        return 0.0f;
    }

    if (present != NULL) {
        *present = true;
    }
    return value;
}

bool parse_bool_field(const std::vector<std::string> &fields, int16_t index, bool *present)
{
    if (present != NULL) {
        *present = false;
    }

    if (index < 0 || static_cast<size_t>(index) >= fields.size()) {
        return false;
    }

    const std::string token = to_lower_ascii(fields[static_cast<size_t>(index)]);
    if (field_is_empty(token)) {
        return false;
    }

    if (present != NULL) {
        *present = true;
    }

    if (token == "1" || token == "true" || token == "yes" || token == "ok") {
        return true;
    }
    if (token == "0" || token == "false" || token == "no") {
        return false;
    }

    const float numeric = std::strtof(token.c_str(), NULL);
    return numeric >= 0.5f;
}

bool parse_time_ms_u64(const ParsedHeader &header, const std::vector<std::string> &fields, uint64_t *out_ms)
{
    if (out_ms == NULL) {
        return false;
    }

    bool present = false;
    const float raw = parse_float_field(fields, header.indices[COL_TIME], &present);
    if (!present || raw < 0.0f) {
        return false;
    }

    double ms_value = static_cast<double>(raw);
    if (header.time_in_us) {
        ms_value = static_cast<double>(raw) / 1000.0;
    } else if (header.time_in_s) {
        ms_value = static_cast<double>(raw) * 1000.0;
    }

    if (ms_value < 0.0 || ms_value > static_cast<double>(UINT64_MAX)) {
        return false;
    }

    *out_ms = static_cast<uint64_t>(ms_value);
    return true;
}

uint32_t clamp_u32_ms(uint64_t ms_value)
{
    if (ms_value > static_cast<uint64_t>(UINT32_MAX)) {
        return UINT32_MAX;
    }
    return static_cast<uint32_t>(ms_value);
}

bool parse_row(
    const ParsedHeader &header,
    const std::vector<std::string> &fields,
    uint32_t time_ms,
    InertialReplayRow *row_out)
{
    if (row_out == NULL) {
        return false;
    }

    bool acc_present[3] = {false, false, false};
    bool gyro_present[3] = {false, false, false};

    const int16_t acc_cols[3] = {
        header.indices[COL_ACC_X],
        header.indices[COL_ACC_Y],
        header.indices[COL_ACC_Z],
    };
    const int16_t gyro_cols[3] = {
        header.indices[COL_GYRO_X],
        header.indices[COL_GYRO_Y],
        header.indices[COL_GYRO_Z],
    };

    for (int axis = 0; axis < 3; ++axis) {
        row_out->imu.accel_mps2[axis] =
            parse_float_field(fields, acc_cols[axis], &acc_present[axis]);
        row_out->imu.gyro_radps[axis] =
            parse_float_field(fields, gyro_cols[axis], &gyro_present[axis]);
    }

    if (!acc_present[0] || !acc_present[1] || !acc_present[2]
        || !gyro_present[0] || !gyro_present[1] || !gyro_present[2]) {
        return false;
    }

    if (header.accel_in_g) {
        for (int axis = 0; axis < 3; ++axis) {
            row_out->imu.accel_mps2[axis] *= kGravityMps2;
        }
    } else {
        const float az = std::fabs(row_out->imu.accel_mps2[2]);
        const float ax = std::fabs(row_out->imu.accel_mps2[0]);
        const float ay = std::fabs(row_out->imu.accel_mps2[1]);
        if (az <= 1.5f && ax <= 2.0f && ay <= 2.0f) {
            for (int axis = 0; axis < 3; ++axis) {
                row_out->imu.accel_mps2[axis] *= kGravityMps2;
            }
        }
    }

    if (header.gyro_in_degps) {
        for (int axis = 0; axis < 3; ++axis) {
            row_out->imu.gyro_radps[axis] *= kDegToRad;
        }
    } else {
        const float gz = std::fabs(row_out->imu.gyro_radps[2]);
        const float gx = std::fabs(row_out->imu.gyro_radps[0]);
        const float gy = std::fabs(row_out->imu.gyro_radps[1]);
        const float gyro_max = (gz > gx) ? ((gz > gy) ? gz : gy) : ((gx > gy) ? gx : gy);
        if (gyro_max > 3.0f) {
            for (int axis = 0; axis < 3; ++axis) {
                row_out->imu.gyro_radps[axis] *= kDegToRad;
            }
        }
    }

    row_out->imu.timestamp_ms = time_ms;
    row_out->imu.valid = true;
    row_out->imu.mag_ut[0] = 0.0f;
    row_out->imu.mag_ut[1] = 0.0f;
    row_out->imu.mag_ut[2] = 0.0f;

    bool lat_present = false;
    bool lon_present = false;
    bool alt_present = false;

    float lat = 0.0f;
    float lon = 0.0f;
    float alt = 0.0f;

    if (header.indices[COL_LAT] >= 0 || header.indices[COL_LON] >= 0 || header.indices[COL_ALT] >= 0) {
        lat = parse_float_field(fields, header.indices[COL_LAT], &lat_present);
        lon = parse_float_field(fields, header.indices[COL_LON], &lon_present);
        alt = parse_float_field(fields, header.indices[COL_ALT], &alt_present);
    } else {
        lat = parse_float_field(fields, header.indices[COL_POS_X], &lat_present);
        lon = parse_float_field(fields, header.indices[COL_POS_Y], &lon_present);
        alt = parse_float_field(fields, header.indices[COL_POS_Z], &alt_present);
    }

    row_out->gnss_valid = lat_present && lon_present && alt_present
        && !is_nan_value(lat) && !is_nan_value(lon) && !is_nan_value(alt);

    row_out->gps.position = vector3d_make(lat, lon, alt);
    row_out->gps.timestamp_ms = time_ms;
    row_out->gps.speed_mps = parse_float_field(fields, header.indices[COL_SPEED], NULL);
    row_out->gps.course_deg = parse_float_field(fields, header.indices[COL_COURSE], NULL);
    {
        const float sat_count = parse_float_field(fields, header.indices[COL_SATELLITES], NULL);
        if (sat_count <= 0.0f) {
            row_out->gps.satellites = 0U;
        } else if (sat_count >= 255.0f) {
            row_out->gps.satellites = 255U;
        } else {
            row_out->gps.satellites = static_cast<uint8_t>(sat_count);
        }
    }

    bool fix_present = false;
    const bool fix_valid = parse_bool_field(fields, header.indices[COL_FIX_VALID], &fix_present);
    if (fix_present) {
        row_out->gps.fix_valid = fix_valid && row_out->gnss_valid;
        row_out->gnss_valid = row_out->gps.fix_valid;
    } else {
        row_out->gps.fix_valid = row_out->gnss_valid;
    }

    if (row_out->gps.satellites == 0U && row_out->gnss_valid) {
        row_out->gps.satellites = 8U;
    }

    row_out->time_ms = time_ms;
    row_out->imu_valid = true;
    return true;
}

bool replay_rows_reserve(InertialReplayLog *log, size_t extra)
{
    if (log == NULL) {
        return false;
    }

    const size_t needed = log->row_count + extra;
    if (needed <= log->row_capacity) {
        return true;
    }

    if (needed > kMaxReplayRows) {
        return false;
    }

    size_t new_capacity = (log->row_capacity == 0U) ? 256U : log->row_capacity;
    while (new_capacity < needed) {
        if (new_capacity > (kMaxReplayRows / 2U)) {
            new_capacity = kMaxReplayRows;
            break;
        }
        new_capacity *= 2U;
    }
    if (new_capacity < needed) {
        return false;
    }

    InertialReplayRow *new_rows = static_cast<InertialReplayRow *>(
        std::realloc(log->rows, new_capacity * sizeof(InertialReplayRow)));
    if (new_rows == NULL) {
        return false;
    }

    log->rows = new_rows;
    log->row_capacity = new_capacity;
    return true;
}

bool replay_rows_push(InertialReplayLog *log, const InertialReplayRow &row)
{
    if (!replay_rows_reserve(log, 1U)) {
        return false;
    }

    log->rows[log->row_count] = row;
    ++log->row_count;
    if (row.time_ms > log->duration_ms) {
        log->duration_ms = row.time_ms;
    }
    return true;
}

void strip_utf8_bom(char *line)
{
    if (line == NULL) {
        return;
    }

    if (std::strncmp(line, "\xEF\xBB\xBF", 3) == 0) {
        std::memmove(line, line + 3, std::strlen(line + 3) + 1U);
    }
}

} // namespace

bool inertial_replay_load(InertialReplayLog *log, const char *csv_path)
{
    if (log == NULL || csv_path == NULL || csv_path[0] == '\0') {
        return false;
    }

    std::memset(log, 0, sizeof(*log));
    std::snprintf(log->source_path, sizeof(log->source_path), "%s", csv_path);

    FILE *file = std::fopen(csv_path, "r");
    if (file == NULL) {
        std::printf("REPLAY: no se pudo abrir '%s'\n", csv_path);
        return false;
    }

    char line[kMaxLineBytes];
    if (std::fgets(line, sizeof(line), file) == NULL) {
        std::printf("REPLAY: CSV vacio '%s'\n", csv_path);
        std::fclose(file);
        return false;
    }
    strip_utf8_bom(line);

    ParsedHeader header{};
    if (!parse_header_line(line, &header)) {
        std::printf(
            "REPLAY: cabecera invalida en '%s' (requiere tiempo + acc_x/y/z + gyro_x/y/z)\n",
            csv_path);
        std::fclose(file);
        return false;
    }

    log->accel_in_g = header.accel_in_g;
    log->gyro_in_degps = header.gyro_in_degps;

    uint64_t first_time_ms = 0U;
    bool first_time_set = false;
    StaticCalibrationAccumulator calib_acc{};
    static_calibration_reset(&calib_acc);
    std::vector<size_t> post_calib_row_indices;

    while (std::fgets(line, sizeof(line), file) != NULL) {
        if (line[0] == '\n' || line[0] == '\r' || line[0] == '#') {
            continue;
        }

        std::vector<std::string> fields;
        split_csv_line(line, &fields);
        if (fields.empty()) {
            continue;
        }

        uint64_t raw_time_ms = 0U;
        if (!parse_time_ms_u64(header, fields, &raw_time_ms)) {
            continue;
        }

        if (!first_time_set) {
            first_time_ms = raw_time_ms;
            first_time_set = true;
        }

        uint32_t time_ms = clamp_u32_ms(raw_time_ms);
        if (raw_time_ms >= first_time_ms) {
            time_ms = clamp_u32_ms(raw_time_ms - first_time_ms);
            log->time_is_relative = true;
        }

        InertialReplayRow row{};
        if (!parse_row(header, fields, time_ms, &row)) {
            continue;
        }

        if (time_ms <= kStaticCalibrationMs) {
            static_calibration_accumulate(&calib_acc, row.imu);
        }

        if (!replay_rows_push(log, row)) {
            std::printf("REPLAY: memoria insuficiente al cargar '%s'\n", csv_path);
            inertial_replay_free(log);
            std::fclose(file);
            return false;
        }

        if (time_ms > kStaticCalibrationMs) {
            post_calib_row_indices.push_back(log->row_count - 1U);
        }
    }

    std::fclose(file);

    if (log->row_count == 0U) {
        std::printf("REPLAY: sin filas IMU validas en '%s'\n", csv_path);
        inertial_replay_free(log);
        return false;
    }

    float accel_bias[3] = {0.0f, 0.0f, 0.0f};
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    float gravity_body[3] = {0.0f, 0.0f, kGravityMps2};

    if (static_calibration_finalize(calib_acc, accel_bias, gyro_bias, gravity_body)) {
        static_calibration_apply_to_log(log, post_calib_row_indices, accel_bias, gyro_bias);
        std::printf(
            "REPLAY: calibracion estatica %zu muestras (t_rel<=%u ms)\n",
            calib_acc.sample_count,
            kStaticCalibrationMs);
        std::printf(
            "REPLAY:   gravedad cuerpo=(%.4f, %.4f, %.4f) m/s2\n",
            gravity_body[0],
            gravity_body[1],
            gravity_body[2]);
        std::printf(
            "REPLAY:   bias_accel=(%.4f, %.4f, %.4f) m/s2 | bias_gyro=(%.6f, %.6f, %.6f) rad/s\n",
            accel_bias[0],
            accel_bias[1],
            accel_bias[2],
            gyro_bias[0],
            gyro_bias[1],
            gyro_bias[2]);
    } else {
        std::printf(
            "REPLAY: sin muestras en ventana de calibracion (t_rel<=%u ms)\n",
            kStaticCalibrationMs);
    }

    std::printf(
        "REPLAY: cargadas %zu muestras | duracion=%u ms | accel=%s | gyro=%s\n",
        log->row_count,
        log->duration_ms,
        log->accel_in_g ? "G->m/s2" : "m/s2(auto)",
        log->gyro_in_degps ? "deg/s->rad/s" : "rad/s(auto)");

    return true;
}

void inertial_replay_free(InertialReplayLog *log)
{
    if (log == NULL) {
        return;
    }

    std::free(log->rows);
    log->rows = NULL;
    log->row_count = 0U;
    log->row_capacity = 0U;
    log->duration_ms = 0U;
}

uint32_t inertial_replay_duration_ms(const InertialReplayLog *log)
{
    if (log == NULL) {
        return 0U;
    }
    return log->duration_ms;
}

size_t inertial_replay_row_count(const InertialReplayLog *log)
{
    if (log == NULL) {
        return 0U;
    }
    return log->row_count;
}

void inertial_replay_pace_reset(void)
{
    g_replay_pace.epoch_valid = false;
}

void inertial_replay_pace_until(uint32_t sim_time_ms)
{
    if (sim_time_ms == 0U) {
        g_replay_pace.epoch = std::chrono::steady_clock::now();
        g_replay_pace.epoch_valid = true;
        return;
    }

    if (!g_replay_pace.epoch_valid) {
        return;
    }

    const auto target = g_replay_pace.epoch
        + std::chrono::milliseconds(static_cast<int64_t>(sim_time_ms));
    const auto now = std::chrono::steady_clock::now();
    if (target > now) {
        std::this_thread::sleep_for(target - now);
    }
}

bool inertial_replay_sample_at(
    const InertialReplayLog *log,
    uint32_t sim_time_ms,
    ImuSample *imu_out,
    GpsSample *gps_out,
    bool *has_imu_sample,
    bool *has_gnss_sample)
{
    if (has_imu_sample != NULL) {
        *has_imu_sample = false;
    }
    if (has_gnss_sample != NULL) {
        *has_gnss_sample = false;
    }

    if (log == NULL || log->row_count == 0U) {
        return false;
    }

    if (sim_time_ms > log->duration_ms) {
        return false;
    }

    size_t exact_index = log->row_count;
    size_t hold_index = log->row_count;

    for (size_t i = 0U; i < log->row_count; ++i) {
        const uint32_t row_time = log->rows[i].time_ms;
        if (row_time == sim_time_ms) {
            exact_index = i;
            break;
        }
        if (row_time < sim_time_ms) {
            hold_index = i;
        } else if (row_time > sim_time_ms) {
            break;
        }
    }

    const InertialReplayRow *selected = NULL;
    bool gnss_from_exact_row = false;

    if (exact_index < log->row_count) {
        selected = &log->rows[exact_index];
        gnss_from_exact_row = true;
    } else if (hold_index < log->row_count) {
        selected = &log->rows[hold_index];
        gnss_from_exact_row = false;
    } else {
        return sim_time_ms <= log->duration_ms;
    }

    if (imu_out != NULL) {
        *imu_out = selected->imu;
        imu_out->timestamp_ms = sim_time_ms;
        imu_out->valid = selected->imu_valid;
    }

    if (gps_out != NULL) {
        *gps_out = selected->gps;
        if (gnss_from_exact_row) {
            gps_out->timestamp_ms = sim_time_ms;
        }
        if (!gnss_from_exact_row) {
            gps_out->fix_valid = false;
        }
    }

    if (has_imu_sample != NULL) {
        *has_imu_sample = selected->imu_valid;
    }

    if (has_gnss_sample != NULL) {
        *has_gnss_sample = gnss_from_exact_row && selected->gnss_valid && selected->gps.fix_valid;
    }

    return true;
}
