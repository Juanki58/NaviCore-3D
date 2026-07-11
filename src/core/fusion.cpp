#include "fusion.hpp"

#include "math_utils.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

#ifndef NAVICORE_GPS_INNOVATION_GATE_M
#define NAVICORE_GPS_INNOVATION_GATE_M 15.0f
#endif

static float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

static float rad_to_deg(float rad)
{
    return rad * (180.0f / M_PI);
}

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float horizontal_speed_from_velocity(float vx, float vy)
{
    const float speed_sq = (vx * vx) + (vy * vy);
    if (speed_sq <= NAVICORE_EPS_SPEED_SQ) {
        return 0.0f;
    }
    return sqrtf(speed_sq);
}

static Vector3D velocity_from_speed_heading(float speed_mps, float heading_deg, float vertical_mps)
{
    if (speed_mps <= NAVICORE_EPS_SPEED_MPS) {
        return vector3d_make(0.0f, 0.0f, vertical_mps);
    }

    const float heading_rad = deg_to_rad(heading_deg);
    return vector3d_make(
        speed_mps * cosf(heading_rad),
        speed_mps * sinf(heading_rad),
        vertical_mps);
}

static void dead_reckoning_apply_position_delta(NavState *state, float north_m, float east_m)
{
    if (fabsf(north_m) <= NAVICORE_EPS_DISPLACEMENT_M &&
        fabsf(east_m) <= NAVICORE_EPS_DISPLACEMENT_M) {
        return;
    }

    const float lat_rad = deg_to_rad(state->position.x);
    state->position.x += north_m / NAVICORE_METERS_PER_DEG_LAT;

    const float cos_lat = cosf(lat_rad);
    if (fabsf(cos_lat) > 1.0e-6f) {
        state->position.y += east_m / (NAVICORE_METERS_PER_DEG_LAT * cos_lat);
    }
}

static float dead_reckoning_confidence_from_fix_age(uint32_t fix_age_ms)
{
    const float age_s = (float)fix_age_ms * 0.001f;
    const float quality = 0.75f - (age_s * 0.05f);
    return clampf(quality, 0.15f, 0.75f);
}

static void dead_reckoning_set_dead_reckoning_confidence(DeadReckoningFilter *filter)
{
    const uint32_t fix_age_ms = (filter->last_gps_timestamp_ms == 0U)
        ? 0U
        : filter->state.timestamp_ms - filter->last_gps_timestamp_ms;

    filter->state.confidence = nav_confidence_make(
        false,
        filter->state.confidence.satellites,
        fix_age_ms,
        dead_reckoning_confidence_from_fix_age(fix_age_ms));
}

static float dead_reckoning_horizontal_distance_m(
    float lat_a_deg,
    float lon_a_deg,
    float lat_b_deg,
    float lon_b_deg)
{
    const float dlat_m = (lat_b_deg - lat_a_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float lat_rad = deg_to_rad((lat_a_deg + lat_b_deg) * 0.5f);
    const float cos_lat = cosf(lat_rad);
    const float dlon_m = (fabsf(cos_lat) > 1.0e-6f)
        ? ((lon_b_deg - lon_a_deg) * NAVICORE_METERS_PER_DEG_LAT * cos_lat)
        : 0.0f;

    return sqrtf((dlat_m * dlat_m) + (dlon_m * dlon_m));
}

static void dead_reckoning_enter_gps_contingency(DeadReckoningFilter *filter, const GpsSample *gps)
{
    filter->state.confidence.satellites = gps->fix_valid ? gps->satellites : 0U;
    filter->state.confidence.gps_trusted = false;
    dead_reckoning_set_dead_reckoning_confidence(filter);
    filter->state.mode = NAV_MODE_DEAD_RECKONING;

    if (gps->timestamp_ms >= filter->state.timestamp_ms) {
        filter->state.timestamp_ms = gps->timestamp_ms;
    }
}

static bool dead_reckoning_gps_passes_innovation_gate(const DeadReckoningFilter *filter, const GpsSample *gps)
{
    if (filter->state.mode == NAV_MODE_INITIALIZING || filter->last_gps_timestamp_ms == 0U) {
        return true;
    }

    const float innovation_m = dead_reckoning_horizontal_distance_m(
        filter->state.position.x,
        filter->state.position.y,
        gps->position.x,
        gps->position.y);

    return innovation_m <= NAVICORE_GPS_INNOVATION_GATE_M;
}

static float dead_reckoning_heading_error_deg(float target_deg, float current_deg)
{
    float error_deg = target_deg - current_deg;

    while (error_deg > 180.0f) {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f) {
        error_deg += 360.0f;
    }

    return error_deg;
}

static void dead_reckoning_update_gps_noise_scale(
    DeadReckoningFilter *filter,
    const SystemHealthMonitor *health_monitor)
{
    filter->gps_noise_covariance_scale = 1.0f;

    if (health_monitor == NULL) {
        return;
    }

    if (health_monitor->update_count == 0U) {
        return;
    }

    if (health_monitor->health_score < NAVICORE_EKF_HEALTH_TRUST_THRESHOLD) {
        const float score = (float)health_monitor->health_score;
        const float safe_score = (score < 1.0f) ? 1.0f : score;

        filter->gps_noise_covariance_scale =
            (float)NAVICORE_EKF_HEALTH_TRUST_THRESHOLD / safe_score;
    }
}

static float dead_reckoning_ekf_kalman_gain(float prior_variance_m2, float measurement_variance_m2)
{
    const float denominator = prior_variance_m2 + measurement_variance_m2;

    if (denominator <= 1.0e-6f) {
        return 0.0f;
    }

    return prior_variance_m2 / denominator;
}

static void dead_reckoning_ekf_apply_gps_measurement(
    DeadReckoningFilter *filter,
    const GpsSample *gps,
    const SystemHealthMonitor *health_monitor)
{
    const Vector3D prior_position = filter->state.position;

    dead_reckoning_update_gps_noise_scale(filter, health_monitor);

    const float measurement_variance_m2 =
        filter->gps_measurement_variance_m2 * filter->gps_noise_covariance_scale;
    const float kalman_gain = dead_reckoning_ekf_kalman_gain(
        filter->position_prior_variance_m2,
        measurement_variance_m2);

    const float lat_rad = deg_to_rad(prior_position.x);
    const float cos_lat = cosf(lat_rad);
    const float innovation_north_m =
        (gps->position.x - prior_position.x) * NAVICORE_METERS_PER_DEG_LAT;
    const float innovation_east_m = (fabsf(cos_lat) > 1.0e-6f)
        ? ((gps->position.y - prior_position.y) * NAVICORE_METERS_PER_DEG_LAT * cos_lat)
        : 0.0f;
    const float innovation_alt_m = gps->position.z - prior_position.z;

    dead_reckoning_apply_position_delta(
        &filter->state,
        kalman_gain * innovation_north_m,
        kalman_gain * innovation_east_m);
    filter->state.position.z = prior_position.z + (kalman_gain * innovation_alt_m);

    filter->position_prior_variance_m2 *= (1.0f - kalman_gain);

    const float heading_error_deg = dead_reckoning_heading_error_deg(
        gps->course_deg,
        filter->state.heading_deg);
    filter->state.heading_deg = navstate_normalize_heading(
        filter->state.heading_deg + (kalman_gain * heading_error_deg));

    const float prior_speed_mps = horizontal_speed_from_velocity(
        filter->state.velocity.x,
        filter->state.velocity.y);
    const float blended_speed_mps = prior_speed_mps +
        (kalman_gain * (gps->speed_mps - prior_speed_mps));

    filter->state.velocity = velocity_from_speed_heading(
        clampf(blended_speed_mps, 0.0f, 40.0f),
        filter->state.heading_deg,
        filter->state.velocity.z);
    filter->state.mode = NAV_MODE_HYBRID;
}

static void dead_reckoning_grow_position_prior_variance(DeadReckoningFilter *filter, float dt_s)
{
    filter->position_prior_variance_m2 +=
        NAVICORE_POSITION_PROCESS_NOISE_M2_PER_S * dt_s;

    if (filter->position_prior_variance_m2 > NAVICORE_POSITION_PRIOR_VARIANCE_MAX_M2) {
        filter->position_prior_variance_m2 = NAVICORE_POSITION_PRIOR_VARIANCE_MAX_M2;
    }
}

static void dead_reckoning_finalize_bias_calibration(DeadReckoningFilter *filter)
{
    if (filter->calibration_samples > 0U) {
        const float inv_count = 1.0f / (float)filter->calibration_samples;

        filter->bias_accel_x = filter->accel_x_sum * inv_count;
        filter->bias_gyro_z = filter->gyro_z_sum * inv_count;
    }

    filter->is_calibrated = true;
}

static void dead_reckoning_bias_calibration_step(DeadReckoningFilter *filter, const ImuSample *imu)
{
    filter->calibration_ticks++;

    if (filter->calibration_ticks <= NAVICORE_BIAS_CALIBRATION_TICKS) {
        const float speed_mps = horizontal_speed_from_velocity(
            filter->state.velocity.x,
            filter->state.velocity.y);

        if (speed_mps <= NAVICORE_EPS_SPEED_MPS) {
            filter->accel_x_sum += imu->accel_mps2[0];
            filter->gyro_z_sum += imu->gyro_radps[2];
            filter->calibration_samples++;
        }

        if (filter->calibration_ticks == NAVICORE_BIAS_CALIBRATION_TICKS) {
            dead_reckoning_finalize_bias_calibration(filter);
        }
    }
}

static void dead_reckoning_apply_imu_bias_correction(
    const ImuSample *imu,
    const DeadReckoningFilter *filter,
    float *forward_accel_out,
    float *yaw_rate_radps_out)
{
    *forward_accel_out = imu->accel_mps2[0];
    *yaw_rate_radps_out = imu->gyro_radps[2];

    if (filter->calibration_ticks > NAVICORE_BIAS_CALIBRATION_TICKS) {
        *forward_accel_out -= filter->bias_accel_x;
        *yaw_rate_radps_out -= filter->bias_gyro_z;
    }
}

void dead_reckoning_init(DeadReckoningFilter *filter, Vector3D initial_position, NavDomain domain)
{
    if (filter == NULL) {
        return;
    }

    filter->state = navstate_zero(domain);
    filter->state.position = initial_position;
    filter->state.mode = NAV_MODE_INITIALIZING;
    filter->state.confidence = nav_confidence_make(false, 0U, 0U, 0.0f);
    filter->imu_weight = 0.35f;
    filter->gps_weight = 0.65f;
    filter->last_gps_timestamp_ms = 0U;
    filter->last_pressure_pa = 0.0f;
    filter->last_pressure_timestamp_ms = 0U;
    filter->pressure_sample_valid = false;
    filter->quality = NAVICORE_QUALITY_NONE;
    filter->imu_predicted_speed_mps = 0.0f;
    filter->imu_speed_prediction_valid = false;
    filter->calibration_ticks = 0U;
    filter->is_calibrated = false;
    filter->bias_accel_x = 0.0f;
    filter->bias_gyro_z = 0.0f;
    filter->accel_x_sum = 0.0f;
    filter->gyro_z_sum = 0.0f;
    filter->calibration_samples = 0U;
    filter->gps_noise_covariance_scale = 1.0f;
    filter->gps_measurement_variance_m2 = NAVICORE_GPS_MEASUREMENT_VARIANCE_M2;
    filter->position_prior_variance_m2 = NAVICORE_GPS_MEASUREMENT_VARIANCE_M2 * 1.5f;
}

bool dead_reckoning_update_imu(DeadReckoningFilter *filter, const ImuSample *imu)
{
    if (filter == NULL || imu == NULL || !imu->valid) {
        return false;
    }

    dead_reckoning_bias_calibration_step(filter, imu);

    const uint32_t prev_ms = filter->state.timestamp_ms;
    if (prev_ms == 0U) {
        filter->state.timestamp_ms = imu->timestamp_ms;
        return true;
    }

    const float dt_s = (float)(imu->timestamp_ms - prev_ms) * 0.001f;
    if (dt_s <= 0.0f) {
        return false;
    }

    dead_reckoning_grow_position_prior_variance(filter, dt_s);

    float forward_accel = 0.0f;
    float yaw_rate_radps = 0.0f;
    dead_reckoning_apply_imu_bias_correction(imu, filter, &forward_accel, &yaw_rate_radps);

    if (fabsf(yaw_rate_radps) > NAVICORE_EPS_GYRO_RADPS) {
        filter->state.heading_deg = navstate_normalize_heading(
            filter->state.heading_deg + (rad_to_deg(yaw_rate_radps) * dt_s));
    }

    float horizontal_speed_mps = horizontal_speed_from_velocity(
        filter->state.velocity.x,
        filter->state.velocity.y);

    if (fabsf(forward_accel) > NAVICORE_EPS_ACCEL_MPS2 || horizontal_speed_mps > NAVICORE_EPS_SPEED_MPS) {
        horizontal_speed_mps = clampf(
            horizontal_speed_mps + (forward_accel * dt_s),
            0.0f,
            40.0f);
    }

    filter->imu_predicted_speed_mps = horizontal_speed_mps;
    filter->imu_speed_prediction_valid = true;

    float vertical_rate_mps = filter->state.velocity.z;
    if (filter->state.domain == NAVICORE_DOMAIN_AIR &&
        fabsf(imu->accel_mps2[2]) > NAVICORE_EPS_ACCEL_MPS2) {
        vertical_rate_mps += imu->accel_mps2[2] * dt_s;
    }

    filter->state.velocity = velocity_from_speed_heading(
        horizontal_speed_mps,
        filter->state.heading_deg,
        vertical_rate_mps);

    const float north_m = filter->state.velocity.x * dt_s;
    const float east_m = filter->state.velocity.y * dt_s;
    dead_reckoning_apply_position_delta(&filter->state, north_m, east_m);

    if (filter->state.domain == NAVICORE_DOMAIN_AIR &&
        fabsf(filter->state.velocity.z) > NAVICORE_EPS_SPEED_MPS) {
        filter->state.position.z += filter->state.velocity.z * dt_s;
    }

    filter->state.timestamp_ms = imu->timestamp_ms;
    filter->state.mode = NAV_MODE_DEAD_RECKONING;
    dead_reckoning_set_dead_reckoning_confidence(filter);
    return true;
}

bool dead_reckoning_update_gps(
    DeadReckoningFilter *filter,
    const GpsSample *gps,
    const SystemHealthMonitor *health_monitor)
{
    if (filter == NULL || gps == NULL) {
        return false;
    }

    if (!gps->fix_valid) {
        dead_reckoning_enter_gps_contingency(filter, gps);
        return true;
    }

    if (!dead_reckoning_gps_passes_innovation_gate(filter, gps)) {
        dead_reckoning_enter_gps_contingency(filter, gps);
        return true;
    }

    const bool blending = filter->state.confidence.gps_trusted;

    if (blending) {
        dead_reckoning_ekf_apply_gps_measurement(filter, gps, health_monitor);
    } else {
        filter->state.position = gps->position;
        filter->state.mode = NAV_MODE_GPS;
        filter->position_prior_variance_m2 = filter->gps_measurement_variance_m2 * 1.5f;
        filter->gps_noise_covariance_scale = 1.0f;

        filter->state.heading_deg = navstate_normalize_heading(gps->course_deg);
        filter->state.velocity = velocity_from_speed_heading(
            gps->speed_mps,
            filter->state.heading_deg,
            filter->state.velocity.z);
    }

    filter->imu_predicted_speed_mps = gps->speed_mps;
    filter->imu_speed_prediction_valid = true;

    filter->state.timestamp_ms = gps->timestamp_ms;
    filter->last_gps_timestamp_ms = gps->timestamp_ms;

    const float quality = clampf(0.55f + ((float)gps->satellites * 0.03f), 0.55f, 0.95f);
    filter->state.confidence = nav_confidence_make(true, gps->satellites, 0U, quality);
    return true;
}

bool dead_reckoning_update_pressure(DeadReckoningFilter *filter, const PressureSample *pressure, float surface_pressure_pa)
{
    (void)surface_pressure_pa;

    if (filter == NULL || pressure == NULL || !pressure->valid || filter->state.domain != NAVICORE_DOMAIN_SEA) {
        return false;
    }

    if (filter->pressure_sample_valid) {
        const uint32_t pressure_dt_ms = pressure->timestamp_ms - filter->last_pressure_timestamp_ms;
        if (pressure_dt_ms > 0U) {
            const float dt_s = (float)pressure_dt_ms * 0.001f;
            filter->state.velocity.z = (pressure->pressure_pa - filter->last_pressure_pa) / dt_s;
        }
    }

    filter->last_pressure_pa = pressure->pressure_pa;
    filter->last_pressure_timestamp_ms = pressure->timestamp_ms;
    filter->pressure_sample_valid = true;

    filter->state.position.z = pressure->pressure_pa;

    if (pressure->timestamp_ms >= filter->state.timestamp_ms) {
        filter->state.timestamp_ms = pressure->timestamp_ms;
    }

    if (filter->state.mode == NAV_MODE_GPS || filter->state.mode == NAV_MODE_HYBRID) {
        filter->state.mode = NAV_MODE_HYBRID;
    } else {
        filter->state.mode = NAV_MODE_DEAD_RECKONING;
        dead_reckoning_set_dead_reckoning_confidence(filter);
    }

    return true;
}

bool dead_reckoning_update_wheel_odometry(DeadReckoningFilter *filter, float speed_mps, bool reverse, uint32_t timestamp_ms)
{
    if (filter == NULL) {
        return false;
    }

    const float ground_speed_mps = fabsf(speed_mps);

    if (filter->imu_speed_prediction_valid) {
        const float speed_delta = fabsf(ground_speed_mps - filter->imu_predicted_speed_mps);
        if (speed_delta > NAVICORE_ODOM_FAULT_THRESHOLD) {
            filter->quality |= NAVICORE_QUALITY_ODOM_FAULT;

            if (timestamp_ms >= filter->state.timestamp_ms) {
                filter->state.timestamp_ms = timestamp_ms;
            }

            if (filter->state.mode != NAV_MODE_GPS && filter->state.mode != NAV_MODE_HYBRID) {
                filter->state.mode = NAV_MODE_DEAD_RECKONING;
                dead_reckoning_set_dead_reckoning_confidence(filter);
            }

            filter->state.confidence.estimate_quality = clampf(
                filter->state.confidence.estimate_quality - 0.10f,
                0.15f,
                0.85f);
            return true;
        }
    }

    filter->quality &= (NavQuality)(~NAVICORE_QUALITY_ODOM_FAULT);

    const float heading_deg = reverse
        ? navstate_normalize_heading(filter->state.heading_deg + 180.0f)
        : filter->state.heading_deg;

    filter->state.velocity = velocity_from_speed_heading(
        ground_speed_mps,
        heading_deg,
        filter->state.velocity.z);

    if (timestamp_ms >= filter->state.timestamp_ms) {
        filter->state.timestamp_ms = timestamp_ms;
    }

    if (filter->state.mode == NAV_MODE_GPS || filter->state.mode == NAV_MODE_HYBRID) {
        filter->state.mode = NAV_MODE_HYBRID;
    } else {
        filter->state.mode = NAV_MODE_DEAD_RECKONING;
        dead_reckoning_set_dead_reckoning_confidence(filter);
        filter->state.confidence.estimate_quality = clampf(
            filter->state.confidence.estimate_quality + 0.12f,
            0.15f,
            0.85f);
    }

    return true;
}

NavQuality dead_reckoning_get_quality(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return NAVICORE_QUALITY_NONE;
    }

    return filter->quality;
}

bool dead_reckoning_has_odom_fault(const DeadReckoningFilter *filter)
{
    if (filter == NULL) {
        return false;
    }

    return (filter->quality & NAVICORE_QUALITY_ODOM_FAULT) != 0U;
}
