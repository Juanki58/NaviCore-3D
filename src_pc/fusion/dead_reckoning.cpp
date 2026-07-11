#include "dead_reckoning.h"

#include "nav_math.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
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
}

bool dead_reckoning_update_imu(DeadReckoningFilter *filter, const ImuSample *imu)
{
    if (filter == NULL || imu == NULL || !imu->valid) {
        return false;
    }

    const uint32_t prev_ms = filter->state.timestamp_ms;
    if (prev_ms == 0U) {
        filter->state.timestamp_ms = imu->timestamp_ms;
        return true;
    }

    const float dt_s = (float)(imu->timestamp_ms - prev_ms) * 0.001f;
    if (dt_s <= 0.0f) {
        return false;
    }

    const float yaw_rate_radps = imu->gyro_radps[2];
    const float forward_accel = imu->accel_mps2[0];

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

bool dead_reckoning_update_gps(DeadReckoningFilter *filter, const GpsSample *gps)
{
    if (filter == NULL || gps == NULL || !gps->fix_valid) {
        return false;
    }

    const bool blending = filter->state.confidence.gps_trusted;
    const float w = filter->gps_weight;

    if (blending) {
        filter->state.position.x = (filter->state.position.x * (1.0f - w)) + (gps->position.x * w);
        filter->state.position.y = (filter->state.position.y * (1.0f - w)) + (gps->position.y * w);
        filter->state.position.z = (filter->state.position.z * (1.0f - w)) + (gps->position.z * w);
        filter->state.mode = NAV_MODE_HYBRID;
    } else {
        filter->state.position = gps->position;
        filter->state.mode = NAV_MODE_GPS;
    }

    filter->state.heading_deg = navstate_normalize_heading(gps->course_deg);
    filter->state.velocity = velocity_from_speed_heading(
        gps->speed_mps,
        filter->state.heading_deg,
        filter->state.velocity.z);

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
