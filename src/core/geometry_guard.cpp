#include "geometry_guard.hpp"

#include <math.h>

#ifndef M_PI_F
#define M_PI_F 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

static float geometry_guard_deg_to_rad(float deg)
{
    return deg * (M_PI_F / 180.0f);
}

static uint8_t geometry_guard_clamp_health_score(int32_t value)
{
    if (value < (int32_t)DIAG_HEALTH_SCORE_MIN) {
        return DIAG_HEALTH_SCORE_MIN;
    }
    if (value > (int32_t)DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return (uint8_t)value;
}

static NavHealthMode geometry_guard_mode_from_score(uint8_t health_score)
{
    if (health_score <= DIAG_HEALTH_SCORE_CRITICAL_MAX) {
        return HEALTH_CRITICAL;
    }
    if (health_score < DIAG_HEALTH_SCORE_NOMINAL_MIN) {
        return HEALTH_DEGRADED;
    }
    return HEALTH_NOMINAL;
}

static void geometry_guard_inject_discontinuity(
    SystemHealthMonitor *monitor,
    float step_distance_m)
{
    if (monitor == NULL) {
        return;
    }

    monitor->last_geometry_error = GEOMETRY_ERROR_DISCONTINUITY;
    monitor->last_geometry_step_m = step_distance_m;

    const int32_t penalized =
        (int32_t)monitor->health_score - (int32_t)GEOMETRY_GUARD_HEALTH_PENALTY;
    monitor->health_score = geometry_guard_clamp_health_score(penalized);
    monitor->mode = geometry_guard_mode_from_score(monitor->health_score);
    monitor->update_count++;
}

static bool geometry_guard_get_last_waypoint(
    const StaticWaypointBuffer *buffer,
    Waypoint *last_out)
{
    if (buffer == NULL || last_out == NULL || buffer->count == 0U) {
        return false;
    }

    const size_t last_index = (buffer->head + buffer->count - 1U) % NAVICORE_MAX_WAYPOINTS;
    *last_out = buffer->items[last_index];
    return true;
}

float geometry_guard_distance_flat_m(float lat_a, float lon_a, float lat_b, float lon_b)
{
    const float north_m = (lat_b - lat_a) * NAVICORE_METERS_PER_DEG_LAT;
    const float lat_rad = geometry_guard_deg_to_rad((lat_a + lat_b) * 0.5f);
    const float cos_lat = cosf(lat_rad);
    const float east_m = (fabsf(cos_lat) > 1.0e-6f)
        ? ((lon_b - lon_a) * NAVICORE_METERS_PER_DEG_LAT * cos_lat)
        : 0.0f;

    return sqrtf((north_m * north_m) + (east_m * east_m));
}

bool geometry_guard_validate_next(
    const StaticWaypointBuffer *buffer,
    float next_x,
    float next_y,
    SystemHealthMonitor *monitor)
{
    if (buffer == NULL || waypoint_buffer_is_empty(buffer)) {
        if (monitor != NULL) {
            monitor->last_geometry_error = GEOMETRY_ERROR_NONE;
        }
        return true;
    }

    if (isnan(next_x) || isnan(next_y) || isinf(next_x) || isinf(next_y)) {
        return false;
    }

    Waypoint last_wp{};
    if (!geometry_guard_get_last_waypoint(buffer, &last_wp)) {
        return true;
    }

    const float step_distance_m = geometry_guard_distance_flat_m(
        last_wp.position.x,
        last_wp.position.y,
        next_x,
        next_y);

    if (monitor != NULL) {
        monitor->last_geometry_step_m = step_distance_m;
    }

    if (step_distance_m > NAVICORE_GEOM_MAX_STEP_M) {
        geometry_guard_inject_discontinuity(monitor, step_distance_m);
        return false;
    }

    if (monitor != NULL) {
        monitor->last_geometry_error = GEOMETRY_ERROR_NONE;
    }

    return true;
}
