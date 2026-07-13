#include "guidance.hpp"

#include "math_utils.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

static float deg_to_rad(float deg)
{
    return deg * (static_cast<float>(M_PI) / 180.0f);
}

static void latlon_to_local_ne_m(
    float ref_lat_deg,
    float ref_lon_deg,
    float lat_deg,
    float lon_deg,
    float *north_m,
    float *east_m)
{
    const float dlat_m = (lat_deg - ref_lat_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float dlon_raw = (lon_deg - ref_lon_deg) * NAVICORE_METERS_PER_DEG_LAT;
    const float mean_lat_rad = deg_to_rad((ref_lat_deg + lat_deg) * 0.5f);

    *north_m = dlat_m;
    *east_m = dlon_raw * cosf(mean_lat_rad);
}

static bool waypoint_buffer_at(
    const StaticWaypointBuffer *buffer,
    size_t index,
    Waypoint *out)
{
    if (buffer == NULL || out == NULL || index >= buffer->count) {
        return false;
    }

    *out = buffer->items[(buffer->head + index) % NAVICORE_MAX_WAYPOINTS];
    return true;
}

static float hypot2f(float north_m, float east_m)
{
    return sqrtf((north_m * north_m) + (east_m * east_m));
}

static PurePursuitOutput pure_pursuit_invalid_output(void)
{
    PurePursuitOutput out{};
    out.yaw_target_rad = 0.0f;
    out.waypoint_completed = false;
    out.valid = false;
    return out;
}

static bool circle_segment_lookahead(
    float p1_n,
    float p1_e,
    float p2_n,
    float p2_e,
    float radius_m,
    float *lookahead_n,
    float *lookahead_e)
{
    const float seg_n = p2_n - p1_n;
    const float seg_e = p2_e - p1_e;
    const float seg_len_sq = (seg_n * seg_n) + (seg_e * seg_e);

    if (seg_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        const float dist_p2 = hypot2f(p2_n, p2_e);
        if (dist_p2 <= NAVICORE_EPS_DISPLACEMENT_M) {
            return false;
        }

        const float scale = (dist_p2 <= radius_m) ? 1.0f : (radius_m / dist_p2);
        *lookahead_n = p2_n * scale;
        *lookahead_e = p2_e * scale;
        return true;
    }

    const float b = 2.0f * ((p1_n * seg_n) + (p1_e * seg_e));
    const float c = (p1_n * p1_n) + (p1_e * p1_e) - (radius_m * radius_m);
    const float disc = (b * b) - (4.0f * seg_len_sq * c);

    float t_best = -1.0f;

    if (disc >= 0.0f) {
        const float sqrt_disc = sqrtf(disc);
        const float inv_2a = 0.5f / seg_len_sq;
        const float t0 = (-b - sqrt_disc) * inv_2a;
        const float t1 = (-b + sqrt_disc) * inv_2a;

        if (t0 >= 0.0f && t0 <= 1.0f) {
            t_best = t0;
        }
        if (t1 >= 0.0f && t1 <= 1.0f && t1 > t_best) {
            t_best = t1;
        }
    }

    if (t_best >= 0.0f) {
        *lookahead_n = p1_n + (t_best * seg_n);
        *lookahead_e = p1_e + (t_best * seg_e);
        return true;
    }

    const float dist_p2 = hypot2f(p2_n, p2_e);
    if (dist_p2 <= radius_m) {
        *lookahead_n = p2_n;
        *lookahead_e = p2_e;
        return true;
    }

    if (dist_p2 > NAVICORE_EPS_DISPLACEMENT_M) {
        const float scale = radius_m / dist_p2;
        *lookahead_n = p2_n * scale;
        *lookahead_e = p2_e * scale;
        return true;
    }

    return false;
}

PurePursuitGuidance::PurePursuitGuidance()
    : look_ahead_distance_m_(kDefaultLookAheadM)
{
}

PurePursuitGuidance::PurePursuitGuidance(float look_ahead_distance_m)
    : look_ahead_distance_m_(look_ahead_distance_m)
{
}

void PurePursuitGuidance::set_look_ahead_distance(float look_ahead_distance_m)
{
    look_ahead_distance_m_ = look_ahead_distance_m;
}

float PurePursuitGuidance::get_look_ahead_distance() const
{
    return look_ahead_distance_m_;
}

PurePursuitOutput PurePursuitGuidance::compute(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index) const
{
    if (look_ahead_distance_m_ <= NAVICORE_EPS_DISPLACEMENT_M) {
        return pure_pursuit_invalid_output();
    }

    if (route.count == 0U || active_waypoint_index >= route.count) {
        return pure_pursuit_invalid_output();
    }

    Waypoint active_wp{};
    if (!waypoint_buffer_at(&route, active_waypoint_index, &active_wp)) {
        return pure_pursuit_invalid_output();
    }

    const float ref_lat = nav_state.position.x;
    const float ref_lon = nav_state.position.y;

    float active_n_m = 0.0f;
    float active_e_m = 0.0f;
    latlon_to_local_ne_m(
        ref_lat,
        ref_lon,
        active_wp.position.x,
        active_wp.position.y,
        &active_n_m,
        &active_e_m);

    const float dist_active_m = hypot2f(active_n_m, active_e_m);
    const float acceptance_m = static_cast<float>(active_wp.arrival_radius_m);

    PurePursuitOutput out{};
    out.waypoint_completed = (dist_active_m <= acceptance_m);
    out.valid = true;

    float lookahead_n_m = active_n_m;
    float lookahead_e_m = active_e_m;

    const bool has_next_leg = (active_waypoint_index + 1U) < route.count;
    if (has_next_leg) {
        Waypoint next_wp{};
        if (!waypoint_buffer_at(&route, active_waypoint_index + 1U, &next_wp)) {
            return pure_pursuit_invalid_output();
        }

        float next_n_m = 0.0f;
        float next_e_m = 0.0f;
        latlon_to_local_ne_m(
            ref_lat,
            ref_lon,
            next_wp.position.x,
            next_wp.position.y,
            &next_n_m,
            &next_e_m);

        if (!circle_segment_lookahead(
                active_n_m,
                active_e_m,
                next_n_m,
                next_e_m,
                look_ahead_distance_m_,
                &lookahead_n_m,
                &lookahead_e_m)) {
            return pure_pursuit_invalid_output();
        }
    } else {
        const float dist_m = hypot2f(active_n_m, active_e_m);
        if (dist_m <= NAVICORE_EPS_DISPLACEMENT_M) {
            out.yaw_target_rad = deg_to_rad(nav_state.heading_deg);
            return out;
        }

        const float scale = (dist_m <= look_ahead_distance_m_)
            ? 1.0f
            : (look_ahead_distance_m_ / dist_m);
        lookahead_n_m = active_n_m * scale;
        lookahead_e_m = active_e_m * scale;
    }

    const float lookahead_len_sq =
        (lookahead_n_m * lookahead_n_m) + (lookahead_e_m * lookahead_e_m);
    if (lookahead_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        out.yaw_target_rad = deg_to_rad(nav_state.heading_deg);
        return out;
    }

    out.yaw_target_rad = atan2f(lookahead_e_m, lookahead_n_m);
    return out;
}

static GuidanceErrors guidance_errors_degenerate(
    float pos_n_m,
    float pos_e_m,
    float track_len_m)
{
    GuidanceErrors errors{};

    const float pos_len_sq = (pos_n_m * pos_n_m) + (pos_e_m * pos_e_m);
    if (pos_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        errors.cross_track_m = 0.0f;
        errors.along_track_m = track_len_m;
        return errors;
    }

    errors.cross_track_m = sqrtf(pos_len_sq);
    errors.along_track_m = track_len_m;
    return errors;
}

GuidanceErrors guidance_compute_errors(
    Vector3D position,
    Waypoint origin,
    Waypoint destination)
{
    const float ref_lat = origin.position.x;
    const float ref_lon = origin.position.y;

    float track_n_m = 0.0f;
    float track_e_m = 0.0f;
    latlon_to_local_ne_m(
        ref_lat,
        ref_lon,
        destination.position.x,
        destination.position.y,
        &track_n_m,
        &track_e_m);

    float pos_n_m = 0.0f;
    float pos_e_m = 0.0f;
    latlon_to_local_ne_m(
        ref_lat,
        ref_lon,
        position.x,
        position.y,
        &pos_n_m,
        &pos_e_m);

    const float track_len_sq = (track_n_m * track_n_m) + (track_e_m * track_e_m);
    if (track_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        return guidance_errors_degenerate(pos_n_m, pos_e_m, 0.0f);
    }

    const float track_len_m = sqrtf(track_len_sq);
    const float inv_track_len_sq = 1.0f / track_len_sq;

    const float along_from_origin_m =
        ((pos_n_m * track_n_m) + (pos_e_m * track_e_m)) * inv_track_len_sq * track_len_m;

    const float proj_n_m = along_from_origin_m * (track_n_m / track_len_m);
    const float proj_e_m = along_from_origin_m * (track_e_m / track_len_m);

    const float cross_n_m = pos_n_m - proj_n_m;
    const float cross_e_m = pos_e_m - proj_e_m;
    const float cross_len_sq = (cross_n_m * cross_n_m) + (cross_e_m * cross_e_m);

    GuidanceErrors errors{};
    errors.along_track_m = track_len_m - along_from_origin_m;

    if (cross_len_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        errors.cross_track_m = 0.0f;
        return errors;
    }

    const float cross_sign = (track_n_m * pos_e_m) - (track_e_m * pos_n_m);
    const float cross_mag_m = sqrtf(cross_len_sq);
    errors.cross_track_m = (cross_sign >= 0.0f) ? cross_mag_m : -cross_mag_m;

    return errors;
}
