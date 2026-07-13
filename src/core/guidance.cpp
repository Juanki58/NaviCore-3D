#include "guidance.hpp"

#include "math_utils.hpp"

#include <math.h>
#include <string.h>

#ifndef NAVICORE_GUIDANCE_PI_F
#define NAVICORE_GUIDANCE_PI_F 3.14159265358979323846f
#endif

#ifndef NAVICORE_METERS_PER_DEG_LAT
#define NAVICORE_METERS_PER_DEG_LAT 111132.954f
#endif

/*
 * Proyeccion local del vehiculo sobre el tramo — solo en .cpp (no expuesta).
 */
typedef struct {
    float pos_n;
    float pos_e;
    float pos_u;
    float t_param;
    float along_from_origin_m;
    float along_remaining_m;
    float cross_track_m;
    float cross_track_signed_m;
    float cross_track_sq_m2;
    float to_dest_n;
    float to_dest_e;
    float to_dest_u;
    float to_dest_sq_m2;
} GuidanceLegProjection;

static float deg_to_rad(float deg)
{
    return deg * (NAVICORE_GUIDANCE_PI_F / 180.0f);
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

static void guidance_position_to_local_neu(
    const Waypoint *origin,
    Vector3D position,
    float *north_m,
    float *east_m,
    float *up_m)
{
    if (origin == NULL || north_m == NULL || east_m == NULL || up_m == NULL) {
        return;
    }

    latlon_to_local_ne_m(
        origin->position.x,
        origin->position.y,
        position.x,
        position.y,
        north_m,
        east_m);
    *up_m = position.z - origin->position.z;
}

static void guidance_waypoint_delta_neu(
    const Waypoint *origin,
    const Waypoint *target,
    float *north_m,
    float *east_m,
    float *up_m)
{
    if (origin == NULL || target == NULL || north_m == NULL || east_m == NULL || up_m == NULL) {
        return;
    }

    latlon_to_local_ne_m(
        origin->position.x,
        origin->position.y,
        target->position.x,
        target->position.y,
        north_m,
        east_m);
    *up_m = target->position.z - origin->position.z;
}

static bool guidance_prepare_leg_geom(
    const Waypoint *origin,
    const Waypoint *destination,
    GuidanceLegGeom *geom)
{
    if (origin == NULL || destination == NULL || geom == NULL) {
        return false;
    }

    guidance_waypoint_delta_neu(
        origin,
        destination,
        &geom->seg_n,
        &geom->seg_e,
        &geom->seg_u);

    const float seg_len_sq_m2 =
        (geom->seg_n * geom->seg_n)
        + (geom->seg_e * geom->seg_e)
        + (geom->seg_u * geom->seg_u);

    if (seg_len_sq_m2 <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        geom->valid = false;
        return false;
    }

    geom->inv_seg_len_sq = 1.0f / seg_len_sq_m2;
    geom->seg_len_m = sqrtf(seg_len_sq_m2);
    geom->valid = true;
    return true;
}

/*
 * Proyeccion lineal r sobre d (sin producto vectorial):
 *   t = (r·d) / |d|^2
 *   r_perp = r - t*d
 *   |e_xt| = |r_perp|  — un solo sqrtf si hace falta magnitud
 */
static void guidance_project_onto_leg(
    const GuidanceLegGeom *geom,
    float pos_n,
    float pos_e,
    float pos_u,
    float dest_n,
    float dest_e,
    float dest_u,
    GuidanceLegProjection *proj)
{
    if (geom == NULL || proj == NULL || !geom->valid) {
        return;
    }

    proj->pos_n = pos_n;
    proj->pos_e = pos_e;
    proj->pos_u = pos_u;

    const float dot_rd =
        (pos_n * geom->seg_n) + (pos_e * geom->seg_e) + (pos_u * geom->seg_u);
    const float t = dot_rd * geom->inv_seg_len_sq;
    proj->t_param = t;

    const float proj_n = t * geom->seg_n;
    const float proj_e = t * geom->seg_e;
    const float proj_u = t * geom->seg_u;

    const float cross_n = pos_n - proj_n;
    const float cross_e = pos_e - proj_e;
    const float cross_u = pos_u - proj_u;

    proj->cross_track_sq_m2 =
        (cross_n * cross_n) + (cross_e * cross_e) + (cross_u * cross_u);

    if (proj->cross_track_sq_m2 <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        proj->cross_track_m = 0.0f;
        proj->cross_track_signed_m = 0.0f;
    } else {
        proj->cross_track_m = sqrtf(proj->cross_track_sq_m2);
        const float horiz_sign = (geom->seg_n * pos_e) - (geom->seg_e * pos_n);
        proj->cross_track_signed_m =
            (horiz_sign >= 0.0f) ? proj->cross_track_m : -proj->cross_track_m;
    }

    proj->along_from_origin_m = dot_rd / geom->seg_len_m;
    proj->along_remaining_m = geom->seg_len_m - proj->along_from_origin_m;
    if (proj->along_remaining_m < 0.0f) {
        proj->along_remaining_m = 0.0f;
    }

    proj->to_dest_n = dest_n - pos_n;
    proj->to_dest_e = dest_e - pos_e;
    proj->to_dest_u = dest_u - pos_u;
    proj->to_dest_sq_m2 =
        (proj->to_dest_n * proj->to_dest_n)
        + (proj->to_dest_e * proj->to_dest_e)
        + (proj->to_dest_u * proj->to_dest_u);
}

static GuidanceOutput guidance_invalid_output(void)
{
    GuidanceOutput out{};
    out.valid = false;
    return out;
}

GuidanceProfile guidance_profile_default(void)
{
    GuidanceProfile profile{};
    profile.cruise_speed_mps = NAVICORE_GUIDANCE_CRUISE_SPEED_MPS;
    profile.arrival_speed_mps = NAVICORE_GUIDANCE_ARRIVAL_SPEED_MPS;
    profile.slowdown_along_track_m = NAVICORE_GUIDANCE_SLOWDOWN_ALONG_M;
    profile.max_climb_mps = NAVICORE_GUIDANCE_MAX_CLIMB_MPS;
    profile.max_descent_mps = NAVICORE_GUIDANCE_MAX_DESCENT_MPS;
    profile.climb_time_constant_s = NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S;
    profile.cross_track_slowdown_m = NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M;
    profile.min_speed_factor = NAVICORE_GUIDANCE_MIN_SPEED_FACTOR;
    return profile;
}

bool guidance_get_leg_waypoints(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index,
    Waypoint *origin_out,
    Waypoint *destination_out)
{
    if (route == NULL || origin_out == NULL || destination_out == NULL
        || route->count == 0U || active_waypoint_index >= route->count) {
        return false;
    }

    if (!waypoint_buffer_at(route, active_waypoint_index, origin_out)) {
        return false;
    }

    const size_t dest_index = active_waypoint_index + 1U;
    if (dest_index < route->count) {
        return waypoint_buffer_at(route, dest_index, destination_out);
    }

    *destination_out = *origin_out;
    return true;
}

GuidanceErrors guidance_compute_errors_3d(
    Vector3D position,
    Waypoint origin,
    Waypoint destination)
{
    GuidanceErrors errors{};

    GuidanceLegGeom geom{};
    if (!guidance_prepare_leg_geom(&origin, &destination, &geom)) {
        float pos_n = 0.0f;
        float pos_e = 0.0f;
        float pos_u = 0.0f;
        guidance_position_to_local_neu(&origin, position, &pos_n, &pos_e, &pos_u);
        const float pos_len_sq = (pos_n * pos_n) + (pos_e * pos_e) + (pos_u * pos_u);
        if (pos_len_sq > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
            errors.cross_track_m = sqrtf(pos_len_sq);
            errors.cross_track_signed_m = errors.cross_track_m;
        }
        return errors;
    }

    float pos_n = 0.0f;
    float pos_e = 0.0f;
    float pos_u = 0.0f;
    guidance_position_to_local_neu(&origin, position, &pos_n, &pos_e, &pos_u);

    GuidanceLegProjection proj{};
    guidance_project_onto_leg(&geom, pos_n, pos_e, pos_u, geom.seg_n, geom.seg_e, geom.seg_u, &proj);

    errors.cross_track_m = proj.cross_track_m;
    errors.cross_track_signed_m = proj.cross_track_signed_m;
    errors.along_track_m = proj.along_remaining_m;
    return errors;
}

GuidanceErrors guidance_compute_leg_errors(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index,
    Vector3D position)
{
    GuidanceErrors errors{};

    Waypoint origin{};
    Waypoint destination{};
    if (!guidance_get_leg_waypoints(route, active_waypoint_index, &origin, &destination)) {
        return errors;
    }

    return guidance_compute_errors_3d(position, origin, destination);
}

static float guidance_desired_speed_mps(
    float along_track_m,
    float cross_track_sq_m2,
    const GuidanceProfile &profile)
{
    const float cruise = profile.cruise_speed_mps;
    const float arrival = profile.arrival_speed_mps;
    const float slowdown = profile.slowdown_along_track_m;
    const float cross_slow = profile.cross_track_slowdown_m;
    const float min_factor = profile.min_speed_factor;

    float along_factor = 1.0f;
    if (slowdown > NAVICORE_EPS_DISPLACEMENT_M && along_track_m < slowdown) {
        along_factor = clampf(along_track_m / slowdown, 0.0f, 1.0f);
    }

    float cross_factor = 1.0f;
    const float cross_slow_sq = cross_slow * cross_slow;
    if (cross_slow_sq > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        cross_factor = clampf(1.0f - (cross_track_sq_m2 / cross_slow_sq), min_factor, 1.0f);
    }

    const float blend_factor = (along_factor < cross_factor) ? along_factor : cross_factor;
    return arrival + ((cruise - arrival) * blend_factor);
}

static float guidance_desired_climb_mps(
    float current_alt_m,
    float origin_alt_m,
    float destination_alt_m,
    float along_remaining_m,
    float leg_length_m,
    const GuidanceProfile &profile)
{
    if (leg_length_m <= NAVICORE_EPS_DISPLACEMENT_M) {
        return 0.0f;
    }

    const float progress = clampf(1.0f - (along_remaining_m / leg_length_m), 0.0f, 1.0f);
    const float target_alt_m = origin_alt_m + (progress * (destination_alt_m - origin_alt_m));
    const float alt_error_m = target_alt_m - current_alt_m;

    if (fabsf(alt_error_m) <= NAVICORE_EPS_DISPLACEMENT_M) {
        return 0.0f;
    }

    const float time_constant_s = (profile.climb_time_constant_s > 0.01f)
        ? profile.climb_time_constant_s
        : NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S;

    return clampf(
        alt_error_m / time_constant_s,
        -profile.max_descent_mps,
        profile.max_climb_mps);
}

static void guidance_fill_commands_from_projection(
    const GuidanceLegGeom *geom,
    const GuidanceLegProjection *proj,
    float look_ahead_distance_m,
    float current_heading_deg,
    float origin_alt_m,
    float destination_alt_m,
    float current_alt_m,
    const GuidanceProfile &profile,
    GuidanceOutput *output)
{
    if (geom == NULL || proj == NULL || output == NULL || !geom->valid) {
        return;
    }

    float t_lookahead = proj->t_param + (look_ahead_distance_m / geom->seg_len_m);
    t_lookahead = clampf(t_lookahead, 0.0f, 1.0f);

    const float lookahead_n = t_lookahead * geom->seg_n;
    const float lookahead_e = t_lookahead * geom->seg_e;

    const float track_heading_rad = atan2f(geom->seg_e, geom->seg_n);
    const float los_correction_rad = atan2f(
        -proj->cross_track_signed_m,
        look_ahead_distance_m);
    output->commands.desired_heading = track_heading_rad + los_correction_rad;

    const float lookahead_len_sq = (lookahead_n * lookahead_n) + (lookahead_e * lookahead_e);
    if (lookahead_len_sq > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        const float pp_heading_rad = atan2f(lookahead_e, lookahead_n);
        const float cross_abs_m = proj->cross_track_m;
        const float blend = clampf(1.0f - (cross_abs_m / look_ahead_distance_m), 0.0f, 1.0f);
        output->commands.desired_heading =
            (blend * pp_heading_rad) + ((1.0f - blend) * output->commands.desired_heading);
    } else {
        output->commands.desired_heading = deg_to_rad(current_heading_deg);
    }

    output->commands.desired_speed = guidance_desired_speed_mps(
        proj->along_remaining_m,
        proj->cross_track_sq_m2,
        profile);
    output->commands.desired_climb = guidance_desired_climb_mps(
        current_alt_m,
        origin_alt_m,
        destination_alt_m,
        proj->along_remaining_m,
        geom->seg_len_m,
        profile);
}

GuidanceOutput guidance_compute_output(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index,
    const GuidanceProfile &profile,
    float look_ahead_distance_m)
{
    if (look_ahead_distance_m <= NAVICORE_EPS_DISPLACEMENT_M
        || route.count == 0U || active_waypoint_index >= route.count) {
        return guidance_invalid_output();
    }

    Waypoint origin{};
    Waypoint destination{};
    if (!guidance_get_leg_waypoints(&route, active_waypoint_index, &origin, &destination)) {
        return guidance_invalid_output();
    }

    GuidanceLegGeom geom{};
    if (!guidance_prepare_leg_geom(&origin, &destination, &geom)) {
        return guidance_invalid_output();
    }

    float pos_n = 0.0f;
    float pos_e = 0.0f;
    float pos_u = 0.0f;
    guidance_position_to_local_neu(&origin, nav_state.position, &pos_n, &pos_e, &pos_u);

    GuidanceLegProjection proj{};
    guidance_project_onto_leg(
        &geom,
        pos_n,
        pos_e,
        pos_u,
        geom.seg_n,
        geom.seg_e,
        geom.seg_u,
        &proj);

    GuidanceOutput output{};
    output.track_errors.cross_track_m = proj.cross_track_m;
    output.track_errors.cross_track_signed_m = proj.cross_track_signed_m;
    output.track_errors.along_track_m = proj.along_remaining_m;

    const float acceptance_m = static_cast<float>(destination.arrival_radius_m);
    const float acceptance_sq_m2 = acceptance_m * acceptance_m;
    const float along_progress_m = geom.seg_len_m - proj.along_remaining_m;

    output.waypoint_completed =
        (proj.to_dest_sq_m2 <= acceptance_sq_m2)
        || ((along_progress_m >= (geom.seg_len_m - acceptance_m))
            && (proj.cross_track_sq_m2 <= acceptance_sq_m2));
    output.valid = true;

    guidance_fill_commands_from_projection(
        &geom,
        &proj,
        look_ahead_distance_m,
        nav_state.heading_deg,
        origin.position.z,
        destination.position.z,
        nav_state.position.z,
        profile,
        &output);

    return output;
}

Guidance3D::Guidance3D()
    : look_ahead_distance_m_(NAVICORE_GUIDANCE_LOOK_AHEAD_M)
    , profile_(guidance_profile_default())
    , cached_leg_index_(SIZE_MAX)
    , leg_cache_{}
{
}

Guidance3D::Guidance3D(float look_ahead_distance_m)
    : look_ahead_distance_m_(look_ahead_distance_m)
    , profile_(guidance_profile_default())
    , cached_leg_index_(SIZE_MAX)
    , leg_cache_{}
{
}

void Guidance3D::set_look_ahead_distance(float look_ahead_distance_m)
{
    look_ahead_distance_m_ = look_ahead_distance_m;
}

void Guidance3D::set_profile(const GuidanceProfile &profile)
{
    profile_ = profile;
}

float Guidance3D::get_look_ahead_distance() const
{
    return look_ahead_distance_m_;
}

bool Guidance3D::prepare_cached_leg(
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index,
    const Waypoint *origin,
    const Waypoint *destination) const
{
    if (active_waypoint_index == cached_leg_index_ && leg_cache_.valid) {
        return true;
    }

    leg_cache_.valid = guidance_prepare_leg_geom(origin, destination, &leg_cache_);
    if (leg_cache_.valid) {
        cached_leg_index_ = active_waypoint_index;
    } else {
        cached_leg_index_ = SIZE_MAX;
    }

    return leg_cache_.valid;
}

GuidanceOutput Guidance3D::compute(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index) const
{
    if (look_ahead_distance_m_ <= NAVICORE_EPS_DISPLACEMENT_M
        || route.count == 0U || active_waypoint_index >= route.count) {
        return guidance_invalid_output();
    }

    Waypoint origin{};
    Waypoint destination{};
    if (!guidance_get_leg_waypoints(&route, active_waypoint_index, &origin, &destination)) {
        return guidance_invalid_output();
    }

    if (!prepare_cached_leg(route, active_waypoint_index, &origin, &destination)) {
        return guidance_invalid_output();
    }

    float pos_n = 0.0f;
    float pos_e = 0.0f;
    float pos_u = 0.0f;
    guidance_position_to_local_neu(&origin, nav_state.position, &pos_n, &pos_e, &pos_u);

    GuidanceLegProjection proj{};
    guidance_project_onto_leg(
        &leg_cache_,
        pos_n,
        pos_e,
        pos_u,
        leg_cache_.seg_n,
        leg_cache_.seg_e,
        leg_cache_.seg_u,
        &proj);

    GuidanceOutput output{};
    output.track_errors.cross_track_m = proj.cross_track_m;
    output.track_errors.cross_track_signed_m = proj.cross_track_signed_m;
    output.track_errors.along_track_m = proj.along_remaining_m;

    const float acceptance_m = static_cast<float>(destination.arrival_radius_m);
    const float acceptance_sq_m2 = acceptance_m * acceptance_m;
    const float along_progress_m = leg_cache_.seg_len_m - proj.along_remaining_m;

    output.waypoint_completed =
        (proj.to_dest_sq_m2 <= acceptance_sq_m2)
        || ((along_progress_m >= (leg_cache_.seg_len_m - acceptance_m))
            && (proj.cross_track_sq_m2 <= acceptance_sq_m2));
    output.valid = true;

    guidance_fill_commands_from_projection(
        &leg_cache_,
        &proj,
        look_ahead_distance_m_,
        nav_state.heading_deg,
        origin.position.z,
        destination.position.z,
        nav_state.position.z,
        profile_,
        &output);

    return output;
}

GuidanceOutput guidance_compute_homing(
    const NavState &nav_state,
    Vector3D target_position,
    const GuidanceProfile &profile)
{
    GuidanceOutput output{};
    output.valid = true;

    float north_m = 0.0f;
    float east_m = 0.0f;
    latlon_to_local_ne_m(
        nav_state.position.x,
        nav_state.position.y,
        target_position.x,
        target_position.y,
        &north_m,
        &east_m);

    const float horiz_dist_sq_m2 = (north_m * north_m) + (east_m * east_m);
    const float alt_error_m = target_position.z - nav_state.position.z;

    output.track_errors.cross_track_m = 0.0f;
    output.track_errors.cross_track_signed_m = 0.0f;

    if (horiz_dist_sq_m2 > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        const float horiz_dist_m = sqrtf(horiz_dist_sq_m2);
        output.track_errors.along_track_m = horiz_dist_m;
        output.commands.desired_heading = atan2f(east_m, north_m);
        output.commands.desired_speed = guidance_desired_speed_mps(horiz_dist_m, 0.0f, profile);
    } else {
        output.track_errors.along_track_m = 0.0f;
        output.commands.desired_heading = deg_to_rad(nav_state.heading_deg);
        output.commands.desired_speed = 0.0f;
    }

    output.commands.desired_climb = guidance_desired_climb_mps(
        nav_state.position.z,
        nav_state.position.z,
        target_position.z,
        output.track_errors.along_track_m,
        output.track_errors.along_track_m + 1.0f,
        profile);

    const float home_accept_sq_m2 =
        NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M * NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M;
    output.waypoint_completed =
        (horiz_dist_sq_m2 <= home_accept_sq_m2) && (fabsf(alt_error_m) <= 2.0f);

    return output;
}
