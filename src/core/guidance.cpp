#include "guidance.hpp"

#include "math_utils.hpp"
#include "waypoint.hpp"

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
    float to_dest_horiz_sq_m2;
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

static float guidance_wrap_pi(float angle_rad)
{
    while (angle_rad > NAVICORE_GUIDANCE_PI_F) {
        angle_rad -= (2.0f * NAVICORE_GUIDANCE_PI_F);
    }
    while (angle_rad < -NAVICORE_GUIDANCE_PI_F) {
        angle_rad += (2.0f * NAVICORE_GUIDANCE_PI_F);
    }
    return angle_rad;
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

static size_t guidance_destination_index(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index)
{
    if (route == NULL || route->count == 0U) {
        return 0U;
    }

    const size_t dest_index = active_waypoint_index + 1U;
    if (dest_index < route->count) {
        return dest_index;
    }

    return route->count - 1U;
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

static float guidance_leg_heading_rad(float seg_n, float seg_e)
{
    return atan2f(seg_e, seg_n);
}

/*
 * Angulo de giro [rad] en el waypoint destino del tramo activo (plano horizontal).
 * 0 = recto; PI = inversion completa.
 */
static float guidance_turn_angle_at_destination(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index)
{
    if (route == NULL || route->count < 3U) {
        return 0.0f;
    }

    const size_t dest_index = guidance_destination_index(route, active_waypoint_index);
    if (dest_index == 0U || (dest_index + 1U) >= route->count) {
        return 0.0f;
    }

    Waypoint prev_wp{};
    Waypoint dest_wp{};
    Waypoint next_wp{};

    if (!waypoint_buffer_at(route, dest_index - 1U, &prev_wp)
        || !waypoint_buffer_at(route, dest_index, &dest_wp)
        || !waypoint_buffer_at(route, dest_index + 1U, &next_wp)) {
        return 0.0f;
    }

    float in_n = 0.0f;
    float in_e = 0.0f;
    float in_u = 0.0f;
    float out_n = 0.0f;
    float out_e = 0.0f;
    float out_u = 0.0f;

    guidance_waypoint_delta_neu(&prev_wp, &dest_wp, &in_n, &in_e, &in_u);
    guidance_waypoint_delta_neu(&dest_wp, &next_wp, &out_n, &out_e, &out_u);

    const float in_horiz_sq = (in_n * in_n) + (in_e * in_e);
    const float out_horiz_sq = (out_n * out_n) + (out_e * out_e);
    if (in_horiz_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)
        || out_horiz_sq <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        return 0.0f;
    }

    const float heading_in_rad = guidance_leg_heading_rad(in_n, in_e);
    const float heading_out_rad = guidance_leg_heading_rad(out_n, out_e);
    return fabsf(guidance_wrap_pi(heading_out_rad - heading_in_rad));
}

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
    proj->to_dest_horiz_sq_m2 =
        (proj->to_dest_n * proj->to_dest_n)
        + (proj->to_dest_e * proj->to_dest_e);
}

static GuidanceOutput guidance_invalid_output(void)
{
    GuidanceOutput out{};
    out.valid = false;
    out.active_waypoint_index = SIZE_MAX;
    return out;
}

static bool guidance_is_route_terminal(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index)
{
    if (route == NULL || route->count == 0U) {
        return true;
    }

    if (route->count == 1U) {
        return true;
    }

    return active_waypoint_index >= (route->count - 2U);
}

GuidanceProfile guidance_profile_default(void)
{
    GuidanceProfile profile{};
    profile.cruise_speed_mps = NAVICORE_GUIDANCE_CRUISE_SPEED_MPS;
    profile.arrival_speed_mps = NAVICORE_GUIDANCE_ARRIVAL_SPEED_MPS;
    profile.min_speed_mps = 0.0f;
    profile.slowdown_along_track_m = NAVICORE_GUIDANCE_SLOWDOWN_ALONG_M;
    profile.turn_slowdown_along_m = NAVICORE_GUIDANCE_TURN_SLOWDOWN_ALONG_M;
    profile.max_climb_mps = NAVICORE_GUIDANCE_MAX_CLIMB_MPS;
    profile.max_descent_mps = NAVICORE_GUIDANCE_MAX_DESCENT_MPS;
    profile.climb_time_constant_s = NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S;
    profile.cross_track_slowdown_m = NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M;
    profile.min_speed_factor = NAVICORE_GUIDANCE_MIN_SPEED_FACTOR;
    profile.home_arrival_radius_m = NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M;
    profile.home_alt_tolerance_m = NAVICORE_GUIDANCE_HOME_ALT_TOLERANCE_M;
    profile.home_pressure_tolerance_pa = NAVICORE_GUIDANCE_HOME_PRESSURE_TOLERANCE_PA;
    profile.terminal_speed_mps = NAVICORE_GUIDANCE_TERMINAL_SPEED_MPS;
    profile.require_terminal_speed_at_home = false;
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

static float guidance_resolve_leg_cruise_mps(
    const Waypoint *destination,
    const GuidanceProfile &profile)
{
    if (destination != NULL && destination->desired_speed_mps > NAVICORE_EPS_SPEED_MPS) {
        return destination->desired_speed_mps;
    }

    return profile.cruise_speed_mps;
}

static float guidance_desired_speed_mps(
    float along_track_m,
    float cross_track_sq_m2,
    float turn_angle_rad,
    float leg_cruise_mps,
    const GuidanceProfile &profile)
{
    const float cruise = leg_cruise_mps;
    const float arrival = (profile.arrival_speed_mps < cruise)
        ? profile.arrival_speed_mps
        : cruise;
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

    float blend_factor = (along_factor < cross_factor) ? along_factor : cross_factor;
    float speed_mps = arrival + ((cruise - arrival) * blend_factor);

    const float turn_slow_m = profile.turn_slowdown_along_m;
    if (turn_angle_rad >= NAVICORE_GUIDANCE_TURN_ANGLE_SLOW_RAD
        && turn_slow_m > NAVICORE_EPS_DISPLACEMENT_M
        && along_track_m < turn_slow_m) {
        const float turn_norm = clampf(
            turn_angle_rad / NAVICORE_GUIDANCE_PI_F,
            0.0f,
            1.0f);
        const float along_turn_factor = clampf(along_track_m / turn_slow_m, 0.0f, 1.0f);
        const float turn_penalty = 1.0f - (turn_norm * (1.0f - min_factor) * (1.0f - along_turn_factor));
        speed_mps *= clampf(turn_penalty, min_factor, 1.0f);
    }

    if (profile.min_speed_mps > NAVICORE_EPS_SPEED_MPS && speed_mps < profile.min_speed_mps) {
        speed_mps = profile.min_speed_mps;
    }

    return speed_mps;
}

static float guidance_desired_climb_mps(
    float alt_error_m,
    float horiz_dist_m,
    float desired_speed_mps,
    const GuidanceProfile &profile)
{
    if (fabsf(alt_error_m) <= NAVICORE_EPS_DISPLACEMENT_M) {
        return 0.0f;
    }

    const float time_constant_s = (profile.climb_time_constant_s > 0.01f)
        ? profile.climb_time_constant_s
        : NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S;

    if (horiz_dist_m <= NAVICORE_EPS_DISPLACEMENT_M) {
        return clampf(
            alt_error_m / time_constant_s,
            -profile.max_descent_mps,
            profile.max_climb_mps);
    }

    const float speed_for_eta = (desired_speed_mps > NAVICORE_EPS_SPEED_MPS)
        ? desired_speed_mps
        : profile.arrival_speed_mps;
    const float eta_s = horiz_dist_m / speed_for_eta;
    const float safe_eta_s = (eta_s > 0.25f) ? eta_s : time_constant_s;

    return clampf(
        alt_error_m / safe_eta_s,
        -profile.max_descent_mps,
        profile.max_climb_mps);
}

static void guidance_fill_commands_from_projection(
    const GuidanceLegGeom *geom,
    const GuidanceLegProjection *proj,
    float look_ahead_distance_m,
    float current_heading_deg,
    float destination_alt_m,
    float current_alt_m,
    float turn_angle_rad,
    float leg_cruise_mps,
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

    const float track_heading_rad = guidance_leg_heading_rad(geom->seg_n, geom->seg_e);
    const float los_correction_rad = atan2f(
        -proj->cross_track_signed_m,
        look_ahead_distance_m);
    output->commands.desired_heading = track_heading_rad + los_correction_rad;

    const float lookahead_len_sq = (lookahead_n * lookahead_n) + (lookahead_e * lookahead_e);
    if (lookahead_len_sq > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        const float pp_heading_rad = guidance_leg_heading_rad(lookahead_n, lookahead_e);
        const float cross_abs_m = proj->cross_track_m;
        const float blend = clampf(1.0f - (cross_abs_m / look_ahead_distance_m), 0.0f, 1.0f);
        output->commands.desired_heading =
            (blend * pp_heading_rad) + ((1.0f - blend) * output->commands.desired_heading);
    } else if (proj->to_dest_horiz_sq_m2 > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        output->commands.desired_heading = guidance_leg_heading_rad(
            proj->to_dest_n,
            proj->to_dest_e);
    } else {
        output->commands.desired_heading = deg_to_rad(current_heading_deg);
    }

    output->commands.desired_speed = guidance_desired_speed_mps(
        proj->along_remaining_m,
        proj->cross_track_sq_m2,
        turn_angle_rad,
        leg_cruise_mps,
        profile);

    const float alt_error_m = destination_alt_m - current_alt_m;
    float horiz_dist_m = 0.0f;
    if (proj->to_dest_horiz_sq_m2 > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        horiz_dist_m = sqrtf(proj->to_dest_horiz_sq_m2);
    }

    output->commands.desired_climb = guidance_desired_climb_mps(
        alt_error_m,
        horiz_dist_m,
        output->commands.desired_speed,
        profile);
}

static void guidance_apply_acceptance(
    GuidanceOutput *output,
    float distance3d_m,
    float acceptance_radius_m,
    bool route_terminal)
{
    if (output == NULL) {
        return;
    }

    const float acceptance_sq_m2 = acceptance_radius_m * acceptance_radius_m;
    const bool within_acceptance =
        (distance3d_m * distance3d_m) <= acceptance_sq_m2;

    output->waypoint_completed = within_acceptance;
    output->route_completed = within_acceptance && route_terminal;
}

GuidanceOutput guidance_compute_output(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index,
    const GuidanceProfile &profile,
    float look_ahead_distance_m,
    float acceptance_radius_m)
{
    GuidanceOutput output = guidance_invalid_output();
    output.active_waypoint_index = active_waypoint_index;

    if (look_ahead_distance_m <= NAVICORE_EPS_DISPLACEMENT_M
        || route.count == 0U || active_waypoint_index >= route.count) {
        return output;
    }

    Waypoint origin{};
    Waypoint destination{};
    if (!guidance_get_leg_waypoints(&route, active_waypoint_index, &origin, &destination)) {
        return output;
    }

    GuidanceLegGeom geom{};
    if (!guidance_prepare_leg_geom(&origin, &destination, &geom)) {
        const float distance3d_m = vector3d_distance_3d_m(nav_state.position, destination.position);
        guidance_apply_acceptance(
            &output,
            distance3d_m,
            acceptance_radius_m,
            guidance_is_route_terminal(&route, active_waypoint_index));
        output.valid = true;
        output.commands.desired_heading = deg_to_rad(nav_state.heading_deg);
        output.commands.desired_speed = 0.0f;
        output.commands.desired_climb = 0.0f;
        return output;
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

    output.track_errors.cross_track_m = proj.cross_track_m;
    output.track_errors.cross_track_signed_m = proj.cross_track_signed_m;
    output.track_errors.along_track_m = proj.along_remaining_m;
    output.valid = true;

    const float turn_angle_rad = guidance_turn_angle_at_destination(&route, active_waypoint_index);
    const float leg_cruise_mps = guidance_resolve_leg_cruise_mps(&destination, profile);
    guidance_fill_commands_from_projection(
        &geom,
        &proj,
        look_ahead_distance_m,
        nav_state.heading_deg,
        destination.position.z,
        nav_state.position.z,
        turn_angle_rad,
        leg_cruise_mps,
        profile,
        &output);

    const float distance3d_m = vector3d_distance_3d_m(nav_state.position, destination.position);
    guidance_apply_acceptance(
        &output,
        distance3d_m,
        acceptance_radius_m,
        guidance_is_route_terminal(&route, active_waypoint_index));

    return output;
}

Guidance3D::Guidance3D()
    : look_ahead_distance_m_(NAVICORE_GUIDANCE_LOOK_AHEAD_M)
    , acceptance_radius_m_(NAVICORE_GUIDANCE_ACCEPTANCE_RADIUS_M)
    , profile_(guidance_profile_default())
    , route_{}
    , active_waypoint_index_(0U)
    , route_loaded_(false)
    , route_completed_(false)
    , cached_leg_index_(SIZE_MAX)
    , leg_cache_{}
{
}

Guidance3D::Guidance3D(float look_ahead_distance_m)
    : look_ahead_distance_m_(look_ahead_distance_m)
    , acceptance_radius_m_(NAVICORE_GUIDANCE_ACCEPTANCE_RADIUS_M)
    , profile_(guidance_profile_default())
    , route_{}
    , active_waypoint_index_(0U)
    , route_loaded_(false)
    , route_completed_(false)
    , cached_leg_index_(SIZE_MAX)
    , leg_cache_{}
{
}

void Guidance3D::set_look_ahead_distance(float look_ahead_distance_m)
{
    look_ahead_distance_m_ = look_ahead_distance_m;
}

void Guidance3D::set_acceptance_radius(float acceptance_radius_m)
{
    acceptance_radius_m_ = acceptance_radius_m;
}

void Guidance3D::set_profile(const GuidanceProfile &profile)
{
    profile_ = profile;
}

const GuidanceProfile &Guidance3D::get_profile() const
{
    return profile_;
}

float Guidance3D::get_look_ahead_distance() const
{
    return look_ahead_distance_m_;
}

float Guidance3D::get_acceptance_radius() const
{
    return acceptance_radius_m_;
}

void Guidance3D::set_route(const StaticWaypointBuffer &route)
{
    memcpy(&route_, &route, sizeof(StaticWaypointBuffer));
    active_waypoint_index_ = 0U;
    route_loaded_ = route_.count > 0U;
    route_completed_ = false;
    cached_leg_index_ = SIZE_MAX;
    leg_cache_.valid = false;
}

void Guidance3D::reset_route()
{
    memset(&route_, 0, sizeof(StaticWaypointBuffer));
    active_waypoint_index_ = 0U;
    route_loaded_ = false;
    route_completed_ = false;
    cached_leg_index_ = SIZE_MAX;
    leg_cache_.valid = false;
}

bool Guidance3D::route_loaded() const
{
    return route_loaded_;
}

bool Guidance3D::route_completed() const
{
    return route_completed_;
}

size_t Guidance3D::active_waypoint_index() const
{
    return active_waypoint_index_;
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

GuidanceOutput Guidance3D::compute_leg(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index,
    float acceptance_radius_m) const
{
    GuidanceOutput output = guidance_invalid_output();
    output.active_waypoint_index = active_waypoint_index;

    if (look_ahead_distance_m_ <= NAVICORE_EPS_DISPLACEMENT_M
        || route.count == 0U || active_waypoint_index >= route.count) {
        return output;
    }

    Waypoint origin{};
    Waypoint destination{};
    if (!guidance_get_leg_waypoints(&route, active_waypoint_index, &origin, &destination)) {
        return output;
    }

    if (!prepare_cached_leg(route, active_waypoint_index, &origin, &destination)) {
        const float distance3d_m = vector3d_distance_3d_m(nav_state.position, destination.position);
        guidance_apply_acceptance(
            &output,
            distance3d_m,
            acceptance_radius_m,
            guidance_is_route_terminal(&route, active_waypoint_index));
        output.valid = true;
        output.commands.desired_heading = deg_to_rad(nav_state.heading_deg);
        output.commands.desired_speed = 0.0f;
        output.commands.desired_climb = 0.0f;
        return output;
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

    output.track_errors.cross_track_m = proj.cross_track_m;
    output.track_errors.cross_track_signed_m = proj.cross_track_signed_m;
    output.track_errors.along_track_m = proj.along_remaining_m;
    output.valid = true;

    const float turn_angle_rad = guidance_turn_angle_at_destination(&route, active_waypoint_index);
    const float leg_cruise_mps = guidance_resolve_leg_cruise_mps(&destination, profile_);
    guidance_fill_commands_from_projection(
        &leg_cache_,
        &proj,
        look_ahead_distance_m_,
        nav_state.heading_deg,
        destination.position.z,
        nav_state.position.z,
        turn_angle_rad,
        leg_cruise_mps,
        profile_,
        &output);

    const float distance3d_m = vector3d_distance_3d_m(nav_state.position, destination.position);
    guidance_apply_acceptance(
        &output,
        distance3d_m,
        acceptance_radius_m,
        guidance_is_route_terminal(&route, active_waypoint_index));

    return output;
}

GuidanceOutput Guidance3D::compute(const NavState &nav_state)
{
    GuidanceOutput output = guidance_invalid_output();

    if (!route_loaded_ || route_.count == 0U) {
        return output;
    }

    if (route_completed_) {
        output.valid = true;
        output.route_completed = true;
        output.active_waypoint_index = active_waypoint_index_;
        output.commands.desired_heading = deg_to_rad(nav_state.heading_deg);
        output.commands.desired_speed = 0.0f;
        output.commands.desired_climb = 0.0f;
        return output;
    }

    if (active_waypoint_index_ >= route_.count) {
        route_completed_ = true;
        output.valid = true;
        output.route_completed = true;
        output.active_waypoint_index = active_waypoint_index_;
        return output;
    }

    output = compute_leg(nav_state, route_, active_waypoint_index_, acceptance_radius_m_);
    output.active_waypoint_index = active_waypoint_index_;

    if (!output.waypoint_completed) {
        return output;
    }

    if (output.route_completed) {
        route_completed_ = true;
        return output;
    }

    if (active_waypoint_index_ + 1U < route_.count) {
        ++active_waypoint_index_;
        cached_leg_index_ = SIZE_MAX;
        leg_cache_.valid = false;

        GuidanceOutput next_leg = compute_leg(
            nav_state,
            route_,
            active_waypoint_index_,
            acceptance_radius_m_);
        next_leg.waypoint_completed = false;
        next_leg.route_completed = false;
        next_leg.active_waypoint_index = active_waypoint_index_;
        return next_leg;
    }

    route_completed_ = true;
    output.route_completed = true;
    return output;
}

GuidanceOutput Guidance3D::compute(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index) const
{
    return compute_leg(nav_state, route, active_waypoint_index, acceptance_radius_m_);
}

static bool guidance_home_position_reached(
    const NavState &nav_state,
    Vector3D home,
    const GuidanceProfile &profile)
{
    const float dist_3d_m = vector3d_distance_3d_m(nav_state.position, home);
    if (dist_3d_m > profile.home_arrival_radius_m) {
        return false;
    }

    const float z_error = home.z - nav_state.position.z;
    if (nav_state.domain == NAVICORE_DOMAIN_SEA) {
        return fabsf(z_error) <= profile.home_pressure_tolerance_pa;
    }

    return fabsf(z_error) <= profile.home_alt_tolerance_m;
}

static float guidance_terminal_speed_floor_mps(const GuidanceProfile &profile)
{
    if (profile.min_speed_mps > NAVICORE_EPS_SPEED_MPS) {
        return profile.min_speed_mps;
    }

    return 0.0f;
}

GuidanceOutput guidance_compute_homing(
    const NavState &nav_state,
    Vector3D target_position,
    const GuidanceProfile &profile)
{
    GuidanceOutput output{};
    output.valid = true;
    output.active_waypoint_index = 0U;

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
    const float homing_transit_mps = NAVICORE_WAYPOINT_DEFAULT_TERMINAL_SPEED_MPS;
    const float speed_floor_mps = guidance_terminal_speed_floor_mps(profile);

    output.track_errors.cross_track_m = 0.0f;
    output.track_errors.cross_track_signed_m = 0.0f;

    if (horiz_dist_sq_m2 > (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        const float horiz_dist_m = sqrtf(horiz_dist_sq_m2);
        output.track_errors.along_track_m = horiz_dist_m;
        output.commands.desired_heading = guidance_leg_heading_rad(north_m, east_m);
        output.commands.desired_speed = guidance_desired_speed_mps(
            horiz_dist_m,
            0.0f,
            0.0f,
            homing_transit_mps,
            profile);
    } else {
        output.track_errors.along_track_m = 0.0f;
        output.commands.desired_heading = deg_to_rad(nav_state.heading_deg);
        output.commands.desired_speed = speed_floor_mps;
    }

    if (output.commands.desired_speed < speed_floor_mps) {
        output.commands.desired_speed = speed_floor_mps;
    }

    output.commands.desired_climb = guidance_desired_climb_mps(
        alt_error_m,
        output.track_errors.along_track_m,
        output.commands.desired_speed,
        profile);

    output.waypoint_completed = guidance_home_position_reached(
        nav_state,
        target_position,
        profile);

    return output;
}

bool guidance_terminal_arrival_satisfied(
    const GuidanceOutput &output,
    const NavState &nav_state,
    const GuidanceProfile &profile)
{
    if (!output.waypoint_completed) {
        return false;
    }

    if (!profile.require_terminal_speed_at_home) {
        return true;
    }

    return navstate_speed_mps(&nav_state) <= profile.terminal_speed_mps;
}
