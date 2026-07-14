#ifndef NAVICORE_GUIDANCE_HPP
#define NAVICORE_GUIDANCE_HPP

#include "NavState.h"
#include "vector3d.h"
#include "waypoint.hpp"

#include <stddef.h>

#ifndef NAVICORE_GUIDANCE_CRUISE_SPEED_MPS
#define NAVICORE_GUIDANCE_CRUISE_SPEED_MPS 15.0f
#endif

#ifndef NAVICORE_GUIDANCE_ARRIVAL_SPEED_MPS
#define NAVICORE_GUIDANCE_ARRIVAL_SPEED_MPS 5.0f
#endif

#ifndef NAVICORE_GUIDANCE_SLOWDOWN_ALONG_M
#define NAVICORE_GUIDANCE_SLOWDOWN_ALONG_M 40.0f
#endif

#ifndef NAVICORE_GUIDANCE_TURN_SLOWDOWN_ALONG_M
#define NAVICORE_GUIDANCE_TURN_SLOWDOWN_ALONG_M 35.0f
#endif

#ifndef NAVICORE_GUIDANCE_TURN_ANGLE_SLOW_RAD
#define NAVICORE_GUIDANCE_TURN_ANGLE_SLOW_RAD 0.5235987755982988f /* ~30 deg */
#endif

#ifndef NAVICORE_GUIDANCE_MAX_CLIMB_MPS
#define NAVICORE_GUIDANCE_MAX_CLIMB_MPS 3.0f
#endif

#ifndef NAVICORE_GUIDANCE_MAX_DESCENT_MPS
#define NAVICORE_GUIDANCE_MAX_DESCENT_MPS 2.0f
#endif

#ifndef NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S
#define NAVICORE_GUIDANCE_CLIMB_TIME_CONSTANT_S 5.0f
#endif

#ifndef NAVICORE_GUIDANCE_LOOK_AHEAD_M
#define NAVICORE_GUIDANCE_LOOK_AHEAD_M 8.0f
#endif

#ifndef NAVICORE_GUIDANCE_ACCEPTANCE_RADIUS_M
#define NAVICORE_GUIDANCE_ACCEPTANCE_RADIUS_M 3.0f
#endif

#ifndef NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M
#define NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M 12.0f
#endif

#ifndef NAVICORE_GUIDANCE_MIN_SPEED_FACTOR
#define NAVICORE_GUIDANCE_MIN_SPEED_FACTOR 0.35f
#endif

#ifndef NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M
#define NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M 10.0f
#endif

#ifndef NAVICORE_GUIDANCE_HOME_ALT_TOLERANCE_M
#define NAVICORE_GUIDANCE_HOME_ALT_TOLERANCE_M 2.0f
#endif

#ifndef NAVICORE_GUIDANCE_HOME_PRESSURE_TOLERANCE_PA
#define NAVICORE_GUIDANCE_HOME_PRESSURE_TOLERANCE_PA 500.0f
#endif

#ifndef NAVICORE_GUIDANCE_TERMINAL_SPEED_MPS
#define NAVICORE_GUIDANCE_TERMINAL_SPEED_MPS 0.05f
#endif

/*
 * Guiado tactico — zero-heap, float/FPU, agnostico de plataforma.
 *
 * Emite consignas cinematicas (rumbo, velocidad escalar, tasa vertical) validas para
 * seguidores no holonomicos: embarcaciones (2D+vel), vehiculos terrestres, ala fija.
 * Plataformas holonomicas (multicopter) deben mapear estas consignas a su controlador
 * en la capa target (p. ej. vector velocidad deseado).
 *
 * Cross-track: proyeccion lineal r_perp = r - t*d, t = (r·d)/|d|^2 (sin cross product).
 */

typedef struct {
    float seg_n;
    float seg_e;
    float seg_u;
    float seg_len_m;
    float inv_seg_len_sq;
    bool valid;
} GuidanceLegGeom;

typedef struct {
    float cross_track_m;
    float along_track_m;
    float cross_track_signed_m;
} GuidanceErrors;

typedef struct {
    float desired_heading; /* rumbo objetivo [rad], 0 = norte */
    float desired_speed;   /* velocidad escalar sobre el plano de rumbo [m/s] */
    float desired_climb;   /* tasa vertical (+ = subir) [m/s] o dZ/dt en dominio mar */
} GuidanceCommands;

typedef struct {
    float cruise_speed_mps;
    float arrival_speed_mps;
    float min_speed_mps; /* piso de velocidad (sustentacion ala fija); 0 = puede detenerse */
    float slowdown_along_track_m;
    float turn_slowdown_along_m;
    float max_climb_mps;
    float max_descent_mps;
    float climb_time_constant_s;
    float cross_track_slowdown_m;
    float min_speed_factor;
    float home_arrival_radius_m;
    float home_alt_tolerance_m;
    float home_pressure_tolerance_pa;
    float terminal_speed_mps;
    bool require_terminal_speed_at_home; /* true solo si la plataforma debe quedar detenida */
} GuidanceProfile;

typedef struct {
    GuidanceErrors track_errors;
    GuidanceCommands commands;
    bool waypoint_completed;
    bool route_completed;
    size_t active_waypoint_index;
    bool valid;
} GuidanceOutput;

class Guidance3D {
public:
    Guidance3D();
    explicit Guidance3D(float look_ahead_distance_m);

    void set_look_ahead_distance(float look_ahead_distance_m);
    void set_acceptance_radius(float acceptance_radius_m);
    void set_profile(const GuidanceProfile &profile);
    const GuidanceProfile &get_profile() const;
    float get_look_ahead_distance() const;
    float get_acceptance_radius() const;

    void set_route(const StaticWaypointBuffer &route);
    void reset_route();
    bool route_loaded() const;
    bool route_completed() const;
    size_t active_waypoint_index() const;

    GuidanceOutput compute(const NavState &nav_state);

    GuidanceOutput compute(
        const NavState &nav_state,
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index) const;

private:
    GuidanceOutput compute_leg(
        const NavState &nav_state,
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index,
        float acceptance_radius_m) const;

    bool prepare_cached_leg(
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index,
        const Waypoint *origin,
        const Waypoint *destination) const;

    float look_ahead_distance_m_;
    float acceptance_radius_m_;
    GuidanceProfile profile_;
    StaticWaypointBuffer route_;
    size_t active_waypoint_index_;
    bool route_loaded_;
    bool route_completed_;
    mutable size_t cached_leg_index_;
    mutable GuidanceLegGeom leg_cache_;
};

GuidanceProfile guidance_profile_default(void);

bool guidance_get_leg_waypoints(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index,
    Waypoint *origin_out,
    Waypoint *destination_out);

GuidanceErrors guidance_compute_errors_3d(
    Vector3D position,
    Waypoint origin,
    Waypoint destination);

GuidanceErrors guidance_compute_leg_errors(
    const StaticWaypointBuffer *route,
    size_t active_waypoint_index,
    Vector3D position);

GuidanceOutput guidance_compute_output(
    const NavState &nav_state,
    const StaticWaypointBuffer &route,
    size_t active_waypoint_index,
    const GuidanceProfile &profile,
    float look_ahead_distance_m,
    float acceptance_radius_m);

GuidanceOutput guidance_compute_homing(
    const NavState &nav_state,
    Vector3D target_position,
    const GuidanceProfile &profile);

bool guidance_terminal_arrival_satisfied(
    const GuidanceOutput &output,
    const NavState &nav_state,
    const GuidanceProfile &profile);

#endif /* NAVICORE_GUIDANCE_HPP */
