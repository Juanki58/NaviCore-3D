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

#ifndef NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M
#define NAVICORE_GUIDANCE_CROSS_TRACK_SLOW_M 12.0f
#endif

#ifndef NAVICORE_GUIDANCE_MIN_SPEED_FACTOR
#define NAVICORE_GUIDANCE_MIN_SPEED_FACTOR 0.35f
#endif

/*
 * Guiado 3D — zero-heap, float/FPU.
 * Cross-track: proyeccion lineal r_perp = r - t*d, t = (r·d)/|d|^2 (sin cross product).
 * WCET @ 100 Hz: 1 sqrtf(|r_perp|) + sqrtf(|d|) solo al cambiar de tramo (cache).
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
    float cross_track_m;  /* |e_xt| perpendicular al pasillo 3D A->B [m] */
    float along_track_m;  /* distancia restante a B a lo largo del segmento [m] */
    float cross_track_signed_m; /* signo en plano horizontal (+ = izquierda del track) */
} GuidanceErrors;

typedef struct {
    float desired_heading; /* rumbo objetivo [rad], 0 = norte, via atan2f */
    float desired_speed;   /* velocidad lineal recomendada [m/s] */
    float desired_climb;   /* tasa de ascenso/descenso (+ = subir) [m/s] */
} GuidanceCommands;

typedef struct {
    float cruise_speed_mps;
    float arrival_speed_mps;
    float slowdown_along_track_m;
    float max_climb_mps;
    float max_descent_mps;
    float climb_time_constant_s;
    float cross_track_slowdown_m;
    float min_speed_factor;
} GuidanceProfile;

typedef struct {
    GuidanceErrors track_errors;
    GuidanceCommands commands;
    bool waypoint_completed;
    bool valid;
} GuidanceOutput;

class Guidance3D {
public:
    Guidance3D();
    explicit Guidance3D(float look_ahead_distance_m);

    void set_look_ahead_distance(float look_ahead_distance_m);
    void set_profile(const GuidanceProfile &profile);
    float get_look_ahead_distance() const;

    GuidanceOutput compute(
        const NavState &nav_state,
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index) const;

private:
    bool prepare_cached_leg(
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index,
        const Waypoint *origin,
        const Waypoint *destination) const;

    float look_ahead_distance_m_;
    GuidanceProfile profile_;
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
    float look_ahead_distance_m);

GuidanceOutput guidance_compute_homing(
    const NavState &nav_state,
    Vector3D target_position,
    const GuidanceProfile &profile);

#endif /* NAVICORE_GUIDANCE_HPP */
