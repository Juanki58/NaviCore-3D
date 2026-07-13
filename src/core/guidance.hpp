#ifndef NAVICORE_GUIDANCE_HPP
#define NAVICORE_GUIDANCE_HPP

#include "NavState.h"
#include "vector3d.h"
#include "waypoint.hpp"

#include <stddef.h>

typedef struct {
    float cross_track_m;  /* desviacion lateral con signo (+ = izquierda del track O->D) */
    float along_track_m;  /* distancia restante al destino a lo largo del track [m] */
} GuidanceErrors;

struct PurePursuitOutput {
    float yaw_target_rad;     /* rumbo absoluto hacia el punto de mira [rad], 0 = norte */
    bool waypoint_completed;  /* true si distancia al WP activo < radio de aceptacion */
    bool valid;               /* false si no hay segmento o ruta insuficiente */
};

class PurePursuitGuidance {
public:
    static constexpr float kDefaultLookAheadM = 5.0f;

    PurePursuitGuidance();
    explicit PurePursuitGuidance(float look_ahead_distance_m);

    void set_look_ahead_distance(float look_ahead_distance_m);
    float get_look_ahead_distance() const;

    PurePursuitOutput compute(
        const NavState &nav_state,
        const StaticWaypointBuffer &route,
        size_t active_waypoint_index) const;

private:
    float look_ahead_distance_m_;
};

GuidanceErrors guidance_compute_errors(
    Vector3D position,
    Waypoint origin,
    Waypoint destination);

#endif /* NAVICORE_GUIDANCE_HPP */
