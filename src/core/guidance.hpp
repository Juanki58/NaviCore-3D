#ifndef NAVICORE_GUIDANCE_HPP
#define NAVICORE_GUIDANCE_HPP

#include "vector3d.h"
#include "waypoint.hpp"

typedef struct {
    float cross_track_m;  /* desviacion lateral con signo (+ = izquierda del track O->D) */
    float along_track_m;  /* distancia restante al destino a lo largo del track [m] */
} GuidanceErrors;

GuidanceErrors guidance_compute_errors(
    Vector3D position,
    Waypoint origin,
    Waypoint destination);

#endif /* NAVICORE_GUIDANCE_HPP */
