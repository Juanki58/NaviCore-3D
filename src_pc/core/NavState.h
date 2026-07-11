#ifndef NAVICORE_NAV_STATE_H
#define NAVICORE_NAV_STATE_H

#include <stdint.h>

#include "vector3d.h"

/*
 * NavState — estado unificado de navegación (NaviCore-3D).
 *
 * Convención de ejes (permanente en todo el núcleo):
 *   position.x = latitud  [°]
 *   position.y = longitud [°]
 *   position.z = altitud [m] en aire / presión hidrostática [Pa] en mar
 *
 *   velocity.x = componente norte  [m/s]
 *   velocity.y = componente este   [m/s]
 *   velocity.z = componente vertical [m/s] o variación hidrostática [Pa/s]
 */

typedef enum {
    NAV_MODE_INITIALIZING,
    NAV_MODE_GPS,
    NAV_MODE_DEAD_RECKONING,
    NAV_MODE_HYBRID
} NavMode;

typedef struct {
    bool gps_trusted;
    uint8_t satellites;
    uint32_t fix_age_ms;
    float estimate_quality; /* 0.0 = sin confianza, 1.0 = máxima confianza */
} NavConfidence;

typedef struct {
    Vector3D position;
    Vector3D velocity;
    float heading_deg; /* rumbo [0, 360) */
    NavDomain domain;
    NavMode mode;
    NavConfidence confidence;
    uint32_t timestamp_ms;
} NavState;

NavConfidence nav_confidence_make(bool gps_trusted, uint8_t satellites, uint32_t fix_age_ms, float estimate_quality);
NavState navstate_make(Vector3D position, Vector3D velocity, float heading_deg, NavDomain domain, NavMode mode, NavConfidence confidence, uint32_t timestamp_ms);
NavState navstate_zero(NavDomain domain);
float navstate_normalize_heading(float heading_deg);
float navstate_speed_mps(const NavState *state);

#endif /* NAVICORE_NAV_STATE_H */
