#ifndef NAVICORE_NAV_STATE_H
#define NAVICORE_NAV_STATE_H

#include <stdbool.h>
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
 *
 * Orden de miembros: mayor a menor tamaño (12 -> 4 -> 1 B) para minimizar padding.
 */

typedef enum {
    NAV_MODE_INITIALIZING,
    NAV_MODE_GPS,
    NAV_MODE_DEAD_RECKONING,
    NAV_MODE_HYBRID
} NavMode;

typedef struct NAVICORE_ALIGNAS(4) {
    uint32_t fix_age_ms;
    float estimate_quality; /* 0.0 = sin confianza, 1.0 = máxima confianza */
    uint8_t satellites;
    bool gps_trusted;
} NavConfidence;

typedef struct NAVICORE_ALIGNAS(4) {
    Vector3D position;
    Vector3D velocity;
    NavConfidence confidence;
    uint32_t timestamp_ms;
    float heading_deg; /* rumbo [0, 360) */
    NavMode mode;
    NavDomain domain;
} NavState;

NAVICORE_STATIC_ASSERT(sizeof(NavConfidence) == 12U, "NavConfidence size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(NavConfidence) % 4U == 0U, "Error de alineación");

NAVICORE_STATIC_ASSERT(sizeof(NavState) == 52U, "NavState size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(NavState) % 4U == 0U, "Error de alineación");

NavConfidence nav_confidence_make(bool gps_trusted, uint8_t satellites, uint32_t fix_age_ms, float estimate_quality);
NavState navstate_make(Vector3D position, Vector3D velocity, float heading_deg, NavDomain domain, NavMode mode, NavConfidence confidence, uint32_t timestamp_ms);
NavState navstate_zero(NavDomain domain);
float navstate_normalize_heading(float heading_deg);
float navstate_speed_mps(const NavState *state);

#endif /* NAVICORE_NAV_STATE_H */
