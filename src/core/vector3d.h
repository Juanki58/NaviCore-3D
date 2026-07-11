#ifndef NAVICORE_VECTOR3D_H
#define NAVICORE_VECTOR3D_H

#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
#define NAVICORE_ALIGNAS(n) alignas(n)
#define NAVICORE_STATIC_ASSERT(cond, msg) static_assert((cond), msg)
#else
#include <stdalign.h>
#define NAVICORE_ALIGNAS(n) _Alignas(n)
#define NAVICORE_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#endif

/*
 * Coordenadas tridimensionales permanentes (NaviCore-3D):
 *   X = Latitud  (grados)
 *   Y = Longitud (grados)
 *   Z = Altitud en aire (metros) / Presión hidrostática bajo el agua (Pa)
 *
 * Miembros: 3 x float (4 B) — sin padding interno.
 */
typedef struct NAVICORE_ALIGNAS(4) {
    float x; /* latitud  [°] */
    float y; /* longitud [°] */
    float z; /* altitud [m] o presión hidrostática [Pa] */
} Vector3D;

NAVICORE_STATIC_ASSERT(sizeof(Vector3D) == 12U, "Vector3D size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(Vector3D) % 4U == 0U, "Error de alineación");

typedef enum {
    NAVICORE_DOMAIN_AIR,
    NAVICORE_DOMAIN_SEA
} NavDomain;

Vector3D vector3d_make(float x, float y, float z);
Vector3D vector3d_zero();
Vector3D vector3d_add(Vector3D a, Vector3D b);
Vector3D vector3d_sub(Vector3D a, Vector3D b);
Vector3D vector3d_scale(Vector3D v, float s);
float vector3d_distance_flat_m(Vector3D a, Vector3D b);
bool vector3d_equal(Vector3D a, Vector3D b, float epsilon);

#endif /* NAVICORE_VECTOR3D_H */
