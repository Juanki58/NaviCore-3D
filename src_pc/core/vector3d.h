#ifndef NAVICORE_VECTOR3D_H
#define NAVICORE_VECTOR3D_H

#include <stdbool.h>

/*
 * Coordenadas tridimensionales permanentes (NaviCore-3D):
 *   X = Latitud  (grados)
 *   Y = Longitud (grados)
 *   Z = Altitud en aire (metros) / Presión hidrostática bajo el agua (Pa)
 */
typedef struct {
    float x; /* latitud  [°] */
    float y; /* longitud [°] */
    float z; /* altitud [m] o presión hidrostática [Pa] */
} Vector3D;

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
