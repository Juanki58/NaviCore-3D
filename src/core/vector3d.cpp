#include "vector3d.h"

#include "math_utils.hpp"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

Vector3D vector3d_make(float x, float y, float z)
{
    Vector3D v = {x, y, z};
    return v;
}

Vector3D vector3d_zero()
{
    return vector3d_make(0.0f, 0.0f, 0.0f);
}

Vector3D vector3d_add(Vector3D a, Vector3D b)
{
    return vector3d_make(a.x + b.x, a.y + b.y, a.z + b.z);
}

Vector3D vector3d_sub(Vector3D a, Vector3D b)
{
    return vector3d_make(a.x - b.x, a.y - b.y, a.z - b.z);
}

Vector3D vector3d_scale(Vector3D v, float s)
{
    return vector3d_make(v.x * s, v.y * s, v.z * s);
}

float vector3d_distance_flat_m(Vector3D a, Vector3D b)
{
    const float dlat_m = (b.x - a.x) * 111132.954f;
    const float dlon_raw = (b.y - a.y) * 111132.954f;

    if (fabsf(dlat_m) <= NAVICORE_EPS_DISPLACEMENT_M &&
        fabsf(dlon_raw) <= NAVICORE_EPS_DISPLACEMENT_M) {
        return 0.0f;
    }

    const float mean_lat_rad = deg_to_rad((a.x + b.x) * 0.5f);
    const float dlon_m = dlon_raw * cosf(mean_lat_rad);
    return sqrtf((dlat_m * dlat_m) + (dlon_m * dlon_m));
}

float vector3d_distance_3d_m(Vector3D a, Vector3D b)
{
    const float dlat_m = (b.x - a.x) * 111132.954f;
    const float dlon_raw = (b.y - a.y) * 111132.954f;
    const float mean_lat_rad = deg_to_rad((a.x + b.x) * 0.5f);
    const float dlon_m = dlon_raw * cosf(mean_lat_rad);
    const float up_m = b.z - a.z;

    const float dist_sq_m2 = (dlat_m * dlat_m) + (dlon_m * dlon_m) + (up_m * up_m);
    if (dist_sq_m2 <= (NAVICORE_EPS_DISPLACEMENT_M * NAVICORE_EPS_DISPLACEMENT_M)) {
        return 0.0f;
    }

    return sqrtf(dist_sq_m2);
}

bool vector3d_equal(Vector3D a, Vector3D b, float epsilon)
{
    return fabsf(a.x - b.x) <= epsilon &&
           fabsf(a.y - b.y) <= epsilon &&
           fabsf(a.z - b.z) <= epsilon;
}
