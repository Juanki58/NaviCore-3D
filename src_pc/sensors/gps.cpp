#include "gps.h"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static float lcg_float(uint32_t *state, float min_value, float max_value)
{
    const float t = static_cast<float>(lcg_next(state) & 0x00FFFFFFU) / static_cast<float>(0x01000000U);
    return min_value + (t * (max_value - min_value));
}

static float deg_to_rad(float deg)
{
    return deg * static_cast<float>(M_PI / 180.0);
}

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed)
{
    if (sim == nullptr) {
        return;
    }

    sim->origin = origin;
    sim->speed_mps = speed_mps;
    sim->course_deg = course_deg;
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out)
{
    if (sim == nullptr || out == nullptr) {
        return false;
    }

    const float dt_s = static_cast<float>(timestamp_ms) * 0.001f;
    const float course_rad = deg_to_rad(sim->course_deg);
    const float north_m = sim->speed_mps * dt_s * std::cos(course_rad);
    const float east_m = sim->speed_mps * dt_s * std::sin(course_rad);

    const float lat_rad = deg_to_rad(sim->origin.x);
    const float dlat = north_m / 111132.954f;
    const float dlon = east_m / (111132.954f * std::cos(lat_rad));

    out->position = vector3d_make(
        sim->origin.x + dlat + lcg_float(&sim->seed, -1.0e-6f, 1.0e-6f),
        sim->origin.y + dlon + lcg_float(&sim->seed, -1.0e-6f, 1.0e-6f),
        sim->origin.z + lcg_float(&sim->seed, -0.05f, 0.05f));
    out->speed_mps = sim->speed_mps;
    out->course_deg = sim->course_deg;
    out->satellites = 10U;
    out->timestamp_ms = timestamp_ms;
    out->fix_valid = true;
    return true;
}
