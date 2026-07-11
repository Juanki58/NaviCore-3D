#include "sensors_sim.hpp"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef SEAWATER_DENSITY_KG_M3
#define SEAWATER_DENSITY_KG_M3 1025.0f
#endif

#ifndef GRAVITY_MPS2
#define GRAVITY_MPS2 9.80665f
#endif

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static float lcg_float(uint32_t *state, float min_value, float max_value)
{
    const float t = (float)(lcg_next(state) & 0x00FFFFFFU) / (float)0x01000000U;
    return min_value + (t * (max_value - min_value));
}

static float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

void imu_simulator_init(ImuSimulator *sim, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    memset(sim, 0, sizeof(*sim));
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float t = (float)timestamp_ms * 0.001f;

    out->accel_mps2[0] = 0.05f * sinf(t * 0.7f) + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[1] = 0.03f * cosf(t * 0.5f) + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[2] = 9.80665f + lcg_float(&sim->seed, -0.02f, 0.02f);

    out->gyro_radps[0] = 0.002f * sinf(t * 1.1f);
    out->gyro_radps[1] = 0.0015f * cosf(t * 0.9f);
    out->gyro_radps[2] = 0.001f * sinf(t * 0.3f);

    out->mag_ut[0] = 22.0f + lcg_float(&sim->seed, -0.5f, 0.5f);
    out->mag_ut[1] = 5.0f + lcg_float(&sim->seed, -0.3f, 0.3f);
    out->mag_ut[2] = 42.0f + lcg_float(&sim->seed, -0.4f, 0.4f);

    out->timestamp_ms = timestamp_ms;
    out->valid = true;
    return true;
}

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    sim->origin = origin;
    sim->speed_mps = speed_mps;
    sim->course_deg = course_deg;
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float dt_s = (float)timestamp_ms * 0.001f;
    const float course_rad = deg_to_rad(sim->course_deg);
    const float north_m = sim->speed_mps * dt_s * cosf(course_rad);
    const float east_m = sim->speed_mps * dt_s * sinf(course_rad);

    const float lat_rad = deg_to_rad(sim->origin.x);
    const float dlat = north_m / 111132.954f;
    const float dlon = east_m / (111132.954f * cosf(lat_rad));

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

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    sim->surface_pressure_pa = surface_pressure_pa;
    sim->depth_m = depth_m;
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float hydrostatic_pa = SEAWATER_DENSITY_KG_M3 * GRAVITY_MPS2 * sim->depth_m;
    const float noise_pa = lcg_float(&sim->seed, -15.0f, 15.0f);

    out->pressure_pa = sim->surface_pressure_pa + hydrostatic_pa + noise_pa;
    out->temperature_c = 12.0f + lcg_float(&sim->seed, -0.2f, 0.2f);
    out->timestamp_ms = timestamp_ms;
    out->valid = true;
    return true;
}

float pressure_depth_from_hydrostatic_pa(float surface_pressure_pa, float pressure_pa)
{
    const float delta_pa = pressure_pa - surface_pressure_pa;
    if (delta_pa <= 0.0f) {
        return 0.0f;
    }
    return delta_pa / (SEAWATER_DENSITY_KG_M3 * GRAVITY_MPS2);
}
