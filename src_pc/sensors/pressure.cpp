#include "pressure.h"

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
    const float t = static_cast<float>(lcg_next(state) & 0x00FFFFFFU) / static_cast<float>(0x01000000U);
    return min_value + (t * (max_value - min_value));
}

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed)
{
    if (sim == nullptr) {
        return;
    }

    sim->surface_pressure_pa = surface_pressure_pa;
    sim->depth_m = depth_m;
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out)
{
    if (sim == nullptr || out == nullptr) {
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
