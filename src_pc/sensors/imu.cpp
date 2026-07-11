#include "imu.h"

#include <cmath>
#include <cstring>

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

void imu_simulator_init(ImuSimulator *sim, uint32_t seed)
{
    if (sim == nullptr) {
        return;
    }

    std::memset(sim, 0, sizeof(*sim));
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out)
{
    if (sim == nullptr || out == nullptr) {
        return false;
    }

    const float t = static_cast<float>(timestamp_ms) * 0.001f;

    out->accel_mps2[0] = 0.05f * std::sin(t * 0.7f) + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[1] = 0.03f * std::cos(t * 0.5f) + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[2] = 9.80665f + lcg_float(&sim->seed, -0.02f, 0.02f);

    out->gyro_radps[0] = 0.002f * std::sin(t * 1.1f);
    out->gyro_radps[1] = 0.0015f * std::cos(t * 0.9f);
    out->gyro_radps[2] = 0.001f * std::sin(t * 0.3f);

    out->mag_ut[0] = 22.0f + lcg_float(&sim->seed, -0.5f, 0.5f);
    out->mag_ut[1] = 5.0f + lcg_float(&sim->seed, -0.3f, 0.3f);
    out->mag_ut[2] = 42.0f + lcg_float(&sim->seed, -0.4f, 0.4f);

    out->timestamp_ms = timestamp_ms;
    out->valid = true;
    return true;
}
