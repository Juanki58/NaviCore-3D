#ifndef NAVICORE_PRESSURE_H
#define NAVICORE_PRESSURE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pressure_pa;
    float temperature_c;
    uint32_t timestamp_ms;
    bool valid;
} PressureSample;

typedef struct {
    float surface_pressure_pa;
    float depth_m;
    uint32_t seed;
} PressureSimulator;

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed);
bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out);
float pressure_depth_from_hydrostatic_pa(float surface_pressure_pa, float pressure_pa);

#endif /* NAVICORE_PRESSURE_H */
