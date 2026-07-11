#ifndef NAVICORE_SENSORS_SIM_HPP
#define NAVICORE_SENSORS_SIM_HPP

#include <stdint.h>

#include "sensor_types.hpp"
#include "vector3d.h"

typedef struct {
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t seed;
} ImuSimulator;

typedef struct {
    Vector3D origin;
    float speed_mps;
    float course_deg;
    uint32_t seed;
} GpsSimulator;

typedef struct {
    float surface_pressure_pa;
    float depth_m;
    uint32_t seed;
} PressureSimulator;

void imu_simulator_init(ImuSimulator *sim, uint32_t seed);
bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out);

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed);
bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out);

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed);
bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out);
float pressure_depth_from_hydrostatic_pa(float surface_pressure_pa, float pressure_pa);

#endif /* NAVICORE_SENSORS_SIM_HPP */
