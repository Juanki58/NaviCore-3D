#ifndef NAVICORE_SENSOR_TYPES_HPP
#define NAVICORE_SENSOR_TYPES_HPP

#include <stdbool.h>
#include <stdint.h>

#include "vector3d.h"

typedef struct {
    float accel_mps2[3];
    float gyro_radps[3];
    float mag_ut[3];
    uint32_t timestamp_ms;
    bool valid;
} ImuSample;

typedef struct {
    Vector3D position;
    float speed_mps;
    float course_deg;
    uint8_t satellites;
    uint32_t timestamp_ms;
    bool fix_valid;
} GpsSample;

typedef struct {
    float pressure_pa;
    float temperature_c;
    uint32_t timestamp_ms;
    bool valid;
} PressureSample;

#endif /* NAVICORE_SENSOR_TYPES_HPP */
