#ifndef NAVICORE_GPS_H
#define NAVICORE_GPS_H

#include <stdbool.h>
#include <stdint.h>

#include "vector3d.h"

typedef struct {
    Vector3D position;
    float speed_mps;
    float course_deg;
    uint8_t satellites;
    uint32_t timestamp_ms;
    bool fix_valid;
} GpsSample;

typedef struct {
    Vector3D origin;
    float speed_mps;
    float course_deg;
    uint32_t seed;
} GpsSimulator;

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed);
bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out);

#endif /* NAVICORE_GPS_H */
