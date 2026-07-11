#ifndef NAVICORE_SENSOR_TYPES_HPP
#define NAVICORE_SENSOR_TYPES_HPP

#include <stdbool.h>
#include <stdint.h>

#include "vector3d.h"

/*
 * Muestas de sensores — orden de miembros de mayor a menor tamaño.
 * Padding residual explicito donde hace falta para multiple exacto de 4 B.
 */

typedef struct NAVICORE_ALIGNAS(4) {
    float accel_mps2[3];
    float gyro_radps[3];
    float mag_ut[3];
    uint32_t timestamp_ms;
    bool valid;
    uint8_t _pad[3];
} ImuSample;

typedef struct NAVICORE_ALIGNAS(4) {
    Vector3D position;
    uint32_t timestamp_ms;
    float speed_mps;
    float course_deg;
    uint8_t satellites;
    bool fix_valid;
    uint8_t _pad[2];
} GpsSample;

typedef struct NAVICORE_ALIGNAS(4) {
    float pressure_pa;
    float temperature_c;
    uint32_t timestamp_ms;
    bool valid;
    uint8_t _pad[3];
} PressureSample;

NAVICORE_STATIC_ASSERT(sizeof(ImuSample) == 44U, "ImuSample size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(ImuSample) % 4U == 0U, "Error de alineación");

NAVICORE_STATIC_ASSERT(sizeof(GpsSample) == 28U, "GpsSample size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(GpsSample) % 4U == 0U, "Error de alineación");

NAVICORE_STATIC_ASSERT(sizeof(PressureSample) == 16U, "PressureSample size mismatch");
NAVICORE_STATIC_ASSERT(sizeof(PressureSample) % 4U == 0U, "Error de alineación");

#endif /* NAVICORE_SENSOR_TYPES_HPP */
