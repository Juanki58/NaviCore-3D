/**
 * @file geometry_guard.hpp
 * @brief Validacion geometrica de waypoints (continuidad espacial, zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostic.hpp"
#include "waypoint.hpp"

/** Distancia maxima permitida entre waypoints consecutivos [m]. */
#define NAVICORE_GEOM_MAX_STEP_M 150.0f

/**
 * @brief Valida que el siguiente waypoint no implique un salto espacial excesivo.
 *
 * Si @p buffer tiene al menos un waypoint, calcula la distancia euclidea [m]
 * respecto al ultimo punto (lat/lon -> metros con sqrtf).
 *
 * Si la distancia supera NAVICORE_GEOM_MAX_STEP_M:
 *   - Registra GEOMETRY_ERROR_DISCONTINUITY en el monitor
 *   - Resta GEOMETRY_GUARD_HEALTH_PENALTY (-15) al health_score
 *   - Devuelve false sin modificar el buffer
 *
 * @return true si el punto es geometricamente valido o el buffer esta vacio.
 */
bool geometry_guard_validate_next(
    const StaticWaypointBuffer *buffer,
    float next_x,
    float next_y,
    SystemHealthMonitor *monitor);

/**
 * @brief Distancia euclidea [m] entre dos puntos lat/lon [grados].
 */
float geometry_guard_distance_flat_m(float lat_a, float lon_a, float lat_b, float lon_b);
