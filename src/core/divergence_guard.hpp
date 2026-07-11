/**
 * @file divergence_guard.hpp
 * @brief Deteccion de divergencia IMU/GPS por innovacion de velocidad (zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostic.hpp"
#include "fusion.hpp"

/** Umbral estatico del cuadrado de innovacion de velocidad [(m/s)^2]. */
#define NAVICORE_DIVERGENCE_INNOVATION_SQ_THRESHOLD 4.0f

/** Ticks consecutivos sobre umbral requeridos para declarar divergencia (> 5). */
#define DIVERGENCE_CONSECUTIVE_TICKS_REQUIRED 5U

/**
 * @brief Evalua divergencia entre velocidad estimada del filtro y GPS.
 *
 * innovation_sq = (v_filter - v_gps)^2
 *
 * Si innovation_sq supera NAVICORE_DIVERGENCE_INNOVATION_SQ_THRESHOLD durante
 * mas de DIVERGENCE_CONSECUTIVE_TICKS_REQUIRED ticks seguidos:
 *   - Registra DIVERGENCE_ERROR_IMU_GPS en el monitor
 *   - Resta DIVERGENCE_GUARD_HEALTH_PENALTY (-30) al health_score (una vez)
 *   - Devuelve true
 *
 * Si vuelve a la normalidad, resetea el contador estatico de forma determinista.
 *
 * @return true si la divergencia IMU/GPS esta confirmada; false en caso contrario.
 */
bool divergence_guard_check(
    DeadReckoningFilter *filter,
    float gps_speed_mps,
    SystemHealthMonitor *monitor);

/**
 * @brief Reinicia el estado estatico del guard (contador y latch).
 */
void divergence_guard_reset(void);
