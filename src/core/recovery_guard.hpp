/**
 * @file recovery_guard.hpp
 * @brief Recuperacion en caliente tras parada segura en HEALTH_CRITICAL (zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostic.hpp"
#include "fusion.hpp"

/** Factor de re-inicializacion de la covarianza P respecto a varianza GPS. */
#define RECOVERY_GUARD_COVARIANCE_GPS_FACTOR 1.5f

/** Umbral de innovacion limpia [(m/s)^2] para acumular ticks de recuperacion. */
#define RECOVERY_GUARD_INNOVATION_SQ_THRESHOLD 1.0f

/** Ticks consecutivos limpios requeridos antes de re-inicializar (30 x 100 ms = 3 s). */
#define RECOVERY_GUARD_CLEAN_TICKS_REQUIRED 30U

/** health_score tras recuperacion en caliente (HEALTH_NOMINAL). */
#define RECOVERY_GUARD_RECOVERED_HEALTH_SCORE 75U

/**
 * @brief Evalua condiciones de recuperacion y ejecuta hot-restart si procede.
 *
 * Requisitos acumulados durante @p RECOVERY_GUARD_CLEAN_TICKS_REQUIRED ticks seguidos:
 *   - @p monitor->shutdown_latched == false
 *   - @p monitor en HEALTH_CRITICAL
 *   - Velocidad horizontal del filtro == 0.0f m/s
 *   - @p current_innovation_sq < RECOVERY_GUARD_INNOVATION_SQ_THRESHOLD
 *
 * Al recuperar (Hot-Restart):
 *   - Reinicia sesgo IMU y covarianza P (GPS_VAR x 1.5)
 *   - health_score = 75, mode = HEALTH_NOMINAL
 *   - Limpia flags de error del monitor y divergence_guard
 *
 * @return true si se ejecuto la re-inicializacion en caliente; false en caso contrario.
 */
bool recovery_guard_step(
    DeadReckoningFilter *filter,
    SystemHealthMonitor *monitor,
    float current_innovation_sq);

/**
 * @brief Reinicia el contador estatico de ticks limpios consecutivos.
 */
void recovery_guard_reset(void);
