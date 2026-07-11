/**
 * @file time_guard.hpp
 * @brief Guardia WCET — conteo de ciclos y validacion determinista (zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostic.hpp"

/** Tasa de refresco simulada en PC (Hz) para convertir tiempo a ciclos logicos. */
#define TIME_GUARD_PC_REFRESH_HZ    60U

/** Ciclos maximos por defecto para un tick CTIMER @ 100 ms (10 Hz). */
#define TIME_GUARD_DEFAULT_MAX_TICKS 6U

/**
 * @brief Inicializa el backend de conteo (stub PC o STimer Ambiq).
 */
void time_guard_init(void);

/**
 * @brief Inicia el conteo de ciclos de la seccion critica actual.
 */
void time_guard_start(void);

/**
 * @brief Detiene el conteo y devuelve los ciclos consumidos.
 */
uint32_t time_guard_stop(void);

/**
 * @brief Valida el tiempo de ejecucion frente al umbral WCET.
 *
 * Si @p execution_ticks supera @p max_allowed_ticks, registra
 * TIME_GUARD_ERROR_WCET en el monitor, aplica penalizacion determinista
 * al health_score y devuelve false.
 *
 * @return true si el tiempo esta dentro del presupuesto; false si hay violacion WCET.
 */
bool time_guard_validate(
    uint32_t execution_ticks,
    uint32_t max_allowed_ticks,
    SystemHealthMonitor *monitor);

/**
 * @brief Devuelve la tasa de refresco del stub PC [Hz].
 */
uint32_t time_guard_pc_refresh_hz(void);
