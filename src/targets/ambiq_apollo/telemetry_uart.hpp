/**
 * @file telemetry_uart.hpp
 * @brief Telemetria CSV por UART para el superloop Ambiq Apollo (sin heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/diagnostic.hpp"
#include "../../core/fusion.hpp"

/**
 * @brief Formatea y transmite una linea CSV con estado, velocidad, orientacion,
 *        salud y fallos del filtro de navegacion.
 * @param filter   Filtro de dead-reckoning (obligatorio).
 * @param monitor  Monitor de salud global; puede ser NULL si aun no esta activo.
 * @param timestamp_ms Marca de tiempo del tick CTIMER (ms).
 * @return true si el frame se formateo y se envio al UART BSP sin truncar.
 * @note Buffer estatico interno de 256 bytes; no reentrante (superloop bare-metal).
 */
bool telemetry_uart_stream(
    const DeadReckoningFilter *filter,
    const SystemHealthMonitor *monitor,
    uint32_t timestamp_ms);
