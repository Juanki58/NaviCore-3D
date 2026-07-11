/**
 * @file power_state_machine.hpp
 * @brief Maquina de estados de energia para Ambiq Apollo (zero-heap, estatica)
 *
 * Gestiona transiciones entre modos de consumo segun la salud del sistema y el
 * estado cinemático del vehiculo. En HEALTH_CRITICAL con vehiculo detenido,
 * ejecuta apagado seguro de perifericos y reposo profundo.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/diagnostic.hpp"

/** Alias semantico: salud del sistema (NavHealthMode del monitor global). */
typedef NavHealthMode SystemHealthMode;

/**
 * @brief Modos de operacion energetica del MCU/perifericos
 */
typedef enum {
    POWER_PERFORMANCE = 0,   /**< Todos los buses activos; ciclo CTIMER @ 10 Hz */
    POWER_CONSERVATION,      /**< Perifericos no criticos deshabilitados */
    POWER_SAFE_SHUTDOWN      /**< UART silenciada, perifericos off, deep sleep forzado */
} PowerState;

/**
 * @brief Inicializa la maquina de estados en POWER_PERFORMANCE.
 * @note Idempotente; no asigna memoria dinamica.
 */
void power_manager_init(void);

/**
 * @brief Evalua salud y movimiento del vehiculo; aplica transiciones y acciones HAL.
 *
 * Reglas de transicion:
 *   - HEALTH_NOMINAL        -> POWER_PERFORMANCE
 *   - HEALTH_DEGRADED       -> POWER_CONSERVATION
 *   - HEALTH_CRITICAL + !stopped -> POWER_CONSERVATION (no apagar en movimiento)
 *   - HEALTH_CRITICAL + stopped  -> POWER_SAFE_SHUTDOWN (latcheo irreversible)
 *
 * @param health_mode     Modo del SystemHealthMonitor (diagnostic.hpp).
 * @param vehicle_stopped true si la velocidad horizontal es despreciable.
 */
void power_manager_update(SystemHealthMode health_mode, bool vehicle_stopped);

/**
 * @brief Devuelve el estado energetico actual.
 */
PowerState power_manager_get_state(void);

/**
 * @brief true tras entrar en POWER_SAFE_SHUTDOWN (no se abandona hasta reset).
 */
bool power_manager_is_shutdown_latched(void);

/**
 * @brief true si la UART de telemetria fue silenciada por apagado seguro.
 */
bool power_manager_is_uart_silenced(void);

/**
 * @brief Mascara de perifericos deshabilitados (bits POWER_PERIPH_*).
 */
uint32_t power_manager_get_disabled_periph_mask(void);

/** Bits de perifericos Ambiq Apollo (shim host / am_hal_pwrctrl_periph_disable). */
#define POWER_PERIPH_UART0   (1U << 0U)
#define POWER_PERIPH_IOM0    (1U << 1U)  /**< SPI IMU */
#define POWER_PERIPH_IOM1    (1U << 2U)  /**< SPI barometro */
#define POWER_PERIPH_GPIO    (1U << 3U)  /**< GNSS INT pin 42 */
