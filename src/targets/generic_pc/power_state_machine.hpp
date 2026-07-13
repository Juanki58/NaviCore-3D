/**
 * @file power_state_machine.hpp
 * @brief Maquina de estados de energia para simulador host (zero-heap, estatica)
 *
 * Gestiona transiciones entre modos de consumo segun la salud del sistema y el
 * estado cinemático del vehiculo. En HEALTH_CRITICAL con vehiculo detenido,
 * ejecuta apagado seguro de perifericos y reposo profundo (stub host).
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
    POWER_PERFORMANCE = 0,   /**< Todos los buses activos; ciclo @ 10 Hz */
    POWER_CONSERVATION,      /**< Perifericos no criticos deshabilitados */
    POWER_SAFE_SHUTDOWN      /**< UART silenciada, perifericos off, deep sleep forzado */
} PowerState;

void power_manager_init(void);
void power_manager_update(SystemHealthMode health_mode, bool vehicle_stopped);
PowerState power_manager_get_state(void);
bool power_manager_is_shutdown_latched(void);
bool power_manager_is_uart_silenced(void);
uint32_t power_manager_get_disabled_periph_mask(void);

#define POWER_PERIPH_UART0   (1U << 0U)
#define POWER_PERIPH_IOM0    (1U << 1U)
#define POWER_PERIPH_IOM1    (1U << 2U)
#define POWER_PERIPH_GPIO    (1U << 3U)
