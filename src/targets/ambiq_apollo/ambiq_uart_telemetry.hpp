/**
 * @file ambiq_uart_telemetry.hpp
 * @brief UART0 telemetria AmbiqSuite — 115200 8N1, TX polled, zero-heap
 */
#pragma once

#include <stdbool.h>

/**
 * @brief Inicializa UART0 @ 115200 baudios, 8N1, sin control de flujo.
 * @return true si el HAL configura el periferico; false en error.
 */
bool ambiq_uart_telemetry_init(void);

/**
 * @brief Transmite una cadena null-terminated carácter a carácter.
 *
 * En silicio usa am_hal_uart_char_transmit_polled.
 * En host (!NAVICORE_AMBIQ_SDK) imprime por stdout.
 */
void ambiq_uart_write_string(const char *str);
