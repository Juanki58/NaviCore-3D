/**
 * @file ambiq_iom_master.hpp
 * @brief Maestro IOM SPI bare-metal (AmbiqSuite HAL, zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Frecuencia SPI fija del bus IOM [Hz]. */
#define AMBIQ_IOM_SPI_CLOCK_HZ 1000000U

/** Numero maximo de instancias IOM soportadas (Apollo4 class). */
#define AMBIQ_IOM_MODULE_MAX 8U

/**
 * @brief Inicializa una instancia IOM en modo SPI @ 1 MHz.
 *
 * @param module Indice de modulo IOM (0..AMBIQ_IOM_MODULE_MAX-1).
 * @return true si el HAL configura y habilita el periferico; false en error.
 */
bool ambiq_iom_spi_init(uint32_t module);

/**
 * @brief Transferencia SPI bloqueante full-duplex sobre IOM.
 *
 * @param module       Modulo IOM previamente inicializado.
 * @param chip_select  Linea CS del periferico esclavo.
 * @param tx_buf       Buffer de transmision (len bytes).
 * @param rx_buf       Buffer de recepcion (len bytes).
 * @param len          Longitud de la transferencia en bytes.
 * @return true si am_hal_iom_blocking_transfer finaliza con exito; false en fallo HAL.
 */
bool ambiq_iom_spi_read_trans(
    uint32_t module,
    uint32_t chip_select,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint32_t len);
