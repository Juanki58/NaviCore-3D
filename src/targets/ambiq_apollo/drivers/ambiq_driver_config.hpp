/**
 * @file ambiq_driver_config.hpp
 * @brief Constantes de hardware para el target Ambiq Apollo (Apollo4 class)
 */
#pragma once

#include <stdint.h>

#define AMBIQ_IOM_IMU_INSTANCE       0U
#define AMBIQ_SPI_IMU_HZ             24000000U
#define AMBIQ_IMU_BURST_LENGTH       12U
#define AMBIQ_IMU_REG_ACCEL_X_OUT_H  0x1FU
#define AMBIQ_IMU_SPI_READ_BIT       0x80U

#define AMBIQ_GPIO_GNSS_INT_PIN      42U
#define AMBIQ_UART_TELEM_INSTANCE    0U
#define AMBIQ_UART_TELEM_BAUD        115200U

#define AMBIQ_TICK_INTERVAL_MS       100U
#define AMBIQ_GNSS_FIX_INTERVAL_TICKS 10U

/** STIMER @ cristal 32.768 kHz — periodo de 100 ms en ciclos. */
#define AMBIQ_STIMER_CLK_HZ            32768U
#define AMBIQ_STIMER_TICK_CYCLES       3277U
#define AMBIQ_STIMER_COMPARE_INSTANCE  0U

#define AMBIQ_DMA_TIMEOUT_CYCLES     500000U

#define AMBIQ_TELEM_FRAME_MAX        96U
