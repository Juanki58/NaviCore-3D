/**
 * @file ambiq_imu_driver.hpp
 * @brief Driver IMU ICM-42688 class — burst SPI via IOM, escalado fisico estricto
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Registro base de datos simulados (ACCEL_X_OUT_H, mapa ICM-42688). */
#define AMBIQ_IMU_DATA_BASE_REG 0x1FU

/** Longitud de la rafaga de datos IMU [bytes] (3 accel + 3 gyro, int16 BE). */
#define AMBIQ_IMU_BURST_DATA_BYTES 12U

/** Chip select del IMU en el bus IOM. */
#define AMBIQ_IMU_SPI_CHIP_SELECT 0U

/** Gravedad estandar [m/s^2]. */
#define AMBIQ_IMU_GRAVITY_MPS2 9.80665f

/** Sensibilidad acelerometro ±4g: 8192 LSB/g. */
#define AMBIQ_IMU_ACCEL_COUNTS_PER_G 8192.0f

/** Escala estricta ADC -> m/s^2 (±4g). */
#define AMBIQ_IMU_ACCEL_SCALE_MPS2 (AMBIQ_IMU_GRAVITY_MPS2 / AMBIQ_IMU_ACCEL_COUNTS_PER_G)

/** Sensibilidad giroscopio ±2000 dps: 32768/2000 LSB/(deg/s). */
#define AMBIQ_IMU_GYRO_COUNTS_PER_DPS 16.384f

#ifndef AMBIQ_IMU_PI_F
#define AMBIQ_IMU_PI_F 3.14159265358979323846f
#endif

/** Escala estricta ADC -> rad/s (±2000 dps). */
#define AMBIQ_IMU_GYRO_SCALE_RADPS \
    ((AMBIQ_IMU_PI_F / 180.0f) / AMBIQ_IMU_GYRO_COUNTS_PER_DPS)

/**
 * @brief Lee una rafaga IMU de 12 bytes y devuelve aceleracion y giro en unidades SI.
 *
 * Utiliza ambiq_iom_spi_read_trans sobre el modulo IOM indicado. Decodifica int16 BE
 * y aplica AMBIQ_IMU_ACCEL_SCALE_MPS2 / AMBIQ_IMU_GYRO_SCALE_RADPS.
 *
 * @param module Modulo IOM (p. ej. AMBIQ_IOM_IMU_INSTANCE).
 * @param acc    Salida aceleracion [m/s^2], 3 ejes.
 * @param gyro   Salida velocidad angular [rad/s], 3 ejes.
 * @return true si la transferencia SPI y el decode son validos; false en error HAL o punteros nulos.
 */
bool ambiq_imu_read_data(uint32_t module, float acc[3], float gyro[3]);
