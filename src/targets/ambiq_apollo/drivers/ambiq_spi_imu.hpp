/**
 * @file ambiq_spi_imu.hpp
 * @brief Driver SPI burst-read para IMU (ICM-42688 class) via IOM + DMA
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensor_types.hpp"

typedef struct {
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
} ImuBurstRaw;

void ambiq_spi_imu_init(void);
bool ambiq_spi_imu_burst_read(ImuBurstRaw *raw_out, uint32_t *cycles_out);
bool ambiq_spi_imu_raw_to_sample(const ImuBurstRaw *raw, ImuSample *sample_out, float mock_phase_rad);
