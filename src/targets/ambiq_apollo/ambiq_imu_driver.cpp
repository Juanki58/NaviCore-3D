/**
 * @file ambiq_imu_driver.cpp
 * @brief Lectura determinista IMU via IOM SPI — zero-heap, compatible superloop Ambiq
 */
#include "ambiq_imu_driver.hpp"

#include "ambiq_iom_master.hpp"
#include "drivers/ambiq_driver_config.hpp"

#include <math.h>

#define AMBIQ_IMU_SPI_FRAME_BYTES (AMBIQ_IMU_BURST_DATA_BYTES + 1U)

static uint8_t g_imu_spi_tx[AMBIQ_IMU_SPI_FRAME_BYTES];
static uint8_t g_imu_spi_rx[AMBIQ_IMU_SPI_FRAME_BYTES];
static bool g_imu_iom_ready = false;
static uint32_t g_imu_bound_module = UINT32_MAX;
static float g_imu_mock_phase_rad = 0.0f;

static bool ambiq_imu_pointers_valid(const float *acc, const float *gyro)
{
    return acc != NULL && gyro != NULL;
}

static bool ambiq_imu_ensure_iom_ready(uint32_t module)
{
    if (g_imu_iom_ready && g_imu_bound_module == module) {
        return true;
    }

    if (!ambiq_iom_spi_init(module)) {
        return false;
    }

    g_imu_iom_ready = true;
    g_imu_bound_module = module;
    return true;
}

static void ambiq_imu_prepare_tx_frame(void)
{
    g_imu_spi_tx[0] = (uint8_t)(AMBIQ_IMU_DATA_BASE_REG | AMBIQ_IMU_SPI_READ_BIT);

    for (uint32_t i = 1U; i < AMBIQ_IMU_SPI_FRAME_BYTES; ++i) {
        g_imu_spi_tx[i] = 0U;
    }
}

#ifndef NAVICORE_AMBIQ_SDK

static void ambiq_imu_fill_mock_burst(void)
{
    g_imu_mock_phase_rad += 0.1f;

    const int16_t ax = (int16_t)(100.0f * sinf(g_imu_mock_phase_rad));
    const int16_t ay = (int16_t)(200.0f * cosf(g_imu_mock_phase_rad));
    const int16_t az = (int16_t)(8192.0f * (AMBIQ_IMU_GRAVITY_MPS2 / AMBIQ_IMU_GRAVITY_MPS2));
    const int16_t gx = 10;
    const int16_t gy = -20;
    const int16_t gz = 0;

    g_imu_spi_rx[1] = (uint8_t)((uint16_t)ax >> 8);
    g_imu_spi_rx[2] = (uint8_t)((uint16_t)ax & 0xFFU);
    g_imu_spi_rx[3] = (uint8_t)((uint16_t)ay >> 8);
    g_imu_spi_rx[4] = (uint8_t)((uint16_t)ay & 0xFFU);
    g_imu_spi_rx[5] = (uint8_t)((uint16_t)az >> 8);
    g_imu_spi_rx[6] = (uint8_t)((uint16_t)az & 0xFFU);
    g_imu_spi_rx[7] = (uint8_t)((uint16_t)gx >> 8);
    g_imu_spi_rx[8] = (uint8_t)((uint16_t)gx & 0xFFU);
    g_imu_spi_rx[9] = (uint8_t)((uint16_t)gy >> 8);
    g_imu_spi_rx[10] = (uint8_t)((uint16_t)gy & 0xFFU);
    g_imu_spi_rx[11] = (uint8_t)((uint16_t)gz >> 8);
    g_imu_spi_rx[12] = (uint8_t)((uint16_t)gz & 0xFFU);
}

#endif /* !NAVICORE_AMBIQ_SDK */

static int16_t ambiq_imu_decode_i16_be(const uint8_t *msb, const uint8_t *lsb)
{
    return (int16_t)(((uint16_t)msb[0] << 8) | (uint16_t)lsb[0]);
}

static bool ambiq_imu_decode_burst(const uint8_t *rx_frame, float acc[3], float gyro[3])
{
    if (rx_frame == NULL) {
        return false;
    }

    const int16_t ax = ambiq_imu_decode_i16_be(&rx_frame[1], &rx_frame[2]);
    const int16_t ay = ambiq_imu_decode_i16_be(&rx_frame[3], &rx_frame[4]);
    const int16_t az = ambiq_imu_decode_i16_be(&rx_frame[5], &rx_frame[6]);
    const int16_t gx = ambiq_imu_decode_i16_be(&rx_frame[7], &rx_frame[8]);
    const int16_t gy = ambiq_imu_decode_i16_be(&rx_frame[9], &rx_frame[10]);
    const int16_t gz = ambiq_imu_decode_i16_be(&rx_frame[11], &rx_frame[12]);

    acc[0] = (float)ax * AMBIQ_IMU_ACCEL_SCALE_MPS2;
    acc[1] = (float)ay * AMBIQ_IMU_ACCEL_SCALE_MPS2;
    acc[2] = (float)az * AMBIQ_IMU_ACCEL_SCALE_MPS2;

    gyro[0] = (float)gx * AMBIQ_IMU_GYRO_SCALE_RADPS;
    gyro[1] = (float)gy * AMBIQ_IMU_GYRO_SCALE_RADPS;
    gyro[2] = (float)gz * AMBIQ_IMU_GYRO_SCALE_RADPS;

    return true;
}

bool ambiq_imu_read_data(uint32_t module, float acc[3], float gyro[3])
{
    if (!ambiq_imu_pointers_valid(acc, gyro)) {
        return false;
    }

    if (!ambiq_imu_ensure_iom_ready(module)) {
        return false;
    }

    ambiq_imu_prepare_tx_frame();

    if (!ambiq_iom_spi_read_trans(
            module,
            AMBIQ_IMU_SPI_CHIP_SELECT,
            g_imu_spi_tx,
            g_imu_spi_rx,
            AMBIQ_IMU_SPI_FRAME_BYTES)) {
        return false;
    }

#ifndef NAVICORE_AMBIQ_SDK
    ambiq_imu_fill_mock_burst();
#endif

    return ambiq_imu_decode_burst(g_imu_spi_rx, acc, gyro);
}
