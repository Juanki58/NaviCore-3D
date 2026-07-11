/**
 * @file ambiq_spi_imu_stub.cpp
 * @brief Stub IOM SPI + DMA burst read para IMU
 */
#include "ambiq_spi_imu.hpp"

#include "ambiq_dma.hpp"
#include "ambiq_driver_config.hpp"

#include <math.h>
#include <string.h>

static uint8_t g_imu_spi_tx[AMBIQ_IMU_BURST_LENGTH + 1U];
static uint8_t g_imu_spi_rx[AMBIQ_IMU_BURST_LENGTH + 1U];
static AmbiqDmaTransaction g_imu_dma{};
static float g_imu_mock_phase_rad = 0.0f;

void ambiq_spi_imu_init(void)
{
    memset(g_imu_spi_tx, 0, sizeof(g_imu_spi_tx));
    memset(g_imu_spi_rx, 0, sizeof(g_imu_spi_rx));

    g_imu_spi_tx[0] = (uint8_t)(AMBIQ_IMU_REG_ACCEL_X_OUT_H | AMBIQ_IMU_SPI_READ_BIT);

    g_imu_dma.channel = AMBIQ_DMA_CHANNEL_SPI_IMU;
    g_imu_dma.tx_buffer = g_imu_spi_tx;
    g_imu_dma.rx_buffer = g_imu_spi_rx;
    g_imu_dma.length = (uint16_t)(AMBIQ_IMU_BURST_LENGTH + 1U);
    g_imu_dma.status = AMBIQ_DMA_STATUS_IDLE;

    g_imu_mock_phase_rad = 0.0f;
}

static void ambiq_spi_imu_fill_mock_burst(void)
{
    g_imu_mock_phase_rad += 0.1f;

    const int16_t ax = (int16_t)(100.0f * sinf(g_imu_mock_phase_rad));
    const int16_t ay = (int16_t)(200.0f * cosf(g_imu_mock_phase_rad));
    const int16_t az = (int16_t)(16384.0f * (9.81f / 9.80665f));
    const int16_t gx = 10;
    const int16_t gy = -20;
    const int16_t gz = 0;

    g_imu_spi_rx[1] = (uint8_t)(ax >> 8);
    g_imu_spi_rx[2] = (uint8_t)(ax & 0xFF);
    g_imu_spi_rx[3] = (uint8_t)(ay >> 8);
    g_imu_spi_rx[4] = (uint8_t)(ay & 0xFF);
    g_imu_spi_rx[5] = (uint8_t)(az >> 8);
    g_imu_spi_rx[6] = (uint8_t)(az & 0xFF);
    g_imu_spi_rx[7] = (uint8_t)(gx >> 8);
    g_imu_spi_rx[8] = (uint8_t)(gx & 0xFF);
    g_imu_spi_rx[9] = (uint8_t)(gy >> 8);
    g_imu_spi_rx[10] = (uint8_t)(gy & 0xFF);
    g_imu_spi_rx[11] = (uint8_t)(gz >> 8);
    g_imu_spi_rx[12] = (uint8_t)(gz & 0xFF);
}

bool ambiq_spi_imu_burst_read(ImuBurstRaw *raw_out, uint32_t *cycles_out)
{
    if (raw_out == NULL) {
        return false;
    }

    /*
     * TODO(Ambiq): am_hal_iom_spi_read(g_IOMHandle, ...);
     * Pre-cargar payload simulado antes del submit DMA stub.
     */
    ambiq_spi_imu_fill_mock_burst();

    if (!ambiq_dma_submit(&g_imu_dma)) {
        return false;
    }

    if (!ambiq_dma_wait_complete(&g_imu_dma, AMBIQ_DMA_TIMEOUT_CYCLES)) {
        return false;
    }

    if (cycles_out != NULL) {
        *cycles_out = g_imu_dma.cycles_elapsed;
    }

    raw_out->accel_raw[0] = (int16_t)((g_imu_spi_rx[1] << 8) | g_imu_spi_rx[2]);
    raw_out->accel_raw[1] = (int16_t)((g_imu_spi_rx[3] << 8) | g_imu_spi_rx[4]);
    raw_out->accel_raw[2] = (int16_t)((g_imu_spi_rx[5] << 8) | g_imu_spi_rx[6]);
    raw_out->gyro_raw[0] = (int16_t)((g_imu_spi_rx[7] << 8) | g_imu_spi_rx[8]);
    raw_out->gyro_raw[1] = (int16_t)((g_imu_spi_rx[9] << 8) | g_imu_spi_rx[10]);
    raw_out->gyro_raw[2] = (int16_t)((g_imu_spi_rx[11] << 8) | g_imu_spi_rx[12]);

    return true;
}

bool ambiq_spi_imu_raw_to_sample(const ImuBurstRaw *raw, ImuSample *sample_out, float mock_phase_rad)
{
    if (raw == NULL || sample_out == NULL) {
        return false;
    }

    (void)mock_phase_rad;

    const float accel_scale = 9.80665f / 16384.0f;
    const float gyro_scale = (3.14159265358979323846f / 180.0f) / 131.0f;

    sample_out->accel_mps2[0] = (float)raw->accel_raw[0] * accel_scale;
    sample_out->accel_mps2[1] = (float)raw->accel_raw[1] * accel_scale;
    sample_out->accel_mps2[2] = (float)raw->accel_raw[2] * accel_scale;

    sample_out->gyro_radps[0] = (float)raw->gyro_raw[0] * gyro_scale;
    sample_out->gyro_radps[1] = (float)raw->gyro_raw[1] * gyro_scale;
    sample_out->gyro_radps[2] = (float)raw->gyro_raw[2] * gyro_scale;

    sample_out->mag_ut[0] = 22.0f;
    sample_out->mag_ut[1] = 5.0f;
    sample_out->mag_ut[2] = 42.0f;
    sample_out->valid = true;

    return true;
}
