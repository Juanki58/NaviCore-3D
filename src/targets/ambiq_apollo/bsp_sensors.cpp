/**
 * @file bsp_sensors.cpp
 * @brief HAL/BSP — orquesta drivers estructurales (CTIMER, SPI/DMA, GPIO, UART, power)
 */
#include "bsp_sensors.hpp"

#include "ambiq_system.hpp"
#include "drivers/ambiq_driver_config.hpp"
#include "drivers/ambiq_dma.hpp"
#include "drivers/ambiq_gpio_gnss.hpp"
#include "drivers/ambiq_power_monitor.hpp"
#include "drivers/ambiq_spi_imu.hpp"
#include "drivers/ambiq_uart_telemetry.hpp"
#include "ambiq_uart_telemetry.hpp"

#include "vector3d.h"

#include <math.h>
#include <string.h>

#define BSP_BARO_SPI_BURST_LENGTH 6U
#define BSP_BARO_REG_PRESS_OUT    0x20U
#define BSP_BARO_SPI_READ_BIT     0x80U

#define BSP_IMU_SPI_BURST_LENGTH  AMBIQ_IMU_BURST_LENGTH
#define BSP_IMU_REG_ACCEL_X_OUT_H AMBIQ_IMU_REG_ACCEL_X_OUT_H
#define BSP_IMU_SPI_READ_BIT      AMBIQ_IMU_SPI_READ_BIT

/** Latencia simulada de transferencia SPI/DMA (stub host, < BSP_SPI_TIMEOUT_US). */
#define BSP_IMU_MOCK_TRANSFER_US  350U
#define BSP_BARO_MOCK_TRANSFER_US 280U

typedef struct {
    int32_t pressure_raw;
    int16_t temperature_raw;
} BaroBurstRaw;

static BspSensorsBusStatus g_bus_status{};
static volatile uint32_t g_bsp_spi_time_us = 0U;
static uint8_t g_imu_spi_tx[BSP_IMU_SPI_BURST_LENGTH + 1U];
static uint8_t g_imu_spi_rx[BSP_IMU_SPI_BURST_LENGTH + 1U];
static AmbiqDmaTransaction g_imu_dma{};
static float g_imu_mock_phase_rad = 0.0f;
static uint8_t g_baro_spi_tx[BSP_BARO_SPI_BURST_LENGTH + 1U];
static uint8_t g_baro_spi_rx[BSP_BARO_SPI_BURST_LENGTH + 1U];
static AmbiqDmaTransaction g_baro_dma{};
static float g_baro_mock_pressure_pa = 101325.0f;

static bool bsp_ctimer_stub_init(void)
{
    /*
     * TODO(Ambiq): am_hal_ctimer_config(AM_HAL_CTIMER_BOTH, ...);
     * Periodo = 1 / BSP_SENSORS_CTIMER_HZ → 100 ms @ 10 Hz.
     */
    g_bus_status.ctimer_armed = true;
    g_bus_status.interrupt_pending = false;
    return true;
}

static bool bsp_ctimer_consume_wake(void)
{
    /*
     * TODO(Ambiq): verificar NVIC CTIMER/STimer y limpiar IRQ pending.
     * Stub host: cada llamada tras WaitNextTick se considera despertar valido.
     */
    if (!g_bus_status.ctimer_armed) {
        return false;
    }

    g_bus_status.interrupt_pending = true;
    return true;
}

static void bsp_ctimer_ack_interrupt(void)
{
    g_bus_status.interrupt_pending = false;
}

static bool bsp_spi_bus_claim(BspSpiBusState *bus_state)
{
    if (bus_state == NULL) {
        return false;
    }

    if (*bus_state == BSP_SPI_BUS_DMA_ACTIVE) {
        return false;
    }

    if (g_bus_status.imu == BSP_SPI_BUS_DMA_ACTIVE ||
        g_bus_status.baro == BSP_SPI_BUS_DMA_ACTIVE) {
        return false;
    }

    *bus_state = BSP_SPI_BUS_DMA_ACTIVE;
    return true;
}

static void bsp_spi_bus_release(BspSpiBusState *bus_state, BspSpiBusState terminal_state)
{
    if (bus_state == NULL) {
        return;
    }

    *bus_state = terminal_state;
}

static uint32_t bsp_spi_time_us(void)
{
    /*
     * TODO(Ambiq): return am_hal_stimer_counter_get() / (AM_HAL_CLKGEN_FREQ_MAX_HZ / 1000000U);
     * Stub host: contador monotono avanzado por el guard de timeout.
     */
    return g_bsp_spi_time_us;
}

static void bsp_spi_time_advance_us(uint32_t delta_us)
{
    g_bsp_spi_time_us += delta_us;
}

static uint32_t bsp_spi_mock_transfer_latency_us(BspSpiBusState *bus_state, uint32_t nominal_us)
{
    (void)bus_state;

    /*
     * Stub: cada 50 ticks fuerza una latencia > BSP_SPI_TIMEOUT_US en barometro
     * para ejercitar la ruta BSP_SPI_BUS_TIMEOUT sin bloquear el resto de ticks.
     */
    const uint32_t tick_index = Ambiq_System_GetTickIndex();
    if (bus_state == &g_bus_status.baro && ((tick_index % 50U) == 49U)) {
        return BSP_SPI_TIMEOUT_US + 500U;
    }

    return nominal_us;
}

static bool bsp_spi_transaction_wait_guard(
    AmbiqDmaTransaction *transaction,
    BspSpiBusState *bus_state,
    uint32_t transfer_latency_us)
{
    if (transaction == NULL || bus_state == NULL) {
        return false;
    }

    const uint32_t deadline_us = bsp_spi_time_us() + BSP_SPI_TIMEOUT_US;
    uint32_t simulated_us = 0U;

    while (simulated_us < transfer_latency_us) {
        if (bsp_spi_time_us() >= deadline_us) {
            ambiq_dma_abort(transaction->channel);
            bsp_spi_bus_release(bus_state, BSP_SPI_BUS_TIMEOUT);
            return false;
        }

        bsp_spi_time_advance_us(1U);
        simulated_us++;
    }

    while (transaction->status == AMBIQ_DMA_STATUS_BUSY) {
        if (bsp_spi_time_us() >= deadline_us) {
            ambiq_dma_abort(transaction->channel);
            bsp_spi_bus_release(bus_state, BSP_SPI_BUS_TIMEOUT);
            return false;
        }
    }

    if (transaction->status == AMBIQ_DMA_STATUS_ERROR) {
        bsp_spi_bus_release(bus_state, BSP_SPI_BUS_ERROR);
        return false;
    }

    if (transaction->status != AMBIQ_DMA_STATUS_COMPLETE) {
        bsp_spi_bus_release(bus_state, BSP_SPI_BUS_ERROR);
        return false;
    }

    return true;
}

static void bsp_spi_imu_init(void)
{
    memset(g_imu_spi_tx, 0, sizeof(g_imu_spi_tx));
    memset(g_imu_spi_rx, 0, sizeof(g_imu_spi_rx));

    g_imu_spi_tx[0] = (uint8_t)(BSP_IMU_REG_ACCEL_X_OUT_H | BSP_IMU_SPI_READ_BIT);

    g_imu_dma.channel = AMBIQ_DMA_CHANNEL_SPI_IMU;
    g_imu_dma.tx_buffer = g_imu_spi_tx;
    g_imu_dma.rx_buffer = g_imu_spi_rx;
    g_imu_dma.length = (uint16_t)(BSP_IMU_SPI_BURST_LENGTH + 1U);
    g_imu_dma.status = AMBIQ_DMA_STATUS_IDLE;

    g_bus_status.imu = BSP_SPI_BUS_IDLE;
    g_imu_mock_phase_rad = 0.0f;
}

static void bsp_spi_imu_fill_mock_burst(void)
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

static bool bsp_spi_imu_burst_read(ImuBurstRaw *raw_out, uint32_t *cycles_out)
{
    if (raw_out == NULL) {
        return false;
    }

    if (!bsp_spi_bus_claim(&g_bus_status.imu)) {
        return false;
    }

    bool success = false;

    /*
     * TODO(Ambiq): am_hal_iom_spi_read(g_IOMHandle, ...);
     */
    bsp_spi_imu_fill_mock_burst();

    if (!ambiq_dma_submit(&g_imu_dma)) {
        bsp_spi_bus_release(&g_bus_status.imu, BSP_SPI_BUS_ERROR);
        return false;
    }

    const uint32_t transfer_us = bsp_spi_mock_transfer_latency_us(
        &g_bus_status.imu, BSP_IMU_MOCK_TRANSFER_US);

    if (bsp_spi_transaction_wait_guard(&g_imu_dma, &g_bus_status.imu, transfer_us)) {
        raw_out->accel_raw[0] = (int16_t)((g_imu_spi_rx[1] << 8) | g_imu_spi_rx[2]);
        raw_out->accel_raw[1] = (int16_t)((g_imu_spi_rx[3] << 8) | g_imu_spi_rx[4]);
        raw_out->accel_raw[2] = (int16_t)((g_imu_spi_rx[5] << 8) | g_imu_spi_rx[6]);
        raw_out->gyro_raw[0] = (int16_t)((g_imu_spi_rx[7] << 8) | g_imu_spi_rx[8]);
        raw_out->gyro_raw[1] = (int16_t)((g_imu_spi_rx[9] << 8) | g_imu_spi_rx[10]);
        raw_out->gyro_raw[2] = (int16_t)((g_imu_spi_rx[11] << 8) | g_imu_spi_rx[12]);

        if (cycles_out != NULL) {
            *cycles_out = g_imu_dma.cycles_elapsed;
        }

        success = true;
    }

    if (success) {
        bsp_spi_bus_release(&g_bus_status.imu, BSP_SPI_BUS_IDLE);
    }

    return success;
}

static void bsp_spi_baro_init(void)
{
    memset(g_baro_spi_tx, 0, sizeof(g_baro_spi_tx));
    memset(g_baro_spi_rx, 0, sizeof(g_baro_spi_rx));

    g_baro_spi_tx[0] = (uint8_t)(BSP_BARO_REG_PRESS_OUT | BSP_BARO_SPI_READ_BIT);

    g_baro_dma.channel = AMBIQ_DMA_CHANNEL_SPI_IMU;
    g_baro_dma.tx_buffer = g_baro_spi_tx;
    g_baro_dma.rx_buffer = g_baro_spi_rx;
    g_baro_dma.length = (uint16_t)(BSP_BARO_SPI_BURST_LENGTH + 1U);
    g_baro_dma.status = AMBIQ_DMA_STATUS_IDLE;

    g_bus_status.baro = BSP_SPI_BUS_IDLE;
    g_baro_mock_pressure_pa = 101325.0f;
}

static void bsp_spi_baro_fill_mock_burst(void)
{
    g_baro_mock_pressure_pa += 2.5f;

    const int32_t pressure_raw = (int32_t)(g_baro_mock_pressure_pa / 0.01f);
    const int16_t temperature_raw = 2100;

    g_baro_spi_rx[1] = (uint8_t)((pressure_raw >> 16) & 0xFF);
    g_baro_spi_rx[2] = (uint8_t)((pressure_raw >> 8) & 0xFF);
    g_baro_spi_rx[3] = (uint8_t)(pressure_raw & 0xFF);
    g_baro_spi_rx[4] = (uint8_t)(temperature_raw >> 8);
    g_baro_spi_rx[5] = (uint8_t)(temperature_raw & 0xFF);
}

static bool bsp_spi_baro_burst_read(BaroBurstRaw *raw_out, uint32_t *cycles_out)
{
    if (raw_out == NULL) {
        return false;
    }

    if (!bsp_spi_bus_claim(&g_bus_status.baro)) {
        return false;
    }

    bool success = false;

    /*
     * TODO(Ambiq): am_hal_iom_spi_read en IOM secundario (MS5611 / BMP390 class).
     */
    bsp_spi_baro_fill_mock_burst();

    if (!ambiq_dma_submit(&g_baro_dma)) {
        bsp_spi_bus_release(&g_bus_status.baro, BSP_SPI_BUS_ERROR);
        return false;
    }

    const uint32_t transfer_us = bsp_spi_mock_transfer_latency_us(
        &g_bus_status.baro, BSP_BARO_MOCK_TRANSFER_US);

    if (bsp_spi_transaction_wait_guard(&g_baro_dma, &g_bus_status.baro, transfer_us)) {
        raw_out->pressure_raw =
            (int32_t)((g_baro_spi_rx[1] << 16) | (g_baro_spi_rx[2] << 8) | g_baro_spi_rx[3]);
        raw_out->temperature_raw = (int16_t)((g_baro_spi_rx[4] << 8) | g_baro_spi_rx[5]);
        success = true;

        if (cycles_out != NULL) {
            *cycles_out = g_baro_dma.cycles_elapsed;
        }
    }

    if (success) {
        bsp_spi_bus_release(&g_bus_status.baro, BSP_SPI_BUS_IDLE);
    }

    return success;
}

static bool bsp_spi_baro_raw_to_pressure_sample(const BaroBurstRaw *raw, PressureSample *sample_out)
{
    if (raw == NULL || sample_out == NULL) {
        return false;
    }

    const float pressure_pa = (float)raw->pressure_raw * 0.01f;
    const float temperature_c = ((float)raw->temperature_raw * 0.01f) - 50.0f;

    sample_out->pressure_pa = pressure_pa;
    sample_out->temperature_c = temperature_c;
    sample_out->valid = true;
    return true;
}

static bool bsp_spi_baro_read_pressure(PressureSample *pressure_out, uint32_t *cycles_out)
{
    if (pressure_out == NULL) {
        return false;
    }

    BaroBurstRaw raw{};
    if (!bsp_spi_baro_burst_read(&raw, cycles_out)) {
        pressure_out->valid = false;
        return false;
    }

    return bsp_spi_baro_raw_to_pressure_sample(&raw, pressure_out);
}

static float bsp_baro_pressure_to_altitude_m(float pressure_pa)
{
    const float sea_level_pa = 101325.0f;
    return 44330.0f * (1.0f - (pressure_pa / sea_level_pa));
}

static void bsp_baro_pressure_to_gps_sample(const PressureSample *pressure, GpsSample *gps_out)
{
    if (pressure == NULL || gps_out == NULL || !pressure->valid) {
        if (gps_out != NULL) {
            gps_out->fix_valid = false;
        }
        return;
    }

    memset(gps_out, 0, sizeof(*gps_out));
    gps_out->fix_valid = true;
    gps_out->position = vector3d_make(0.0f, 0.0f, bsp_baro_pressure_to_altitude_m(pressure->pressure_pa));
    gps_out->speed_mps = 0.0f;
    gps_out->course_deg = 0.0f;
    gps_out->satellites = 0U;
    gps_out->timestamp_ms = pressure->timestamp_ms;
}

bool bsp_sensors_init(void)
{
    memset(&g_bus_status, 0, sizeof(g_bus_status));

    if (!bsp_ctimer_stub_init()) {
        return false;
    }

    ambiq_dma_init();
    ambiq_spi_imu_init();
    bsp_spi_imu_init();
    bsp_spi_baro_init();
    g_bsp_spi_time_us = 0U;
    ambiq_gpio_gnss_init();
    if (!ambiq_uart_telemetry_init()) {
        return false;
    }
    ambiq_power_monitor_init();

    g_bus_status.imu = BSP_SPI_BUS_IDLE;
    g_bus_status.baro = BSP_SPI_BUS_IDLE;

    return g_bus_status.ctimer_armed;
}

void bsp_sensors_get_bus_status(BspSensorsBusStatus *status_out)
{
    if (status_out == NULL) {
        return;
    }

    *status_out = g_bus_status;
}

bool bsp_sensors_orchestrate_tick(DeadReckoningFilter *nav_filter)
{
    if (nav_filter == NULL) {
        return false;
    }

    if (!bsp_ctimer_consume_wake()) {
        return false;
    }

    const uint32_t timestamp_ms = Ambiq_System_GetTickIndex() * AMBIQ_TICK_INTERVAL_MS;

    ImuSample imu{};
    GpsSample gps{};
    PressureSample baro{};

    Ambiq_BSP_ReadIMU(&imu);
    imu.timestamp_ms = timestamp_ms;

    uint32_t baro_cycles = 0U;
    if (bsp_spi_baro_read_pressure(&baro, &baro_cycles)) {
        baro.timestamp_ms = timestamp_ms;
        ambiq_power_add_cycles(baro_cycles + 180U);
    } else {
        baro.valid = false;
    }

    Ambiq_BSP_ReadGNSS(&gps);
    gps.timestamp_ms = timestamp_ms;

    if (!imu.valid) {
        bsp_ctimer_ack_interrupt();
        return false;
    }

    dead_reckoning_update_imu(nav_filter, &imu, NULL);

    if (gps.fix_valid) {
        dead_reckoning_update_gps(nav_filter, &gps, NULL);
    } else if (baro.valid) {
        GpsSample baro_gps{};
        bsp_baro_pressure_to_gps_sample(&baro, &baro_gps);
        baro_gps.timestamp_ms = timestamp_ms;
        dead_reckoning_update_gps(nav_filter, &baro_gps, NULL);
    }

    if (baro.valid && nav_filter->state.domain == NAVICORE_DOMAIN_SEA) {
        dead_reckoning_update_pressure(nav_filter, &baro, 101325.0f);
    }

    nav_filter->state.timestamp_ms = timestamp_ms;

    Ambiq_BSP_TransmitState(&nav_filter->state);
    bsp_ctimer_ack_interrupt();

    return true;
}

void Ambiq_BSP_ReadIMU(NaviCore::IMUMeasurement *imu_out)
{
    if (imu_out == NULL) {
        return;
    }

    ImuBurstRaw raw{};
    uint32_t spi_cycles = 0U;

    if (bsp_spi_imu_burst_read(&raw, &spi_cycles) &&
        ambiq_spi_imu_raw_to_sample(&raw, imu_out, 0.0f)) {
        ambiq_power_add_cycles(spi_cycles + 330U);
    } else {
        imu_out->valid = false;
    }
}

void Ambiq_BSP_ReadGNSS(NaviCore::GNSSMeasurement *gnss_out)
{
    if (gnss_out == NULL) {
        return;
    }

    const uint32_t tick_index = Ambiq_System_GetTickIndex();

    if (!ambiq_gpio_gnss_read_fix(gnss_out, tick_index)) {
        gnss_out->fix_valid = false;
        return;
    }

    if (gnss_out->fix_valid) {
        ambiq_power_set_current_ua(15.6f);
        ambiq_power_add_cycles(820U);
    } else {
        ambiq_power_set_current_ua(4.2f);
    }
}

void Ambiq_BSP_TransmitState(const NaviCore::NavState *state_in)
{
    if (state_in == NULL) {
        return;
    }

    if (ambiq_uart_transmit_navstate(state_in)) {
        ambiq_power_add_cycles(210U);
    }
}

void Ambiq_BSP_GetPowerMetrics(PowerMetrics *metrics_out)
{
    ambiq_power_get_metrics(metrics_out);
}
