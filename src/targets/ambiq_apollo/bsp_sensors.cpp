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

#include "vector3d.h"

#include <string.h>

#define BSP_BARO_SPI_BURST_LENGTH 6U
#define BSP_BARO_REG_PRESS_OUT    0x20U
#define BSP_BARO_SPI_READ_BIT     0x80U

typedef struct {
    int32_t pressure_raw;
    int16_t temperature_raw;
} BaroBurstRaw;

static BspSensorsBusStatus g_bus_status{};
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

static void bsp_spi_bus_release(BspSpiBusState *bus_state, bool success)
{
    if (bus_state == NULL) {
        return;
    }

    *bus_state = success ? BSP_SPI_BUS_IDLE : BSP_SPI_BUS_ERROR;
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

    if (ambiq_dma_submit(&g_baro_dma) &&
        ambiq_dma_wait_complete(&g_baro_dma, AMBIQ_DMA_TIMEOUT_CYCLES)) {
        raw_out->pressure_raw =
            (int32_t)((g_baro_spi_rx[1] << 16) | (g_baro_spi_rx[2] << 8) | g_baro_spi_rx[3]);
        raw_out->temperature_raw = (int16_t)((g_baro_spi_rx[4] << 8) | g_baro_spi_rx[5]);
        success = true;

        if (cycles_out != NULL) {
            *cycles_out = g_baro_dma.cycles_elapsed;
        }
    }

    bsp_spi_bus_release(&g_bus_status.baro, success);
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
    bsp_spi_baro_init();
    ambiq_gpio_gnss_init();
    ambiq_uart_telemetry_init();
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

    dead_reckoning_update_imu(nav_filter, &imu);

    if (gps.fix_valid) {
        dead_reckoning_update_gps(nav_filter, &gps);
    } else if (baro.valid) {
        GpsSample baro_gps{};
        bsp_baro_pressure_to_gps_sample(&baro, &baro_gps);
        baro_gps.timestamp_ms = timestamp_ms;
        dead_reckoning_update_gps(nav_filter, &baro_gps);
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

    if (!bsp_spi_bus_claim(&g_bus_status.imu)) {
        imu_out->valid = false;
        return;
    }

    ImuBurstRaw raw{};
    uint32_t spi_cycles = 0U;
    bool success = false;

    if (ambiq_spi_imu_burst_read(&raw, &spi_cycles) &&
        ambiq_spi_imu_raw_to_sample(&raw, imu_out, 0.0f)) {
        ambiq_power_add_cycles(spi_cycles + 330U);
        success = true;
    } else {
        imu_out->valid = false;
    }

    bsp_spi_bus_release(&g_bus_status.imu, success);
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
