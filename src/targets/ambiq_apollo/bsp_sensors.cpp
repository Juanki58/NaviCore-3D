/**
 * @file bsp_sensors.cpp
 * @brief HAL/BSP — orquesta drivers estructurales (SPI/DMA, GPIO, UART, power)
 */
#include "bsp_sensors.hpp"

#include "ambiq_system.hpp"
#include "drivers/ambiq_driver_config.hpp"
#include "drivers/ambiq_gpio_gnss.hpp"
#include "drivers/ambiq_power_monitor.hpp"
#include "drivers/ambiq_spi_imu.hpp"
#include "drivers/ambiq_uart_telemetry.hpp"

void Ambiq_BSP_ReadIMU(NaviCore::IMUMeasurement *imu_out)
{
    if (imu_out == NULL) {
        return;
    }

    ImuBurstRaw raw{};
    uint32_t spi_cycles = 0U;

    if (!ambiq_spi_imu_burst_read(&raw, &spi_cycles)) {
        imu_out->valid = false;
        return;
    }

    if (!ambiq_spi_imu_raw_to_sample(&raw, imu_out, 0.0f)) {
        imu_out->valid = false;
        return;
    }

    ambiq_power_add_cycles(spi_cycles + 330U);
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
