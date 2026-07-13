#include "bsp_sensors.hpp"

#include "bsp_gnss.hpp"
#include "bsp_power.hpp"
#include "bsp_wt61c.hpp"
#include "hw_config.hpp"

#include "../../core/sensor_types.hpp"

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <stdio.h>

namespace {

uint32_t g_tick_count = 0U;
uint8_t g_battery_low_streak = 0U;
bool g_battery_low_latched = false;

} /* namespace */

bool pico2_bsp_sensors_init(void)
{
    if (!pico2_bsp_power_init()) {
        printf("Aviso: UPS I2C no responde (addr 0x%02X)\n", PICO2_POWER_I2C_ADDR);
    }

    if (!pico2_bsp_wt61c_init()) {
        printf("Error: WT61C UART0 init\n");
        return false;
    }

    if (!pico2_bsp_gnss_init()) {
        printf("Error: NEO-M9N UART1 init\n");
        return false;
    }

    printf(
        "BSP Comarruga: WT61C @ UART%d %u baud | NEO-M9N @ UART%d %u baud | UPS I2C1\n",
        PICO2_IMU_UART == uart0 ? 0 : 1,
        PICO2_IMU_UART_BAUD,
        PICO2_GNSS_UART == uart0 ? 0 : 1,
        PICO2_GNSS_UART_BAUD);

    return true;
}

void pico2_bsp_sensors_rx_pump(void)
{
    for (uint8_t round = 0U; round < PICO2_UART_PUMP_MAX_ROUNDS; ++round) {
        bool activity = false;

        if (pico2_bsp_wt61c_rx_pending()) {
            pico2_bsp_wt61c_rx_pump(PICO2_UART_RX_BUDGET);
            activity = true;
        }
        if (pico2_bsp_gnss_rx_pending()) {
            pico2_bsp_gnss_rx_pump(PICO2_UART_RX_BUDGET);
            activity = true;
        }

        if (!activity) {
            break;
        }
    }
}

void pico2_bsp_sensors_housekeeping(uint32_t nav_tick_count)
{
    if (pico2_bsp_power_is_offline()) {
        return;
    }

    pico2_bsp_power_poll(nav_tick_count);

    uint16_t battery_mv = 0U;
    if (!pico2_bsp_power_consume_battery(&battery_mv) || battery_mv == 0U) {
        return;
    }

    if (battery_mv < PICO2_BATTERY_LOW_MV) {
        if (g_battery_low_streak < 255U) {
            ++g_battery_low_streak;
        }
    } else if (battery_mv >= PICO2_BATTERY_LOW_CLEAR_MV) {
        g_battery_low_streak = 0U;
        g_battery_low_latched = false;
    }

    if (!g_battery_low_latched
        && g_battery_low_streak >= PICO2_BATTERY_LOW_DEBOUNCE) {
        g_battery_low_latched = true;
        printf("Aviso: bateria baja %u mV\n", battery_mv);
    }
}

bool pico2_bsp_sensors_can_sleep(void)
{
    return !pico2_bsp_wt61c_rx_pending()
        && !pico2_bsp_gnss_rx_pending();
}

bool pico2_bsp_sensors_tick(
    DeadReckoningFilter *nav_filter,
    uint32_t timestamp_ms,
    bool *gps_fix_valid_out)
{
    if (nav_filter == nullptr) {
        return false;
    }

    g_tick_count++;

    ImuSample imu{};
    if (pico2_bsp_wt61c_poll(&imu)) {
        imu.timestamp_ms = timestamp_ms;
        dead_reckoning_update_imu(nav_filter, &imu, nullptr);
    }

    GpsSample gps{};
    const bool gps_fix_valid = pico2_bsp_gnss_poll(&gps);
    if (gps_fix_valid_out != nullptr) {
        *gps_fix_valid_out = gps_fix_valid;
    }

    if (gps_fix_valid) {
        gps.timestamp_ms = timestamp_ms;
        dead_reckoning_update_gps(nav_filter, &gps, nullptr);
    }

    nav_filter->state.timestamp_ms = timestamp_ms;
    return true;
}
