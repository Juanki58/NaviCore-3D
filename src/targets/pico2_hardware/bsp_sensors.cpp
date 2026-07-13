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
SensorConfidenceFlags g_sensor_confidence{};

struct UartOverflowWindow {
    uint32_t window_start_ms = 0U;
    uint32_t last_overflow_total = 0U;
    uint16_t events_in_window = 0U;
    bool confidence_degraded = false;
};

UartOverflowWindow g_imu_overflow_window{};
UartOverflowWindow g_gnss_overflow_window{};

float sensors_clampf(float value, float min_v, float max_v)
{
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

void sensors_overflow_window_update(
    UartOverflowWindow *window,
    uint8_t uart_id,
    uint32_t timestamp_ms)
{
    if (window == nullptr) {
        return;
    }

    const uint32_t total = pico2_bsp_uart_get_overflow_count(uart_id);
    if (total > window->last_overflow_total) {
        window->events_in_window = static_cast<uint16_t>(
            window->events_in_window + (total - window->last_overflow_total));
        window->last_overflow_total = total;
    }

    if (window->window_start_ms == 0U) {
        window->window_start_ms = timestamp_ms;
    }

    if ((timestamp_ms - window->window_start_ms) >= PICO2_RING_OVERFLOW_WINDOW_MS) {
        window->window_start_ms = timestamp_ms;
        window->events_in_window = 0U;
        window->confidence_degraded = false;
    }

    if (window->events_in_window >= PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD) {
        window->confidence_degraded = true;
    }
}

void sensors_update_confidence_flags(uint32_t timestamp_ms)
{
    sensors_overflow_window_update(
        &g_imu_overflow_window,
        PICO2_UART_ID_IMU,
        timestamp_ms);
    sensors_overflow_window_update(
        &g_gnss_overflow_window,
        PICO2_UART_ID_GNSS,
        timestamp_ms);

    g_sensor_confidence.imu_degraded = g_imu_overflow_window.confidence_degraded;
    g_sensor_confidence.gnss_degraded = g_gnss_overflow_window.confidence_degraded;
}

void sensors_apply_degraded_confidence(DeadReckoningFilter *nav_filter)
{
    if (nav_filter == nullptr) {
        return;
    }

    if (!g_sensor_confidence.imu_degraded && !g_sensor_confidence.gnss_degraded) {
        return;
    }

    float quality = nav_filter->state.confidence.estimate_quality;

    if (g_sensor_confidence.imu_degraded) {
        quality *= PICO2_RING_DEGRADED_QUALITY_FACTOR;
    }

    if (g_sensor_confidence.gnss_degraded) {
        nav_filter->state.confidence.gps_trusted = false;
        quality *= PICO2_RING_DEGRADED_QUALITY_FACTOR;
    }

    nav_filter->state.confidence.estimate_quality = sensors_clampf(quality, 0.0f, 1.0f);
}

} /* namespace */

bool pico2_bsp_sensors_init(void)
{
    g_sensor_confidence = SensorConfidenceFlags{};
    g_imu_overflow_window = UartOverflowWindow{};
    g_gnss_overflow_window = UartOverflowWindow{};

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

uint32_t pico2_bsp_uart_get_overflow_count(uint8_t uart_id)
{
    switch (uart_id) {
    case PICO2_UART_ID_IMU:
        return pico2_bsp_wt61c_rx_overflow_count();
    case PICO2_UART_ID_GNSS:
        return pico2_bsp_gnss_rx_overflow_count();
    default:
        return 0U;
    }
}

void pico2_bsp_sensors_get_confidence_flags(SensorConfidenceFlags *flags_out)
{
    if (flags_out == nullptr) {
        return;
    }

    *flags_out = g_sensor_confidence;
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

    sensors_update_confidence_flags(timestamp_ms);

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

    sensors_apply_degraded_confidence(nav_filter);

    nav_filter->state.timestamp_ms = timestamp_ms;
    return true;
}
