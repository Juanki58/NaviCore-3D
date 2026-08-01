#include "bsp_sensors.hpp"

#include "bsp_gnss.hpp"
#include "bsp_imu_secondary.hpp"
#include "bsp_power.hpp"
#include "bsp_wt61c.hpp"
#include "hw_config.hpp"

#include "../../core/imu_cross_check.hpp"
#include "../../core/sensor_types.hpp"

#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <stdio.h>

#ifndef NAVICORE_INS_EKF_PI_F
#define NAVICORE_INS_EKF_PI_F 3.14159265358979323846f
#endif

namespace {

uint32_t g_tick_count = 0U;
uint8_t g_battery_low_streak = 0U;
bool g_battery_low_latched = false;
bool g_ins_seeded = false;
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
    /* imu_cross_fail is latched/cleared in sensors_tick after dual-IMU compare */

    /* Cable pull / UART hang: silence is a first-class degrade, not only overflow. */
    if (pico2_bsp_wt61c_silence_ms() >= PICO2_IMU_SILENCE_DEGRADE_MS) {
        g_sensor_confidence.imu_degraded = true;
    }
}

float sensors_course_deg_to_yaw_rad(float course_deg)
{
    return course_deg * (NAVICORE_INS_EKF_PI_F / 180.0f);
}

void sensors_apply_degraded_confidence(NavState *nav_state)
{
    if (nav_state == nullptr) {
        return;
    }

    if (!g_sensor_confidence.imu_degraded && !g_sensor_confidence.gnss_degraded
        && !g_sensor_confidence.imu_cross_fail) {
        return;
    }

    float quality = nav_state->confidence.estimate_quality;

    if (g_sensor_confidence.imu_degraded) {
        quality *= PICO2_RING_DEGRADED_QUALITY_FACTOR;
    }

    if (g_sensor_confidence.gnss_degraded) {
        nav_state->confidence.gps_trusted = false;
        quality *= PICO2_RING_DEGRADED_QUALITY_FACTOR;
    }

    if (g_sensor_confidence.imu_cross_fail) {
        quality *= PICO2_RING_DEGRADED_QUALITY_FACTOR;
    }

    nav_state->confidence.estimate_quality = sensors_clampf(quality, 0.0f, 1.0f);
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

    if (pico2_bsp_imu_secondary_init()) {
        printf("BSP: secondary IMU vigilante present (I2C0)\n");
    } else if (PICO2_SECONDARY_IMU_ENABLE) {
        printf("Aviso: secondary IMU enabled but not responding (cross-check idle)\n");
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

void pico2_bsp_sensors_set_imu_degraded(bool degraded)
{
    g_sensor_confidence.imu_degraded = degraded;
    if (degraded) {
        g_imu_overflow_window.confidence_degraded = true;
    }
}

void pico2_bsp_sensors_set_gnss_degraded(bool degraded)
{
    g_sensor_confidence.gnss_degraded = degraded;
    if (degraded) {
        g_gnss_overflow_window.confidence_degraded = true;
    }
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
    InsEkfFilter *ins_filter,
    NavState *nav_state_out,
    uint32_t timestamp_ms,
    bool *gps_fix_valid_out)
{
    if (ins_filter == nullptr || nav_state_out == nullptr) {
        return false;
    }

    g_tick_count++;

    sensors_update_confidence_flags(timestamp_ms);

    GpsSample gps{};
    const bool gps_fix_valid = pico2_bsp_gnss_poll(&gps);
    if (gps_fix_valid_out != nullptr) {
        *gps_fix_valid_out = gps_fix_valid;
    }

    if (gps_fix_valid && !g_ins_seeded) {
        ins_ekf_init(
            ins_filter,
            gps.position,
            sensors_course_deg_to_yaw_rad(gps.course_deg),
            NAVICORE_DOMAIN_AIR);
        g_ins_seeded = true;
    }

    ImuSample imu{};
    if (g_ins_seeded && pico2_bsp_wt61c_poll(&imu)) {
        imu.timestamp_ms = timestamp_ms;
        (void)ins_ekf_predict(ins_filter, &imu);

        ImuSample secondary{};
        if (pico2_bsp_imu_secondary_poll(&secondary)) {
            const ImuCrossCheckResult xcheck = imu_cross_check_evaluate(&imu, &secondary);
            g_sensor_confidence.imu_cross_fail = xcheck.disagree;
        } else {
            g_sensor_confidence.imu_cross_fail = false;
        }
    }

    const GpsSample *last_gps_ptr = nullptr;
    if (g_ins_seeded && gps_fix_valid) {
        gps.timestamp_ms = timestamp_ms;
        if (!ins_ekf_update_gnss(ins_filter, &gps)) {
            pico2_bsp_sensors_set_gnss_degraded(true);
        }
        last_gps_ptr = &gps;
    }

    if (g_ins_seeded) {
        ins_ekf_export_nav_state(ins_filter, nav_state_out, timestamp_ms, last_gps_ptr);
        sensors_apply_degraded_confidence(nav_state_out);
    } else {
        *nav_state_out = navstate_zero(NAVICORE_DOMAIN_AIR);
        nav_state_out->timestamp_ms = timestamp_ms;
        nav_state_out->mode = NAV_MODE_INITIALIZING;
    }

    return true;
}
