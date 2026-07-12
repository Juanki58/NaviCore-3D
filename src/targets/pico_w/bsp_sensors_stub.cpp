/**
 * @file bsp_sensors_stub.cpp
 * @brief Lectura de sensores Pico W — stub hasta conectar SPI IMU + UART GNSS
 */
#include "bsp_sensors_stub.hpp"

#include "../../core/sensor_types.hpp"
#include "../../core/vector3d.h"

#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <math.h>
#include <string.h>

namespace {

float g_mock_phase_rad = 0.0f;
uint32_t g_tick_count = 0U;

} /* namespace */

bool pico_bsp_sensors_init(void)
{
    /*
     * Fase 2: spi_init(spi0, 1 MHz), gpio_set_function para SCK/MOSI/MISO/CS
     *         uart_init(uart0, 9600), gpio_set_function para TX/RX GNSS
     */
    g_mock_phase_rad = 0.0f;
    g_tick_count = 0U;
    return true;
}

bool pico_bsp_sensors_tick(DeadReckoningFilter *nav_filter, uint32_t timestamp_ms, bool *gps_fix_valid_out)
{
    if (nav_filter == nullptr) {
        return false;
    }

    g_tick_count++;
    g_mock_phase_rad += 0.01f;

    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = timestamp_ms;
    imu.accel_mps2[0] = 0.05f * sinf(g_mock_phase_rad);
    imu.accel_mps2[1] = 0.05f * cosf(g_mock_phase_rad);
    imu.accel_mps2[2] = 9.81f;
    imu.gyro_radps[0] = 0.0f;
    imu.gyro_radps[1] = 0.0f;
    imu.gyro_radps[2] = 0.002f;

    dead_reckoning_update_imu(nav_filter, &imu, nullptr);

    const bool gps_fix_valid = ((g_tick_count % 100U) >= 10U);
    if (gps_fix_valid_out != nullptr) {
        *gps_fix_valid_out = gps_fix_valid;
    }

    if (gps_fix_valid) {
        GpsSample gps{};
        gps.fix_valid = true;
        gps.timestamp_ms = timestamp_ms;
        gps.position = vector3d_make(41.2606f, 1.6769f, 12.0f);
        gps.speed_mps = 8.0f;
        gps.course_deg = 90.0f;
        gps.satellites = 12U;
        dead_reckoning_update_gps(nav_filter, &gps, nullptr);
    }

    nav_filter->state.timestamp_ms = timestamp_ms;
    return true;
}
