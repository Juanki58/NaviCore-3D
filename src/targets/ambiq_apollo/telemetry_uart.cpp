/**
 * @file telemetry_uart.cpp
 * @brief Formateo CSV sin heap y TX UART simulada para el target Ambiq Apollo
 */
#include "telemetry_uart.hpp"

#include "ambiq_uart_telemetry.hpp"
#include "drivers/ambiq_driver_config.hpp"

#include <stdio.h>
#include <stdint.h>

#define TELEMETRY_UART_BUFFER_BYTES 256U

static char g_telemetry_uart_buffer[TELEMETRY_UART_BUFFER_BYTES];

static bool telemetry_uart_format_csv_line(
    const DeadReckoningFilter *filter,
    const SystemHealthMonitor *monitor,
    uint32_t timestamp_ms,
    int *written_out)
{
    if (filter == NULL || written_out == NULL) {
        return false;
    }

    const NavState *state = &filter->state;

    uint8_t health_score = 0U;
    uint8_t health_mode = (uint8_t)HEALTH_NOMINAL;
    uint8_t bsp_bus_status = DIAG_BSP_BUS_IDLE;

    if (monitor != NULL) {
        health_score = monitor->health_score;
        health_mode = (uint8_t)monitor->mode;
        bsp_bus_status = monitor->last_bsp_bus_status;
    }

    const uint8_t odom_fault = dead_reckoning_has_odom_fault(filter) ? 1U : 0U;
    const uint8_t filter_quality = (uint8_t)dead_reckoning_get_quality(filter);

    const int written = snprintf(
        g_telemetry_uart_buffer,
        TELEMETRY_UART_BUFFER_BYTES,
        "%u,%u,%.4f,%u,%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%.1f,%.4f,%.4f,%u,%u,%u,%u,%u\r\n",
        timestamp_ms,
        (unsigned)state->mode,
        state->confidence.estimate_quality,
        (unsigned)state->confidence.satellites,
        state->position.x,
        state->position.y,
        state->position.z,
        state->velocity.x,
        state->velocity.y,
        state->velocity.z,
        state->heading_deg,
        filter->roll_rad,
        filter->pitch_rad,
        (unsigned)health_score,
        (unsigned)health_mode,
        (unsigned)bsp_bus_status,
        (unsigned)odom_fault,
        (unsigned)filter_quality);

    if (written <= 0 || written >= (int)TELEMETRY_UART_BUFFER_BYTES) {
        return false;
    }

    *written_out = written;
    return true;
}

static bool telemetry_uart_submit_transfer(int byte_count)
{
    if (byte_count <= 0) {
        return false;
    }

    (void)byte_count;
    ambiq_uart_write_string(g_telemetry_uart_buffer);
    return true;
}

bool telemetry_uart_stream(
    const DeadReckoningFilter *filter,
    const SystemHealthMonitor *monitor,
    uint32_t timestamp_ms)
{
    int written = 0;

    if (!telemetry_uart_format_csv_line(filter, monitor, timestamp_ms, &written)) {
        return false;
    }

    return telemetry_uart_submit_transfer(written);
}
