/**
 * @file telemetry_uart.cpp
 * @brief Formateo CSV sin heap y TX UART simulada para el target Ambiq Apollo
 */
#include "telemetry_uart.hpp"

#include "drivers/ambiq_driver_config.hpp"

#include <stdio.h>
#include <stdint.h>

#define TELEMETRY_UART_BUFFER_BYTES 256U

#define TELEMETRY_UART_HAL_STATUS_SUCCESS 0U
#define TELEMETRY_UART_HAL_STATUS_FAIL    1U

/*
 * Shim minimo de am_hal_uart_transfer para builds host/stub.
 * En silicio: enlazar contra am_hal_uart.h del SDK Ambiq y eliminar este bloque.
 */
typedef struct {
    uint8_t *pui8Data;
    uint32_t ui32NumBytes;
    uint32_t ui32TimeoutMs;
    uint32_t ui32ErrorStatus;
} am_hal_uart_transfer_t;

extern "C" uint32_t am_hal_uart_transfer(void *pHandle, am_hal_uart_transfer_t *psTransfer)
{
    if (pHandle == NULL || psTransfer == NULL || psTransfer->pui8Data == NULL) {
        return TELEMETRY_UART_HAL_STATUS_FAIL;
    }

    if (psTransfer->ui32NumBytes == 0U) {
        return TELEMETRY_UART_HAL_STATUS_FAIL;
    }

    /*
     * Simulacion: los bytes del buffer se copian al registro/FIFO de TX del UART BSP.
     * TODO(Ambiq): sustituir por am_hal_uart_transfer real y esperar ISR TX complete.
     */
    psTransfer->ui32ErrorStatus = 0U;
    return TELEMETRY_UART_HAL_STATUS_SUCCESS;
}

static char g_telemetry_uart_buffer[TELEMETRY_UART_BUFFER_BYTES];
static void *g_telemetry_uart_handle = NULL;

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

    /*
     * TODO(Ambiq): g_telemetry_uart_handle = resultado de
     *              am_hal_uart_initialize(AMBIQ_UART_TELEM_INSTANCE, ...);
     */
    (void)AMBIQ_UART_TELEM_INSTANCE;
    (void)AMBIQ_UART_TELEM_BAUD;

    am_hal_uart_transfer_t transfer{};
    transfer.pui8Data = (uint8_t *)g_telemetry_uart_buffer;
    transfer.ui32NumBytes = (uint32_t)byte_count;
    transfer.ui32TimeoutMs = 0U;
    transfer.ui32ErrorStatus = 0U;

    const uint32_t status = am_hal_uart_transfer(g_telemetry_uart_handle, &transfer);
    return (status == TELEMETRY_UART_HAL_STATUS_SUCCESS);
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
