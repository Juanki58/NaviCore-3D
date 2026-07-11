/**
 * @file ambiq_uart_telemetry.cpp
 * @brief Driver UART0 nativo Ambiq — configure + TX polled, stub host stdout
 */
#include "ambiq_uart_telemetry.hpp"

#include "drivers/ambiq_driver_config.hpp"

#include <stdio.h>
#include <stdint.h>

static void *g_uart_handle = NULL;
static bool g_uart_ready = false;

#ifdef NAVICORE_AMBIQ_SDK

#include "am_hal_uart.h"
#include "am_hal_pwrctrl.h"

static am_hal_uart_config_t g_uart_config = {
    .ui32BaudRate = AMBIQ_UART_TELEM_BAUD,
    .eDataBits = AM_HAL_UART_DATA_BITS_8,
    .eParity = AM_HAL_UART_PARITY_NONE,
    .eStopBits = AM_HAL_UART_ONE_STOP_BIT,
    .eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE,
    .eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
    .eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16,
};

static bool ambiq_uart_sdk_power_enable(void)
{
    return am_hal_pwrctrl_periph_enable(AM_HAL_PWRCTRL_PERIPH_UART0) ==
        AM_HAL_STATUS_SUCCESS;
}

static bool ambiq_uart_sdk_configure(void)
{
    g_uart_config.ui32BaudRate = AMBIQ_UART_TELEM_BAUD;
    g_uart_config.eDataBits = AM_HAL_UART_DATA_BITS_8;
    g_uart_config.eParity = AM_HAL_UART_PARITY_NONE;
    g_uart_config.eStopBits = AM_HAL_UART_ONE_STOP_BIT;
    g_uart_config.eFlowControl = AM_HAL_UART_FLOW_CTRL_NONE;
    g_uart_config.eTXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16;
    g_uart_config.eRXFifoLevel = AM_HAL_UART_FIFO_LEVEL_16;

    if (am_hal_uart_initialize(AMBIQ_UART_TELEM_INSTANCE, &g_uart_handle) !=
        AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    if (am_hal_uart_power_control(g_uart_handle, AM_HAL_SYSCTRL_WAKE, false) !=
        AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    if (am_hal_uart_configure(g_uart_handle, &g_uart_config) != AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    /*
     * TODO(Ambiq): am_hal_gpio_pinconfig() para TX/RX segun BSP de la placa.
     */
    return true;
}

static void ambiq_uart_sdk_write_char(char ch)
{
    if (!g_uart_ready || g_uart_handle == NULL) {
        return;
    }

    (void)am_hal_uart_char_transmit_polled(g_uart_handle, (uint8_t)ch);
}

#endif /* NAVICORE_AMBIQ_SDK */

static void ambiq_uart_host_write_char(char ch)
{
    (void)fputc(ch, stdout);
}

bool ambiq_uart_telemetry_init(void)
{
    if (g_uart_ready) {
        return true;
    }

#ifdef NAVICORE_AMBIQ_SDK
    if (!ambiq_uart_sdk_power_enable()) {
        return false;
    }

    if (!ambiq_uart_sdk_configure()) {
        return false;
    }
#endif

    g_uart_ready = true;
    return true;
}

void ambiq_uart_write_string(const char *str)
{
    if (str == NULL) {
        return;
    }

    if (!g_uart_ready) {
        if (!ambiq_uart_telemetry_init()) {
            return;
        }
    }

    for (const char *cursor = str; *cursor != '\0'; ++cursor) {
#ifdef NAVICORE_AMBIQ_SDK
        ambiq_uart_sdk_write_char(*cursor);
#else
        ambiq_uart_host_write_char(*cursor);
#endif
    }

#ifndef NAVICORE_AMBIQ_SDK
    (void)fflush(stdout);
#endif
}
