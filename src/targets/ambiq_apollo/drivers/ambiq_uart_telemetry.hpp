/**
 * @file ambiq_uart_telemetry.hpp
 * @brief UART telemetria NavState (FIFO / DMA TX)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"

bool ambiq_uart_transmit_navstate(const NavState *state);
