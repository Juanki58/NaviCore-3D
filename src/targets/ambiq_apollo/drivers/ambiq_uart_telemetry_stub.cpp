/**
 * @file ambiq_uart_telemetry_stub.cpp
 * @brief Formateo NavState ASCII — delega TX a ambiq_uart_write_string
 */
#include "../ambiq_uart_telemetry.hpp"
#include "ambiq_uart_telemetry.hpp"

#include "ambiq_driver_config.hpp"

#include <stdio.h>

static char g_uart_tx_frame[AMBIQ_TELEM_FRAME_MAX];

bool ambiq_uart_transmit_navstate(const NavState *state)
{
    if (state == NULL) {
        return false;
    }

    const int written = snprintf(
        g_uart_tx_frame,
        AMBIQ_TELEM_FRAME_MAX,
        "NAV,t=%u,m=%u,q=%.3f,px=%.6f,py=%.6f,pz=%.1f,vx=%.2f,vy=%.2f,vz=%.2f,h=%.1f\r\n",
        state->timestamp_ms,
        (unsigned)state->mode,
        state->confidence.estimate_quality,
        state->position.x,
        state->position.y,
        state->position.z,
        state->velocity.x,
        state->velocity.y,
        state->velocity.z,
        state->heading_deg);

    if (written <= 0 || written >= AMBIQ_TELEM_FRAME_MAX) {
        return false;
    }

    ambiq_uart_write_string(g_uart_tx_frame);
    return true;
}
