/**
 * @file ambiq_uart_telemetry_stub.cpp
 * @brief Stub UART + DMA TX para telemetria civil
 */
#include "ambiq_uart_telemetry.hpp"

#include "ambiq_dma.hpp"
#include "ambiq_driver_config.hpp"

#include <stdio.h>
#include <string.h>

static char g_uart_tx_frame[AMBIQ_TELEM_FRAME_MAX];
static AmbiqDmaTransaction g_uart_dma{};

void ambiq_uart_telemetry_init(void)
{
    g_uart_dma.channel = AMBIQ_DMA_CHANNEL_UART_TX;
    g_uart_dma.tx_buffer = (const uint8_t *)g_uart_tx_frame;
    g_uart_dma.rx_buffer = NULL;
    g_uart_dma.length = 0U;
    g_uart_dma.status = AMBIQ_DMA_STATUS_IDLE;

    /* TODO(Ambiq): am_hal_uart_initialize(AMBIQ_UART_TELEM_INSTANCE, ...); */
}

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

    g_uart_dma.length = (uint16_t)written;

    if (!ambiq_dma_submit(&g_uart_dma)) {
        return false;
    }

    /* TODO(Ambiq): am_hal_uart_transfer(...); esperar ISR TX complete */
    return ambiq_dma_wait_complete(&g_uart_dma, AMBIQ_DMA_TIMEOUT_CYCLES);
}
