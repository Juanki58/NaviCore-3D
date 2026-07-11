/**
 * @file ambiq_dma_stub.cpp
 * @brief Stub DMA — sustituir por am_hal_dma_* del SDK Ambiq
 */
#include "ambiq_dma.hpp"

#include "ambiq_driver_config.hpp"

#include <string.h>

static AmbiqDmaTransaction *g_active_spi = NULL;
static AmbiqDmaTransaction *g_active_uart = NULL;

void ambiq_dma_init(void)
{
    g_active_spi = NULL;
    g_active_uart = NULL;
}

static bool ambiq_dma_register_active(AmbiqDmaTransaction *transaction)
{
    if (transaction == NULL || transaction->length == 0U) {
        return false;
    }

    if (transaction->channel == AMBIQ_DMA_CHANNEL_SPI_IMU) {
        if (g_active_spi != NULL && g_active_spi->status == AMBIQ_DMA_STATUS_BUSY) {
            return false;
        }
        g_active_spi = transaction;
    } else {
        if (g_active_uart != NULL && g_active_uart->status == AMBIQ_DMA_STATUS_BUSY) {
            return false;
        }
        g_active_uart = transaction;
    }

    return true;
}

bool ambiq_dma_submit(AmbiqDmaTransaction *transaction)
{
    if (!ambiq_dma_register_active(transaction)) {
        return false;
    }

    transaction->status = AMBIQ_DMA_STATUS_BUSY;
    transaction->cycles_elapsed = 0U;

    /*
     * TODO(Ambiq): am_hal_dma_transfer(...);
     * SPI RX lo rellena el periferico; UART es TX-only en este stub.
     */
    if (transaction->channel != AMBIQ_DMA_CHANNEL_SPI_IMU &&
        transaction->rx_buffer != NULL &&
        transaction->tx_buffer != NULL) {
        memcpy(transaction->rx_buffer, transaction->tx_buffer, transaction->length);
    }

    transaction->cycles_elapsed = 120U;
    transaction->status = AMBIQ_DMA_STATUS_COMPLETE;
    return true;
}

bool ambiq_dma_wait_complete(AmbiqDmaTransaction *transaction, uint32_t timeout_cycles)
{
    (void)timeout_cycles;

    if (transaction == NULL) {
        return false;
    }

    if (transaction->status == AMBIQ_DMA_STATUS_ERROR) {
        return false;
    }

    return transaction->status == AMBIQ_DMA_STATUS_COMPLETE;
}

void ambiq_dma_abort(AmbiqDmaChannel channel)
{
    if (channel == AMBIQ_DMA_CHANNEL_SPI_IMU) {
        if (g_active_spi != NULL) {
            g_active_spi->status = AMBIQ_DMA_STATUS_IDLE;
            g_active_spi = NULL;
        }
    } else if (g_active_uart != NULL) {
        g_active_uart->status = AMBIQ_DMA_STATUS_IDLE;
        g_active_uart = NULL;
    }
}
