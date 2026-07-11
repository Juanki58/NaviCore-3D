/**
 * @file ambiq_dma.hpp
 * @brief Abstraccion DMA para transferencias SPI/UART sin bloqueo del CPU
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AMBIQ_DMA_STATUS_IDLE = 0,
    AMBIQ_DMA_STATUS_BUSY,
    AMBIQ_DMA_STATUS_COMPLETE,
    AMBIQ_DMA_STATUS_ERROR
} AmbiqDmaStatus;

typedef enum {
    AMBIQ_DMA_CHANNEL_SPI_IMU = 0,
    AMBIQ_DMA_CHANNEL_UART_TX
} AmbiqDmaChannel;

typedef struct {
    AmbiqDmaChannel channel;
    uint8_t *rx_buffer;
    const uint8_t *tx_buffer;
    uint16_t length;
    volatile AmbiqDmaStatus status;
    uint32_t cycles_elapsed;
} AmbiqDmaTransaction;

void ambiq_dma_init(void);
bool ambiq_dma_submit(AmbiqDmaTransaction *transaction);
bool ambiq_dma_wait_complete(AmbiqDmaTransaction *transaction, uint32_t timeout_cycles);
void ambiq_dma_abort(AmbiqDmaChannel channel);
