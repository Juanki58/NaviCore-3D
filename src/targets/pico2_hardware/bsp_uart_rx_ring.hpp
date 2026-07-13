#pragma once

#include <stdint.h>

#include "hw_config.hpp"

/**
 * Ring buffer SPSC: productor = ISR UART, consumidor = rx_pump() en main.
 * Tamaño >> 32 bytes (FIFO PL011) para absorber bloqueos de I2C/Wi-Fi en core 0.
 */
template <uint16_t Size>
struct UartRxRing {
    static_assert((Size & (Size - 1U)) == 0U, "UartRxRing: Size debe ser potencia de 2");

    uint8_t buf[Size] = {};
    volatile uint16_t w = 0U;
    volatile uint16_t r = 0U;
    volatile uint32_t overflow = 0U;

    void push(uint8_t byte)
    {
        const uint16_t next = static_cast<uint16_t>((w + 1U) & (Size - 1U));
        if (next == r) {
            ++overflow;
            return;
        }
        buf[w] = byte;
        w = next;
    }

    bool pop(uint8_t *byte_out)
    {
        if (r == w || byte_out == nullptr) {
            return false;
        }
        *byte_out = buf[r];
        r = static_cast<uint16_t>((r + 1U) & (Size - 1U));
        return true;
    }

    bool empty(void) const
    {
        return r == w;
    }

    uint16_t count(void) const
    {
        if (w >= r) {
            return static_cast<uint16_t>(w - r);
        }
        return static_cast<uint16_t>(Size - r + w);
    }
};

using ImuUartRxRing = UartRxRing<PICO2_IMU_UART_RING_SIZE>;
using GnssUartRxRing = UartRxRing<PICO2_GNSS_UART_RING_SIZE>;
