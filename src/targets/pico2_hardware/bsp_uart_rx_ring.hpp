#pragma once

#include <atomic>
#include <stdint.h>

#include "hw_config.hpp"

/**
 * Ring buffer SPSC: productor = ISR UART, consumidor = rx_pump() en main.
 *
 * Orden de memoria (Cortex-M33 @ -O3):
 *   push() (ISR):  byte en buf[w] → w.store(release)
 *   pop()  (main): w.load(acquire) antes de leer byte
 *   r: release en pop(), acquire en push() para detectar lleno
 */
template <uint16_t Size>
struct UartRxRing {
    static_assert((Size & (Size - 1U)) == 0U, "UartRxRing: Size debe ser potencia de 2");

    uint8_t buf[Size] = {};
    std::atomic<uint16_t> w{0U};
    std::atomic<uint16_t> r{0U};
    std::atomic<uint32_t> overflow_count{0U};

    void push(uint8_t byte)
    {
        const uint16_t wi = w.load(std::memory_order_relaxed);
        const uint16_t next = static_cast<uint16_t>((wi + 1U) & (Size - 1U));
        const uint16_t ri = r.load(std::memory_order_acquire);
        if (next == ri) {
            overflow_count.fetch_add(1U, std::memory_order_relaxed);
            return;
        }

        buf[wi] = byte;
        w.store(next, std::memory_order_release);
    }

    bool pop(uint8_t *byte_out)
    {
        if (byte_out == nullptr) {
            return false;
        }

        const uint16_t ri = r.load(std::memory_order_relaxed);
        const uint16_t wi = w.load(std::memory_order_acquire);
        if (ri == wi) {
            return false;
        }

        *byte_out = buf[ri];
        r.store(
            static_cast<uint16_t>((ri + 1U) & (Size - 1U)),
            std::memory_order_release);
        return true;
    }

    bool empty(void) const
    {
        return r.load(std::memory_order_acquire)
            == w.load(std::memory_order_acquire);
    }

    uint16_t count(void) const
    {
        const uint16_t wi = w.load(std::memory_order_acquire);
        const uint16_t ri = r.load(std::memory_order_acquire);
        if (wi >= ri) {
            return static_cast<uint16_t>(wi - ri);
        }
        return static_cast<uint16_t>(Size - ri + wi);
    }

    uint32_t overflow_count_load(void) const
    {
        return overflow_count.load(std::memory_order_relaxed);
    }
};

using ImuUartRxRing = UartRxRing<PICO2_IMU_UART_RING_SIZE>;
using GnssUartRxRing = UartRxRing<PICO2_GNSS_UART_RING_SIZE>;
