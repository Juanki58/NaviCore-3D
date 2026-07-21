#include "bsp_wt61c.hpp"
#include "bsp_uart_rx_ring.hpp"
#include "hw_config.hpp"

#include "../../core/wt61c_parser.hpp"

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

namespace {

ImuUartRxRing g_rx_ring;
Wt61cStreamParser g_parser{};
uint32_t g_last_overflow_count = 0U;
uint64_t g_last_good_sample_us = 0U;

void wt61c_check_overflow_contamination(void)
{
    const uint32_t overflow_total = g_rx_ring.overflow_count_load();
    if (overflow_total > g_last_overflow_count) {
        g_last_overflow_count = overflow_total;
        wt61c_stream_mark_contaminated(&g_parser);
    }
}

void wt61c_uart_drain_hw_to_ring(void)
{
    while (uart_is_readable(PICO2_IMU_UART)) {
        g_rx_ring.push(static_cast<uint8_t>(uart_getc(PICO2_IMU_UART)));
    }
    uart_get_hw(PICO2_IMU_UART)->icr = UART_UARTICR_RXIC_BITS;
}

} /* namespace */

static void wt61c_uart_rx_isr(void)
{
    wt61c_uart_drain_hw_to_ring();
}

bool pico2_bsp_wt61c_init(void)
{
    g_rx_ring = ImuUartRxRing{};
    wt61c_stream_reset(&g_parser);

    uart_init(PICO2_IMU_UART, PICO2_IMU_UART_BAUD);
    gpio_set_function(PICO2_IMU_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO2_IMU_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enable(PICO2_IMU_UART, true);

    irq_set_exclusive_handler(
        PICO2_IMU_UART == uart0 ? UART0_IRQ : UART1_IRQ,
        wt61c_uart_rx_isr);
    irq_set_enabled(PICO2_IMU_UART == uart0 ? UART0_IRQ : UART1_IRQ, true);
    uart_set_irq_enables(PICO2_IMU_UART, true, false);

    g_last_overflow_count = 0U;
    g_last_good_sample_us = 0U;
    return true;
}

bool pico2_bsp_wt61c_rx_pending(void)
{
    return !g_rx_ring.empty();
}

void pico2_bsp_wt61c_rx_pump(uint16_t byte_budget)
{
    const uint64_t now = time_us_64();
    wt61c_check_overflow_contamination();
    wt61c_stream_check_timeout(&g_parser, now);

    uint8_t byte = 0U;
    while (byte_budget > 0U && g_rx_ring.pop(&byte)) {
        wt61c_stream_feed(&g_parser, byte, now);
        --byte_budget;
    }
}

bool pico2_bsp_wt61c_poll(ImuSample *imu_out)
{
    wt61c_stream_check_timeout(&g_parser, time_us_64());
    if (!wt61c_stream_try_sample(&g_parser, imu_out)) {
        return false;
    }
    g_last_good_sample_us = time_us_64();
    return true;
}

uint32_t pico2_bsp_wt61c_rx_overflow_count(void)
{
    return g_rx_ring.overflow_count_load();
}

uint32_t pico2_bsp_wt61c_silence_ms(void)
{
    if (g_last_good_sample_us == 0U) {
        return 0xFFFFFFFFU;
    }
    const uint64_t age_us = time_us_64() - g_last_good_sample_us;
    return static_cast<uint32_t>(age_us / 1000ULL);
}
