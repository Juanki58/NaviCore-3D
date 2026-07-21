#include "bsp_gnss.hpp"
#include "bsp_uart_rx_ring.hpp"
#include "hw_config.hpp"

#include "../../core/nmea_parser.hpp"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <string.h>

namespace {

NmeaLineAssembler g_nmea{};
char g_pending_line[PICO2_GNSS_LINE_MAX];
bool g_line_pending = false;

GnssUartRxRing g_rx_ring;

uint32_t g_last_overflow_count = 0U;

void gnss_check_overflow_contamination(void)
{
    const uint32_t overflow_total = g_rx_ring.overflow_count_load();
    if (overflow_total > g_last_overflow_count) {
        g_last_overflow_count = overflow_total;
        nmea_line_assembler_mark_contaminated(&g_nmea);
        const uint32_t irq_state = save_and_disable_interrupts();
        g_line_pending = false;
        memset(g_pending_line, 0, sizeof(g_pending_line));
        restore_interrupts(irq_state);
    }
}

void gnss_uart_drain_hw_to_ring(void)
{
    while (uart_is_readable(PICO2_GNSS_UART)) {
        g_rx_ring.push(static_cast<uint8_t>(uart_getc(PICO2_GNSS_UART)));
    }
    uart_get_hw(PICO2_GNSS_UART)->icr = UART_UARTICR_RXIC_BITS;
}

} /* namespace */

static void gnss_uart_rx_isr(void)
{
    gnss_uart_drain_hw_to_ring();
}

bool pico2_bsp_gnss_init(void)
{
    g_rx_ring = GnssUartRxRing{};
    nmea_line_assembler_reset(&g_nmea);

    uart_init(PICO2_GNSS_UART, PICO2_GNSS_UART_BAUD);
    gpio_set_function(PICO2_GNSS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO2_GNSS_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enable(PICO2_GNSS_UART, true);

    irq_set_exclusive_handler(
        PICO2_GNSS_UART == uart0 ? UART0_IRQ : UART1_IRQ,
        gnss_uart_rx_isr);
    irq_set_enabled(PICO2_GNSS_UART == uart0 ? UART0_IRQ : UART1_IRQ, true);
    uart_set_irq_enables(PICO2_GNSS_UART, true, false);

    g_line_pending = false;
    g_last_overflow_count = 0U;
    memset(g_pending_line, 0, sizeof(g_pending_line));
    return true;
}

bool pico2_bsp_gnss_rx_pending(void)
{
    return !g_rx_ring.empty();
}

void pico2_bsp_gnss_rx_pump(uint16_t byte_budget)
{
    gnss_check_overflow_contamination();

    uint8_t byte = 0U;
    char line[PICO2_GNSS_LINE_MAX];
    while (byte_budget > 0U && g_rx_ring.pop(&byte)) {
        if (nmea_line_assembler_feed(&g_nmea, byte, line, sizeof(line))) {
            const uint32_t irq_state = save_and_disable_interrupts();
            if (!g_line_pending) {
                memcpy(g_pending_line, line, sizeof(g_pending_line));
                g_line_pending = true;
            }
            restore_interrupts(irq_state);
        }
        --byte_budget;
    }
}

bool pico2_bsp_gnss_poll(GpsSample *gps_out)
{
    if (!g_line_pending || g_pending_line[0] != '$') {
        return false;
    }

    const bool parsed = nmea_try_parse_sentence(g_pending_line, gps_out);
    const uint32_t irq_state = save_and_disable_interrupts();
    g_line_pending = false;
    memset(g_pending_line, 0, sizeof(g_pending_line));
    restore_interrupts(irq_state);
    return parsed;
}

uint32_t pico2_bsp_gnss_rx_overflow_count(void)
{
    return g_rx_ring.overflow_count_load();
}
