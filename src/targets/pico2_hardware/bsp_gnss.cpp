#include "bsp_gnss.hpp"
#include "bsp_uart_rx_ring.hpp"
#include "hw_config.hpp"

#include "../../core/vector3d.h"

#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <stdlib.h>
#include <string.h>

namespace {

char g_rx_line[PICO2_GNSS_LINE_MAX];
uint8_t g_rx_len = 0U;

char g_pending_line[PICO2_GNSS_LINE_MAX];
bool g_line_pending = false;

GnssUartRxRing g_rx_ring;

bool nmea_checksum_ok(const char *sentence)
{
    if (sentence == nullptr || sentence[0] != '$') {
        return false;
    }

    const char *star = strchr(sentence, '*');
    if (star == nullptr || star[1] == '\0' || star[2] == '\0' || star[3] != '\0') {
        return false;
    }

    uint8_t checksum = 0U;
    for (const char *p = sentence + 1; p < star; ++p) {
        const uint8_t ch = static_cast<uint8_t>(*p);
        if (ch < 0x20U || ch > 0x7EU) {
            return false;
        }
        checksum = static_cast<uint8_t>(checksum ^ ch);
    }

    char hex[3] = {star[1], star[2], '\0'};
    char *end = nullptr;
    const unsigned long expected = strtoul(hex, &end, 16);
    if (end == nullptr || *end != '\0') {
        return false;
    }

    return checksum == static_cast<uint8_t>(expected);
}

bool nmea_parse_lat_lon(const char *field, float *deg_out)
{
    if (field == nullptr || field[0] == '\0' || deg_out == nullptr) {
        return false;
    }

    char buf[16];
    strncpy(buf, field, sizeof(buf) - 1U);
    buf[sizeof(buf) - 1U] = '\0';

    char *dot = strchr(buf, '.');
    if (dot == nullptr || dot - buf < 2) {
        return false;
    }

    const int deg_digits = (dot - buf > 4) ? 3 : 2;
    char deg_part[4] = {0, 0, 0, 0};
    char min_part[16] = {0};

    strncpy(deg_part, buf, static_cast<size_t>(deg_digits));
    strncpy(min_part, buf + deg_digits, sizeof(min_part) - 1U);

    const float degrees = static_cast<float>(atof(deg_part));
    const float minutes = static_cast<float>(atof(min_part));
    *deg_out = degrees + (minutes / 60.0f);
    return true;
}

const char *nmea_field(const char *sentence, uint8_t index)
{
    const char *cursor = sentence;
    uint8_t current = 0U;

    while (*cursor != '\0') {
        if (current == index) {
            return cursor;
        }

        while (*cursor != '\0' && *cursor != ',') {
            ++cursor;
        }

        if (*cursor == ',') {
            ++cursor;
        }

        ++current;
    }

    return nullptr;
}

bool nmea_parse_gga(const char *sentence, GpsSample *gps_out)
{
    if (gps_out == nullptr) {
        return false;
    }

    if (strncmp(sentence, "$GNGGA", 6) != 0 && strncmp(sentence, "$GPGGA", 6) != 0) {
        return false;
    }

    const char *quality = nmea_field(sentence, 6U);
    const char *sats = nmea_field(sentence, 7U);
    const char *lat = nmea_field(sentence, 2U);
    const char *lat_hemi = nmea_field(sentence, 3U);
    const char *lon = nmea_field(sentence, 4U);
    const char *lon_hemi = nmea_field(sentence, 5U);
    const char *alt = nmea_field(sentence, 9U);

    if (quality == nullptr || quality[0] == '0' || quality[0] == '\0') {
        return false;
    }

    float lat_deg = 0.0f;
    float lon_deg = 0.0f;
    if (!nmea_parse_lat_lon(lat, &lat_deg) || !nmea_parse_lat_lon(lon, &lon_deg)) {
        return false;
    }

    if (lat_hemi != nullptr && lat_hemi[0] == 'S') {
        lat_deg = -lat_deg;
    }
    if (lon_hemi != nullptr && lon_hemi[0] == 'W') {
        lon_deg = -lon_deg;
    }

    gps_out->fix_valid = true;
    gps_out->position = vector3d_make(lat_deg, lon_deg, alt != nullptr ? static_cast<float>(atof(alt)) : 0.0f);
    gps_out->speed_mps = 0.0f;
    gps_out->course_deg = 0.0f;
    gps_out->satellites = (sats != nullptr && sats[0] != '\0') ? static_cast<uint8_t>(atoi(sats)) : 0U;
    return true;
}

bool gnss_line_is_gga_candidate(void)
{
    if (g_rx_len < 6U) {
        return true;
    }

    return (strncmp(g_rx_line, "$GNGGA", 6) == 0)
        || (strncmp(g_rx_line, "$GPGGA", 6) == 0);
}

void gnss_discard_rx_line(void)
{
    g_rx_len = 0U;
    memset(g_rx_line, 0, sizeof(g_rx_line));
}

void gnss_commit_rx_line(void)
{
    if (g_rx_len == 0U || g_rx_line[0] != '$' || !gnss_line_is_gga_candidate()) {
        gnss_discard_rx_line();
        return;
    }

    g_rx_line[g_rx_len] = '\0';
    const uint32_t irq_state = save_and_disable_interrupts();
    if (g_line_pending) {
        restore_interrupts(irq_state);
        gnss_discard_rx_line();
        return;
    }

    memcpy(g_pending_line, g_rx_line, static_cast<size_t>(g_rx_len) + 1U);
    g_line_pending = true;
    restore_interrupts(irq_state);
    gnss_discard_rx_line();
}

void gnss_feed_byte(uint8_t byte)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        gnss_commit_rx_line();
        return;
    }

    if (g_rx_len + 1U >= PICO2_GNSS_LINE_MAX) {
        gnss_discard_rx_line();
        return;
    }

    g_rx_line[g_rx_len++] = static_cast<char>(byte);

    if (!gnss_line_is_gga_candidate()) {
        gnss_discard_rx_line();
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

    uart_init(PICO2_GNSS_UART, PICO2_GNSS_UART_BAUD);
    gpio_set_function(PICO2_GNSS_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO2_GNSS_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enable(PICO2_GNSS_UART, true);

    irq_set_exclusive_handler(
        PICO2_GNSS_UART == uart0 ? UART0_IRQ : UART1_IRQ,
        gnss_uart_rx_isr);
    irq_set_enabled(PICO2_GNSS_UART == uart0 ? UART0_IRQ : UART1_IRQ, true);
    uart_set_irq_enables(PICO2_GNSS_UART, true, false);

    gnss_discard_rx_line();
    g_line_pending = false;
    memset(g_pending_line, 0, sizeof(g_pending_line));
    return true;
}

bool pico2_bsp_gnss_rx_pending(void)
{
    return !g_rx_ring.empty();
}

void pico2_bsp_gnss_rx_pump(uint16_t byte_budget)
{
    uint8_t byte = 0U;
    while (byte_budget > 0U && g_rx_ring.pop(&byte)) {
        gnss_feed_byte(byte);
        --byte_budget;
    }
}

bool pico2_bsp_gnss_poll(GpsSample *gps_out)
{
    if (!g_line_pending || g_pending_line[0] != '$') {
        return false;
    }

    if (!nmea_checksum_ok(g_pending_line)) {
        const uint32_t irq_state = save_and_disable_interrupts();
        g_line_pending = false;
        memset(g_pending_line, 0, sizeof(g_pending_line));
        restore_interrupts(irq_state);
        return false;
    }

    const bool parsed = nmea_parse_gga(g_pending_line, gps_out);
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
