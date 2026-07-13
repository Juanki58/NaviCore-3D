#include "safe_log.hpp"

#include <stdio.h>
#include <string.h>

#include "tusb.h"

namespace {

constexpr uint8_t kSafeLogLineCount = 24U;
constexpr size_t kSafeLogLineMax = 96U;

char g_log_ring[kSafeLogLineCount][kSafeLogLineMax];
uint8_t g_log_ring_head = 0U;
uint8_t g_log_ring_count = 0U;

void safe_log_ring_push(const char *line)
{
    char *slot = g_log_ring[g_log_ring_head];
    strncpy(slot, line, kSafeLogLineMax - 1U);
    slot[kSafeLogLineMax - 1U] = '\0';

    g_log_ring_head = static_cast<uint8_t>((g_log_ring_head + 1U) % kSafeLogLineCount);
    if (g_log_ring_count < kSafeLogLineCount) {
        ++g_log_ring_count;
    }
}

bool safe_log_ring_pop(char *line_out, size_t line_out_max)
{
    if (g_log_ring_count == 0U || line_out == nullptr || line_out_max == 0U) {
        return false;
    }

    const uint8_t tail = static_cast<uint8_t>(
        (g_log_ring_head + kSafeLogLineCount - g_log_ring_count) % kSafeLogLineCount);
    strncpy(line_out, g_log_ring[tail], line_out_max - 1U);
    line_out[line_out_max - 1U] = '\0';
    --g_log_ring_count;
    return true;
}

size_t safe_log_cdc_write_nonblocking(const char *data, size_t len)
{
    size_t written = 0U;
    while (written < len) {
        const uint32_t avail = tud_cdc_write_available();
        if (avail == 0U) {
            break;
        }

        const uint32_t chunk = static_cast<uint32_t>(
            (len - written) < static_cast<size_t>(avail)
                ? (len - written)
                : static_cast<size_t>(avail));
        const uint32_t sent = tud_cdc_write(
            reinterpret_cast<const uint8_t *>(data + written),
            chunk);
        if (sent == 0U) {
            break;
        }
        written += static_cast<size_t>(sent);
    }
    return written;
}

void safe_log_emit_line(const char *line)
{
    if (line == nullptr || line[0] == '\0') {
        return;
    }

    tud_task();

    if (tud_cdc_connected()) {
        const size_t len = strlen(line);
        const size_t written = safe_log_cdc_write_nonblocking(line, len);
        if (written < len) {
            safe_log_ring_push(line);
        }
        return;
    }

    safe_log_ring_push(line);
}

} /* namespace */

void safe_log_init(void)
{
    g_log_ring_head = 0U;
    g_log_ring_count = 0U;
    memset(g_log_ring, 0, sizeof(g_log_ring));
}

void safe_log_flush_pending(void)
{
    tud_task();

    if (!tud_cdc_connected()) {
        return;
    }

    char line[kSafeLogLineMax];
    while (g_log_ring_count > 0U) {
        if (!safe_log_ring_pop(line, sizeof(line))) {
            break;
        }

        const size_t len = strlen(line);
        const size_t written = safe_log_cdc_write_nonblocking(line, len);
        if (written < len) {
            safe_log_ring_push(line);
            break;
        }
    }
}

void safe_log(const char *message)
{
    safe_log_emit_line(message);
}

void safe_logf(const char *fmt, ...)
{
    if (fmt == nullptr) {
        return;
    }

    char line[kSafeLogLineMax];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    safe_log_emit_line(line);
}

bool safe_log_pending_count(uint8_t *count_out)
{
    if (count_out != nullptr) {
        *count_out = g_log_ring_count;
    }
    return g_log_ring_count > 0U;
}
