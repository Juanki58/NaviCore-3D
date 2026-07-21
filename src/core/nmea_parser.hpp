/**
 * @file nmea_parser.hpp
 * @brief Pure NMEA GGA parser + line assembler (no UART / no heap).
 *
 * Extracted from Pico BSP so libFuzzer / ASan can hammer wire formats without
 * linking the Pico SDK. Fail-closed: corrupt / oversize / bad checksum → drop.
 */
#pragma once

#include "sensor_types.hpp"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef NAVICORE_NMEA_LINE_MAX
#define NAVICORE_NMEA_LINE_MAX 96U
#endif

#ifdef __cplusplus
extern "C" {
#endif

bool nmea_checksum_ok(const char *sentence);
bool nmea_parse_lat_lon(const char *field, float *deg_out);
const char *nmea_field(const char *sentence, uint8_t index);
bool nmea_parse_gga(const char *sentence, GpsSample *gps_out);

/** Parse a complete NUL-terminated sentence: checksum + GGA → GpsSample. */
bool nmea_try_parse_sentence(const char *sentence, GpsSample *gps_out);

typedef struct {
    char line[NAVICORE_NMEA_LINE_MAX];
    uint8_t len;
    bool stream_contaminated;
} NmeaLineAssembler;

void nmea_line_assembler_reset(NmeaLineAssembler *as);
/**
 * Feed one UART byte. When a line is complete and is a GGA candidate,
 * copies into out_line (NUL-terminated, capacity out_cap) and returns true.
 * Never writes past out_cap / never overflows the internal buffer.
 */
bool nmea_line_assembler_feed(
    NmeaLineAssembler *as,
    uint8_t byte,
    char *out_line,
    size_t out_cap);

/** Mark stream contaminated (e.g. ring overflow); discards until next '\\n'. */
void nmea_line_assembler_mark_contaminated(NmeaLineAssembler *as);

#ifdef __cplusplus
}
#endif
