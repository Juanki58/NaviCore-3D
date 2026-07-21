/**
 * @file ubx_parser.hpp
 * @brief Fail-closed u-blox UBX frame sync + Fletcher checksum (no heap).
 *
 * Firmware today speaks NMEA on the NEO-M9N, but binary UBX is the other wire
 * surface a spoofer / defective module can emit. This validator rejects
 * oversize / truncated / bad-CK frames without buffer overflow.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef NAVICORE_UBX_MAX_PAYLOAD
#define NAVICORE_UBX_MAX_PAYLOAD 256U
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UBX_PARSE_OK = 0,
    UBX_PARSE_NEED_MORE = 1,
    UBX_PARSE_BAD_SYNC = 2,
    UBX_PARSE_OVERSIZE = 3,
    UBX_PARSE_BAD_CHECKSUM = 4,
} UbxParseStatus;

typedef struct {
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_len;
    uint8_t payload[NAVICORE_UBX_MAX_PAYLOAD];
} UbxFrame;

typedef struct {
    uint8_t state;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t payload_len;
    uint16_t payload_idx;
    uint8_t payload[NAVICORE_UBX_MAX_PAYLOAD];
    uint8_t ck_a;
    uint8_t ck_b;
    uint8_t ck_a_rx;
} UbxStreamParser;

void ubx_stream_reset(UbxStreamParser *p);
/**
 * Feed one byte. On UBX_PARSE_OK, *out is filled with a checksum-valid frame.
 * Never writes past payload[]. Oversize length → reset + UBX_PARSE_OVERSIZE.
 */
UbxParseStatus ubx_stream_feed(UbxStreamParser *p, uint8_t byte, UbxFrame *out);

/** Validate a complete buffer that already starts at 0xB5 0x62. */
bool ubx_frame_validate(const uint8_t *data, size_t len, UbxFrame *out);

#ifdef __cplusplus
}
#endif
