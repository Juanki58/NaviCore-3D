#include "ubx_parser.hpp"

#include <string.h>

enum {
    kUbxStSync1 = 0,
    kUbxStSync2 = 1,
    kUbxStClass = 2,
    kUbxStId = 3,
    kUbxStLenLo = 4,
    kUbxStLenHi = 5,
    kUbxStPayload = 6,
    kUbxStCkA = 7,
    kUbxStCkB = 8,
};

void ubx_stream_reset(UbxStreamParser *p)
{
    if (p == nullptr) {
        return;
    }
    memset(p, 0, sizeof(*p));
    p->state = kUbxStSync1;
}

static void ubx_ck_add(UbxStreamParser *p, uint8_t byte)
{
    p->ck_a = static_cast<uint8_t>(p->ck_a + byte);
    p->ck_b = static_cast<uint8_t>(p->ck_b + p->ck_a);
}

static void ubx_emit(const UbxStreamParser *p, UbxFrame *out)
{
    if (out == nullptr) {
        return;
    }
    out->msg_class = p->msg_class;
    out->msg_id = p->msg_id;
    out->payload_len = p->payload_len;
    if (p->payload_len > 0U) {
        memcpy(out->payload, p->payload, p->payload_len);
    }
}

UbxParseStatus ubx_stream_feed(UbxStreamParser *p, uint8_t byte, UbxFrame *out)
{
    if (p == nullptr) {
        return UBX_PARSE_BAD_SYNC;
    }

    switch (p->state) {
    case kUbxStSync1:
        if (byte == 0xB5U) {
            p->state = kUbxStSync2;
        }
        return UBX_PARSE_NEED_MORE;

    case kUbxStSync2:
        if (byte == 0x62U) {
            p->state = kUbxStClass;
            p->ck_a = 0U;
            p->ck_b = 0U;
            p->payload_idx = 0U;
            return UBX_PARSE_NEED_MORE;
        }
        p->state = (byte == 0xB5U) ? kUbxStSync2 : kUbxStSync1;
        return UBX_PARSE_BAD_SYNC;

    case kUbxStClass:
        p->msg_class = byte;
        ubx_ck_add(p, byte);
        p->state = kUbxStId;
        return UBX_PARSE_NEED_MORE;

    case kUbxStId:
        p->msg_id = byte;
        ubx_ck_add(p, byte);
        p->state = kUbxStLenLo;
        return UBX_PARSE_NEED_MORE;

    case kUbxStLenLo:
        p->payload_len = byte;
        ubx_ck_add(p, byte);
        p->state = kUbxStLenHi;
        return UBX_PARSE_NEED_MORE;

    case kUbxStLenHi:
        p->payload_len = static_cast<uint16_t>(
            p->payload_len | (static_cast<uint16_t>(byte) << 8));
        ubx_ck_add(p, byte);
        if (p->payload_len > NAVICORE_UBX_MAX_PAYLOAD) {
            ubx_stream_reset(p);
            return UBX_PARSE_OVERSIZE;
        }
        p->payload_idx = 0U;
        p->state = (p->payload_len == 0U) ? kUbxStCkA : kUbxStPayload;
        return UBX_PARSE_NEED_MORE;

    case kUbxStPayload:
        if (p->payload_idx < NAVICORE_UBX_MAX_PAYLOAD) {
            p->payload[p->payload_idx] = byte;
        }
        ubx_ck_add(p, byte);
        ++p->payload_idx;
        if (p->payload_idx >= p->payload_len) {
            p->state = kUbxStCkA;
        }
        return UBX_PARSE_NEED_MORE;

    case kUbxStCkA:
        p->ck_a_rx = byte;
        p->state = kUbxStCkB;
        return UBX_PARSE_NEED_MORE;

    case kUbxStCkB: {
        const bool ok = (p->ck_a_rx == p->ck_a) && (byte == p->ck_b);
        if (ok) {
            ubx_emit(p, out);
            ubx_stream_reset(p);
            return UBX_PARSE_OK;
        }
        ubx_stream_reset(p);
        return UBX_PARSE_BAD_CHECKSUM;
    }

    default:
        ubx_stream_reset(p);
        return UBX_PARSE_BAD_SYNC;
    }
}

bool ubx_frame_validate(const uint8_t *data, size_t len, UbxFrame *out)
{
    if (data == nullptr || len < 8U) {
        return false;
    }

    UbxStreamParser p{};
    ubx_stream_reset(&p);
    for (size_t i = 0; i < len; ++i) {
        const UbxParseStatus st = ubx_stream_feed(&p, data[i], out);
        if (st == UBX_PARSE_OK) {
            return true;
        }
        if (st == UBX_PARSE_OVERSIZE) {
            return false;
        }
    }
    return false;
}
