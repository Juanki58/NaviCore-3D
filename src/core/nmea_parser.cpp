#include "nmea_parser.hpp"

#include "vector3d.h"

#include <stdlib.h>
#include <string.h>

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
    if (dot == nullptr || (dot - buf) < 2) {
        return false;
    }

    const int deg_digits = ((dot - buf) > 4) ? 3 : 2;
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
    if (sentence == nullptr) {
        return nullptr;
    }

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
    if (sentence == nullptr || gps_out == nullptr) {
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
    gps_out->position = vector3d_make(
        lat_deg,
        lon_deg,
        alt != nullptr ? static_cast<float>(atof(alt)) : 0.0f);
    gps_out->speed_mps = 0.0f;
    gps_out->course_deg = 0.0f;
    gps_out->satellites =
        (sats != nullptr && sats[0] != '\0') ? static_cast<uint8_t>(atoi(sats)) : 0U;
    return true;
}

bool nmea_try_parse_sentence(const char *sentence, GpsSample *gps_out)
{
    if (!nmea_checksum_ok(sentence)) {
        return false;
    }
    return nmea_parse_gga(sentence, gps_out);
}

static bool nmea_line_is_gga_candidate(const char *line, uint8_t len)
{
    if (len < 6U) {
        return true;
    }
    return (strncmp(line, "$GNGGA", 6) == 0) || (strncmp(line, "$GPGGA", 6) == 0);
}

void nmea_line_assembler_reset(NmeaLineAssembler *as)
{
    if (as == nullptr) {
        return;
    }
    as->len = 0U;
    as->stream_contaminated = false;
    memset(as->line, 0, sizeof(as->line));
}

void nmea_line_assembler_mark_contaminated(NmeaLineAssembler *as)
{
    if (as == nullptr) {
        return;
    }
    as->stream_contaminated = true;
    as->len = 0U;
    memset(as->line, 0, sizeof(as->line));
}

bool nmea_line_assembler_feed(
    NmeaLineAssembler *as,
    uint8_t byte,
    char *out_line,
    size_t out_cap)
{
    if (as == nullptr) {
        return false;
    }

    if (as->stream_contaminated) {
        if (byte == '\n') {
            as->stream_contaminated = false;
            as->len = 0U;
            memset(as->line, 0, sizeof(as->line));
        }
        return false;
    }

    if (byte == '\r') {
        return false;
    }

    if (byte == '\n') {
        if (as->len == 0U || as->line[0] != '$'
            || !nmea_line_is_gga_candidate(as->line, as->len)) {
            as->len = 0U;
            memset(as->line, 0, sizeof(as->line));
            return false;
        }

        as->line[as->len] = '\0';
        if (out_line != nullptr && out_cap > 0U) {
            const size_t copy_n =
                (static_cast<size_t>(as->len) + 1U < out_cap)
                    ? static_cast<size_t>(as->len) + 1U
                    : out_cap - 1U;
            memcpy(out_line, as->line, copy_n);
            out_line[copy_n] = '\0';
            if (copy_n < static_cast<size_t>(as->len) + 1U) {
                /* Truncated output capacity — treat as drop (fail-closed). */
                as->len = 0U;
                memset(as->line, 0, sizeof(as->line));
                return false;
            }
        }
        as->len = 0U;
        memset(as->line, 0, sizeof(as->line));
        return out_line != nullptr && out_cap > 0U;
    }

    if (as->len + 1U >= NAVICORE_NMEA_LINE_MAX) {
        as->len = 0U;
        memset(as->line, 0, sizeof(as->line));
        return false;
    }

    as->line[as->len++] = static_cast<char>(byte);
    if (!nmea_line_is_gga_candidate(as->line, as->len)) {
        as->len = 0U;
        memset(as->line, 0, sizeof(as->line));
    }
    return false;
}
