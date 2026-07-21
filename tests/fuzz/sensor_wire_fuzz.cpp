/**
 * @file sensor_wire_fuzz.cpp
 * @brief libFuzzer entry for NMEA / UBX / WT61C wire parsers.
 *
 * Build (Clang):
 *   cmake -S . -B build_fuzz -G Ninja \
 *     -DCMAKE_CXX_COMPILER=clang++ -DNAVICORE_BUILD_FUZZERS=ON
 *   ./build_fuzz/navicore_sensor_wire_fuzz tests/fuzz/corpus -max_total_time=60
 *
 * AFL (standalone stdin driver — same binary with NAVICORE_FUZZ_STANDALONE):
 *   afl-fuzz -i tests/fuzz/corpus -o findings -- ./navicore_sensor_wire_fuzz_standalone
 */
#include "nmea_parser.hpp"
#include "ubx_parser.hpp"
#include "wt61c_parser.hpp"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (data == nullptr) {
        return 0;
    }

    /* Selector nibble: exercise all three surfaces from one corpus. */
    const uint8_t sel = (size > 0U) ? (data[0] & 0x03U) : 0U;
    const uint8_t *payload = (size > 1U) ? (data + 1) : data;
    const size_t plen = (size > 1U) ? (size - 1U) : size;

    if (sel == 0U) {
        NmeaLineAssembler as{};
        nmea_line_assembler_reset(&as);
        char line[NAVICORE_NMEA_LINE_MAX];
        GpsSample gps{};
        for (size_t i = 0; i < plen; ++i) {
            if (nmea_line_assembler_feed(&as, payload[i], line, sizeof(line))) {
                (void)nmea_try_parse_sentence(line, &gps);
            }
        }
        /* Also treat whole blob as a sentence (NUL-safe copy). */
        char sentence[NAVICORE_NMEA_LINE_MAX];
        const size_t n =
            (plen < sizeof(sentence) - 1U) ? plen : (sizeof(sentence) - 1U);
        memcpy(sentence, payload, n);
        sentence[n] = '\0';
        (void)nmea_try_parse_sentence(sentence, &gps);
        return 0;
    }

    if (sel == 1U) {
        UbxStreamParser ubx{};
        ubx_stream_reset(&ubx);
        UbxFrame frame{};
        for (size_t i = 0; i < plen; ++i) {
            (void)ubx_stream_feed(&ubx, payload[i], &frame);
        }
        (void)ubx_frame_validate(payload, plen, &frame);
        return 0;
    }

    /* sel == 2 or 3: WT61C + mid-frame timeout stress */
    Wt61cStreamParser wt{};
    wt61c_stream_reset(&wt);
    uint64_t now = 1000U;
    ImuSample imu{};
    for (size_t i = 0; i < plen; ++i) {
        if ((i & 7U) == 0U) {
            now += NAVICORE_WT61C_FRAME_TIMEOUT_US + 1U;
            wt61c_stream_check_timeout(&wt, now);
        }
        wt61c_stream_feed(&wt, payload[i], now);
        ++now;
        (void)wt61c_stream_try_sample(&wt, &imu);
    }
    if ((plen & 1U) != 0U) {
        wt61c_stream_mark_contaminated(&wt);
    }
    return 0;
}

#if defined(NAVICORE_FUZZ_STANDALONE)
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

static int run_file(const char *path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "cannot open %s\n", path);
        return 1;
    }
    std::vector<uint8_t> buf(
        (std::istreambuf_iterator<char>(in)),
        std::istreambuf_iterator<char>());
    LLVMFuzzerTestOneInput(buf.data(), buf.size());
    return 0;
}

int main(int argc, char **argv)
{
    if (argc <= 1) {
        /* AFL / pipe: read stdin once */
        std::vector<uint8_t> buf(
            (std::istreambuf_iterator<char>(std::cin)),
            std::istreambuf_iterator<char>());
        LLVMFuzzerTestOneInput(buf.data(), buf.size());
        return 0;
    }
    int rc = 0;
    for (int i = 1; i < argc; ++i) {
        rc |= run_file(argv[i]);
    }
    return rc;
}
#endif
