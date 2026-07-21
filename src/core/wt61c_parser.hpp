/**
 * @file wt61c_parser.hpp
 * @brief Pure WitMotion WT61C 0x55 frame sync (no UART / injectable time).
 */
#pragma once

#include "sensor_types.hpp"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NAVICORE_WT61C_FRAME_LEN
#define NAVICORE_WT61C_FRAME_LEN 11U
#endif

#ifndef NAVICORE_WT61C_FRAME_TIMEOUT_US
#define NAVICORE_WT61C_FRAME_TIMEOUT_US 5000U
#endif

typedef struct {
    uint8_t frame[NAVICORE_WT61C_FRAME_LEN];
    uint8_t frame_idx;
    uint64_t last_rx_us;
    bool stream_contaminated;
    bool discard_next_frame;
    float accel_mps2[3];
    float gyro_radps[3];
    bool accel_valid;
    bool gyro_valid;
} Wt61cStreamParser;

bool wt61c_checksum_ok(const uint8_t *frame);
void wt61c_stream_reset(Wt61cStreamParser *p);
void wt61c_stream_mark_contaminated(Wt61cStreamParser *p);
/** Mid-frame silence: if now_us - last_rx > timeout, drop partial frame. */
void wt61c_stream_check_timeout(Wt61cStreamParser *p, uint64_t now_us);
void wt61c_stream_feed(Wt61cStreamParser *p, uint8_t byte, uint64_t now_us);
/** Returns true when both accel and gyro have been seen since last poll. */
bool wt61c_stream_try_sample(Wt61cStreamParser *p, ImuSample *imu_out);

#ifdef __cplusplus
}
#endif
