#include "wt61c_parser.hpp"

#include <string.h>

namespace {

constexpr uint8_t kWt61cHeader = 0x55U;
constexpr uint8_t kWt61cTypeAccel = 0x51U;
constexpr uint8_t kWt61cTypeGyro = 0x52U;
constexpr float kWt61cAccelScale = 16.0f * 9.80665f / 32768.0f;
constexpr float kWt61cGyroScale = 2000.0f * 0.0174532925f / 32768.0f;

int16_t wt61c_read_int16(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

void wt61c_consume_frame(Wt61cStreamParser *p, const uint8_t *frame)
{
    if (!wt61c_checksum_ok(frame)) {
        return;
    }

    if (frame[1] != kWt61cTypeAccel && frame[1] != kWt61cTypeGyro) {
        return;
    }

    if (p->stream_contaminated || p->discard_next_frame) {
        p->stream_contaminated = false;
        p->discard_next_frame = false;
        return;
    }

    const int16_t x = wt61c_read_int16(frame[2], frame[3]);
    const int16_t y = wt61c_read_int16(frame[4], frame[5]);
    const int16_t z = wt61c_read_int16(frame[6], frame[7]);

    if (frame[1] == kWt61cTypeAccel) {
        p->accel_mps2[0] = static_cast<float>(x) * kWt61cAccelScale;
        p->accel_mps2[1] = static_cast<float>(y) * kWt61cAccelScale;
        p->accel_mps2[2] = static_cast<float>(z) * kWt61cAccelScale;
        p->accel_valid = true;
        return;
    }

    p->gyro_radps[0] = static_cast<float>(x) * kWt61cGyroScale;
    p->gyro_radps[1] = static_cast<float>(y) * kWt61cGyroScale;
    p->gyro_radps[2] = static_cast<float>(z) * kWt61cGyroScale;
    p->gyro_valid = true;
}

} /* namespace */

bool wt61c_checksum_ok(const uint8_t *frame)
{
    if (frame == nullptr) {
        return false;
    }
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < 10U; ++i) {
        sum = static_cast<uint8_t>(sum + frame[i]);
    }
    return sum == frame[10];
}

void wt61c_stream_reset(Wt61cStreamParser *p)
{
    if (p == nullptr) {
        return;
    }
    memset(p, 0, sizeof(*p));
}

void wt61c_stream_mark_contaminated(Wt61cStreamParser *p)
{
    if (p == nullptr) {
        return;
    }
    p->stream_contaminated = true;
    p->discard_next_frame = false;
    p->frame_idx = 0U;
    p->accel_valid = false;
    p->gyro_valid = false;
}

void wt61c_stream_check_timeout(Wt61cStreamParser *p, uint64_t now_us)
{
    if (p == nullptr || p->frame_idx == 0U || p->last_rx_us == 0U) {
        return;
    }
    if ((now_us - p->last_rx_us) > NAVICORE_WT61C_FRAME_TIMEOUT_US) {
        p->frame_idx = 0U;
    }
}

void wt61c_stream_feed(Wt61cStreamParser *p, uint8_t byte, uint64_t now_us)
{
    if (p == nullptr) {
        return;
    }

    if (p->stream_contaminated) {
        if (byte != kWt61cHeader) {
            return;
        }
        p->last_rx_us = now_us;
        p->frame[0] = byte;
        p->frame_idx = 1U;
        return;
    }

    if (p->frame_idx == 0U) {
        if (byte != kWt61cHeader) {
            return;
        }
        p->last_rx_us = now_us;
        p->frame[0] = byte;
        p->frame_idx = 1U;
        return;
    }

    p->last_rx_us = now_us;

    if (p->frame_idx >= NAVICORE_WT61C_FRAME_LEN) {
        p->frame_idx = 0U;
        return;
    }

    p->frame[p->frame_idx++] = byte;
    if (p->frame_idx < NAVICORE_WT61C_FRAME_LEN) {
        return;
    }

    wt61c_consume_frame(p, p->frame);
    p->frame_idx = 0U;
}

bool wt61c_stream_try_sample(Wt61cStreamParser *p, ImuSample *imu_out)
{
    if (p == nullptr || imu_out == nullptr || !p->accel_valid || !p->gyro_valid) {
        return false;
    }

    imu_out->valid = true;
    imu_out->accel_mps2[0] = p->accel_mps2[0];
    imu_out->accel_mps2[1] = p->accel_mps2[1];
    imu_out->accel_mps2[2] = p->accel_mps2[2];
    imu_out->gyro_radps[0] = p->gyro_radps[0];
    imu_out->gyro_radps[1] = p->gyro_radps[1];
    imu_out->gyro_radps[2] = p->gyro_radps[2];
    imu_out->mag_ut[0] = 0.0f;
    imu_out->mag_ut[1] = 0.0f;
    imu_out->mag_ut[2] = 0.0f;
    /* Consume pair so silence detection can re-arm. */
    p->accel_valid = false;
    p->gyro_valid = false;
    return true;
}
