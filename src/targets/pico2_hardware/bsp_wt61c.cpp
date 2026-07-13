#include "bsp_wt61c.hpp"
#include "bsp_uart_rx_ring.hpp"
#include "hw_config.hpp"

#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include <string.h>

namespace {

constexpr uint8_t kWt61cHeader = 0x55U;
constexpr uint8_t kWt61cTypeAccel = 0x51U;
constexpr uint8_t kWt61cTypeGyro = 0x52U;
constexpr uint8_t kWt61cFrameLen = 11U;

constexpr float kWt61cAccelScale = 16.0f * 9.80665f / 32768.0f;
constexpr float kWt61cGyroScale = 2000.0f * 0.0174532925f / 32768.0f;

ImuUartRxRing g_rx_ring;

uint8_t g_frame[kWt61cFrameLen];
uint8_t g_frame_idx = 0U;
uint64_t g_last_rx_us = 0U;

float g_accel_mps2[3] = {0.0f, 0.0f, 0.0f};
float g_gyro_radps[3] = {0.0f, 0.0f, 0.0f};
bool g_accel_valid = false;
bool g_gyro_valid = false;

int16_t wt61c_read_int16(uint8_t low, uint8_t high)
{
    return static_cast<int16_t>(
        (static_cast<uint16_t>(high) << 8) | low);
}

bool wt61c_checksum_ok(const uint8_t *frame)
{
    uint8_t sum = 0U;
    for (uint8_t i = 0U; i < 10U; ++i) {
        sum = static_cast<uint8_t>(sum + frame[i]);
    }
    return sum == frame[10];
}

void wt61c_reset_sync(void)
{
    g_frame_idx = 0U;
}

void wt61c_check_frame_timeout(void)
{
    if (g_frame_idx == 0U || g_last_rx_us == 0U) {
        return;
    }

    if ((time_us_64() - g_last_rx_us) > PICO2_IMU_FRAME_TIMEOUT_US) {
        wt61c_reset_sync();
    }
}

void wt61c_consume_frame(const uint8_t *frame)
{
    if (!wt61c_checksum_ok(frame)) {
        return;
    }

    const int16_t x = wt61c_read_int16(frame[2], frame[3]);
    const int16_t y = wt61c_read_int16(frame[4], frame[5]);
    const int16_t z = wt61c_read_int16(frame[6], frame[7]);

    if (frame[1] == kWt61cTypeAccel) {
        g_accel_mps2[0] = static_cast<float>(x) * kWt61cAccelScale;
        g_accel_mps2[1] = static_cast<float>(y) * kWt61cAccelScale;
        g_accel_mps2[2] = static_cast<float>(z) * kWt61cAccelScale;
        g_accel_valid = true;
        return;
    }

    if (frame[1] == kWt61cTypeGyro) {
        g_gyro_radps[0] = static_cast<float>(x) * kWt61cGyroScale;
        g_gyro_radps[1] = static_cast<float>(y) * kWt61cGyroScale;
        g_gyro_radps[2] = static_cast<float>(z) * kWt61cGyroScale;
        g_gyro_valid = true;
    }
}

void wt61c_feed_byte(uint8_t byte)
{
    if (g_frame_idx == 0U) {
        if (byte != kWt61cHeader) {
            return;
        }
        g_last_rx_us = time_us_64();
        g_frame[0] = byte;
        g_frame_idx = 1U;
        return;
    }

    g_last_rx_us = time_us_64();

    if (g_frame_idx >= kWt61cFrameLen) {
        wt61c_reset_sync();
        return;
    }

    g_frame[g_frame_idx++] = byte;
    if (g_frame_idx < kWt61cFrameLen) {
        return;
    }

    wt61c_consume_frame(g_frame);
    wt61c_reset_sync();
}

void wt61c_uart_drain_hw_to_ring(void)
{
    while (uart_is_readable(PICO2_IMU_UART)) {
        g_rx_ring.push(static_cast<uint8_t>(uart_getc(PICO2_IMU_UART)));
    }
    uart_get_hw(PICO2_IMU_UART)->icr = UART_UARTICR_RXIC_BITS;
}

} /* namespace */

static void wt61c_uart_rx_isr(void)
{
    wt61c_uart_drain_hw_to_ring();
}

bool pico2_bsp_wt61c_init(void)
{
    g_rx_ring = ImuUartRxRing{};

    uart_init(PICO2_IMU_UART, PICO2_IMU_UART_BAUD);
    gpio_set_function(PICO2_IMU_UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(PICO2_IMU_UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_fifo_enable(PICO2_IMU_UART, true);

    irq_set_exclusive_handler(
        PICO2_IMU_UART == uart0 ? UART0_IRQ : UART1_IRQ,
        wt61c_uart_rx_isr);
    irq_set_enabled(PICO2_IMU_UART == uart0 ? UART0_IRQ : UART1_IRQ, true);
    uart_set_irq_enables(PICO2_IMU_UART, true, false);

    wt61c_reset_sync();
    g_last_rx_us = 0U;
    g_accel_valid = false;
    g_gyro_valid = false;
    memset(g_accel_mps2, 0, sizeof(g_accel_mps2));
    memset(g_gyro_radps, 0, sizeof(g_gyro_radps));
    return true;
}

bool pico2_bsp_wt61c_rx_pending(void)
{
    return !g_rx_ring.empty();
}

void pico2_bsp_wt61c_rx_pump(uint16_t byte_budget)
{
    wt61c_check_frame_timeout();

    uint8_t byte = 0U;
    while (byte_budget > 0U && g_rx_ring.pop(&byte)) {
        wt61c_feed_byte(byte);
        --byte_budget;
    }
}

bool pico2_bsp_wt61c_poll(ImuSample *imu_out)
{
    wt61c_check_frame_timeout();

    if (imu_out == nullptr || !g_accel_valid || !g_gyro_valid) {
        return false;
    }

    imu_out->valid = true;
    imu_out->accel_mps2[0] = g_accel_mps2[0];
    imu_out->accel_mps2[1] = g_accel_mps2[1];
    imu_out->accel_mps2[2] = g_accel_mps2[2];
    imu_out->gyro_radps[0] = g_gyro_radps[0];
    imu_out->gyro_radps[1] = g_gyro_radps[1];
    imu_out->gyro_radps[2] = g_gyro_radps[2];
    imu_out->mag_ut[0] = 0.0f;
    imu_out->mag_ut[1] = 0.0f;
    imu_out->mag_ut[2] = 0.0f;
    return true;
}
