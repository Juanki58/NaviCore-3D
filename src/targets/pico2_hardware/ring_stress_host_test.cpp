/**
 * @file ring_stress_host_test.cpp
 * @brief Test de estrés host (g++ -O3) para UartRxRing SPSC.
 *
 * Simulación cooperativa (sin jitter de planificador OS):
 *   - Bytes UART generados a 115200 B/s en ambos rings
 *   - rx_pump() entre eventos (como while(true) en main)
 *   - Bloqueo I2C de PICO2_I2C_STEP_TIMEOUT_US cada 100 ms sin pump
 *
 * Build & run:
 *   cmake --build build --target ring_stress_test
 *   ./build/ring_stress_test
 *
 * Manual (equivalente):
 *   g++ -std=c++17 -O3 -I src/targets/pico2_hardware \
 *       src/targets/pico2_hardware/ring_stress_host_test.cpp -o ring_stress_host_test
 *   ./ring_stress_host_test
 */
#include "bsp_uart_rx_ring.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstdint>

namespace {

constexpr uint32_t kStressDurationS = 60U;
constexpr uint64_t kStressDurationUs = static_cast<uint64_t>(kStressDurationS) * 1000000ULL;
constexpr uint64_t kMainLoopStepUs = 10ULL;
constexpr uint64_t kImuBytePeriodUs =
    (10ULL * 1000000ULL) / static_cast<uint64_t>(PICO2_IMU_UART_BAUD);
constexpr uint64_t kGnssBytePeriodUs =
    (10ULL * 1000000ULL) / static_cast<uint64_t>(PICO2_GNSS_UART_BAUD);
constexpr uint64_t kHousekeepingPeriodUs =
    static_cast<uint64_t>(PICO2_BATTERY_POLL_TICKS * PICO2_NAV_TICK_MS) * 1000ULL;
constexpr uint64_t kI2cBlockUs = static_cast<uint64_t>(PICO2_I2C_STEP_TIMEOUT_US);

ImuUartRxRing g_imu_ring;
GnssUartRxRing g_gnss_ring;

uint64_t g_imu_pushed = 0U;
uint64_t g_gnss_pushed = 0U;

template <uint16_t Size>
void uart_push_due_bytes(
    uint64_t virtual_us,
    uint64_t *next_push_us,
    uint64_t period_us,
    UartRxRing<Size> *ring,
    uint64_t *counter)
{
    while (*next_push_us <= virtual_us) {
        ring->push(static_cast<uint8_t>((*counter)++ & 0xFFU));
        *next_push_us += period_us;
    }
}

void uart_push_due_both(uint64_t virtual_us, uint64_t *imu_next, uint64_t *gnss_next)
{
    uart_push_due_bytes<PICO2_IMU_UART_RING_SIZE>(
        virtual_us, imu_next, kImuBytePeriodUs, &g_imu_ring, &g_imu_pushed);
    uart_push_due_bytes<PICO2_GNSS_UART_RING_SIZE>(
        virtual_us, gnss_next, kGnssBytePeriodUs, &g_gnss_ring, &g_gnss_pushed);
}

void rx_pump_both(void)
{
    uint8_t byte = 0U;

    for (uint8_t round = 0U; round < PICO2_UART_PUMP_MAX_ROUNDS; ++round) {
        bool activity = false;

        for (uint16_t budget = PICO2_UART_RX_BUDGET; budget > 0U && g_imu_ring.pop(&byte); --budget) {
            activity = true;
        }
        for (uint16_t budget = PICO2_UART_RX_BUDGET; budget > 0U && g_gnss_ring.pop(&byte); --budget) {
            activity = true;
        }

        if (!activity) {
            break;
        }
    }
}

void simulate_i2c_block(uint64_t *virtual_us, uint64_t *imu_next, uint64_t *gnss_next)
{
    const uint64_t block_end = *virtual_us + kI2cBlockUs;
    while (*virtual_us < block_end) {
        uart_push_due_both(*virtual_us, imu_next, gnss_next);
        *virtual_us += kMainLoopStepUs;
    }
}

} /* namespace */

int main(void)
{
    std::printf(
        "ring_stress_host_test: %u s | IMU/GNSS %u baud | ring %u B | -O3 | sim cooperativa\n",
        kStressDurationS,
        PICO2_IMU_UART_BAUD,
        PICO2_IMU_UART_RING_SIZE);

    uint64_t virtual_us = 0U;
    uint64_t imu_next = 0U;
    uint64_t gnss_next = 0U;
    uint64_t next_housekeeping = kHousekeepingPeriodUs;

    while (virtual_us < kStressDurationUs) {
        uart_push_due_both(virtual_us, &imu_next, &gnss_next);
        rx_pump_both();

        if (virtual_us >= next_housekeeping) {
            simulate_i2c_block(&virtual_us, &imu_next, &gnss_next);
            next_housekeeping += kHousekeepingPeriodUs;
        }

        virtual_us += kMainLoopStepUs;
    }

    rx_pump_both();

    const uint32_t imu_ovf = g_imu_ring.overflow_count_load();
    const uint32_t gnss_ovf = g_gnss_ring.overflow_count_load();

    std::printf(
        "Resultado: IMU pushed=%llu overflow=%u | GNSS pushed=%llu overflow=%u\n",
        static_cast<unsigned long long>(g_imu_pushed),
        imu_ovf,
        static_cast<unsigned long long>(g_gnss_pushed),
        gnss_ovf);

    if (imu_ovf != 0U || gnss_ovf != 0U) {
        std::fprintf(
            stderr,
            "FALLO: overflow detectado (esperado 0 con margen %u B salvo saturacion intencional)\n",
            PICO2_IMU_UART_RING_SIZE);
        return EXIT_FAILURE;
    }

    std::printf("OK: overflow_count == 0 en ambos rings tras %u s\n", kStressDurationS);
    return EXIT_SUCCESS;
}
