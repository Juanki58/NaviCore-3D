/**
 * @file hw_config.hpp
 * @brief Hardware validado — laboratorio físico Comarruga (Pico 2 W)
 *
 * MCU:     Raspberry Pi Pico 2 W (RP2350, dual Cortex-M33 @ 150 MHz)
 * Energía: Waveshare UPS Module — I2C1 @ 0x43, celdas AVESO 14500
 * IMU:     WitMotion WT61C-232 — AHRS con Kalman integrado, UART0 @ 115200
 * GNSS:    u-blox NEO-M9N — 4 constelaciones concurrentes, UART1 @ 115200
 *
 * Arquitectura de tiempo real (core 0 único para navegación):
 *   - ISR UART: vacía FIFO hardware PL011 (32 B) → ring buffer software.
 *   - rx_pump(): consume el ring en cada iteración del while(true) de main.
 *   - PICO2_UART_RX_BUDGET limita el WCET de UNA ronda de pump (no el throughput).
 *   - PICO2_UART_PUMP_MAX_ROUNDS multiplica el drenaje por iteración del bucle.
 *   - El tick @ 100 Hz solo fusiona muestras ya parseadas; NO hace I2C.
 *   - pico2_bsp_power_poll(): FSM cooperativa UPS; un paso por llamada (≤ 2 ms).
 *   - Tras PICO2_I2C_RECOVER_MAX secuencias 9-pulsos fallidas → UPS OFFLINE permanente.
 *   - cyw43_poll() es periférico "blando" sin timeout SDK — ver nota abajo.
 *   - WT61C: checksum de trama obligatorio antes de actualizar IMU (bsp_wt61c.cpp).
 *
 * Invariante WDT (3 pasos I2C fallidos × 2 ms = 6 ms << 50 ms WDT):
 *   PICO2_I2C_STEP_TIMEOUT_US × PICO2_I2C_RECOVER_AFTER < PICO2_WDT_TIMEOUT_MS × 1000
 *
 * Invariante UART vs FSM (static_assert en bsp_power.cpp):
 *   PICO2_I2C_STEP_TIMEOUT_US < tiempo_equivalente(PICO2_UART_RX_BUDGET @ 115200)
 *
 * Ring SPSC: w/r son std::atomic<uint16_t> (release/acquire). Validar con
 * ring_stress_host_test.cpp @ -O3 durante 60 s.
 *
 * WCET: GP22 = sensors_tick(), GP21 = cyw43_poll(), max_loop_time_us cada 1 s.
 */
#pragma once

#include <stdint.h>

/* --- WitMotion WT61C-232 (UART0) --- */
#define PICO2_IMU_UART              uart0
#define PICO2_IMU_UART_BAUD          115200U
#define PICO2_IMU_UART_TX_PIN        0U
#define PICO2_IMU_UART_RX_PIN        1U
#define PICO2_IMU_UART_RING_SIZE     512U

/* --- u-blox NEO-M9N (UART1) --- */
#define PICO2_GNSS_UART              uart1
#define PICO2_GNSS_UART_BAUD         115200U
#define PICO2_GNSS_UART_TX_PIN       4U
#define PICO2_GNSS_UART_RX_PIN       5U
#define PICO2_GNSS_UART_RING_SIZE    512U

/* --- Waveshare UPS Module (I2C1) --- */
#define PICO2_POWER_I2C              i2c1
#define PICO2_POWER_I2C_SDA_PIN      6U
#define PICO2_POWER_I2C_SCL_PIN      7U
#define PICO2_POWER_I2C_HZ           100000U
#define PICO2_POWER_I2C_ADDR         0x43U

/* --- Bucle de navegación --- */
#define PICO2_NAV_TICK_MS            10U
#define PICO2_BATTERY_LOW_MV         3300U
#define PICO2_BATTERY_LOW_CLEAR_MV   3400U

/* --- Límites de robustez (aviónica / tiempo real) --- */
#define PICO2_UART_RX_BUDGET         64U   /* bytes máx. por UART y por ronda de pump */
#define PICO2_UART_PUMP_MAX_ROUNDS   8U    /* rondas/iteración → hasta 512 B/UART/loop */
#define PICO2_IMU_FRAME_TIMEOUT_US   5000U
#define PICO2_GNSS_LINE_MAX          96U
#define PICO2_I2C_STEP_TIMEOUT_US    2000U /* deadline por paso FSM (≤ 2 ms) */
#define PICO2_I2C_RECOVERY_SCL_LOW_US  5U
#define PICO2_I2C_RECOVERY_SCL_HIGH_US 5U
#define PICO2_I2C_RECOVERY_PULSE_US  (PICO2_I2C_RECOVERY_SCL_LOW_US + PICO2_I2C_RECOVERY_SCL_HIGH_US)
#define PICO2_I2C_RECOVER_AFTER      3U
#define PICO2_I2C_RECOVER_MAX        3U    /* secuencias 9-pulsos fallidas → OFFLINE permanente */
#define PICO2_WDT_TIMEOUT_MS         50U

/* --- Métricas de control (Fase 2 gate — osciloscopio en banco) --- */
#define PICO2_GPIO_BENCHMARK         22U   /* GP22: HIGH durante pico2_bsp_sensors_tick() */
#define PICO2_GPIO_BENCHMARK_WIFI      21U   /* GP21: HIGH durante cyw43_arch_poll() */
#define PICO2_LOOP_METRICS_REPORT_MS 1000U /* reporte max_loop_time_us por ventana */

/* --- Eficiencia --- */
#define PICO2_BATTERY_POLL_TICKS     10U
#define PICO2_BATTERY_LOW_DEBOUNCE   3U

/* --- UART ring overflow → degradación de confianza --- */
#define PICO2_UART_ID_IMU             0U
#define PICO2_UART_ID_GNSS             1U
#define PICO2_RING_OVERFLOW_WINDOW_MS  1000U
#define PICO2_RING_OVERFLOW_DEGRADE_THRESHOLD 3U
#define PICO2_RING_DEGRADED_QUALITY_FACTOR    0.50f
