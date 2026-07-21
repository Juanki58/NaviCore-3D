/**
 * @file health_policy.hpp
 * @brief Host-portable classification of Pico health_monitor reactions.
 *
 * Same thresholds as hw_config.hpp (mirrored numerically). Used by unit /
 * safety-inject tests so fault injection proves *policy*, not just "no crash".
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HEALTH_POLICY_NOMINAL = 0,
    HEALTH_POLICY_DEGRADED = 1,
    HEALTH_POLICY_CRITICAL = 2,
} HealthPolicyLevel;

/* Mirrors src/targets/pico2_hardware/hw_config.hpp (keep in sync). */
#ifndef HEALTH_POLICY_LOOP_DEGRADED_US
#define HEALTH_POLICY_LOOP_DEGRADED_US 8000U
#endif
#ifndef HEALTH_POLICY_LOOP_CRITICAL_US
#define HEALTH_POLICY_LOOP_CRITICAL_US 25000U
#endif
#ifndef HEALTH_POLICY_TICK_BACKLOG_DEGRADED
#define HEALTH_POLICY_TICK_BACKLOG_DEGRADED 1U
#endif
#ifndef HEALTH_POLICY_TICK_BACKLOG_CRITICAL
#define HEALTH_POLICY_TICK_BACKLOG_CRITICAL 3U
#endif
#ifndef HEALTH_POLICY_LOOP_OVERRUN_DEGRADED
#define HEALTH_POLICY_LOOP_OVERRUN_DEGRADED 3U
#endif
#ifndef HEALTH_POLICY_RING_OVERFLOW_DEGRADE
#define HEALTH_POLICY_RING_OVERFLOW_DEGRADE 3U
#endif
#ifndef HEALTH_POLICY_UART_OVERFLOW_PER_S_MAX
#define HEALTH_POLICY_UART_OVERFLOW_PER_S_MAX 3U
#endif
#ifndef HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX
#define HEALTH_POLICY_I2C_RECOVERY_OFFLINE_MAX 5U
#endif
#ifndef HEALTH_POLICY_IMU_SILENCE_DEGRADE_MS
#define HEALTH_POLICY_IMU_SILENCE_DEGRADE_MS 200U /* 20 ticks @ 100 Hz */
#endif
#ifndef HEALTH_POLICY_GNSS_SILENCE_DEGRADE_MS
#define HEALTH_POLICY_GNSS_SILENCE_DEGRADE_MS 5000U
#endif
#ifndef HEALTH_POLICY_UART_FRAME_TIMEOUT_US
#define HEALTH_POLICY_UART_FRAME_TIMEOUT_US 5000U
#endif

typedef struct {
    uint32_t last_loop_us;
    uint32_t max_tick_backlog;
    uint8_t consecutive_overrun;
    uint32_t uart0_overflows_total;
    uint32_t uart1_overflows_total;
    uint16_t uart0_overflows_in_window;
    uint16_t uart1_overflows_in_window;
    uint32_t i2c_recoveries;
    bool imu_degraded;
    bool gnss_degraded;
    bool power_offline;
    uint32_t imu_silence_ms;
    uint32_t gnss_silence_ms;
    uint64_t task_idle_us;
    uint32_t task_max_idle_us; /* 0 = skip starvation check */
    bool imu_cross_check_fail;
} HealthPolicyInput;

typedef struct {
    HealthPolicyLevel level;
    bool wifi_should_disable;
    bool imu_should_degrade;
    bool gnss_should_degrade;
    bool power_should_force_offline;
    bool controlled_restart;
    bool nav_update_allowed;
    const char *primary_event; /* static string id */
} HealthPolicyDecision;

HealthPolicyLevel health_policy_classify_level(const HealthPolicyInput *in);
HealthPolicyDecision health_policy_evaluate(const HealthPolicyInput *in);

bool health_policy_imu_silence_is_degraded(uint32_t silence_ms);
bool health_policy_gnss_silence_is_degraded(uint32_t silence_ms);
bool health_policy_uart_overflow_rate_trips(uint16_t events_in_window);
bool health_policy_task_starvation_is_critical(uint64_t idle_us, uint32_t max_idle_us);
bool health_policy_nav_update_allowed(uint32_t pending_before_consume);

#ifdef __cplusplus
}
#endif
