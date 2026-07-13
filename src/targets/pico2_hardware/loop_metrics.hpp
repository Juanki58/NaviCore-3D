#pragma once

#include <stdint.h>

enum class SystemHealth : uint8_t {
    NOMINAL = 0,
    DEGRADED = 1,
    CRITICAL = 2,
};

struct RuntimeHealth {
    uint32_t max_loop_us;
    uint32_t max_tick_us;
    uint32_t max_wifi_us;
    uint32_t max_housekeeping_us;
    uint32_t max_rxpump_us;
    uint32_t max_logging_us;
    uint32_t uart0_overflows;
    uint32_t uart1_overflows;
    uint32_t missed_ticks;
    uint32_t max_tick_backlog;
    uint32_t loop_budget_exceeded;
    uint32_t wifi_skipped_budget;
    uint32_t i2c_recoveries;
};

void loop_metrics_init(void);
void loop_metrics_on_loop_complete(uint64_t loop_time_us);
void loop_metrics_report_due(void);
uint32_t loop_metrics_max_loop_time_us(void);
void loop_metrics_record_rx_pump_us(uint32_t elapsed_us);
void loop_metrics_record_tick_us(uint32_t elapsed_us);
void loop_metrics_record_housekeeping_us(uint32_t elapsed_us);
void loop_metrics_record_wifi_us(uint32_t elapsed_us);
void loop_metrics_record_logging_us(uint32_t elapsed_us);
void loop_metrics_add_missed_ticks(uint32_t count);
void loop_metrics_record_tick_backlog(uint32_t pending_before_consume);
void loop_metrics_add_wifi_skipped(void);
void loop_metrics_add_i2c_recovery(void);
void loop_metrics_sync_uart_overflows(uint32_t uart0_total, uint32_t uart1_total);
void loop_metrics_update_system_health(
    uint32_t last_loop_us,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline);
void loop_metrics_set_system_health(SystemHealth health);
const RuntimeHealth *loop_metrics_health(void);
SystemHealth loop_metrics_system_health(void);
