#pragma once

#include "loop_metrics.hpp"
#include "task_monitor.hpp"

#include <stdbool.h>
#include <stdint.h>

enum class SystemHealth : uint8_t {
    NOMINAL = 0,
    DEGRADED = 1,
    CRITICAL = 2,
};

enum class HealthPolicyRecovery : uint8_t {
    Recoverable = 0,
    Permanent = 1,
};

struct HealthPolicyDescriptor {
    const char *event_id;
    HealthPolicyRecovery recovery;
    SystemHealth classification;
};

void health_monitor_init(void);
bool health_monitor_wifi_poll_allowed(void);
bool health_monitor_nav_update_allowed(uint32_t pending_before_consume);
void health_monitor_check_task_deadline(TaskId id, uint32_t max_idle_us, const char *label);
void health_monitor_on_loop_complete(
    uint32_t loop_us,
    uint32_t nav_timestamp_ms,
    bool imu_degraded,
    bool gnss_degraded,
    bool power_offline);
const RuntimeHealth *health_monitor_runtime(void);
SystemHealth health_monitor_system_health(void);
const HealthPolicyDescriptor *health_monitor_policy_table(uint8_t *count);
