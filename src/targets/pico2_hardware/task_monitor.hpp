#pragma once

#include <stdint.h>

struct TaskMonitor {
    uint32_t last_execution_tick;
    uint32_t executions;
};

enum class TaskId : uint8_t {
    RxPump = 0,
    NavTick,
    Housekeeping,
    Wifi,
    Logging,
    Loop,
    Count
};

void task_monitor_init(void);
void task_monitor_record(TaskId id, uint32_t nav_tick);
const TaskMonitor *task_monitor_get(TaskId id);
