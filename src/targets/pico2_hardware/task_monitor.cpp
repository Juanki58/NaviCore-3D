#include "task_monitor.hpp"

namespace {

TaskMonitor g_monitors[static_cast<uint8_t>(TaskId::Count)]{};

} /* namespace */

void task_monitor_init(void)
{
    for (uint8_t i = 0U; i < static_cast<uint8_t>(TaskId::Count); ++i) {
        g_monitors[i] = TaskMonitor{};
    }
}

void task_monitor_record(TaskId id, uint32_t nav_tick)
{
    const uint8_t idx = static_cast<uint8_t>(id);
    if (idx >= static_cast<uint8_t>(TaskId::Count)) {
        return;
    }

    TaskMonitor *monitor = &g_monitors[idx];
    monitor->last_execution_tick = nav_tick;
    if (monitor->executions < 0xFFFFFFFFU) {
        ++monitor->executions;
    }
}

const TaskMonitor *task_monitor_get(TaskId id)
{
    const uint8_t idx = static_cast<uint8_t>(id);
    if (idx >= static_cast<uint8_t>(TaskId::Count)) {
        return nullptr;
    }

    return &g_monitors[idx];
}
