#include "time_guard.hpp"

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <time.h>
#endif

static struct {
    bool session_active;
    bool platform_ready;
    uint32_t start_ticks;
#ifdef _WIN32
    LARGE_INTEGER qpc_frequency;
    LARGE_INTEGER qpc_start;
#else
    struct timespec mono_start;
#endif
} g_time_guard{};

static uint32_t time_guard_clamp_score(int32_t value)
{
    if (value < (int32_t)DIAG_HEALTH_SCORE_MIN) {
        return DIAG_HEALTH_SCORE_MIN;
    }
    if (value > (int32_t)DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return (uint32_t)value;
}

static NavHealthMode time_guard_mode_from_score(uint8_t health_score)
{
    if (health_score <= DIAG_HEALTH_SCORE_CRITICAL_MAX) {
        return HEALTH_CRITICAL;
    }
    if (health_score < DIAG_HEALTH_SCORE_NOMINAL_MIN) {
        return HEALTH_DEGRADED;
    }
    return HEALTH_NOMINAL;
}

static void time_guard_inject_wcet_error(
    SystemHealthMonitor *monitor,
    uint32_t execution_ticks,
    uint32_t max_allowed_ticks)
{
    monitor->last_time_guard_error = TIME_GUARD_ERROR_WCET;
    monitor->last_execution_ticks = execution_ticks;
    monitor->last_max_allowed_ticks = max_allowed_ticks;

    const int32_t penalized = (int32_t)monitor->health_score - (int32_t)TIME_GUARD_WCET_PENALTY;
    monitor->health_score = (uint8_t)time_guard_clamp_score(penalized);
    monitor->mode = time_guard_mode_from_score(monitor->health_score);
    monitor->update_count++;
}

#ifdef NAVICORE_TARGET_AMBIQ

extern "C" uint32_t am_hal_stimer_counter_get(void);

static uint32_t time_guard_read_platform_ticks(void)
{
    /*
     * TODO(Ambiq): escalar STimer a ciclos logicos @ CTIMER_HZ.
     * Stub host/Ambiq sin SDK: contador monotono emulado.
     */
    return am_hal_stimer_counter_get();
}

#else

static uint32_t time_guard_elapsed_refresh_ticks(void)
{
#ifdef _WIN32
    LARGE_INTEGER now{};
    QueryPerformanceCounter(&now);

    const double elapsed_s =
        (double)(now.QuadPart - g_time_guard.qpc_start.QuadPart) /
        (double)g_time_guard.qpc_frequency.QuadPart;

    const double refresh_ticks = elapsed_s * (double)TIME_GUARD_PC_REFRESH_HZ;
    if (refresh_ticks <= 0.0) {
        return 0U;
    }
    return (uint32_t)refresh_ticks;
#else
    struct timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);

    const int64_t start_ns =
        ((int64_t)g_time_guard.mono_start.tv_sec * 1000000000LL) +
        (int64_t)g_time_guard.mono_start.tv_nsec;
    const int64_t now_ns =
        ((int64_t)now.tv_sec * 1000000000LL) + (int64_t)now.tv_nsec;
    const int64_t delta_ns = now_ns - start_ns;

    if (delta_ns <= 0) {
        return 0U;
    }

    const int64_t ns_per_refresh = 1000000000LL / (int64_t)TIME_GUARD_PC_REFRESH_HZ;
    return (uint32_t)(delta_ns / ns_per_refresh);
#endif
}

static uint32_t time_guard_read_platform_ticks(void)
{
    return time_guard_elapsed_refresh_ticks();
}

#endif /* NAVICORE_TARGET_AMBIQ */

void time_guard_init(void)
{
    g_time_guard.session_active = false;
    g_time_guard.start_ticks = 0U;

#ifdef _WIN32
    QueryPerformanceFrequency(&g_time_guard.qpc_frequency);
#endif

    g_time_guard.platform_ready = true;
}

void time_guard_start(void)
{
    if (!g_time_guard.platform_ready) {
        time_guard_init();
    }

#ifdef _WIN32
    QueryPerformanceCounter(&g_time_guard.qpc_start);
#else
#ifndef NAVICORE_TARGET_AMBIQ
    clock_gettime(CLOCK_MONOTONIC, &g_time_guard.mono_start);
#endif
#endif

    g_time_guard.start_ticks = time_guard_read_platform_ticks();
    g_time_guard.session_active = true;
}

uint32_t time_guard_stop(void)
{
    if (!g_time_guard.session_active) {
        return 0U;
    }

    const uint32_t end_ticks = time_guard_read_platform_ticks();
    g_time_guard.session_active = false;

    if (end_ticks >= g_time_guard.start_ticks) {
        return end_ticks - g_time_guard.start_ticks;
    }

    return 0U;
}

bool time_guard_validate(
    uint32_t execution_ticks,
    uint32_t max_allowed_ticks,
    SystemHealthMonitor *monitor)
{
    if (monitor == NULL) {
        return false;
    }

    if (execution_ticks <= max_allowed_ticks) {
        monitor->last_time_guard_error = TIME_GUARD_ERROR_NONE;
        monitor->last_execution_ticks = execution_ticks;
        monitor->last_max_allowed_ticks = max_allowed_ticks;
        return true;
    }

    time_guard_inject_wcet_error(monitor, execution_ticks, max_allowed_ticks);
    return false;
}

uint32_t time_guard_pc_refresh_hz(void)
{
    return TIME_GUARD_PC_REFRESH_HZ;
}

#if defined(NAVICORE_TARGET_AMBIQ) && !defined(NAVICORE_AMBIQ_SDK)

extern "C" uint32_t am_hal_stimer_counter_get(void)
{
    static uint32_t g_stimer_stub = 0U;
    g_stimer_stub += 1U;
    return g_stimer_stub;
}

#endif
