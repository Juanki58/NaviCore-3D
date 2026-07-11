#ifndef NAVICORE_DIAGNOSTIC_HPP
#define NAVICORE_DIAGNOSTIC_HPP

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    HEALTH_NOMINAL = 0,
    HEALTH_DEGRADED,
    HEALTH_CRITICAL
} NavHealthMode;

/** Valores de bsp_bus_status (alineados con BspSpiBusState del BSP Ambiq). */
#define DIAG_BSP_BUS_IDLE        0U
#define DIAG_BSP_BUS_DMA_ACTIVE  1U
#define DIAG_BSP_BUS_ERROR       2U
#define DIAG_BSP_BUS_TIMEOUT     3U

#define DIAG_HEALTH_SCORE_MIN           0U
#define DIAG_HEALTH_SCORE_MAX           100U
#define DIAG_HEALTH_SCORE_CRITICAL_MAX   39U
#define DIAG_HEALTH_SCORE_DEGRADED_MIN   40U
#define DIAG_HEALTH_SCORE_NOMINAL_MIN    70U

#define DIAG_BSP_PENALTY_DMA_ACTIVE  5U
#define DIAG_BSP_PENALTY_ERROR      30U
#define DIAG_BSP_PENALTY_TIMEOUT    50U
#define DIAG_BSP_PENALTY_UNKNOWN    20U

/** Codigos de error de tiempo de ejecucion (time_guard). */
#define TIME_GUARD_ERROR_NONE       0U
#define TIME_GUARD_ERROR_WCET       1U

#define TIME_GUARD_WCET_PENALTY     40U

/** Codigos de error de compensacion de patinaje (slip_compensation). */
#define SLIP_COMP_ERROR_NONE        0U
#define SLIP_COMP_ERROR_SLIP        1U

#define SLIP_COMP_HEALTH_PENALTY    25U

typedef struct {
    NavHealthMode mode;
    uint8_t health_score;
    uint8_t last_filter_quality;
    uint8_t last_bsp_bus_status;
    uint8_t last_time_guard_error;
    uint8_t last_slip_comp_error;
    uint32_t last_execution_ticks;
    uint32_t last_max_allowed_ticks;
    float last_slip_ratio;
    uint32_t update_count;
} SystemHealthMonitor;

void diagnostic_update(
    SystemHealthMonitor *monitor,
    uint8_t filter_quality,
    uint8_t bsp_bus_status);

static inline uint8_t diagnostic_filter_quality_from_float(float estimate_quality)
{
    float scaled = estimate_quality * 100.0f;

    if (scaled < 0.0f) {
        scaled = 0.0f;
    } else if (scaled > 100.0f) {
        scaled = 100.0f;
    }

    return (uint8_t)scaled;
}

static inline bool diagnostic_requires_safe_stop(const SystemHealthMonitor *monitor)
{
    if (monitor == NULL) {
        return true;
    }

    return monitor->mode == HEALTH_CRITICAL;
}

#endif /* NAVICORE_DIAGNOSTIC_HPP */
