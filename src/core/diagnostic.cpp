#include "diagnostic.hpp"

static uint8_t diagnostic_clamp_score_u8(int32_t value)
{
    if (value < (int32_t)DIAG_HEALTH_SCORE_MIN) {
        return DIAG_HEALTH_SCORE_MIN;
    }
    if (value > (int32_t)DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return (uint8_t)value;
}

static uint8_t diagnostic_clamp_filter_quality(uint8_t filter_quality)
{
    if (filter_quality > DIAG_HEALTH_SCORE_MAX) {
        return DIAG_HEALTH_SCORE_MAX;
    }
    return filter_quality;
}

static uint8_t diagnostic_bsp_status_penalty(uint8_t bsp_bus_status)
{
    switch (bsp_bus_status) {
    case DIAG_BSP_BUS_IDLE:
        return 0U;
    case DIAG_BSP_BUS_DMA_ACTIVE:
        return DIAG_BSP_PENALTY_DMA_ACTIVE;
    case DIAG_BSP_BUS_ERROR:
        return DIAG_BSP_PENALTY_ERROR;
    case DIAG_BSP_BUS_TIMEOUT:
        return DIAG_BSP_PENALTY_TIMEOUT;
    default:
        return DIAG_BSP_PENALTY_UNKNOWN;
    }
}

static NavHealthMode diagnostic_mode_from_score(uint8_t health_score)
{
    if (health_score <= DIAG_HEALTH_SCORE_CRITICAL_MAX) {
        return HEALTH_CRITICAL;
    }
    if (health_score < DIAG_HEALTH_SCORE_NOMINAL_MIN) {
        return HEALTH_DEGRADED;
    }
    return HEALTH_NOMINAL;
}

void diagnostic_update(
    SystemHealthMonitor *monitor,
    uint8_t filter_quality,
    uint8_t bsp_bus_status)
{
    if (monitor == NULL) {
        return;
    }

    const uint8_t clamped_quality = diagnostic_clamp_filter_quality(filter_quality);
    const uint8_t bsp_penalty = diagnostic_bsp_status_penalty(bsp_bus_status);
    const uint8_t health_score = diagnostic_clamp_score_u8(
        (int32_t)clamped_quality - (int32_t)bsp_penalty);

    monitor->health_score = health_score;
    monitor->mode = diagnostic_mode_from_score(health_score);
    monitor->last_filter_quality = clamped_quality;
    monitor->last_bsp_bus_status = bsp_bus_status;
    monitor->update_count++;
}
