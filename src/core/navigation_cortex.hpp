/**
 * @file navigation_cortex.hpp
 * @brief Corteza de navegacion unificada — decision, eventos y telemetria (zero-heap)
 */
#ifndef NAVICORE_NAVIGATION_CORTEX_HPP
#define NAVICORE_NAVIGATION_CORTEX_HPP

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
#include "diagnostic.hpp"
#include "fusion.hpp"

#define NAV_CORTEX_MAX_EVENTS_PER_TICK 4U

#define NAV_CORTEX_TELEMETRY_FULL        0U
#define NAV_CORTEX_TELEMETRY_EVENT_ONLY  1U
#define NAV_CORTEX_TELEMETRY_SILENT      2U

#define NAV_CORTEX_CONTINGENCY_NONE          0U
#define NAV_CORTEX_CONTINGENCY_WIDEN_WAYPOINT  1U

enum NavigationEventId : uint8_t {
    NAV_EVENT_NONE = 0U,
    NAV_EVENT_SAFE_STOP = 1U,
    NAV_EVENT_HOT_RESTART = 2U,
    NAV_EVENT_HEALTH_DEGRADED = 3U,
    NAV_EVENT_HEALTH_CRITICAL = 4U,
    NAV_EVENT_HEALTH_NOMINAL = 5U,
    NAV_EVENT_GPS_LOST = 6U,
    NAV_EVENT_GPS_RESTORED = 7U,
    NAV_EVENT_WCET_VIOLATION = 8U,
    NAV_EVENT_POWER_CONSERVATION = 9U,
    NAV_EVENT_PREDICTIVE_DEGRADE = 10U,
};

typedef struct {
    uint8_t id;
    uint8_t param;
} NavigationEvent;

typedef struct {
    bool requires_safe_stop;
    bool hot_restart;
    bool predictive_degrade;
    uint8_t telemetry_tier;
    uint8_t contingency_flags;
    uint8_t event_count;
    NavigationEvent events[NAV_CORTEX_MAX_EVENTS_PER_TICK];
} NavigationDecision;

typedef struct {
    NavHealthMode prev_health;
    NavMode prev_nav_mode;
    bool prev_gps_valid;
    bool prev_safe_stop;
    uint8_t health_ema;
} NavigationCortexState;

typedef struct {
    DeadReckoningFilter *filter;
    SystemHealthMonitor *monitor;
    const NavState *nav_state;
    bool gps_fix_valid;
    bool skip_diagnostic_update;
    uint8_t filter_quality;
    uint8_t bsp_bus_status;
} NavigationCortexInput;

void navigation_cortex_init(NavigationCortexState *state);

void navigation_cortex_step(
    NavigationCortexState *state,
    const NavigationCortexInput *input,
    NavigationDecision *decision);

#endif /* NAVICORE_NAVIGATION_CORTEX_HPP */
