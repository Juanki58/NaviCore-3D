/**
 * @file nav_mode_policy.hpp
 * @brief Explicit NavMode selection + confidence (integrator-facing contract).
 *
 * Production Pico path (`ins_ekf_export_nav_state`) must call this — do not
 * fork mode logic elsewhere without updating docs/NAV_MODE_DEGRADATION.md.
 */
#pragma once

#include "NavState.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** GNSS accept is "recent" for HYBRID if within this window (ms). */
#ifndef NAV_MODE_GNSS_RECENT_MS
#define NAV_MODE_GNSS_RECENT_MS 2000U
#endif

typedef struct {
    bool initialized;
    bool gps_fix_valid;
    bool gnss_accepted_recent; /* last accept age ≤ NAV_MODE_GNSS_RECENT_MS */
    bool gnss_outlier;
    uint32_t fix_age_ms; /* age since last accept; used in DR */
    uint8_t satellites;
    bool imu_cross_check_fail; /* vigilante IMU disagreement */
} NavModeSelectInput;

typedef struct {
    NavMode mode;
    NavConfidence confidence;
    const char *reason; /* static string id for logs / tests */
} NavModeSelectResult;

NavModeSelectResult nav_mode_select(const NavModeSelectInput *in);

/** Human-readable mode name (never NULL). */
const char *nav_mode_policy_name(NavMode mode);

#ifdef __cplusplus
}
#endif
