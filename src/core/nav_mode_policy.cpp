#include "nav_mode_policy.hpp"

#include <stddef.h>

const char *nav_mode_policy_name(NavMode mode)
{
    switch (mode) {
    case NAV_MODE_INITIALIZING:
        return "INITIALIZING";
    case NAV_MODE_GPS:
        return "GPS";
    case NAV_MODE_DEAD_RECKONING:
        return "DEAD_RECKONING";
    case NAV_MODE_HYBRID:
        return "HYBRID";
    default:
        return "UNKNOWN";
    }
}

static float nav_mode_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

NavModeSelectResult nav_mode_select(const NavModeSelectInput *in)
{
    NavModeSelectResult out{};
    out.mode = NAV_MODE_INITIALIZING;
    out.confidence = nav_confidence_make(false, 0U, 0U, 0.0f);
    out.reason = "null_input";

    if (in == nullptr || !in->initialized) {
        out.reason = "initializing";
        return out;
    }

    if (in->gnss_outlier) {
        out.mode = NAV_MODE_DEAD_RECKONING;
        out.confidence = nav_confidence_make(false, 0U, in->fix_age_ms, 0.25f);
        out.reason = "gnss_outlier";
    } else if (in->gps_fix_valid && in->gnss_accepted_recent) {
        const float quality =
            nav_mode_clampf(0.55f + (static_cast<float>(in->satellites) * 0.03f), 0.55f, 0.95f);
        out.mode = NAV_MODE_HYBRID;
        out.confidence = nav_confidence_make(true, in->satellites, 0U, quality);
        out.reason = "gnss_recent_hybrid";
    } else if (in->gps_fix_valid) {
        out.mode = NAV_MODE_GPS;
        out.confidence = nav_confidence_make(true, in->satellites, 0U, 0.65f);
        out.reason = "gps_fix_stale_accept";
    } else {
        const float quality = nav_confidence_quality_from_fix_age_ms(in->fix_age_ms);
        out.mode = NAV_MODE_DEAD_RECKONING;
        out.confidence = nav_confidence_make(false, 0U, in->fix_age_ms, quality);
        out.reason = "no_fix_dead_reckoning";
    }

    if (in->imu_cross_check_fail) {
        out.confidence.estimate_quality =
            nav_mode_clampf(out.confidence.estimate_quality * 0.50f, 0.0f, 1.0f);
        out.reason = "imu_cross_check_quality_penalty";
    }

    return out;
}
