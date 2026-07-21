/**
 * @file estimate_mode.hpp
 * @brief Generic estimate operating modes ↔ NavMode product façade.
 *
 * Use EstimateMode when talking about reusable state estimation.
 * NavMode remains the ABI for integrators of the navigation product.
 */
#pragma once

#include "NavState.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EST_MODE_UNINITIALIZED = 0,
    EST_MODE_AIDED = 1,       /* fresh aiding accepted recently */
    EST_MODE_AIDED_STALE = 2, /* aiding present but accept window stale */
    EST_MODE_COAST = 3        /* unaided / reject / outlier */
} EstimateMode;

static inline EstimateMode estimate_mode_from_nav_mode(NavMode mode)
{
    switch (mode) {
    case NAV_MODE_INITIALIZING:
        return EST_MODE_UNINITIALIZED;
    case NAV_MODE_HYBRID:
        return EST_MODE_AIDED;
    case NAV_MODE_GPS:
        return EST_MODE_AIDED_STALE;
    case NAV_MODE_DEAD_RECKONING:
        return EST_MODE_COAST;
    default:
        return EST_MODE_UNINITIALIZED;
    }
}

static inline NavMode nav_mode_from_estimate_mode(EstimateMode mode)
{
    switch (mode) {
    case EST_MODE_UNINITIALIZED:
        return NAV_MODE_INITIALIZING;
    case EST_MODE_AIDED:
        return NAV_MODE_HYBRID;
    case EST_MODE_AIDED_STALE:
        return NAV_MODE_GPS;
    case EST_MODE_COAST:
        return NAV_MODE_DEAD_RECKONING;
    default:
        return NAV_MODE_INITIALIZING;
    }
}

static inline const char *estimate_mode_name(EstimateMode mode)
{
    switch (mode) {
    case EST_MODE_UNINITIALIZED:
        return "UNINITIALIZED";
    case EST_MODE_AIDED:
        return "AIDED";
    case EST_MODE_AIDED_STALE:
        return "AIDED_STALE";
    case EST_MODE_COAST:
        return "COAST";
    default:
        return "UNKNOWN";
    }
}

#ifdef __cplusplus
}
#endif
