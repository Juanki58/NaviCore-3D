/**
 * @file estimate_quality.hpp
 * @brief Generic estimate trust (score + aiding age) extracted from NavConfidence.
 *
 * NavConfidence keeps GNSS overlays (satellites, gps_trusted) for the nav product.
 * Callers that only need integrity / mission guards can use EstimateQualityView.
 */
#pragma once

#include "NavState.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float score;           /* 0..1 — same semantics as NavConfidence.estimate_quality */
    uint32_t aiding_age_ms; /* age since last accepted aiding (fix_age_ms in nav) */
    bool aiding_trusted;   /* generic; maps from gps_trusted in nav façade */
} EstimateQualityView;

static inline EstimateQualityView estimate_quality_from_nav_confidence(NavConfidence c)
{
    EstimateQualityView v{};
    v.score = c.estimate_quality;
    v.aiding_age_ms = c.fix_age_ms;
    v.aiding_trusted = c.gps_trusted;
    return v;
}

/** Age-based coast quality (shared with nav_confidence_quality_from_fix_age_ms). */
static inline float estimate_quality_from_aiding_age_ms(uint32_t aiding_age_ms)
{
    return nav_confidence_quality_from_fix_age_ms(aiding_age_ms);
}

#ifdef __cplusplus
}
#endif
