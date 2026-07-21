#include "NavState.h"

#include "math_utils.hpp"

#include <math.h>

#ifndef NAVICORE_CONFIDENCE_AGE_SLOPE_PER_S
#define NAVICORE_CONFIDENCE_AGE_SLOPE_PER_S 0.05f
#endif

#ifndef NAVICORE_CONFIDENCE_AGE_QUALITY_MAX
#define NAVICORE_CONFIDENCE_AGE_QUALITY_MAX 0.75f
#endif

#ifndef NAVICORE_CONFIDENCE_AGE_QUALITY_MIN
#define NAVICORE_CONFIDENCE_AGE_QUALITY_MIN 0.15f
#endif

static float nav_confidence_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

float nav_confidence_quality_from_fix_age_ms(uint32_t fix_age_ms)
{
    const float age_s = (float)fix_age_ms * 0.001f;
    const float quality =
        NAVICORE_CONFIDENCE_AGE_QUALITY_MAX - (age_s * NAVICORE_CONFIDENCE_AGE_SLOPE_PER_S);
    return nav_confidence_clampf(
        quality,
        NAVICORE_CONFIDENCE_AGE_QUALITY_MIN,
        NAVICORE_CONFIDENCE_AGE_QUALITY_MAX);
}

NavConfidence nav_confidence_make(bool gps_trusted, uint8_t satellites, uint32_t fix_age_ms, float estimate_quality)
{
    NavConfidence confidence{};
    float quality = estimate_quality;

    if (quality < 0.0f) {
        quality = 0.0f;
    } else if (quality > 1.0f) {
        quality = 1.0f;
    }

    confidence.gps_trusted = gps_trusted;
    confidence.satellites = satellites;
    confidence.fix_age_ms = fix_age_ms;
    confidence.estimate_quality = quality;
    return confidence;
}

NavState navstate_make(
    Vector3D position,
    Vector3D velocity,
    float heading_deg,
    NavDomain domain,
    NavMode mode,
    NavConfidence confidence,
    uint32_t timestamp_ms)
{
    NavState state{};
    state.position = position;
    state.velocity = velocity;
    state.heading_deg = navstate_normalize_heading(heading_deg);
    state.domain = domain;
    state.mode = mode;
    state.confidence = confidence;
    state.timestamp_ms = timestamp_ms;
    return state;
}

NavState navstate_zero(NavDomain domain)
{
    const NavConfidence confidence = nav_confidence_make(false, 0U, 0U, 0.0f);
    return navstate_make(
        vector3d_zero(),
        vector3d_zero(),
        0.0f,
        domain,
        NAV_MODE_INITIALIZING,
        confidence,
        0U);
}

float navstate_normalize_heading(float heading_deg)
{
    float heading = fmodf(heading_deg, 360.0f);
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}

float navstate_speed_mps(const NavState *state)
{
    if (state == nullptr) {
        return 0.0f;
    }

    const float vx = state->velocity.x;
    const float vy = state->velocity.y;
    const float speed_sq = (vx * vx) + (vy * vy);
    if (speed_sq <= NAVICORE_EPS_SPEED_SQ) {
        return 0.0f;
    }

    return sqrtf(speed_sq);
}
