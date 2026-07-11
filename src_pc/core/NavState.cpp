#include "NavState.h"

#include "nav_math.h"

#include <math.h>

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
