/**
 * @file nhc_ops_policy.hpp
 * @brief Operational NHC policy frozen from GAP-3 (NHC matrix).
 *
 * Finding: naive high-rate / always-on NHC can *worsen* coast vs NHC-off
 * (exit drift 1408 m vs 493 m on super-tunnel). Production defaults must not
 * silently re-enable ALWAYS.
 *
 * This header is intentional policy + helpers — not EKF math.
 * Artefacts: docs/nhc_experiments/manifest.json · docs/NHC_OPS_POLICY.md
 */
#pragma once

#include "ins_ekf.hpp"
#include "ins_ekf_v2.hpp"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    /** Recommended default — matches ins_ekf_init (nhc_enabled=false). */
    NHC_OPS_OFF = 0,
    /**
     * Allowed production mode: NHC only when GNSS accept gap ≥ threshold
     * (same idea as ins_ekf_v2_maybe_update_nhc).
     */
    NHC_OPS_GAP_TRIGGERED = 1,
    /**
     * Experimental / A-B only (super-tunnel B_always). Not production-safe.
     * CI must keep nhc_ops_policy_is_production_safe(ALWAYS) == false.
     */
    NHC_OPS_ALWAYS = 2
} NhcOpsPolicy;

/** Compile-time / product default after GAP-3. */
#ifndef NAVICORE_NHC_OPS_POLICY_DEFAULT
#define NAVICORE_NHC_OPS_POLICY_DEFAULT NHC_OPS_OFF
#endif

static inline NhcOpsPolicy nhc_ops_policy_default(void)
{
    return (NhcOpsPolicy)NAVICORE_NHC_OPS_POLICY_DEFAULT;
}

/** OFF and GAP_TRIGGERED are allowed for shipping products; ALWAYS is not. */
static inline bool nhc_ops_policy_is_production_safe(NhcOpsPolicy policy)
{
    return policy == NHC_OPS_OFF || policy == NHC_OPS_GAP_TRIGGERED;
}

/**
 * Whether an NHC measurement update should run this tick.
 * @param feature_armed  filter nhc_enabled (must be true for any update)
 * @param gnss_gap_s     seconds since last GNSS accept (0 if unknown → treat as 0)
 * @param gap_thr_s      GAP_TRIGGERED threshold (use NAVICORE_INS_EKF_V2_NHC_GNSS_GAP_S)
 */
static inline bool nhc_ops_should_update(
    NhcOpsPolicy policy,
    bool feature_armed,
    float gnss_gap_s,
    float gap_thr_s)
{
    if (!feature_armed || policy == NHC_OPS_OFF) {
        return false;
    }
    if (policy == NHC_OPS_ALWAYS) {
        return true;
    }
    /* NHC_OPS_GAP_TRIGGERED */
    if (gap_thr_s < 0.0f) {
        gap_thr_s = NAVICORE_INS_EKF_V2_NHC_GNSS_GAP_S;
    }
    return gnss_gap_s >= gap_thr_s;
}

/**
 * Arm/disarm the filter flag from policy.
 * GAP_TRIGGERED and ALWAYS both set nhc_enabled=true; callers must still
 * gate ticks with nhc_ops_should_update or ins_ekf_v2_maybe_update_nhc.
 */
static inline void nhc_ops_apply_feature_arm(InsEkfFilter *filter, NhcOpsPolicy policy)
{
    if (filter == NULL) {
        return;
    }
    ins_ekf_set_nhc_enabled(filter, policy != NHC_OPS_OFF);
}

#ifdef __cplusplus
}
#endif
