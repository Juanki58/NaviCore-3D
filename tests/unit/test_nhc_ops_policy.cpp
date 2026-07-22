/**
 * Freeze GAP-3 operational NHC policy in CI.
 * ALWAYS must never be production-safe; init default remains NHC off.
 */
#include <catch2/catch_test_macros.hpp>

#include "ins_ekf.hpp"
#include "nhc_ops_policy.hpp"
#include "vector3d.h"

TEST_CASE("NHC ops: default is OFF and production-safe", "[nhc_ops][GAP-3]")
{
    REQUIRE(nhc_ops_policy_default() == NHC_OPS_OFF);
    REQUIRE(nhc_ops_policy_is_production_safe(NHC_OPS_OFF));
    REQUIRE(nhc_ops_policy_is_production_safe(NHC_OPS_GAP_TRIGGERED));
    REQUIRE_FALSE(nhc_ops_policy_is_production_safe(NHC_OPS_ALWAYS));
}

TEST_CASE("NHC ops: tick gate matches GAP-3 intent", "[nhc_ops][GAP-3]")
{
    constexpr float kGapThr = 1.0f;

    REQUIRE_FALSE(nhc_ops_should_update(NHC_OPS_OFF, true, 10.0f, kGapThr));
    REQUIRE_FALSE(nhc_ops_should_update(NHC_OPS_GAP_TRIGGERED, false, 10.0f, kGapThr));
    REQUIRE_FALSE(nhc_ops_should_update(NHC_OPS_GAP_TRIGGERED, true, 0.2f, kGapThr));
    REQUIRE(nhc_ops_should_update(NHC_OPS_GAP_TRIGGERED, true, 1.0f, kGapThr));
    REQUIRE(nhc_ops_should_update(NHC_OPS_GAP_TRIGGERED, true, 5.0f, kGapThr));

    /* ALWAYS ignores gap — lab only; still requires feature arm. */
    REQUIRE_FALSE(nhc_ops_should_update(NHC_OPS_ALWAYS, false, 0.0f, kGapThr));
    REQUIRE(nhc_ops_should_update(NHC_OPS_ALWAYS, true, 0.0f, kGapThr));
}

TEST_CASE("NHC ops: ins_ekf_init leaves feature disarmed", "[nhc_ops][GAP-3]")
{
    InsEkfFilter ekf{};
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);

    REQUIRE_FALSE(ins_ekf_nhc_enabled(&ekf));

    nhc_ops_apply_feature_arm(&ekf, NHC_OPS_OFF);
    REQUIRE_FALSE(ins_ekf_nhc_enabled(&ekf));

    nhc_ops_apply_feature_arm(&ekf, NHC_OPS_GAP_TRIGGERED);
    REQUIRE(ins_ekf_nhc_enabled(&ekf));

    nhc_ops_apply_feature_arm(&ekf, NHC_OPS_ALWAYS);
    REQUIRE(ins_ekf_nhc_enabled(&ekf));

    nhc_ops_apply_feature_arm(&ekf, NHC_OPS_OFF);
    REQUIRE_FALSE(ins_ekf_nhc_enabled(&ekf));
}
