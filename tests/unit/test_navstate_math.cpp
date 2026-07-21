#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "math_utils.hpp"
#include "NavState.h"
#include "vector3d.h"

using Catch::Matchers::WithinAbs;

TEST_CASE("navstate_normalize_heading wraps to [0, 360)", "[NavState][math]")
{
    REQUIRE_THAT(navstate_normalize_heading(0.0f), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(navstate_normalize_heading(360.0f), WithinAbs(0.0f, 1e-5f));
    REQUIRE_THAT(navstate_normalize_heading(450.0f), WithinAbs(90.0f, 1e-5f));
    REQUIRE_THAT(navstate_normalize_heading(-90.0f), WithinAbs(270.0f, 1e-5f));
    REQUIRE_THAT(navstate_normalize_heading(-720.0f), WithinAbs(0.0f, 1e-5f));
}

TEST_CASE("nav_confidence_make clamps estimate_quality", "[NavState]")
{
    const NavConfidence lo = nav_confidence_make(true, 8U, 100U, -0.5f);
    REQUIRE(lo.estimate_quality == 0.0f);
    const NavConfidence hi = nav_confidence_make(false, 0U, 0U, 2.0f);
    REQUIRE(hi.estimate_quality == 1.0f);
}

TEST_CASE("navstate_zero initializes safe defaults", "[NavState]")
{
    const NavState s = navstate_zero(NAVICORE_DOMAIN_AIR);
    REQUIRE(s.mode == NAV_MODE_INITIALIZING);
    REQUIRE(s.domain == NAVICORE_DOMAIN_AIR);
    REQUIRE(s.confidence.gps_trusted == false);
    REQUIRE(s.timestamp_ms == 0U);
    REQUIRE_THAT(s.heading_deg, WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("navstate_speed_mps uses EPS floor (no spurious sqrt)", "[NavState][math_utils]")
{
    NavState s = navstate_zero(NAVICORE_DOMAIN_AIR);
    s.velocity = vector3d_make(0.001f, 0.001f, 0.0f); /* |v|^2 << EPS_SPEED_SQ */
    REQUIRE_THAT(navstate_speed_mps(&s), WithinAbs(0.0f, 1e-9f));

    s.velocity = vector3d_make(3.0f, 4.0f, 0.0f);
    REQUIRE_THAT(navstate_speed_mps(&s), WithinAbs(5.0f, 1e-5f));

    REQUIRE_THAT(navstate_speed_mps(nullptr), WithinAbs(0.0f, 1e-9f));
}

TEST_CASE("math_utils EPS constants are consistent", "[math_utils]")
{
    REQUIRE(NAVICORE_EPS_SPEED_SQ
            == (NAVICORE_EPS_SPEED_MPS * NAVICORE_EPS_SPEED_MPS));
    REQUIRE(NAVICORE_EPS_SPEED_MPS > 0.0f);
    REQUIRE(NAVICORE_EPS_GYRO_RADPS > 0.0f);
}
