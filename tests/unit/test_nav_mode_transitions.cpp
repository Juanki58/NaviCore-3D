#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "imu_cross_check.hpp"
#include "nav_mode_policy.hpp"

#include <cmath>
#include <cstring>
#include <string>

TEST_CASE("nav mode: uninitialized → INITIALIZING", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = false;
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_INITIALIZING);
    REQUIRE(r.confidence.estimate_quality == 0.0f);
    REQUIRE(std::string(r.reason) == "initializing");
}

TEST_CASE("nav mode: recent GNSS → HYBRID with sats quality", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = true;
    in.gps_fix_valid = true;
    in.gnss_accepted_recent = true;
    in.satellites = 10U;
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_HYBRID);
    REQUIRE(r.confidence.gps_trusted);
    REQUIRE(r.confidence.estimate_quality == Catch::Approx(0.85f).margin(1e-4f));
    REQUIRE(std::string(r.reason) == "gnss_recent_hybrid");
}

TEST_CASE("nav mode: fix valid but stale accept → GPS", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = true;
    in.gps_fix_valid = true;
    in.gnss_accepted_recent = false;
    in.satellites = 8U;
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_GPS);
    REQUIRE(r.confidence.estimate_quality == Catch::Approx(0.65f).margin(1e-4f));
}

TEST_CASE("nav mode: no fix → DEAD_RECKONING age quality", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = true;
    in.gps_fix_valid = false;
    in.fix_age_ms = 0U;
    const float q0 = nav_confidence_quality_from_fix_age_ms(0U);
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_DEAD_RECKONING);
    REQUIRE_FALSE(r.confidence.gps_trusted);
    REQUIRE(r.confidence.estimate_quality == Catch::Approx(q0).margin(1e-4f));
}

TEST_CASE("nav mode: GNSS outlier → DR quality 0.25", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = true;
    in.gps_fix_valid = true;
    in.gnss_accepted_recent = true;
    in.gnss_outlier = true;
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_DEAD_RECKONING);
    REQUIRE(r.confidence.estimate_quality == Catch::Approx(0.25f).margin(1e-4f));
}

TEST_CASE("nav mode: HYBRID→DR path when fix drops", "[navmode]")
{
    NavModeSelectInput hybrid{};
    hybrid.initialized = true;
    hybrid.gps_fix_valid = true;
    hybrid.gnss_accepted_recent = true;
    hybrid.satellites = 12U;
    REQUIRE(nav_mode_select(&hybrid).mode == NAV_MODE_HYBRID);

    NavModeSelectInput dr = hybrid;
    dr.gps_fix_valid = false;
    dr.gnss_accepted_recent = false;
    dr.fix_age_ms = 5000U;
    REQUIRE(nav_mode_select(&dr).mode == NAV_MODE_DEAD_RECKONING);
}

TEST_CASE("nav mode: DR→HYBRID reacquire", "[navmode]")
{
    NavModeSelectInput dr{};
    dr.initialized = true;
    dr.fix_age_ms = 3000U;
    REQUIRE(nav_mode_select(&dr).mode == NAV_MODE_DEAD_RECKONING);

    NavModeSelectInput hy = dr;
    hy.gps_fix_valid = true;
    hy.gnss_accepted_recent = true;
    hy.satellites = 9U;
    REQUIRE(nav_mode_select(&hy).mode == NAV_MODE_HYBRID);
}

TEST_CASE("nav mode: IMU cross-check halves quality without forcing DR", "[navmode]")
{
    NavModeSelectInput in{};
    in.initialized = true;
    in.gps_fix_valid = true;
    in.gnss_accepted_recent = true;
    in.satellites = 10U;
    in.imu_cross_check_fail = true;
    const NavModeSelectResult r = nav_mode_select(&in);
    REQUIRE(r.mode == NAV_MODE_HYBRID);
    REQUIRE(r.confidence.estimate_quality == Catch::Approx(0.425f).margin(1e-3f));
    REQUIRE(std::string(r.reason) == "imu_cross_check_quality_penalty");
}

TEST_CASE("imu cross-check: agreement within thresholds", "[imu_cross]")
{
    ImuSample a{};
    ImuSample b{};
    a.valid = true;
    b.valid = true;
    a.accel_mps2[2] = 9.81f;
    b.accel_mps2[2] = 9.81f;
    const ImuCrossCheckResult r = imu_cross_check_evaluate(&a, &b);
    REQUIRE_FALSE(r.secondary_missing);
    REQUIRE_FALSE(r.disagree);
}

TEST_CASE("imu cross-check: large accel delta → disagree", "[imu_cross]")
{
    ImuSample a{};
    ImuSample b{};
    a.valid = true;
    b.valid = true;
    a.accel_mps2[0] = 0.0f;
    b.accel_mps2[0] = IMU_CROSS_ACCEL_MAX_DELTA_MPS2 + 1.0f;
    const ImuCrossCheckResult r = imu_cross_check_evaluate(&a, &b);
    REQUIRE(r.disagree);
}

TEST_CASE("imu cross-check: missing secondary is not a false positive", "[imu_cross]")
{
    ImuSample a{};
    a.valid = true;
    const ImuCrossCheckResult r = imu_cross_check_evaluate(&a, nullptr);
    REQUIRE(r.secondary_missing);
    REQUIRE_FALSE(r.disagree);
}
