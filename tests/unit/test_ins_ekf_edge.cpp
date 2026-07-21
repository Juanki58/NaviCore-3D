#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "estimate_mode.hpp"
#include "ins_ekf.hpp"
#include "meas_reject.hpp"
#include "sensor_types.hpp"
#include "vector3d.h"

#include <cmath>

namespace {

ImuSample make_finite_imu(uint32_t t_ms)
{
    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = t_ms;
    imu.accel_mps2[0] = 0.0f;
    imu.accel_mps2[1] = 0.0f;
    imu.accel_mps2[2] = -9.80665f;
    imu.gyro_radps[0] = 0.0f;
    imu.gyro_radps[1] = 0.0f;
    imu.gyro_radps[2] = 0.0f;
    return imu;
}

} /* namespace */

TEST_CASE("meas reject aliases match GNSS macros", "[ekf_edge][meas_reject]")
{
    REQUIRE(INS_EKF_GNSS_REJECT_NIS == MEAS_REJECT_NIS);
    REQUIRE(INS_EKF_GNSS_REJECT_S_SINGULAR == MEAS_REJECT_S_SINGULAR);
    REQUIRE(INS_EKF_GNSS_REJECT_INCONSISTENT == MEAS_REJECT_INCONSISTENT);
}

TEST_CASE("estimate_mode ↔ NavMode round-trip", "[ekf_edge][estimate_mode]")
{
    REQUIRE(estimate_mode_from_nav_mode(NAV_MODE_HYBRID) == EST_MODE_AIDED);
    REQUIRE(estimate_mode_from_nav_mode(NAV_MODE_GPS) == EST_MODE_AIDED_STALE);
    REQUIRE(estimate_mode_from_nav_mode(NAV_MODE_DEAD_RECKONING) == EST_MODE_COAST);
    REQUIRE(nav_mode_from_estimate_mode(EST_MODE_COAST) == NAV_MODE_DEAD_RECKONING);
}

TEST_CASE("EKF predict before init → false", "[ekf_edge]")
{
    InsEkfFilter ekf{};
    /* zero-init: not initialized */
    ImuSample imu = make_finite_imu(10U);
    REQUIRE_FALSE(ins_ekf_predict(&ekf, &imu));
}

TEST_CASE("EKF predict dt path: null / invalid IMU → false, state unchanged", "[ekf_edge]")
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    const float p0 = ekf.pos_[0];

    REQUIRE_FALSE(ins_ekf_predict(&ekf, nullptr));
    REQUIRE(ekf.pos_[0] == p0);

    ImuSample bad = make_finite_imu(20U);
    bad.valid = false;
    REQUIRE_FALSE(ins_ekf_predict(&ekf, &bad));
    REQUIRE(ekf.pos_[0] == p0);
}

TEST_CASE("EKF predict rejects NaN accel; state frozen", "[ekf_edge]")
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    ImuSample good = make_finite_imu(10U);
    REQUIRE(ins_ekf_predict(&ekf, &good));

    const float pn = ekf.pos_[0];
    ImuSample nan_imu = make_finite_imu(20U);
    nan_imu.accel_mps2[0] = NAN;
    REQUIRE_FALSE(ins_ekf_predict(&ekf, &nan_imu));
    REQUIRE(ekf.pos_[0] == pn);
}

TEST_CASE("EKF GNSS update before init → false", "[ekf_edge]")
{
    InsEkfFilter ekf{};
    GpsSample gps{};
    gps.fix_valid = true;
    gps.position = vector3d_make(41.3874f, 2.1686f, 12.0f);
    gps.satellites = 8U;
    REQUIRE_FALSE(ins_ekf_update_gnss(&ekf, &gps));
}

TEST_CASE("EKF GNSS invalid fix → false (fail-closed)", "[ekf_edge]")
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    ImuSample imu = make_finite_imu(10U);
    (void)ins_ekf_predict(&ekf, &imu);

    GpsSample gps{};
    gps.fix_valid = false;
    gps.position = origin;
    REQUIRE_FALSE(ins_ekf_update_gnss(&ekf, &gps));
}

TEST_CASE("EKF two predicts @100Hz: horizontal motion bounded", "[ekf_edge]")
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    InsEkfFilter ekf{};
    ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    ekf.vel_[0] = 10.0f; /* 10 m/s north */

    ImuSample a = make_finite_imu(10U);
    ImuSample b = make_finite_imu(20U);
    REQUIRE(ins_ekf_predict(&ekf, &a));
    REQUIRE(ins_ekf_predict(&ekf, &b));
    /* ~0.2 m north in 20 ms at 10 m/s — civil envelope */
    REQUIRE(ekf.pos_[0] == Catch::Approx(0.2f).margin(0.05f));
}
