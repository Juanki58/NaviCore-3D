/**
 * Property-based tests (RapidCheck) — invariants over randomly generated inputs.
 *
 * Unlike Monte Carlo scenario scripts, these state *always-true* properties and
 * let RapidCheck search for counterexamples (with shrinking).
 *
 * Generators use bounded integer ranges cast to float — unrestricted
 * gen::arbitrary<float>() hits Inf/huge values and causes RC_PRE give-up.
 */
#include <catch2/catch_test_macros.hpp>

#include <rapidcheck.h>

#include "NavState.h"
#include "geodesy.hpp"
#include "ins_ekf.hpp"
#include "ins_ekf_math.hpp"
#include "meas_reject.hpp"
#include "sensor_types.hpp"
#include "vector3d.h"

#include <cmath>
#include <cstdint>

namespace {

constexpr float kTickDtS = 0.01f; /* 100 Hz */
constexpr float kMaxSaneSpeedMps = 80.0f;
constexpr float kMaxSaneAccelMps2 = 40.0f;
constexpr float kMaxPosJumpM =
    (kMaxSaneSpeedMps * kTickDtS)
    + (0.5f * kMaxSaneAccelMps2 * kTickDtS * kTickDtS)
    + 0.50f;

float horiz_ned_m(float n0, float e0, float n1, float e1)
{
    const float dn = n1 - n0;
    const float de = e1 - e0;
    return std::sqrt((dn * dn) + (de * de));
}

/** Uniform float in [lo, hi] via milli-units (avoids wild IEEE floats). */
rc::Gen<float> genBoundedFloat(float lo, float hi)
{
    const int lo_i = static_cast<int>(lo * 1000.0f);
    const int hi_i = static_cast<int>(hi * 1000.0f);
    return rc::gen::map(rc::gen::inRange(lo_i, hi_i + 1), [](int v) {
        return static_cast<float>(v) * 0.001f;
    });
}

ImuSample make_level_imu(uint32_t t_ms)
{
    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = t_ms;
    imu.accel_mps2[0] = 0.0f;
    imu.accel_mps2[1] = 0.0f;
    imu.accel_mps2[2] = 9.80665f;
    return imu;
}

/** Seed continuous-track GNSS + short coast so consistency gate (reason=3) can fire. */
void seed_ekf_for_integrity_gate(InsEkfFilter *ekf, const Vector3D &origin)
{
    ins_ekf_init(ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
    ins_ekf_set_consistency_check_enabled(ekf, true);
    ins_ekf_set_gnss_obs_mode(ekf, INS_EKF_GNSS_OBS_POS);
    ekf->vel_[0] = 10.0f;
    ekf->vel_[1] = 0.0f;
    ekf->vel_[2] = 0.0f;

    GpsSample good{};
    good.fix_valid = true;
    good.timestamp_ms = 50U;
    good.position = origin;
    good.speed_mps = 10.0f;
    good.course_deg = 0.0f;
    good.satellites = 12U;
    RC_ASSERT(ins_ekf_update_gnss(ekf, &good));

    for (uint32_t t_ms = 60U; t_ms <= 200U; t_ms += 10U) {
        ImuSample imu = make_level_imu(t_ms);
        RC_ASSERT(ins_ekf_predict(ekf, &imu));
    }
}

GpsSample gps_at_ned_offset(const InsEkfFilter &ekf, float dn_m, float de_m, uint32_t t_ms)
{
    float lat = 0.0f;
    float lon = 0.0f;
    float alt = 0.0f;
    geodesy::ned_to_lla(
        ekf.ref_lat_deg,
        ekf.ref_lon_deg,
        ekf.ref_alt_m,
        ekf.pos_[0] + dn_m,
        ekf.pos_[1] + de_m,
        ekf.pos_[2],
        &lat,
        &lon,
        &alt);

    GpsSample gps{};
    gps.fix_valid = true;
    gps.timestamp_ms = t_ms;
    gps.position = vector3d_make(lat, lon, alt);
    gps.speed_mps = 10.0f;
    gps.course_deg = 0.0f;
    gps.satellites = 12U;
    return gps;
}

} /* namespace */

TEST_CASE("PROP: DR quality never rises when fix_age increases", "[rapidcheck][NavState]")
{
    const bool ok = rc::check(
        "quality(age_a) >= quality(age_b) when age_a <= age_b",
        [] {
            const auto age_a = *rc::gen::inRange<uint32_t>(0U, 3600000U);
            const auto age_b = *rc::gen::inRange<uint32_t>(0U, 3600000U);
            const uint32_t young = (age_a <= age_b) ? age_a : age_b;
            const uint32_t old = (age_a <= age_b) ? age_b : age_a;

            const float qa = nav_confidence_quality_from_fix_age_ms(young);
            const float qb = nav_confidence_quality_from_fix_age_ms(old);
            RC_ASSERT(qa >= qb - 1.0e-6f);
            RC_ASSERT(qa >= 0.15f - 1.0e-6f);
            RC_ASSERT(qa <= 0.75f + 1.0e-6f);
            RC_ASSERT(qb >= 0.15f - 1.0e-6f);
            RC_ASSERT(qb <= 0.75f + 1.0e-6f);
        });
    REQUIRE(ok);
}

TEST_CASE("PROP: heading normalize always lands in [0, 360)", "[rapidcheck][NavState]")
{
    const bool ok = rc::check([] {
        const float heading = *genBoundedFloat(-1.0e5f, 1.0e5f);
        const float n = navstate_normalize_heading(heading);
        RC_ASSERT(std::isfinite(n));
        RC_ASSERT(n >= 0.0f);
        RC_ASSERT(n < 360.0f);
    });
    REQUIRE(ok);
}

TEST_CASE("PROP: quat_normalize yields unit or identity (no /0)", "[rapidcheck][ins_ekf_math]")
{
    const bool ok = rc::check([] {
        float q[4] = {
            *genBoundedFloat(-10.0f, 10.0f),
            *genBoundedFloat(-10.0f, 10.0f),
            *genBoundedFloat(-10.0f, 10.0f),
            *genBoundedFloat(-10.0f, 10.0f),
        };
        navicore_quat_normalize(q);
        for (int i = 0; i < 4; ++i) {
            RC_ASSERT(std::isfinite(q[i]));
        }
        const float n2 =
            (q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]);
        RC_ASSERT(std::fabs(n2 - 1.0f) < 1.0e-4f);
    });
    REQUIRE(ok);
}

TEST_CASE("PROP: invert2x2 success ⇒ S·S⁻¹ ≈ I", "[rapidcheck][ins_ekf_math]")
{
    const bool ok = rc::check([] {
        const float a = *genBoundedFloat(-50.0f, 50.0f);
        const float b = *genBoundedFloat(-50.0f, 50.0f);
        const float c = *genBoundedFloat(-50.0f, 50.0f);
        const float d = *genBoundedFloat(-50.0f, 50.0f);
        const float s[2][2] = {{a, b}, {c, d}};
        float inv[2][2]{};
        if (!navicore_mat_invert2x2(s, inv)) {
            return;
        }
        const float i00 = a * inv[0][0] + b * inv[1][0];
        const float i01 = a * inv[0][1] + b * inv[1][1];
        const float i10 = c * inv[0][0] + d * inv[1][0];
        const float i11 = c * inv[0][1] + d * inv[1][1];
        RC_ASSERT(std::fabs(i00 - 1.0f) < 5.0e-3f);
        RC_ASSERT(std::fabs(i11 - 1.0f) < 5.0e-3f);
        RC_ASSERT(std::fabs(i01) < 5.0e-3f);
        RC_ASSERT(std::fabs(i10) < 5.0e-3f);
    });
    REQUIRE(ok);
}

TEST_CASE(
    "PROP: EKF predict @100Hz — horizontal jump bounded for sane |v|,|a|",
    "[rapidcheck][ins_ekf]")
{
    const bool ok = rc::check(
        "one 10 ms predict cannot teleport under civil envelope",
        [] {
            const float vn = *genBoundedFloat(-kMaxSaneSpeedMps, kMaxSaneSpeedMps);
            const float ve = *genBoundedFloat(-kMaxSaneSpeedMps, kMaxSaneSpeedMps);
            const float ax = *genBoundedFloat(-kMaxSaneAccelMps2, kMaxSaneAccelMps2);
            const float ay = *genBoundedFloat(-kMaxSaneAccelMps2, kMaxSaneAccelMps2);
            const float az =
                *genBoundedFloat(9.80665f - kMaxSaneAccelMps2, 9.80665f + kMaxSaneAccelMps2);

            InsEkfFilter ekf{};
            const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
            ins_ekf_init(&ekf, origin, 0.0f, NAVICORE_DOMAIN_AIR);
            ekf.vel_[0] = vn;
            ekf.vel_[1] = ve;
            ekf.vel_[2] = 0.0f;

            const float n0 = ekf.pos_[0];
            const float e0 = ekf.pos_[1];

            ImuSample imu{};
            imu.valid = true;
            imu.timestamp_ms = 10U;
            imu.accel_mps2[0] = ax;
            imu.accel_mps2[1] = ay;
            imu.accel_mps2[2] = az;

            RC_ASSERT(ins_ekf_predict(&ekf, &imu));
            const float jump = horiz_ned_m(n0, e0, ekf.pos_[0], ekf.pos_[1]);
            RC_ASSERT(std::isfinite(jump));
            RC_ASSERT(jump <= kMaxPosJumpM);
        });
    REQUIRE(ok);
}

TEST_CASE(
    "PROP: GNSS teleport on short gap → reject INCONSISTENT; state/P hold",
    "[rapidcheck][integrity][ins_ekf]")
{
    const bool ok = rc::check(
        "SW spoof jump on continuous track must trip consistency gate",
        [] {
            /* 200–800 m North — always above CONSISTENCY_MAX_POS_JUMP_M (120 m). */
            const float jump_m = *genBoundedFloat(200.0f, 800.0f);

            InsEkfFilter ekf{};
            const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
            seed_ekf_for_integrity_gate(&ekf, origin);

            const float pos_n0 = ekf.pos_[0];
            const float pos_e0 = ekf.pos_[1];
            const float p_nn0 = ekf.cov.P[INS_ERR_POS_N][INS_ERR_POS_N];
            const float p_ee0 = ekf.cov.P[INS_ERR_POS_E][INS_ERR_POS_E];

            GpsSample spoof = gps_at_ned_offset(ekf, jump_m, 0.0f, 250U);
            const bool accepted = ins_ekf_update_gnss(&ekf, &spoof);

            RC_ASSERT(!accepted);
            RC_ASSERT(ekf.gnss_last_reject_reason == MEAS_REJECT_INCONSISTENT);
            RC_ASSERT(ins_ekf_gnss_consistency_last_suspect(&ekf));

            /* Fail-closed: rejected aiding must not pull state or collapse P. */
            RC_ASSERT(std::fabs(ekf.pos_[0] - pos_n0) < 1.0e-3f);
            RC_ASSERT(std::fabs(ekf.pos_[1] - pos_e0) < 1.0e-3f);
            RC_ASSERT(ekf.cov.P[INS_ERR_POS_N][INS_ERR_POS_N] >= p_nn0 * 0.99f);
            RC_ASSERT(ekf.cov.P[INS_ERR_POS_E][INS_ERR_POS_E] >= p_ee0 * 0.99f);
        });
    REQUIRE(ok);
}

TEST_CASE(
    "PROP: tiny GNSS nudge on short gap is never reason=INCONSISTENT",
    "[rapidcheck][integrity][ins_ekf]")
{
    const bool ok = rc::check(
        "sub-metre multipath-scale jump must not trip spoof consistency",
        [] {
            const float dn = *genBoundedFloat(-2.0f, 2.0f);
            const float de = *genBoundedFloat(-2.0f, 2.0f);

            InsEkfFilter ekf{};
            const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
            seed_ekf_for_integrity_gate(&ekf, origin);

            GpsSample nudge = gps_at_ned_offset(ekf, dn, de, 250U);
            (void)ins_ekf_update_gnss(&ekf, &nudge);

            /* May accept or NIS-reject; must not classify as physical spoof. */
            RC_ASSERT(ekf.gnss_last_reject_reason != MEAS_REJECT_INCONSISTENT);
            RC_ASSERT(!ins_ekf_gnss_consistency_last_suspect(&ekf));
        });
    REQUIRE(ok);
}
