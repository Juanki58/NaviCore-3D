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
#include "ins_ekf.hpp"
#include "ins_ekf_math.hpp"
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
