#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "ins_ekf_math.hpp"

#include <cmath>

using Catch::Matchers::WithinAbs;

TEST_CASE("quat_normalize: unit quaternion unchanged", "[ins_ekf_math][quat]")
{
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    navicore_quat_normalize(q);
    REQUIRE_THAT(q[0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q[1], WithinAbs(0.0f, 1e-6f));
}

TEST_CASE("quat_normalize: scales non-unit quat", "[ins_ekf_math][quat]")
{
    float q[4] = {2.0f, 0.0f, 0.0f, 0.0f};
    navicore_quat_normalize(q);
    REQUIRE_THAT(q[0], WithinAbs(1.0f, 1e-5f));
    const float n =
        std::sqrt((q[0] * q[0]) + (q[1] * q[1]) + (q[2] * q[2]) + (q[3] * q[3]));
    REQUIRE_THAT(n, WithinAbs(1.0f, 1e-5f));
}

TEST_CASE("quat_normalize: zero / near-zero → identity (no /0)", "[ins_ekf_math][quat]")
{
    float q[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    navicore_quat_normalize(q);
    REQUIRE_THAT(q[0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(q[1], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q[2], WithinAbs(0.0f, 1e-6f));
    REQUIRE_THAT(q[3], WithinAbs(0.0f, 1e-6f));

    float tiny[4] = {1e-8f, 1e-8f, 1e-8f, 1e-8f};
    navicore_quat_normalize(tiny);
    REQUIRE_THAT(tiny[0], WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("mat_invert2x2: identity", "[ins_ekf_math][matrix]")
{
    const float s[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    float inv[2][2]{};
    REQUIRE(navicore_mat_invert2x2(s, inv));
    REQUIRE_THAT(inv[0][0], WithinAbs(1.0f, 1e-6f));
    REQUIRE_THAT(inv[1][1], WithinAbs(1.0f, 1e-6f));
}

TEST_CASE("mat_invert2x2: singular / ill-conditioned → false", "[ins_ekf_math][matrix]")
{
    const float singular[2][2] = {{1.0f, 2.0f}, {2.0f, 4.0f}};
    float inv[2][2] = {{9.0f, 9.0f}, {9.0f, 9.0f}};
    REQUIRE_FALSE(navicore_mat_invert2x2(singular, inv));

    const float near_sing[2][2] = {{1.0f, 0.0f}, {0.0f, 1e-20f}};
    REQUIRE_FALSE(navicore_mat_invert2x2(near_sing, inv));
}

TEST_CASE("mat_invert3x3: identity and singular", "[ins_ekf_math][matrix]")
{
    const float I[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f},
    };
    float inv[3][3]{};
    REQUIRE(navicore_mat_invert3x3(I, inv));
    REQUIRE_THAT(inv[2][2], WithinAbs(1.0f, 1e-6f));

    const float singular[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {2.0f, 3.0f, 0.0f}, /* row3 = 2*r0+3*r1 → det 0 */
    };
    REQUIRE_FALSE(navicore_mat_invert3x3(singular, inv));
}

TEST_CASE("mat_invert null args → false", "[ins_ekf_math][matrix]")
{
    float inv[2][2]{};
    const float s[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
    REQUIRE_FALSE(navicore_mat_invert2x2(nullptr, inv));
    REQUIRE_FALSE(navicore_mat_invert2x2(s, nullptr));
}
