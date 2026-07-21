#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "fusion.hpp"
#include "sensor_types.hpp"

#include <cmath>

using Catch::Matchers::WithinAbs;

TEST_CASE("dead_reckoning_init sets INITIALIZING NavState", "[fusion]")
{
    DeadReckoningFilter f{};
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    dead_reckoning_init(&f, origin, NAVICORE_DOMAIN_AIR);
    REQUIRE(f.state.mode == NAV_MODE_INITIALIZING);
    REQUIRE(f.state.domain == NAVICORE_DOMAIN_AIR);
    REQUIRE_THAT(f.state.position.x, WithinAbs(origin.x, 1e-5f));
}

TEST_CASE("dead_reckoning_update_imu rejects null / invalid / NaN", "[fusion][edge]")
{
    DeadReckoningFilter f{};
    dead_reckoning_init(&f, vector3d_make(0.0f, 0.0f, 0.0f), NAVICORE_DOMAIN_AIR);

    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = 10U;
    imu.accel_mps2[2] = 9.81f;

    REQUIRE_FALSE(dead_reckoning_update_imu(nullptr, &imu, nullptr));
    REQUIRE_FALSE(dead_reckoning_update_imu(&f, nullptr, nullptr));

    imu.valid = false;
    REQUIRE_FALSE(dead_reckoning_update_imu(&f, &imu, nullptr));

    imu.valid = true;
    imu.accel_mps2[0] = NAN;
    REQUIRE_FALSE(dead_reckoning_update_imu(&f, &imu, nullptr));

    imu.accel_mps2[0] = 0.0f;
    imu.gyro_radps[1] = INFINITY;
    REQUIRE_FALSE(dead_reckoning_update_imu(&f, &imu, nullptr));
}

TEST_CASE("dead_reckoning_update_imu accepts finite stationary sample", "[fusion]")
{
    DeadReckoningFilter f{};
    dead_reckoning_init(&f, vector3d_make(41.0f, 2.0f, 10.0f), NAVICORE_DOMAIN_AIR);

    ImuSample imu{};
    imu.valid = true;
    imu.timestamp_ms = 0U;
    imu.accel_mps2[2] = NAVICORE_GRAVITY_MPS2;
    REQUIRE(dead_reckoning_update_imu(&f, &imu, nullptr));

    imu.timestamp_ms = 100U;
    REQUIRE(dead_reckoning_update_imu(&f, &imu, nullptr));
}
