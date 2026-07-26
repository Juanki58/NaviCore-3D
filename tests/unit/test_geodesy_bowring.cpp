#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>

#include "geodesy.hpp"

using Catch::Matchers::WithinAbs;

namespace {

constexpr float kBarcelonaLatDeg = 41.3874f;
constexpr float kBarcelonaLonDeg = 2.1686f;
constexpr float kBarcelonaAltM = 12.0f;

/** Horizontal + vertical residual of LLA roundtrip, in metres at `ref`. */
void lla_roundtrip_ned_error_m(
    const geodesy::LLA &orig,
    float *out_err_n_m,
    float *out_err_e_m,
    float *out_err_d_m)
{
    const geodesy::ECEF ecef = geodesy::lla_to_ecef(orig);
    const geodesy::LLA back = geodesy::ecef_to_lla(ecef);
    const geodesy::NED err = geodesy::lla_to_ned(back, orig);
    if (out_err_n_m != nullptr) {
        *out_err_n_m = err.north_m;
    }
    if (out_err_e_m != nullptr) {
        *out_err_e_m = err.east_m;
    }
    if (out_err_d_m != nullptr) {
        *out_err_d_m = err.down_m;
    }
}

void require_lla_roundtrip_mm(const geodesy::LLA &orig)
{
    float en = 0.0f;
    float ee = 0.0f;
    float ed = 0.0f;
    lla_roundtrip_ned_error_m(orig, &en, &ee, &ed);
    /* Pre-fix Bowring bias was ~30 m; demand millimetre-class closure. */
    REQUIRE_THAT(en, WithinAbs(0.0f, 1.0e-3f));
    REQUIRE_THAT(ee, WithinAbs(0.0f, 1.0e-3f));
    REQUIRE_THAT(ed, WithinAbs(0.0f, 1.0e-3f));
}

} /* namespace */

TEST_CASE("ecef_to_lla Bowring: Barcelona LLA->ECEF->LLA closes to mm", "[geodesy][bowring]")
{
    const geodesy::LLA barcelona =
        geodesy::lla(kBarcelonaLatDeg, kBarcelonaLonDeg, kBarcelonaAltM);
    require_lla_roundtrip_mm(barcelona);

    const geodesy::LLA back = geodesy::ecef_to_lla(geodesy::lla_to_ecef(barcelona));
    REQUIRE_THAT(back.lat_deg, WithinAbs(kBarcelonaLatDeg, 1.0e-7f));
    REQUIRE_THAT(back.lon_deg, WithinAbs(kBarcelonaLonDeg, 1.0e-7f));
    REQUIRE_THAT(back.alt_m, WithinAbs(kBarcelonaAltM, 1.0e-3f));
}

TEST_CASE("ecef_to_lla Bowring: ned_to_lla(ref,0,0,0) returns ref", "[geodesy][bowring]")
{
    const geodesy::LLA ref =
        geodesy::lla(kBarcelonaLatDeg, kBarcelonaLonDeg, kBarcelonaAltM);
    const geodesy::NED zero{0.0f, 0.0f, 0.0f};
    const geodesy::LLA back = geodesy::ned_to_lla(zero, ref);

    REQUIRE_THAT(back.lat_deg, WithinAbs(ref.lat_deg, 1.0e-7f));
    REQUIRE_THAT(back.lon_deg, WithinAbs(ref.lon_deg, 1.0e-7f));
    REQUIRE_THAT(back.alt_m, WithinAbs(ref.alt_m, 1.0e-3f));

    /* Same check in local metres — pre-fix was ~30 m away. */
    const geodesy::NED err = geodesy::lla_to_ned(back, ref);
    REQUIRE_THAT(err.north_m, WithinAbs(0.0f, 1.0e-3f));
    REQUIRE_THAT(err.east_m, WithinAbs(0.0f, 1.0e-3f));
    REQUIRE_THAT(err.down_m, WithinAbs(0.0f, 1.0e-3f));
}

TEST_CASE("ecef_to_lla Bowring: roundtrip outside Barcelona latitude", "[geodesy][bowring]")
{
    /* Near equator (Quito-ish). */
    require_lla_roundtrip_mm(geodesy::lla(-0.1807f, -78.4678f, 2850.0f));
    /* Southern hemisphere (Sydney-ish). */
    require_lla_roundtrip_mm(geodesy::lla(-33.8688f, 151.2093f, 20.0f));
    /* High latitude. */
    require_lla_roundtrip_mm(geodesy::lla(64.1466f, -21.9426f, 50.0f));
}
