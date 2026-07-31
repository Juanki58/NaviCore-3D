#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "technique_arbiter.hpp"

#include <string>

static ArbiterEnvContext make_field_ctx()
{
    ArbiterEnvContext ctx{};
    ctx.domain = ARB_DOMAIN_FIELD;
    ctx.gnss_fix_valid = true;
    ctx.gnss_accepted_recent = true;
    ctx.gnss_outlier = false;
    ctx.imu_ok = true;
    ctx.humidity_pct = NAV_ARB_SENSOR_UNKNOWN;
    ctx.light_lux = NAV_ARB_SENSOR_UNKNOWN;
    ctx.temperature_c = NAV_ARB_SENSOR_UNKNOWN;
    return ctx;
}

TEST_CASE("arbiter: field + fresh GNSS prefers GNSS", "[arbiter]")
{
    ArbiterEnvContext ctx = make_field_ctx();
    TechniqueScore scores[NAV_TECH_COUNT];
    technique_score_all(&ctx, scores);

    REQUIRE(scores[NAV_TECH_GNSS].viability > scores[NAV_TECH_IMU_DR].viability);
    REQUIRE(scores[NAV_TECH_CAMERA].viability == Catch::Approx(0.0f));
    REQUIRE(scores[NAV_TECH_ACOUSTIC].viability == Catch::Approx(0.0f));

    const ArbiterDecision d = technique_arbitrate_from_env(&ctx, NAV_TECH_NONE);
    REQUIRE(d.selected == NAV_TECH_GNSS);
    REQUIRE(d.switched);
    REQUIRE(std::string(d.reason) == "bootstrap");
}

TEST_CASE("arbiter: GNSS outlier elevates IMU_DR", "[arbiter]")
{
    ArbiterEnvContext ctx = make_field_ctx();
    ctx.gnss_outlier = true;
    TechniqueScore scores[NAV_TECH_COUNT];
    technique_score_all(&ctx, scores);

    REQUIRE(scores[NAV_TECH_GNSS].viability < 0.1f);
    REQUIRE(scores[NAV_TECH_IMU_DR].viability > scores[NAV_TECH_GNSS].viability);

    const ArbiterDecision d = technique_arbitrate_from_env(&ctx, NAV_TECH_GNSS);
    REQUIRE(d.best == NAV_TECH_IMU_DR);
    REQUIRE(d.selected == NAV_TECH_IMU_DR);
    REQUIRE(d.switched);
    REQUIRE(std::string(d.reason) == "hysteresis_switch");
}

TEST_CASE("arbiter: hysteresis prevents flicker near thresholds", "[arbiter]")
{
    TechniqueScore scores[NAV_TECH_COUNT]{};
    for (uint32_t i = 0; i < (uint32_t)NAV_TECH_COUNT; ++i) {
        scores[i].id = (NavTechnique)i;
        scores[i].viability = 0.0f;
    }
    /* Current GNSS still above EXIT; IMU slightly better but not decisive. */
    scores[NAV_TECH_GNSS].viability = 0.45f;   /* > EXIT 0.40 */
    scores[NAV_TECH_IMU_DR].viability = 0.60f; /* > ENTER 0.55 */

    const ArbiterDecision hold =
        technique_arbitrate(scores, (uint32_t)NAV_TECH_COUNT, NAV_TECH_GNSS, 0.55f, 0.40f);
    REQUIRE(hold.best == NAV_TECH_IMU_DR);
    REQUIRE(hold.selected == NAV_TECH_GNSS);
    REQUIRE_FALSE(hold.switched);
    REQUIRE(std::string(hold.reason) == "hysteresis_hold");

    /* GNSS collapses below EXIT → switch allowed. */
    scores[NAV_TECH_GNSS].viability = 0.35f;
    const ArbiterDecision sw =
        technique_arbitrate(scores, (uint32_t)NAV_TECH_COUNT, NAV_TECH_GNSS, 0.55f, 0.40f);
    REQUIRE(sw.selected == NAV_TECH_IMU_DR);
    REQUIRE(sw.switched);
    REQUIRE(std::string(sw.reason) == "hysteresis_switch");
}

TEST_CASE("arbiter: submerged GNSS prior is zero", "[arbiter]")
{
    ArbiterEnvContext ctx{};
    ctx.domain = ARB_DOMAIN_MARITIME_SUBMERGED;
    ctx.imu_ok = true;
    ctx.gnss_fix_valid = true;
    ctx.gnss_accepted_recent = true;
    ctx.humidity_pct = NAV_ARB_SENSOR_UNKNOWN;
    ctx.light_lux = NAV_ARB_SENSOR_UNKNOWN;
    ctx.temperature_c = NAV_ARB_SENSOR_UNKNOWN;

    TechniqueScore scores[NAV_TECH_COUNT];
    technique_score_all(&ctx, scores);
    REQUIRE(scores[NAV_TECH_GNSS].viability == Catch::Approx(0.0f));
    REQUIRE(scores[NAV_TECH_IMU_DR].viability > 0.0f);
    /* Acoustic stub remains 0 until a real DVL path exists. */
    REQUIRE(scores[NAV_TECH_ACOUSTIC].viability == Catch::Approx(0.0f));
}

TEST_CASE("arbiter: rejects inverted thresholds", "[arbiter]")
{
    TechniqueScore scores[2]{};
    scores[0].id = NAV_TECH_GNSS;
    scores[0].viability = 0.9f;
    scores[1].id = NAV_TECH_IMU_DR;
    scores[1].viability = 0.5f;
    const ArbiterDecision d = technique_arbitrate(scores, 2U, NAV_TECH_NONE, 0.40f, 0.55f);
    REQUIRE(std::string(d.reason) == "bad_thresholds");
}
