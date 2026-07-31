#include "technique_arbiter.hpp"

#include <stddef.h>

const char *nav_technique_name(NavTechnique tech)
{
    switch (tech) {
    case NAV_TECH_NONE:
        return "NONE";
    case NAV_TECH_GNSS:
        return "GNSS";
    case NAV_TECH_IMU_DR:
        return "IMU_DR";
    case NAV_TECH_CAMERA:
        return "CAMERA";
    case NAV_TECH_LIDAR:
        return "LIDAR";
    case NAV_TECH_ACOUSTIC:
        return "ACOUSTIC";
    case NAV_TECH_MAG:
        return "MAG";
    case NAV_TECH_BARO:
        return "BARO";
    default:
        return "UNKNOWN";
    }
}

const char *arbiter_domain_name(ArbiterDomain domain)
{
    switch (domain) {
    case ARB_DOMAIN_AIR:
        return "AIR";
    case ARB_DOMAIN_ROAD:
        return "ROAD";
    case ARB_DOMAIN_FIELD:
        return "FIELD";
    case ARB_DOMAIN_MARITIME:
        return "MARITIME";
    case ARB_DOMAIN_MARITIME_SUBMERGED:
        return "MARITIME_SUBMERGED";
    default:
        return "UNKNOWN";
    }
}

static float arb_clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

/** Base applicability prior by domain (engineering priors, not field-fitted). */
static float base_viability(NavTechnique tech, ArbiterDomain domain)
{
    switch (tech) {
    case NAV_TECH_GNSS:
        switch (domain) {
        case ARB_DOMAIN_AIR:
        case ARB_DOMAIN_ROAD:
            return 0.95f;
        case ARB_DOMAIN_FIELD:
            return 0.70f; /* canopy / shed — degraded prior */
        case ARB_DOMAIN_MARITIME:
            return 0.85f;
        case ARB_DOMAIN_MARITIME_SUBMERGED:
            return 0.0f;
        default:
            return 0.0f;
        }
    case NAV_TECH_IMU_DR:
        /* Always a short-horizon backup; submerged still usable briefly. */
        return 0.75f;
    case NAV_TECH_CAMERA:
        switch (domain) {
        case ARB_DOMAIN_AIR:
        case ARB_DOMAIN_ROAD:
        case ARB_DOMAIN_FIELD:
            return 0.0f; /* not fused yet — stub */
        case ARB_DOMAIN_MARITIME:
        case ARB_DOMAIN_MARITIME_SUBMERGED:
            return 0.0f;
        default:
            return 0.0f;
        }
    case NAV_TECH_LIDAR:
        return 0.0f; /* stub */
    case NAV_TECH_ACOUSTIC:
        return (domain == ARB_DOMAIN_MARITIME_SUBMERGED) ? 0.0f /* stub until DVL path */
                                                         : 0.0f;
    case NAV_TECH_MAG:
        return 0.15f; /* weak heading prior only; not fused as primary */
    case NAV_TECH_BARO:
        switch (domain) {
        case ARB_DOMAIN_AIR:
            return 0.20f; /* altitude aid potential; not ESKF update yet */
        case ARB_DOMAIN_MARITIME:
        case ARB_DOMAIN_MARITIME_SUBMERGED:
            return 0.25f; /* depth / pressure potential */
        default:
            return 0.0f;
        }
    case NAV_TECH_NONE:
    default:
        return 0.0f;
    }
}

static float score_gnss(const ArbiterEnvContext *ctx)
{
    float score = base_viability(NAV_TECH_GNSS, ctx->domain);
    if (score <= 0.0f) {
        return 0.0f;
    }
    if (ctx->gnss_outlier) {
        return 0.05f;
    }
    if (!ctx->gnss_fix_valid) {
        return 0.0f;
    }
    if (!ctx->gnss_accepted_recent) {
        score *= 0.55f; /* stale accept — still present but weak */
    }
    return arb_clampf(score, 0.0f, 1.0f);
}

static float score_imu_dr(const ArbiterEnvContext *ctx)
{
    float score = base_viability(NAV_TECH_IMU_DR, ctx->domain);
    if (!ctx->imu_ok) {
        return 0.0f;
    }
    /* IMU becomes relatively more attractive when GNSS is bad. */
    if (ctx->gnss_outlier || !ctx->gnss_fix_valid) {
        score = arb_clampf(score + 0.15f, 0.0f, 1.0f);
    } else if (ctx->gnss_accepted_recent) {
        score *= 0.85f; /* still viable backup, not preferred */
    }
    return arb_clampf(score, 0.0f, 1.0f);
}

static float env_optical_penalty(const ArbiterEnvContext *ctx)
{
    float m = 1.0f;
    if (ctx->temperature_c > 60.0f && ctx->temperature_c != NAV_ARB_SENSOR_UNKNOWN) {
        m *= 0.30f;
    }
    if (ctx->humidity_pct >= 0.0f && ctx->humidity_pct > 90.0f) {
        m *= 0.20f;
    }
    if (ctx->light_lux >= 0.0f && ctx->light_lux < 10.0f) {
        m *= 0.40f;
    }
    return m;
}

static float score_camera(const ArbiterEnvContext *ctx)
{
    float score = base_viability(NAV_TECH_CAMERA, ctx->domain);
    if (score <= 0.0f) {
        return 0.0f;
    }
    return arb_clampf(score * env_optical_penalty(ctx), 0.0f, 1.0f);
}

static float score_lidar(const ArbiterEnvContext *ctx)
{
    float score = base_viability(NAV_TECH_LIDAR, ctx->domain);
    if (score <= 0.0f) {
        return 0.0f;
    }
    if (ctx->humidity_pct >= 0.0f && ctx->humidity_pct > 95.0f) {
        score *= 0.30f; /* fog / heavy precip prior */
    }
    return arb_clampf(score, 0.0f, 1.0f);
}

static float score_acoustic(const ArbiterEnvContext *ctx)
{
    /* Stub: DVL/sonar not fused. Prior would be high when submerged. */
    (void)ctx;
    return 0.0f;
}

static float score_mag(const ArbiterEnvContext *ctx)
{
    return base_viability(NAV_TECH_MAG, ctx->domain);
}

static float score_baro(const ArbiterEnvContext *ctx)
{
    return base_viability(NAV_TECH_BARO, ctx->domain);
}

static float trend_gnss(const ArbiterEnvContext *ctx)
{
    if (ctx->gnss_outlier) {
        return -1.0f;
    }
    if (ctx->gnss_fix_valid && ctx->gnss_accepted_recent) {
        return 0.5f;
    }
    if (ctx->gnss_fix_valid) {
        return -0.25f;
    }
    return -0.75f;
}

void technique_score_all(const ArbiterEnvContext *ctx, TechniqueScore scores[NAV_TECH_COUNT])
{
    for (uint32_t i = 0U; i < (uint32_t)NAV_TECH_COUNT; ++i) {
        scores[i].id = (NavTechnique)i;
        scores[i].viability = 0.0f;
        scores[i].confidence_trend = 0.0f;
    }
    if (ctx == nullptr) {
        return;
    }

    scores[NAV_TECH_NONE].viability = 0.0f;

    scores[NAV_TECH_GNSS].viability = score_gnss(ctx);
    scores[NAV_TECH_GNSS].confidence_trend = trend_gnss(ctx);

    scores[NAV_TECH_IMU_DR].viability = score_imu_dr(ctx);
    scores[NAV_TECH_IMU_DR].confidence_trend = ctx->imu_ok ? 0.0f : -1.0f;

    scores[NAV_TECH_CAMERA].viability = score_camera(ctx);
    scores[NAV_TECH_LIDAR].viability = score_lidar(ctx);
    scores[NAV_TECH_ACOUSTIC].viability = score_acoustic(ctx);
    scores[NAV_TECH_MAG].viability = score_mag(ctx);
    scores[NAV_TECH_BARO].viability = score_baro(ctx);
}

static const TechniqueScore *find_score(const TechniqueScore *scores, uint32_t count, NavTechnique id)
{
    for (uint32_t i = 0U; i < count; ++i) {
        if (scores[i].id == id) {
            return &scores[i];
        }
    }
    return nullptr;
}

ArbiterDecision technique_arbitrate(const TechniqueScore *scores,
                                    uint32_t count,
                                    NavTechnique current,
                                    float enter_threshold,
                                    float exit_threshold)
{
    ArbiterDecision out{};
    out.selected = current;
    out.best = NAV_TECH_NONE;
    out.switched = false;
    out.reason = "null_scores";

    if (scores == nullptr || count == 0U) {
        return out;
    }
    if (!(enter_threshold > exit_threshold)) {
        out.reason = "bad_thresholds";
        return out;
    }

    float best_v = -1.0f;
    NavTechnique best = NAV_TECH_NONE;
    for (uint32_t i = 0U; i < count; ++i) {
        if (scores[i].id == NAV_TECH_NONE) {
            continue;
        }
        if (scores[i].viability > best_v) {
            best_v = scores[i].viability;
            best = scores[i].id;
        }
    }
    out.best = best;

    if (best == NAV_TECH_NONE || best_v < 0.0f) {
        out.reason = "no_candidate";
        return out;
    }

    if (current == NAV_TECH_NONE) {
        if (best_v >= enter_threshold) {
            out.selected = best;
            out.switched = true;
            out.reason = "bootstrap";
        } else {
            out.reason = "bootstrap_below_enter";
        }
        return out;
    }

    if (best == current) {
        out.selected = current;
        out.reason = "hold_best";
        return out;
    }

    const TechniqueScore *cur = find_score(scores, count, current);
    const float cur_v = (cur != nullptr) ? cur->viability : 0.0f;

    if (best_v > enter_threshold && cur_v < exit_threshold) {
        out.selected = best;
        out.switched = true;
        out.reason = "hysteresis_switch";
        return out;
    }

    out.selected = current;
    out.reason = "hysteresis_hold";
    return out;
}

ArbiterDecision technique_arbitrate_from_env(const ArbiterEnvContext *ctx, NavTechnique current)
{
    TechniqueScore scores[NAV_TECH_COUNT];
    technique_score_all(ctx, scores);
    return technique_arbitrate(scores, (uint32_t)NAV_TECH_COUNT, current, NAV_ARB_ENTER_THRESHOLD,
                               NAV_ARB_EXIT_THRESHOLD);
}
