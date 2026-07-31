/**
 * @file technique_arbiter.hpp
 * @brief Multi-technique viability scores + hysteresis (N-way, not GPS-vs-IMU only).
 *
 * CRITICAL SCOPE
 * --------------
 * This module scores *aiding / modality techniques* (GNSS, IMU coast, camera, …).
 * It does **not** replace `nav_mode_select` / NavMode (GPS / HYBRID / DR).
 * NavMode is the product façade over the ESKF; HYBRID means GNSS+INS fused —
 * that is not the same as "picking one technique as winner".
 *
 * Until wired deliberately into the EKF export path, this arbiter is a
 * testable building block for field/off-road matrices. Stubs return 0 viability
 * for modalities that are not fused yet (camera, LiDAR, acoustic, …).
 *
 * TinyML / STEMMA: feed EnvContext labels later; keep this arbiter rule-based.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Application domain for the applicability matrix (independent of NavDomain ABI). */
typedef enum {
    ARB_DOMAIN_AIR = 0,
    ARB_DOMAIN_ROAD = 1,
    ARB_DOMAIN_FIELD = 2,
    ARB_DOMAIN_MARITIME = 3,
    ARB_DOMAIN_MARITIME_SUBMERGED = 4
} ArbiterDomain;

typedef enum {
    NAV_TECH_NONE = 0,
    NAV_TECH_GNSS = 1,
    NAV_TECH_IMU_DR = 2,
    NAV_TECH_CAMERA = 3,
    NAV_TECH_LIDAR = 4,
    NAV_TECH_ACOUSTIC = 5,
    NAV_TECH_MAG = 6,
    NAV_TECH_BARO = 7,
    NAV_TECH_COUNT = 8
} NavTechnique;

typedef struct {
    NavTechnique id;
    float viability;         /* 0..1 this cycle */
    float confidence_trend;  /* -1..+1 degrading..improving; 0 unknown */
} TechniqueScore;

/**
 * Environment / mission facts for scoring. Unknown optional sensors: leave
 * humidity/light/temp at NAV_ARB_SENSOR_UNKNOWN so they do not penalize.
 */
#ifndef NAV_ARB_SENSOR_UNKNOWN
#define NAV_ARB_SENSOR_UNKNOWN (-1.0f)
#endif

typedef struct {
    ArbiterDomain domain;
    bool gnss_fix_valid;
    bool gnss_accepted_recent;
    bool gnss_outlier;
    bool imu_ok;
    float humidity_pct;   /* NAV_ARB_SENSOR_UNKNOWN or 0..100 */
    float light_lux;      /* NAV_ARB_SENSOR_UNKNOWN or >= 0 */
    float temperature_c;  /* NAV_ARB_SENSOR_UNKNOWN or deg C */
} ArbiterEnvContext;

/** Enter must be strictly greater than exit to create a deadband. */
#ifndef NAV_ARB_ENTER_THRESHOLD
#define NAV_ARB_ENTER_THRESHOLD 0.55f
#endif
#ifndef NAV_ARB_EXIT_THRESHOLD
#define NAV_ARB_EXIT_THRESHOLD 0.40f
#endif

typedef struct {
    NavTechnique selected;
    NavTechnique best;
    bool switched;
    const char *reason; /* static string */
} ArbiterDecision;

const char *nav_technique_name(NavTechnique tech);
const char *arbiter_domain_name(ArbiterDomain domain);

/** Fill scores[0..NAV_TECH_COUNT-1]; scores[i].id == (NavTechnique)i. */
void technique_score_all(const ArbiterEnvContext *ctx, TechniqueScore scores[NAV_TECH_COUNT]);

/**
 * Hysteresis: switch only if best != current AND best > enter AND current < exit.
 * If current is NONE, adopt best when best >= enter (bootstrap).
 */
ArbiterDecision technique_arbitrate(const TechniqueScore *scores,
                                    uint32_t count,
                                    NavTechnique current,
                                    float enter_threshold,
                                    float exit_threshold);

/** Convenience: score_all + arbitrate with default thresholds. */
ArbiterDecision technique_arbitrate_from_env(const ArbiterEnvContext *ctx, NavTechnique current);

#ifdef __cplusplus
}
#endif
