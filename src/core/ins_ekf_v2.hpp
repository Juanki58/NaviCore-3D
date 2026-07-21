#pragma once

#include "ins_ekf.hpp"

#include <stdint.h>

/*
 * EKF v2 — núcleos reescritos (predict, GNSS pos/vel, actitud/seed, NHC condicional).
 * Opera sobre el mismo InsEkfFilter (auditorías / lab intactos). v1 no se modifica.
 */

enum InsEkfCoreVersion : uint8_t {
    INS_EKF_CORE_V1 = 0,
    INS_EKF_CORE_V2 = 1,
};

#ifndef NAVICORE_INS_EKF_V2_NHC_GNSS_GAP_S
/** Doc 17: NHC solo si gap GNSS (desde último accept) supera esta gracia [s]. */
#define NAVICORE_INS_EKF_V2_NHC_GNSS_GAP_S 1.0f
#endif

#ifndef NAVICORE_INS_EKF_V2_POLISH_NHC_GAP_S
/** Polish: NHC entre fixes GNSS (~1 Hz) sin ALWAYS agresivo. */
#define NAVICORE_INS_EKF_V2_POLISH_NHC_GAP_S 0.25f
#endif

#ifndef NAVICORE_INS_EKF_V2_POLISH_VEL_MIN_MPS
#define NAVICORE_INS_EKF_V2_POLISH_VEL_MIN_MPS 3.0f
#endif

#ifndef NAVICORE_INS_EKF_V2_POLISH_COURSE_YAW_MAX_DEG
#define NAVICORE_INS_EKF_V2_POLISH_COURSE_YAW_MAX_DEG 35.0f
#endif

#ifndef NAVICORE_INS_EKF_V2_POLISH_VEL_R_SCALE
/** Inflar R_vel del teléfono (más tolerancia → más accepts coherentes). */
#define NAVICORE_INS_EKF_V2_POLISH_VEL_R_SCALE 2.25f
#endif

#ifndef NAVICORE_INS_EKF_V2_POLISH_POS_INNOV_SOFT_M
/** Por encima de este innov_h, inflar R_pos (update suave, no abandono). */
#define NAVICORE_INS_EKF_V2_POLISH_POS_INNOV_SOFT_M 25.0f
#endif

/**
 * Posición: umbral NIS 3-DoF laxo (v2 baseline sin polish).
 * Velocidad sigue en χ² 2-DoF @ 99%.
 */
#ifndef NAVICORE_INS_EKF_V2_NIS_POS_THRESHOLD
#define NAVICORE_INS_EKF_V2_NIS_POS_THRESHOLD 50.0f
#endif

/** Predict cinemático v2: R·(f−b)−g, integración trapezoidal p, presupuesto Δv, sin NHC. */
bool ins_ekf_v2_predict(InsEkfFilter *filter, const ImuSample *imu);

/**
 * GNSS v2: pos y vel separados. Baseline: pos siempre. Polish: R adaptativo + vel coherente.
 */
bool ins_ekf_v2_update_gnss(InsEkfFilter *filter, const GpsSample *gps);

bool ins_ekf_v2_seed_from_gnss(
    InsEkfFilter *filter,
    const GpsSample *gps,
    NavDomain domain);

bool ins_ekf_v2_maybe_update_nhc(InsEkfFilter *filter, uint32_t now_imu_ms);

void ins_ekf_v2_set_polish(InsEkfFilter *filter, bool enabled);
bool ins_ekf_v2_polish_enabled(const InsEkfFilter *filter);

const char *ins_ekf_core_version_name(InsEkfCoreVersion version);
bool ins_ekf_core_version_parse(const char *text, InsEkfCoreVersion *out_version);
