/**
 * @file ins_ekf_math.hpp
 * @brief Pure math kernels used by the ESKF — isolated for unit tests (no Sim/orchestration).
 *
 * Covers: quaternion normalize (zero-norm safe), 2x2/3x3 invert (singular → false).
 */
#ifndef NAVICORE_INS_EKF_MATH_HPP
#define NAVICORE_INS_EKF_MATH_HPP

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Det |S| below this → treat as singular (no 1/det). */
#ifndef NAVICORE_MAT_SINGULAR_DET_EPS
#define NAVICORE_MAT_SINGULAR_DET_EPS 1.0e-12f
#endif

/** ||q||^2 below this → reset to identity quaternion (avoid /0). */
#ifndef NAVICORE_QUAT_NORM_EPS_SQ
#define NAVICORE_QUAT_NORM_EPS_SQ 1.0e-12f
#endif

/**
 * Normalize Hamilton quaternion [w,x,y,z] in place.
 * If near-zero norm: sets identity (1,0,0,0) — no division by zero.
 */
void navicore_quat_normalize(float q[4]);

/** Invert 2×2; returns false if |det| <= NAVICORE_MAT_SINGULAR_DET_EPS. */
bool navicore_mat_invert2x2(const float s[2][2], float inv_out[2][2]);

/** Invert 3×3; returns false if |det| <= NAVICORE_MAT_SINGULAR_DET_EPS. */
bool navicore_mat_invert3x3(const float s[3][3], float inv_out[3][3]);

#ifdef __cplusplus
}
#endif

#endif /* NAVICORE_INS_EKF_MATH_HPP */
