/**
 * @file imu_cross_check.hpp
 * @brief Cheap dual-IMU "vigilante": compare secondary vs primary, flag only.
 *
 * Does not replace the primary IMU in the EKF — only detects disagreement
 * (faulty primary, bad mount, cable noise) without avionics-grade TMR cost.
 */
#pragma once

#include "sensor_types.hpp"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef IMU_CROSS_ACCEL_MAX_DELTA_MPS2
#define IMU_CROSS_ACCEL_MAX_DELTA_MPS2 3.0f /* ~0.3 g */
#endif
#ifndef IMU_CROSS_GYRO_MAX_DELTA_RADPS
#define IMU_CROSS_GYRO_MAX_DELTA_RADPS 0.50f /* ~28.6 deg/s */
#endif

typedef struct {
    float accel_delta_mps2; /* max |Δ| over axes */
    float gyro_delta_radps;
    bool disagree;
    bool secondary_missing;
} ImuCrossCheckResult;

/**
 * Compare primary (EKF) vs secondary (vigilante). If secondary is NULL or
 * !valid → secondary_missing (no false positive disagreement).
 */
ImuCrossCheckResult imu_cross_check_evaluate(
    const ImuSample *primary,
    const ImuSample *secondary);

#ifdef __cplusplus
}
#endif
