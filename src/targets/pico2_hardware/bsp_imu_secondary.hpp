/**
 * @file bsp_imu_secondary.hpp
 * @brief Optional cheap vigilante IMU (MPU-6050 class) on I2C0.
 *
 * Not fused into the EKF — only compared against the primary WT61C via
 * `imu_cross_check_evaluate`. Enable with PICO2_SECONDARY_IMU_ENABLE=1.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/sensor_types.hpp"

bool pico2_bsp_imu_secondary_init(void);
/** Returns false if disabled, absent, or I2C fault (not an error for the EKF). */
bool pico2_bsp_imu_secondary_poll(ImuSample *imu_out);
bool pico2_bsp_imu_secondary_present(void);
