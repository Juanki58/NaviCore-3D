#include "imu_cross_check.hpp"

#include <math.h>

static float imu_cross_fabsf(float v)
{
    return (v < 0.0f) ? -v : v;
}

ImuCrossCheckResult imu_cross_check_evaluate(
    const ImuSample *primary,
    const ImuSample *secondary)
{
    ImuCrossCheckResult r{};
    r.secondary_missing = true;

    if (primary == nullptr || !primary->valid) {
        return r;
    }
    if (secondary == nullptr || !secondary->valid) {
        return r;
    }

    r.secondary_missing = false;
    float max_a = 0.0f;
    float max_g = 0.0f;
    for (int i = 0; i < 3; ++i) {
        const float da = imu_cross_fabsf(primary->accel_mps2[i] - secondary->accel_mps2[i]);
        const float dg = imu_cross_fabsf(primary->gyro_radps[i] - secondary->gyro_radps[i]);
        if (da > max_a) {
            max_a = da;
        }
        if (dg > max_g) {
            max_g = dg;
        }
    }
    r.accel_delta_mps2 = max_a;
    r.gyro_delta_radps = max_g;
    r.disagree = (max_a > IMU_CROSS_ACCEL_MAX_DELTA_MPS2)
        || (max_g > IMU_CROSS_GYRO_MAX_DELTA_RADPS);
    return r;
}
