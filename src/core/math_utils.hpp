#ifndef NAVICORE_MATH_UTILS_HPP
#define NAVICORE_MATH_UTILS_HPP

/*
 * Umbrales para saltar sqrt/trig cuando el vehiculo esta detenido.
 * Pensado para FPU simple en ARM Cortex-M (sqrtf/sinf/cosf).
 */
#define NAVICORE_EPS_SPEED_MPS      0.01f
#define NAVICORE_EPS_SPEED_SQ       (NAVICORE_EPS_SPEED_MPS * NAVICORE_EPS_SPEED_MPS)
#define NAVICORE_EPS_DISPLACEMENT_M 0.001f
#define NAVICORE_EPS_GYRO_RADPS     1.0e-4f
#define NAVICORE_EPS_ACCEL_MPS2     0.05f

#endif /* NAVICORE_MATH_UTILS_HPP */
