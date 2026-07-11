/**
 * @file slip_compensation.hpp
 * @brief Deteccion de patinaje y escalado exponencial de ruido odométrico (zero-heap)
 */
#pragma once

#include <stdint.h>

#include "diagnostic.hpp"
#include "fusion.hpp"

/** Ganancia exponencial del ruido de odometria por unidad de exceso de slip ratio. */
#define SLIP_COMP_EXP_NOISE_GAIN 10.0f

/** Escala maxima del ruido odométrico (evita colapso numerico). */
#define SLIP_COMP_MAX_NOISE_SCALE 64.0f

/**
 * @brief Evalua patinaje comparando velocidad IMU vs ruedas.
 *
 * Calcula slip_ratio = |v_imu - v_wheel| / max(|v_imu|, |v_wheel|).
 * Si slip_ratio > NAVICORE_SLIP_RATIO_THRESHOLD (0.15f):
 *   - Activa filter->slip_fault_active
 *   - Penaliza health_score del monitor (-SLIP_COMP_HEALTH_PENALTY)
 *   - Escala odom_noise_covariance_scale = exp(exceso * SLIP_COMP_EXP_NOISE_GAIN)
 *
 * @param filter           Filtro de navegacion (velocidad IMU predicha + matrices).
 * @param wheel_speed_mps  Velocidad de las ruedas [m/s] (puede ser negativa en reverse).
 * @param monitor          Monitor de salud; puede ser NULL.
 */
void slip_compensation_evaluate(
    DeadReckoningFilter *filter,
    float wheel_speed_mps,
    SystemHealthMonitor *monitor);

/**
 * @brief Devuelve el slip ratio almacenado en el filtro tras la ultima evaluacion.
 */
float slip_compensation_get_last_ratio(const DeadReckoningFilter *filter);
