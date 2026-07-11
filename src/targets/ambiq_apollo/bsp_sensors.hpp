/**
 * @file bsp_sensors.hpp
 * @brief Abstraccion de Hardware (HAL/BSP) para NaviCore-Ambiq
 * @note Define la interfaz de lectura directa por registros e interrupciones
 */
#pragma once

#include <stdint.h>

#include "../../core/fusion.hpp"

namespace NaviCore {

using IMUMeasurement = ImuSample;
using GNSSMeasurement = GpsSample;
using NavState = ::NavState;

} /* namespace NaviCore */

struct PowerMetrics {
    float voltage_core_mv;
    float current_draw_ua;
    uint32_t active_cycles;
};

/**
 * @brief Lee los datos del giroscopio y acelerometro mediante rafaga SPI/DMA
 */
void Ambiq_BSP_ReadIMU(NaviCore::IMUMeasurement *imu_out);

/**
 * @brief Lee los datos del receptor GNSS si la linea de interrupcion esta en alta
 */
void Ambiq_BSP_ReadGNSS(NaviCore::GNSSMeasurement *gnss_out);

/**
 * @brief Transmite el estado actual a traves del bus serie o telemetria dedicada
 */
void Ambiq_BSP_TransmitState(const NaviCore::NavState *state_in);

/**
 * @brief Registra y expone las metricas de consumo del ultimo tick de ejecucion
 */
void Ambiq_BSP_GetPowerMetrics(PowerMetrics *metrics_out);
