/**
 * @file api_ingest.hpp
 * @brief API de Ingestión Universal para NaviCore-3D
 * @note Permite acoplar el núcleo matemático a cualquier vehículo de forma inmediata
 */
#pragma once

#include <stdint.h>

#include "NavState.h"
#include "sensor_types.hpp"

namespace NaviCore {

using ImuSample = ::ImuSample;
using GpsSample = ::GpsSample;
using NavState = ::NavState;

/**
 * @brief Vista del estado de navegación para HMI / bus del vehículo
 * @note pos_x = latitud [°], pos_y = longitud [°]
 */
struct VehicleNavOutput {
    float pos_x;
    float pos_y;
    float pos_z;
    float heading_deg;
    float quality;
};

/**
 * @brief Estructura unificada para vehículos terrestres (Coches/Rovers)
 * Permite inyectar la velocidad real de las ruedas (odometría)
 */
struct WheelOdometry {
    float speed_mps;       /* Velocidad en metros por segundo */
    bool reverse;          /* Marcha atrás activa */
    uint32_t timestamp_ms; /* Tiempo del bus del vehículo */
};

/**
 * @brief Inicializa el núcleo de navegación (dominio y posición inicial)
 */
void Initialize(NavDomain domain, Vector3D initial_position);

/**
 * @brief Inyecta una lectura de IMU en el Core desde cualquier origen
 */
void Ingest_IMU(const ImuSample &imu_data);

/**
 * @brief Inyecta una lectura de GNSS/GPS en el Core desde cualquier origen
 */
void Ingest_GNSS(const GpsSample &gnss_data);

/**
 * @brief Inyecta la velocidad de las ruedas (Exclusivo para coches/rovers)
 * @note Ayuda a eliminar la deriva (drift) en túneles de forma drástica
 */
void Ingest_WheelOdometry(const WheelOdometry &odo_data);

/**
 * @brief Devuelve el estado de navegación actual para que el vehículo lo use
 */
void Get_CurrentState(::NavState *state_out);

/**
 * @brief Devuelve pos_x/pos_y listos para el navegador del vehículo
 */
void Get_VehicleNavOutput(VehicleNavOutput *output);

} /* namespace NaviCore */
