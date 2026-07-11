/**
 * @file bsp_sensors.hpp
 * @brief Abstraccion de Hardware (HAL/BSP) para NaviCore-Ambiq
 * @note Define la interfaz de lectura directa por registros e interrupciones
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "../../core/fusion.hpp"
#include "../../core/sensor_types.hpp"

namespace NaviCore {

using IMUMeasurement = ImuSample;
using GNSSMeasurement = GpsSample;
using NavState = ::NavState;

} /* namespace NaviCore */

/** Frecuencia del bucle critico de sensores (CTIMER / STimer). */
#define BSP_SENSORS_CTIMER_HZ        10U
#define BSP_SENSORS_CTIMER_PERIOD_MS (1000U / BSP_SENSORS_CTIMER_HZ)

/** Umbral de espera maxima para una transaccion SPI/DMA (microsegundos). */
#define BSP_SPI_TIMEOUT_US           5000U

typedef enum {
    BSP_SPI_BUS_IDLE = 0,
    BSP_SPI_BUS_DMA_ACTIVE,
    BSP_SPI_BUS_ERROR,
    BSP_SPI_BUS_TIMEOUT
} BspSpiBusState;

/** Alias de diagnostico equivalente a BSP_SPI_BUS_TIMEOUT. */
#define BSP_BUS_TIMEOUT BSP_SPI_BUS_TIMEOUT

typedef struct {
    BspSpiBusState imu;
    BspSpiBusState baro;
    bool ctimer_armed;
    bool interrupt_pending;
} BspSensorsBusStatus;

struct PowerMetrics {
    float voltage_core_mv;
    float current_draw_ua;
    uint32_t active_cycles;
};

/**
 * @brief Inicializa CTIMER @ 10 Hz y los buses SPI (IMU + barometro) con soporte DMA.
 * @return true si todos los stubs de hardware quedaron listos para el ciclo critico.
 */
bool bsp_sensors_init(void);

/**
 * @brief Ciclo critico bare-metal: despertar por interrupcion, lectura IMU/barometro,
 *        empaquetado en ImuSample/GpsSample y actualizacion del filtro de fusion.
 * @param nav_filter Filtro de navegacion (estado global del vehiculo).
 * @return true si el tick se ejecuto con muestras IMU validas.
 */
bool bsp_sensors_orchestrate_tick(DeadReckoningFilter *nav_filter);

/**
 * @brief Expone el estado actual de buses SPI y bandera de interrupcion CTIMER.
 */
void bsp_sensors_get_bus_status(BspSensorsBusStatus *status_out);

/**
 * @brief Lee los datos del giroscopio y acelerometro mediante rafaga SPI/DMA.
 */
void Ambiq_BSP_ReadIMU(NaviCore::IMUMeasurement *imu_out);

/**
 * @brief Lee los datos del receptor GNSS si la linea de interrupcion esta en alta.
 */
void Ambiq_BSP_ReadGNSS(NaviCore::GNSSMeasurement *gnss_out);

/**
 * @brief Transmite el estado actual a traves del bus serie o telemetria dedicada.
 */
void Ambiq_BSP_TransmitState(const NaviCore::NavState *state_in);

/**
 * @brief Registra y expone las metricas de consumo del ultimo tick de ejecucion.
 */
void Ambiq_BSP_GetPowerMetrics(PowerMetrics *metrics_out);
