#ifndef NAVICORE_INERTIAL_REPLAY_HPP
#define NAVICORE_INERTIAL_REPLAY_HPP

#include <stdbool.h>
#include <stdint.h>

#include "sensor_types.hpp"

/*
 * Lector síncrono de logs inerciales CSV para Software-in-the-Loop.
 *
 * Columnas mínimas (orden libre, resolución por cabecera):
 *   tiempo: timestamp | time_us | time_ms | t_ms | t | time_s
 *   IMU:    acc_x/y/z, gyro_x/y/z  (m/s² o G; rad/s o deg/s — autodetectado)
 *   GNSS:   lat/lon/alt  o  pos_x/pos_y/pos_z  (opcional por fila)
 *
 * Filas sin GNSS válido permiten auditar propagación pura del EKF.
 */

typedef struct {
    uint32_t time_ms;
    ImuSample imu;
    GpsSample gps;
    bool imu_valid;
    bool gnss_valid;
} InertialReplayRow;

typedef struct {
    InertialReplayRow *rows;
    size_t row_count;
    size_t row_capacity;
    uint32_t duration_ms;
    bool time_is_relative;
    bool accel_in_g;
    bool gyro_in_degps;
    char source_path[512];
} InertialReplayLog;

bool inertial_replay_load(InertialReplayLog *log, const char *csv_path);
void inertial_replay_free(InertialReplayLog *log);

uint32_t inertial_replay_duration_ms(const InertialReplayLog *log);
size_t inertial_replay_row_count(const InertialReplayLog *log);

/*
 * Obtiene la muestra para el instante de simulación sim_time_ms (paso nominal 10 ms).
 *
 * imu_out / gps_out: datos listos para predict() / update_gnss().
 * has_imu_sample:    true si hay IMU utilizable en este tick (fila exacta o hold ante huecos).
 * has_gnss_sample: true solo si la fila alineada al tick incluye GNSS válido.
 *
 * Retorna false cuando sim_time_ms supera el final del log.
 */
bool inertial_replay_sample_at(
    const InertialReplayLog *log,
    uint32_t sim_time_ms,
    ImuSample *imu_out,
    GpsSample *gps_out,
    bool *has_imu_sample,
    bool *has_gnss_sample);

#endif /* NAVICORE_INERTIAL_REPLAY_HPP */
