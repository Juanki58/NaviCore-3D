#ifndef NAVICORE_IMU_H
#define NAVICORE_IMU_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float accel_mps2[3];   /* cuerpo: adelante, derecha, abajo */
    float gyro_radps[3];   /* roll, pitch, yaw rates */
    float mag_ut[3];       /* magnetómetro (microteslas) */
    uint32_t timestamp_ms;
    bool valid;
} ImuSample;

typedef struct {
    float accel_bias[3];
    float gyro_bias[3];
    uint32_t seed;
} ImuSimulator;

void imu_simulator_init(ImuSimulator *sim, uint32_t seed);
bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out);

#endif /* NAVICORE_IMU_H */
