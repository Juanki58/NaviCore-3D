#ifndef NAVICORE_FUSION_HPP
#define NAVICORE_FUSION_HPP

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
#include "sensor_types.hpp"

typedef struct {
    NavState state;
    float imu_weight;
    float gps_weight;
    uint32_t last_gps_timestamp_ms;
    float last_pressure_pa;
    uint32_t last_pressure_timestamp_ms;
    bool pressure_sample_valid;
} DeadReckoningFilter;

void dead_reckoning_init(DeadReckoningFilter *filter, Vector3D initial_position, NavDomain domain);
bool dead_reckoning_update_imu(DeadReckoningFilter *filter, const ImuSample *imu);
bool dead_reckoning_update_gps(DeadReckoningFilter *filter, const GpsSample *gps);
bool dead_reckoning_update_pressure(DeadReckoningFilter *filter, const PressureSample *pressure, float surface_pressure_pa);
bool dead_reckoning_update_wheel_odometry(DeadReckoningFilter *filter, float speed_mps, bool reverse, uint32_t timestamp_ms);

#endif /* NAVICORE_FUSION_HPP */
