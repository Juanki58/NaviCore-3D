#ifndef NAVICORE_FUSION_HPP
#define NAVICORE_FUSION_HPP

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
#include "sensor_types.hpp"

typedef uint8_t NavQuality;

#define NAVICORE_QUALITY_NONE         0x00U
#define NAVICORE_QUALITY_ODOM_FAULT   0x01U

#ifndef NAVICORE_ODOM_FAULT_THRESHOLD
#define NAVICORE_ODOM_FAULT_THRESHOLD 5.0f
#endif

#ifndef NAVICORE_BIAS_CALIBRATION_TICKS
#define NAVICORE_BIAS_CALIBRATION_TICKS 20U
#endif

typedef struct {
    NavState state;
    float imu_weight;
    float gps_weight;
    uint32_t last_gps_timestamp_ms;
    float last_pressure_pa;
    uint32_t last_pressure_timestamp_ms;
    bool pressure_sample_valid;
    NavQuality quality;
    float imu_predicted_speed_mps;
    bool imu_speed_prediction_valid;
    float bias_accel_x;
    float bias_gyro_z;
    float bias_accel_x_sum;
    float bias_gyro_z_sum;
    uint32_t bias_sample_count;
    uint32_t calibration_tick_count;
    bool bias_calibration_complete;
    float last_odom_speed_mps;
    float pending_bias_accel_x;
    float pending_bias_gyro_z;
    bool pending_imu_sample_valid;
} DeadReckoningFilter;

void dead_reckoning_init(DeadReckoningFilter *filter, Vector3D initial_position, NavDomain domain);
bool dead_reckoning_update_imu(DeadReckoningFilter *filter, const ImuSample *imu);
bool dead_reckoning_update_gps(DeadReckoningFilter *filter, const GpsSample *gps);
bool dead_reckoning_update_pressure(DeadReckoningFilter *filter, const PressureSample *pressure, float surface_pressure_pa);
bool dead_reckoning_update_wheel_odometry(DeadReckoningFilter *filter, float speed_mps, bool reverse, uint32_t timestamp_ms);

NavQuality dead_reckoning_get_quality(const DeadReckoningFilter *filter);
bool dead_reckoning_has_odom_fault(const DeadReckoningFilter *filter);

#endif /* NAVICORE_FUSION_HPP */
