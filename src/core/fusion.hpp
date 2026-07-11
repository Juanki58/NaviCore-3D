#ifndef NAVICORE_FUSION_HPP
#define NAVICORE_FUSION_HPP

#include <stdbool.h>
#include <stdint.h>

#include "NavState.h"
#include "diagnostic.hpp"
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

#ifndef NAVICORE_EKF_HEALTH_TRUST_THRESHOLD
#define NAVICORE_EKF_HEALTH_TRUST_THRESHOLD 80U
#endif

#ifndef NAVICORE_GPS_MEASUREMENT_VARIANCE_M2
#define NAVICORE_GPS_MEASUREMENT_VARIANCE_M2 25.0f
#endif

#ifndef NAVICORE_POSITION_PROCESS_NOISE_M2_PER_S
#define NAVICORE_POSITION_PROCESS_NOISE_M2_PER_S 2.0f
#endif

#ifndef NAVICORE_POSITION_PRIOR_VARIANCE_MAX_M2
#define NAVICORE_POSITION_PRIOR_VARIANCE_MAX_M2 400.0f
#endif

#ifndef NAVICORE_GRAVITY_MPS2
#define NAVICORE_GRAVITY_MPS2 9.81f
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
    uint32_t calibration_ticks;
    bool is_calibrated;
    float bias_accel_x;
    float bias_gyro_z;
    float accel_x_sum;
    float gyro_z_sum;
    uint32_t calibration_samples;
    float gps_noise_covariance_scale;
    float gps_measurement_variance_m2;
    float position_prior_variance_m2;
    float pitch_rad;
    float roll_rad;
} DeadReckoningFilter;

void dead_reckoning_init(DeadReckoningFilter *filter, Vector3D initial_position, NavDomain domain);
bool dead_reckoning_update_imu(
    DeadReckoningFilter *filter,
    const ImuSample *imu,
    const SystemHealthMonitor *health_monitor);
bool dead_reckoning_update_gps(
    DeadReckoningFilter *filter,
    const GpsSample *gps,
    const SystemHealthMonitor *health_monitor);
bool dead_reckoning_update_pressure(DeadReckoningFilter *filter, const PressureSample *pressure, float surface_pressure_pa);
bool dead_reckoning_update_wheel_odometry(DeadReckoningFilter *filter, float speed_mps, bool reverse, uint32_t timestamp_ms);

NavQuality dead_reckoning_get_quality(const DeadReckoningFilter *filter);
bool dead_reckoning_has_odom_fault(const DeadReckoningFilter *filter);

#endif /* NAVICORE_FUSION_HPP */
