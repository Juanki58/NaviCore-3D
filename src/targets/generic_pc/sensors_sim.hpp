#ifndef NAVICORE_SENSORS_SIM_HPP
#define NAVICORE_SENSORS_SIM_HPP

#include <random>
#include <stdint.h>

#include "sensor_types.hpp"
#include "vector3d.h"

typedef enum {
    SCENARIO_CLEAN = 0,
    SCENARIO_GPS_LOSS,
    SCENARIO_IMU_DRIFT,
    SCENARIO_ODOM_LOSS,
} SensorScenario;

#define SENSOR_FAULT_GPS_LOSS_START_TICK_DEFAULT 3U
#define SENSOR_FAULT_ODOM_LOSS_START_TICK_DEFAULT 100U

typedef struct {
    float accel_bias[3];
    float gyro_bias[3];
    float commanded_yaw_rate_radps;
    float commanded_forward_accel_mps2;
    std::mt19937 rng;
} ImuSimulator;

typedef struct {
    Vector3D origin;
    Vector3D position;
    float speed_mps;
    float vertical_speed_mps;
    float course_deg;
    float yaw_rate_radps;
    std::mt19937 rng;
    uint32_t last_timestamp_ms;
} GpsSimulator;

typedef struct {
    float surface_pressure_pa;
    float depth_m;
    std::mt19937 rng;
} PressureSimulator;

typedef struct {
    SensorScenario scenario;
    uint32_t tick_index;
    uint32_t gps_loss_start_tick;
    uint32_t odom_fault_start_tick;
    float imu_accel_drift_per_tick[3];
    float imu_gyro_drift_per_tick[3];
    float imu_accel_drift_accum[3];
    float imu_gyro_drift_accum[3];
} SensorFaultInjection;

typedef struct {
    ImuSimulator imu;
    GpsSimulator gps;
    SensorFaultInjection faults;
} SensorsSimulation;

void sensors_simulation_set_default_seed(uint32_t seed);
uint32_t sensors_simulation_get_default_seed(void);

void imu_simulator_init(ImuSimulator *sim, uint32_t seed);
bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out);
void imu_simulator_apply_measurement_model(ImuSimulator *sim, ImuSample *sample);

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed);
bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out);

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed);
bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out);
float pressure_depth_from_hydrostatic_pa(float surface_pressure_pa, float pressure_pa);

void sensor_fault_injection_init(SensorFaultInjection *inj, SensorScenario scenario);
void sensor_fault_injection_reset(SensorFaultInjection *inj);
const char *sensor_scenario_name(SensorScenario scenario);

void sensors_simulation_init(
    SensorsSimulation *ctx,
    SensorScenario scenario,
    Vector3D origin,
    float speed_mps,
    float course_deg,
    uint32_t seed);
bool sensors_simulation_tick(
    SensorsSimulation *ctx,
    uint32_t timestamp_ms,
    ImuSample *imu_out,
    GpsSample *gps_out);
bool sensors_simulation_read_wheel_odometry(const SensorsSimulation *ctx, float *speed_mps);

void sensors_simulation_apply_heading_control(
    SensorsSimulation *ctx,
    float course_deg,
    float yaw_rate_radps);

void sensors_simulation_apply_guidance_control(
    SensorsSimulation *ctx,
    float heading_rad,
    float desired_speed_mps,
    float desired_climb_mps,
    float current_heading_deg,
    float dt_s);

void sensors_simulation_apply_actuator_forces(
    SensorsSimulation *ctx,
    float forward_accel_mps2,
    float yaw_rate_cmd_radps,
    float vertical_accel_mps2,
    float dt_s);

#endif /* NAVICORE_SENSORS_SIM_HPP */
