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

#define IMU_SIM_GPS_DELAY_MS 80U
#define IMU_SIM_GPS_DELAY_TICKS 8U
#define IMU_SIM_GPS_RING_CAPACITY 16U

#define IMU_SIM_SAMPLE_RATE_HZ 100.0f
#define IMU_SIM_SAMPLE_DT_S (1.0f / IMU_SIM_SAMPLE_RATE_HZ)

/*
 * Parametros estocasticos IEEE Std 952-1997 (MEMS tactico representativo).
 * Gyro: ARW [deg/sqrt(h)], Bias Instability [deg/h], RRW [deg/sqrt(h^3)].
 * Accel: VRW [m/s/sqrt(h)], Bias Instability [m/s^2], ARW [m/s^2/sqrt(h)].
 */
#define IMU_SIM_GYRO_ARW_DEG_SQRT_H 0.20f
#define IMU_SIM_GYRO_BIAS_INSTABILITY_DEG_H 0.05f
#define IMU_SIM_GYRO_RRW_DEG_SQRT_H3 0.005f

#define IMU_SIM_ACCEL_VRW_MPS_SQRT_H 0.08f
#define IMU_SIM_ACCEL_BIAS_INSTABILITY_MPS2 0.001f
#define IMU_SIM_ACCEL_ARW_MPS2_SQRT_H 0.0005f

/** EKF bias RW alineado con RRW del simulador (sigma_tick^2 / dt). */
#define IMU_SIM_ACCEL_BIAS_WALK_STD_PER_TICK 0.0001f
#define IMU_SIM_GYRO_BIAS_WALK_STD_PER_TICK  1.0e-6f

typedef struct {
    float turn_on_accel_bias[3];
    float turn_on_gyro_bias[3];
    float accel_rrw[3];
    float gyro_rrw[3];
    float accel_pink_pole_fast[3];
    float accel_pink_pole_slow[3];
    float gyro_pink_pole_fast[3];
    float gyro_pink_pole_slow[3];
    float accel_scale[3];
    float gyro_scale[3];
    float accel_misalign[9];
    float gyro_misalign[9];
    float commanded_yaw_rate_radps;
    float commanded_forward_accel_mps2;
    uint32_t last_sim_clock_ms;
    uint32_t last_output_timestamp_ms;
    bool scale_misalign_enabled;
    std::mt19937 rng;
} ImuSimulator;

typedef struct {
    Vector3D origin;
    Vector3D position;
    float speed_mps;
    float vertical_speed_mps;
    float course_deg;
    float yaw_rate_radps;
    GpsSample current_truth;
    GpsSample ring[IMU_SIM_GPS_RING_CAPACITY];
    uint8_t ring_count;
    uint8_t ring_write;
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
void imu_simulator_set_scale_misalign_enabled(ImuSimulator *sim, bool enabled);
void imu_simulator_step_bias_random_walk(ImuSimulator *sim);
void imu_simulator_fill_kinematics(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out);
bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out);
void imu_simulator_apply_measurement_model(
    ImuSimulator *sim,
    ImuSample *sample,
    uint32_t sim_clock_ms);

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed);
bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out);
bool gps_simulator_get_truth(const GpsSimulator *sim, GpsSample *out);

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
void sensors_simulation_apply_step_faults(
    SensorsSimulation *ctx,
    ImuSample *imu,
    GpsSample *gps);
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
