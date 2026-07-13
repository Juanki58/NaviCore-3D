#include "sensors_sim.hpp"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef SEAWATER_DENSITY_KG_M3
#define SEAWATER_DENSITY_KG_M3 1025.0f
#endif

#ifndef GRAVITY_MPS2
#define GRAVITY_MPS2 9.80665f
#endif

static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

static float lcg_float(uint32_t *state, float min_value, float max_value)
{
    const float t = (float)(lcg_next(state) & 0x00FFFFFFU) / (float)0x01000000U;
    return min_value + (t * (max_value - min_value));
}

static float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

static void sensor_fault_apply_defaults(SensorFaultInjection *inj, SensorScenario scenario)
{
    inj->scenario = scenario;
    inj->tick_index = 0U;
    inj->gps_loss_start_tick = SENSOR_FAULT_GPS_LOSS_START_TICK_DEFAULT;

    memset(inj->imu_accel_drift_per_tick, 0, sizeof(inj->imu_accel_drift_per_tick));
    memset(inj->imu_gyro_drift_per_tick, 0, sizeof(inj->imu_gyro_drift_per_tick));
    memset(inj->imu_accel_drift_accum, 0, sizeof(inj->imu_accel_drift_accum));
    memset(inj->imu_gyro_drift_accum, 0, sizeof(inj->imu_gyro_drift_accum));

    if (scenario == SCENARIO_IMU_DRIFT) {
        inj->imu_accel_drift_per_tick[0] = 0.05f;
        inj->imu_accel_drift_per_tick[1] = 0.02f;
        inj->imu_gyro_drift_per_tick[2] = 0.002f;
    }

    if (scenario == SCENARIO_ODOM_LOSS) {
        inj->gps_loss_start_tick = SENSOR_FAULT_ODOM_LOSS_START_TICK_DEFAULT;
        inj->odom_fault_start_tick = SENSOR_FAULT_ODOM_LOSS_START_TICK_DEFAULT;
    }
}

static void sensor_fault_apply_imu(SensorFaultInjection *inj, ImuSample *sample)
{
    if (inj == NULL || sample == NULL || inj->scenario != SCENARIO_IMU_DRIFT) {
        return;
    }

    for (int axis = 0; axis < 3; ++axis) {
        inj->imu_accel_drift_accum[axis] += inj->imu_accel_drift_per_tick[axis];
        inj->imu_gyro_drift_accum[axis] += inj->imu_gyro_drift_per_tick[axis];

        sample->accel_mps2[axis] += inj->imu_accel_drift_accum[axis];
        sample->gyro_radps[axis] += inj->imu_gyro_drift_accum[axis];
    }
}

static void sensor_fault_apply_gps(SensorFaultInjection *inj, GpsSample *sample)
{
    if (inj == NULL || sample == NULL) {
        return;
    }

    if (inj->scenario != SCENARIO_GPS_LOSS && inj->scenario != SCENARIO_ODOM_LOSS) {
        return;
    }

    if (inj->tick_index >= inj->gps_loss_start_tick) {
        sample->satellites = 0U;
        sample->fix_valid = false;
    }
}

void imu_simulator_init(ImuSimulator *sim, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    memset(sim, 0, sizeof(*sim));
    sim->seed = (seed == 0U) ? 1U : seed;
    sim->commanded_yaw_rate_radps = 0.0f;
    sim->commanded_forward_accel_mps2 = 0.0f;
}

bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float t = (float)timestamp_ms * 0.001f;

    out->accel_mps2[0] = sim->commanded_forward_accel_mps2
        + (0.05f * sinf(t * 0.7f))
        + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[1] = 0.03f * cosf(t * 0.5f) + lcg_float(&sim->seed, -0.01f, 0.01f);
    out->accel_mps2[2] = 9.80665f + lcg_float(&sim->seed, -0.02f, 0.02f);

    out->gyro_radps[0] = 0.002f * sinf(t * 1.1f);
    out->gyro_radps[1] = 0.0015f * cosf(t * 0.9f);
    out->gyro_radps[2] = sim->commanded_yaw_rate_radps + (0.001f * sinf(t * 0.3f));

    out->mag_ut[0] = 22.0f + lcg_float(&sim->seed, -0.5f, 0.5f);
    out->mag_ut[1] = 5.0f + lcg_float(&sim->seed, -0.3f, 0.3f);
    out->mag_ut[2] = 42.0f + lcg_float(&sim->seed, -0.4f, 0.4f);

    out->timestamp_ms = timestamp_ms;
    out->valid = true;
    return true;
}

void gps_simulator_init(GpsSimulator *sim, Vector3D origin, float speed_mps, float course_deg, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    sim->origin = origin;
    sim->position = origin;
    sim->speed_mps = speed_mps;
    sim->vertical_speed_mps = 0.0f;
    sim->course_deg = course_deg;
    sim->seed = (seed == 0U) ? 1U : seed;
    sim->last_timestamp_ms = 0U;
}

bool gps_simulator_read(GpsSimulator *sim, uint32_t timestamp_ms, GpsSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    uint32_t dt_ms = 0U;
    if (sim->last_timestamp_ms < timestamp_ms) {
        dt_ms = timestamp_ms - sim->last_timestamp_ms;
    }

    const float dt_s = (float)dt_ms * 0.001f;
    const float course_rad = deg_to_rad(sim->course_deg);
    const float north_m = sim->speed_mps * dt_s * cosf(course_rad);
    const float east_m = sim->speed_mps * dt_s * sinf(course_rad);

    const float lat_rad = deg_to_rad(sim->position.x);
    const float dlat = north_m / 111132.954f;
    const float dlon = east_m / (111132.954f * cosf(lat_rad));

    sim->position.x += dlat;
    sim->position.y += dlon;
    sim->position.z += sim->vertical_speed_mps * dt_s;
    sim->last_timestamp_ms = timestamp_ms;

    out->position = vector3d_make(
        sim->position.x + lcg_float(&sim->seed, -1.0e-6f, 1.0e-6f),
        sim->position.y + lcg_float(&sim->seed, -1.0e-6f, 1.0e-6f),
        sim->position.z + lcg_float(&sim->seed, -0.05f, 0.05f));
    out->speed_mps = sim->speed_mps;
    out->course_deg = sim->course_deg;
    out->satellites = 10U;
    out->timestamp_ms = timestamp_ms;
    out->fix_valid = true;
    return true;
}

void pressure_simulator_init(PressureSimulator *sim, float surface_pressure_pa, float depth_m, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    sim->surface_pressure_pa = surface_pressure_pa;
    sim->depth_m = depth_m;
    sim->seed = (seed == 0U) ? 1U : seed;
}

bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float hydrostatic_pa = SEAWATER_DENSITY_KG_M3 * GRAVITY_MPS2 * sim->depth_m;
    const float noise_pa = lcg_float(&sim->seed, -15.0f, 15.0f);

    out->pressure_pa = sim->surface_pressure_pa + hydrostatic_pa + noise_pa;
    out->temperature_c = 12.0f + lcg_float(&sim->seed, -0.2f, 0.2f);
    out->timestamp_ms = timestamp_ms;
    out->valid = true;
    return true;
}

float pressure_depth_from_hydrostatic_pa(float surface_pressure_pa, float pressure_pa)
{
    const float delta_pa = pressure_pa - surface_pressure_pa;
    if (delta_pa <= 0.0f) {
        return 0.0f;
    }
    return delta_pa / (SEAWATER_DENSITY_KG_M3 * GRAVITY_MPS2);
}

void sensor_fault_injection_init(SensorFaultInjection *inj, SensorScenario scenario)
{
    if (inj == NULL) {
        return;
    }

    memset(inj, 0, sizeof(*inj));
    sensor_fault_apply_defaults(inj, scenario);
}

void sensor_fault_injection_reset(SensorFaultInjection *inj)
{
    if (inj == NULL) {
        return;
    }

    const SensorScenario scenario = inj->scenario;
    sensor_fault_apply_defaults(inj, scenario);
}

const char *sensor_scenario_name(SensorScenario scenario)
{
    switch (scenario) {
    case SCENARIO_CLEAN:
        return "CLEAN";
    case SCENARIO_GPS_LOSS:
        return "GPS_LOSS";
    case SCENARIO_IMU_DRIFT:
        return "IMU_DRIFT";
    case SCENARIO_ODOM_LOSS:
        return "ODOM_LOSS";
    default:
        return "UNKNOWN";
    }
}

void sensors_simulation_init(
    SensorsSimulation *ctx,
    SensorScenario scenario,
    Vector3D origin,
    float speed_mps,
    float course_deg,
    uint32_t seed)
{
    if (ctx == NULL) {
        return;
    }

    memset(ctx, 0, sizeof(*ctx));
    imu_simulator_init(&ctx->imu, seed);
    gps_simulator_init(&ctx->gps, origin, speed_mps, course_deg, seed + 1U);
    sensor_fault_injection_init(&ctx->faults, scenario);
}

bool sensors_simulation_tick(
    SensorsSimulation *ctx,
    uint32_t timestamp_ms,
    ImuSample *imu_out,
    GpsSample *gps_out)
{
    if (ctx == NULL || imu_out == NULL || gps_out == NULL) {
        return false;
    }

    if (!imu_simulator_read(&ctx->imu, timestamp_ms, imu_out)) {
        return false;
    }
    if (!gps_simulator_read(&ctx->gps, timestamp_ms, gps_out)) {
        return false;
    }

    sensor_fault_apply_imu(&ctx->faults, imu_out);
    sensor_fault_apply_gps(&ctx->faults, gps_out);

    ctx->faults.tick_index++;
    return true;
}

bool sensors_simulation_read_wheel_odometry(const SensorsSimulation *ctx, float *speed_mps)
{
    if (ctx == NULL || speed_mps == NULL) {
        return false;
    }

    if (ctx->faults.scenario == SCENARIO_ODOM_LOSS
        && ctx->faults.tick_index > ctx->faults.odom_fault_start_tick) {
        *speed_mps = 0.0f;
        return true;
    }

    *speed_mps = ctx->gps.speed_mps;
    return true;
}

void sensors_simulation_apply_heading_control(
    SensorsSimulation *ctx,
    float course_deg,
    float yaw_rate_radps)
{
    if (ctx == NULL) {
        return;
    }

    ctx->gps.course_deg = course_deg;
    ctx->imu.commanded_yaw_rate_radps = yaw_rate_radps;
    ctx->imu.commanded_forward_accel_mps2 = 0.0f;
}

static float sim_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float sim_heading_delta_deg(float from_deg, float to_deg)
{
    float delta = to_deg - from_deg;
    while (delta > 180.0f) {
        delta -= 360.0f;
    }
    while (delta < -180.0f) {
        delta += 360.0f;
    }
    return delta;
}

void sensors_simulation_apply_guidance_control(
    SensorsSimulation *ctx,
    float heading_rad,
    float desired_speed_mps,
    float desired_climb_mps,
    float current_heading_deg,
    float dt_s)
{
    if (ctx == NULL || dt_s <= 0.0f) {
        return;
    }

    const float heading_deg = heading_rad * (180.0f / M_PI);
    float normalized_heading = heading_deg;
    while (normalized_heading < 0.0f) {
        normalized_heading += 360.0f;
    }
    while (normalized_heading >= 360.0f) {
        normalized_heading -= 360.0f;
    }

    const float delta_heading_deg = sim_heading_delta_deg(current_heading_deg, normalized_heading);
    const float yaw_rate_radps = (delta_heading_deg * (M_PI / 180.0f)) / dt_s;

    constexpr float kMaxAccelMps2 = 2.5f;
    const float speed_error_mps = desired_speed_mps - ctx->gps.speed_mps;
    const float accel_mps2 = sim_clampf(speed_error_mps / dt_s, -kMaxAccelMps2, kMaxAccelMps2);

    ctx->gps.course_deg = normalized_heading;
    ctx->gps.vertical_speed_mps = desired_climb_mps;
    ctx->gps.speed_mps = sim_clampf(
        ctx->gps.speed_mps + (accel_mps2 * dt_s),
        0.0f,
        40.0f);
    ctx->imu.commanded_yaw_rate_radps = yaw_rate_radps;
    ctx->imu.commanded_forward_accel_mps2 = accel_mps2;
}
