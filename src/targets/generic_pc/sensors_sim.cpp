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

namespace {

constexpr float kImuAccelNoiseHalfRangeMps2 = 0.01f;
constexpr float kImuAccelZNoiseHalfRangeMps2 = 0.02f;
constexpr float kImuMagNoiseHalfRangeUt = 0.5f;
constexpr float kImuAccelBiasStdMps2 = 0.05f;
constexpr float kImuGyroBiasStdRadps = 0.001f;

uint32_t g_default_simulation_seed = 71U;

uint32_t normalize_seed(uint32_t seed)
{
    return (seed == 0U) ? 1U : seed;
}

float uniform_float(std::mt19937 &rng, float min_value, float max_value)
{
    std::uniform_real_distribution<float> dist(min_value, max_value);
    return dist(rng);
}

float gaussian_float(std::mt19937 &rng, float mean, float stddev)
{
    std::normal_distribution<float> dist(mean, stddev);
    return dist(rng);
}

float deg_to_rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

void sensor_fault_apply_defaults(SensorFaultInjection *inj, SensorScenario scenario)
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

void sensor_fault_apply_imu(SensorFaultInjection *inj, ImuSample *sample)
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

void sensor_fault_apply_gps(SensorFaultInjection *inj, GpsSample *sample)
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

void init_imu_biases(ImuSimulator *sim)
{
    if (sim == NULL) {
        return;
    }

    for (int axis = 0; axis < 3; ++axis) {
        sim->accel_bias[axis] = gaussian_float(sim->rng, 0.0f, kImuAccelBiasStdMps2);
        sim->gyro_bias[axis] = gaussian_float(sim->rng, 0.0f, kImuGyroBiasStdRadps);
    }
}

} /* namespace */

void sensors_simulation_set_default_seed(uint32_t seed)
{
    g_default_simulation_seed = normalize_seed(seed);
}

uint32_t sensors_simulation_get_default_seed(void)
{
    return g_default_simulation_seed;
}

void imu_simulator_init(ImuSimulator *sim, uint32_t seed)
{
    if (sim == NULL) {
        return;
    }

    sim->commanded_yaw_rate_radps = 0.0f;
    sim->commanded_forward_accel_mps2 = 0.0f;
    memset(sim->accel_bias, 0, sizeof(sim->accel_bias));
    memset(sim->gyro_bias, 0, sizeof(sim->gyro_bias));
    sim->rng = std::mt19937(normalize_seed(seed));
    init_imu_biases(sim);
}

void imu_simulator_apply_measurement_model(ImuSimulator *sim, ImuSample *sample)
{
    if (sim == NULL || sample == NULL) {
        return;
    }

    for (int axis = 0; axis < 3; ++axis) {
        sample->accel_mps2[axis] += sim->accel_bias[axis];
        sample->gyro_radps[axis] += sim->gyro_bias[axis];
    }

    sample->accel_mps2[0] += uniform_float(sim->rng, -kImuAccelNoiseHalfRangeMps2, kImuAccelNoiseHalfRangeMps2);
    sample->accel_mps2[1] += uniform_float(sim->rng, -kImuAccelNoiseHalfRangeMps2, kImuAccelNoiseHalfRangeMps2);
    sample->accel_mps2[2] += uniform_float(sim->rng, -kImuAccelZNoiseHalfRangeMps2, kImuAccelZNoiseHalfRangeMps2);

    sample->gyro_radps[0] += uniform_float(sim->rng, -kImuGyroBiasStdRadps, kImuGyroBiasStdRadps);
    sample->gyro_radps[1] += uniform_float(sim->rng, -kImuGyroBiasStdRadps, kImuGyroBiasStdRadps);
    sample->gyro_radps[2] += uniform_float(sim->rng, -kImuGyroBiasStdRadps, kImuGyroBiasStdRadps);

    sample->mag_ut[0] = 22.0f + uniform_float(sim->rng, -kImuMagNoiseHalfRangeUt, kImuMagNoiseHalfRangeUt);
    sample->mag_ut[1] = 5.0f + uniform_float(sim->rng, -0.3f, 0.3f);
    sample->mag_ut[2] = 42.0f + uniform_float(sim->rng, -0.4f, 0.4f);
}

bool imu_simulator_read(ImuSimulator *sim, uint32_t timestamp_ms, ImuSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float t = (float)timestamp_ms * 0.001f;

    out->accel_mps2[0] = sim->commanded_forward_accel_mps2 + (0.05f * sinf(t * 0.7f));
    out->accel_mps2[1] = 0.03f * cosf(t * 0.5f);
    out->accel_mps2[2] = 9.80665f;

    out->gyro_radps[0] = 0.002f * sinf(t * 1.1f);
    out->gyro_radps[1] = 0.0015f * cosf(t * 0.9f);
    out->gyro_radps[2] = sim->commanded_yaw_rate_radps + (0.001f * sinf(t * 0.3f));

    imu_simulator_apply_measurement_model(sim, out);

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
    sim->yaw_rate_radps = 0.0f;
    sim->rng = std::mt19937(normalize_seed(seed));
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
        sim->position.x + uniform_float(sim->rng, -1.0e-6f, 1.0e-6f),
        sim->position.y + uniform_float(sim->rng, -1.0e-6f, 1.0e-6f),
        sim->position.z + uniform_float(sim->rng, -0.05f, 0.05f));
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
    sim->rng = std::mt19937(normalize_seed(seed));
}

bool pressure_simulator_read(PressureSimulator *sim, uint32_t timestamp_ms, PressureSample *out)
{
    if (sim == NULL || out == NULL) {
        return false;
    }

    const float hydrostatic_pa = SEAWATER_DENSITY_KG_M3 * GRAVITY_MPS2 * sim->depth_m;
    const float noise_pa = uniform_float(sim->rng, -15.0f, 15.0f);

    out->pressure_pa = sim->surface_pressure_pa + hydrostatic_pa + noise_pa;
    out->temperature_c = 12.0f + uniform_float(sim->rng, -0.2f, 0.2f);
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

    memset(&ctx->faults, 0, sizeof(ctx->faults));
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
    ctx->gps.yaw_rate_radps = yaw_rate_radps;
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

static float sim_normalize_course_deg(float course_deg)
{
    float normalized = course_deg;
    while (normalized < 0.0f) {
        normalized += 360.0f;
    }
    while (normalized >= 360.0f) {
        normalized -= 360.0f;
    }
    return normalized;
}

void sensors_simulation_apply_actuator_forces(
    SensorsSimulation *ctx,
    float forward_accel_mps2,
    float yaw_rate_cmd_radps,
    float vertical_accel_mps2,
    float dt_s)
{
    if (ctx == NULL || dt_s <= 0.0f) {
        return;
    }

    constexpr float kMaxForwardAccelMps2 = 3.5f;
    constexpr float kMaxYawRateSlewRadps2 = 4.0f;
    constexpr float kMaxVerticalAccelMps2 = 2.5f;
    constexpr float kMaxYawRateRadps = 1.2f;
    constexpr float kMaxSpeedMps = 40.0f;
    constexpr float kMaxVerticalSpeedMps = 8.0f;

    const float fwd_accel = sim_clampf(forward_accel_mps2, -kMaxForwardAccelMps2, kMaxForwardAccelMps2);
    const float vert_accel = sim_clampf(vertical_accel_mps2, -kMaxVerticalAccelMps2, kMaxVerticalAccelMps2);

    const float yaw_rate_target = sim_clampf(yaw_rate_cmd_radps, -kMaxYawRateRadps, kMaxYawRateRadps);
    const float max_yaw_rate_delta = kMaxYawRateSlewRadps2 * dt_s;
    float yaw_rate_delta = yaw_rate_target - ctx->gps.yaw_rate_radps;
    yaw_rate_delta = sim_clampf(yaw_rate_delta, -max_yaw_rate_delta, max_yaw_rate_delta);
    const float yaw_rate = sim_clampf(
        ctx->gps.yaw_rate_radps + yaw_rate_delta,
        -kMaxYawRateRadps,
        kMaxYawRateRadps);

    float speed = ctx->gps.speed_mps + (fwd_accel * dt_s);
    speed = sim_clampf(speed, 0.0f, kMaxSpeedMps);

    float vertical_speed = ctx->gps.vertical_speed_mps + (vert_accel * dt_s);
    vertical_speed = sim_clampf(vertical_speed, -kMaxVerticalSpeedMps, kMaxVerticalSpeedMps);

    const float course_delta_deg = yaw_rate * dt_s * (180.0f / M_PI);
    const float course_deg = sim_normalize_course_deg(ctx->gps.course_deg + course_delta_deg);

    ctx->gps.yaw_rate_radps = yaw_rate;
    ctx->gps.speed_mps = speed;
    ctx->gps.vertical_speed_mps = vertical_speed;
    ctx->gps.course_deg = course_deg;
    ctx->imu.commanded_yaw_rate_radps = yaw_rate;
    ctx->imu.commanded_forward_accel_mps2 = fwd_accel;
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
