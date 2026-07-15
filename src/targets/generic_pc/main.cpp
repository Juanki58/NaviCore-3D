#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cerrno>
#include <cstring>
#include <chrono>

#ifdef _WIN32
#include <direct.h>
#include <windows.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

#include "vector3d.h"
#include "NavState.h"
#include "fusion.hpp"
#include "guidance.hpp"
#include "mission.hpp"
#include "ins_ekf.hpp"
#include "waypoint.hpp"
#include "sensors_sim.hpp"
#include "inertial_replay.hpp"
#include "diagnostic.hpp"
#include "command_ingestor.hpp"
#include "navigation_cortex.hpp"
#include "power_state_machine.hpp"
#include "time_guard.hpp"
#include "telemetry_interface.hpp"
#include "pid.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr uint32_t kStepMs = 100U;
constexpr uint32_t kEkfStepMs = 10U;
constexpr float kEkfDtS = NAVICORE_INS_EKF_DT_S;
constexpr float kCruiseSpeedMps = 15.0f;
constexpr float kDegradedCruiseSpeedMps = 8.0f;
constexpr float kCruiseCourseDeg = 90.0f;
constexpr float kSurfacePressurePa = 101325.0f;
constexpr float kSubmersionPressureRatePaS = 10000.0f;
constexpr float kSafeStopDecelMps2 = 3.0f;
constexpr float kVehicleStoppedSpeedMps = 0.05f;

constexpr uint32_t kFaultInjectGpsLossMs = 5000U;
constexpr uint32_t kFaultInjectRadioCmdMs = 10000U;
constexpr uint32_t kFaultInjectSpiTimeoutMs = 15000U;
constexpr uint32_t kFaultInjectDurationMs = 30000U;

constexpr uint32_t kHighDemandRadioBurstMs = 5000U;
constexpr uint32_t kHighDemandWcetStressMs = 10000U;
constexpr uint32_t kHighDemandDurationMs = 20000U;
constexpr size_t kHighDemandRadioBurstCount = 100U;
constexpr uint32_t kHighDemandWcetArtificialDelayMs = 150U;
constexpr float kHighDemandRadioStepDeg = 0.000120f;
constexpr float kHighDemandGeomViolationStepDeg = 0.0020f;

constexpr const char *kTelemetryCsvPath = TELEMETRY_LOGGER_DEFAULT_PATH;
constexpr const char *kTelemetryUnityHost = UNITY_TELEMETRY_DEFAULT_HOST;
constexpr const char *kHighDemandScenarioName = "HIGH_DEMAND_STRESS_TEST";
constexpr float kAmbientTemperatureC = 25.0f;
constexpr float kSubmarineTemperatureC = 10.0f;

constexpr SensorScenario kSelectedScenario = SCENARIO_ODOM_LOSS;

constexpr float kWaypointLonStepDeg = 0.00018f;
constexpr float kGuidanceLookAheadM = 8.0f;
constexpr float kSquareLegStepDeg = 0.00018f;
constexpr float kSquareApproachLonOffsetDeg = 0.00009f;
constexpr float kSquareArrivalRadiusM = 2.0f;
constexpr size_t kSquareWaypointCount = 4U;
constexpr size_t kMission3dWaypointCount = 4U;
constexpr uint32_t kMissionCleanNominalTicks = 6000U;
constexpr uint32_t kMissionCleanMaxTicks = 36000U;
constexpr uint32_t kMissionAutoStartReadyTicks = 3U;
constexpr uint32_t kGpsStalenessThresholdMs = 1500U;
constexpr float kWaypointArrivalRadiusNominalM = 5.0f;
constexpr float kWaypointArrivalRadiusDegradedM = 15.0f;
constexpr float kWaypointStraightSpeedMps = NAVICORE_WAYPOINT_DEFAULT_TRANSIT_SPEED_MPS;
constexpr float kWaypointCornerSpeedMps = NAVICORE_WAYPOINT_DEFAULT_TERMINAL_SPEED_MPS;

/*
 * Perfil de ejecución del simulador PC (compile-time + CLI).
 *   SCENARIO_CLEAN       — misión 3D sin inyección de fallos (validación guiado).
 *   SCENARIO_HIGH_STRESS — WCET artificial, ráfagas radio y estrés de colas.
 */
enum class SimPrimaryScenario : uint8_t {
    SCENARIO_CLEAN = 0,
    SCENARIO_HIGH_STRESS = 1,
};

#ifndef NAVICORE_SIM_PRIMARY_SCENARIO
#define NAVICORE_SIM_PRIMARY_SCENARIO 0
#endif

constexpr SimPrimaryScenario kCompileTimeSimScenario =
    (NAVICORE_SIM_PRIMARY_SCENARIO == 1)
        ? SimPrimaryScenario::SCENARIO_HIGH_STRESS
        : SimPrimaryScenario::SCENARIO_CLEAN;

const char *sim_primary_scenario_name(SimPrimaryScenario scenario)
{
    switch (scenario) {
    case SimPrimaryScenario::SCENARIO_CLEAN:
        return "SCENARIO_CLEAN";
    case SimPrimaryScenario::SCENARIO_HIGH_STRESS:
        return "SCENARIO_HIGH_STRESS";
    default:
        return "UNKNOWN";
    }
}

struct MissionGuidanceSnapshot {
    GuidanceCommands commands;
    bool guidance_valid;
    bool return_home_active;
    size_t active_waypoint_index;
};

struct ClosedLoopPidPlant {
    PIDController speed_pid;
    PIDController yaw_pid;
    PIDController altitude_pid;
    TelemetryPidSnapshot telemetry;
};

struct GpsDeadReckoningState {
    uint32_t last_fresh_gps_ms;
    uint32_t prev_gps_timestamp_ms;
    bool active;
    bool initialized;
};

struct GpsDeadReckoningEval {
    bool allow_gnss_update;
    bool dead_reckoning_active;
};

GpsDeadReckoningEval gps_dead_reckoning_step(
    GpsDeadReckoningState *state,
    const GpsSample *gps,
    uint32_t t_ms,
    bool sample_is_fresh)
{
    GpsDeadReckoningEval result{};
    result.allow_gnss_update = false;
    result.dead_reckoning_active = (state != NULL) ? state->active : false;

    if (state == NULL || gps == NULL) {
        return result;
    }

    const bool timestamp_changed = (gps->timestamp_ms != state->prev_gps_timestamp_ms);
    state->prev_gps_timestamp_ms = gps->timestamp_ms;

    const bool fresh_valid = gps->fix_valid && sample_is_fresh
        && (timestamp_changed || !state->initialized);

    if (fresh_valid) {
        state->last_fresh_gps_ms = t_ms;
        state->initialized = true;
    }

    const bool gps_stale = state->initialized
        && ((t_ms - state->last_fresh_gps_ms) > kGpsStalenessThresholdMs);

    const bool was_active = state->active;

    if (gps_stale) {
        state->active = true;
        if (!was_active) {
            std::printf(
                "⚠️ GPS PERDIDO: Iniciando navegación por estima (Dead Reckoning)\n");
        }
    } else if (fresh_valid && was_active) {
        state->active = false;
        std::printf("✅ GPS RECUPERADO: Sincronizando posición\n");
    } else if (!gps_stale) {
        state->active = false;
    }

    result.dead_reckoning_active = state->active;
    result.allow_gnss_update = gps->fix_valid && !gps_stale;
    return result;
}

float horizontal_distance_m(Vector3D a, Vector3D b)
{
    const float dlat_m = (b.x - a.x) * 111132.954f;
    const float mean_lat_rad = (a.x + b.x) * 0.5f * (static_cast<float>(M_PI) / 180.0f);
    const float dlon_m = (b.y - a.y) * 111132.954f * std::cos(mean_lat_rad);
    return std::sqrt((dlat_m * dlat_m) + (dlon_m * dlon_m));
}

uint32_t contingency_arrival_radius_m(NavHealthMode health_mode)
{
    return static_cast<uint32_t>(
        (health_mode == HEALTH_DEGRADED)
            ? kWaypointArrivalRadiusDegradedM
            : kWaypointArrivalRadiusNominalM);
}

float contingency_cruise_speed_mps(NavHealthMode health_mode)
{
    return (health_mode == HEALTH_DEGRADED) ? kDegradedCruiseSpeedMps : kCruiseSpeedMps;
}

const char *nav_mode_name(NavMode mode)
{
    switch (mode) {
    case NAV_MODE_INITIALIZING:
        return "INITIALIZING";
    case NAV_MODE_GPS:
        return "GPS";
    case NAV_MODE_DEAD_RECKONING:
        return "DEAD_RECKONING";
    case NAV_MODE_HYBRID:
        return "HYBRID";
    default:
        return "UNKNOWN";
    }
}

const char *health_mode_name(NavHealthMode mode)
{
    switch (mode) {
    case HEALTH_NOMINAL:
        return "NOMINAL";
    case HEALTH_DEGRADED:
        return "DEGRADED";
    case HEALTH_CRITICAL:
        return "CRITICAL";
    default:
        return "UNKNOWN";
    }
}

const char *power_state_name(PowerState state)
{
    switch (state) {
    case POWER_PERFORMANCE:
        return "PERFORMANCE";
    case POWER_CONSERVATION:
        return "CONSERVATION";
    case POWER_SAFE_SHUTDOWN:
        return "SAFE_SHUTDOWN";
    default:
        return "UNKNOWN";
    }
}

uint8_t telemetry_scenario_id_from_sensor(SensorScenario scenario)
{
    switch (scenario) {
    case SCENARIO_CLEAN:
        return TELEM_SCENARIO_CLEAN;
    case SCENARIO_GPS_LOSS:
        return TELEM_SCENARIO_GPS_LOSS;
    case SCENARIO_IMU_DRIFT:
        return TELEM_SCENARIO_IMU_DRIFT;
    case SCENARIO_ODOM_LOSS:
        return TELEM_SCENARIO_ODOM_LOSS;
    default:
        return TELEM_SCENARIO_UNKNOWN;
    }
}

uint8_t telemetry_scenario_id_from_name(const char *scenario)
{
    if (scenario == NULL) {
        return TELEM_SCENARIO_UNKNOWN;
    }

    if (std::strcmp(scenario, kHighDemandScenarioName) == 0) {
        return TELEM_SCENARIO_HIGH_DEMAND;
    }
    if (std::strcmp(scenario, "FAULT_INJECTION") == 0) {
        return TELEM_SCENARIO_FAULT_INJECTION;
    }
    if (std::strcmp(scenario, "SUBMARINE") == 0) {
        return TELEM_SCENARIO_SUBMARINE;
    }
    if (std::strcmp(scenario, sensor_scenario_name(SCENARIO_CLEAN)) == 0) {
        return TELEM_SCENARIO_CLEAN;
    }
    if (std::strcmp(scenario, sensor_scenario_name(SCENARIO_GPS_LOSS)) == 0) {
        return TELEM_SCENARIO_GPS_LOSS;
    }
    if (std::strcmp(scenario, sensor_scenario_name(SCENARIO_IMU_DRIFT)) == 0) {
        return TELEM_SCENARIO_IMU_DRIFT;
    }
    if (std::strcmp(scenario, sensor_scenario_name(SCENARIO_ODOM_LOSS)) == 0) {
        return TELEM_SCENARIO_ODOM_LOSS;
    }

    return TELEM_SCENARIO_UNKNOWN;
}

uint8_t pc_simulate_worst_bsp_bus_status(bool odom_fault_active, float filter_quality)
{
    uint8_t imu_bus = DIAG_BSP_BUS_IDLE;
    uint8_t baro_bus = DIAG_BSP_BUS_IDLE;

    if (odom_fault_active) {
        imu_bus = DIAG_BSP_BUS_ERROR;
    }

    if (filter_quality < 0.20f) {
        baro_bus = DIAG_BSP_BUS_TIMEOUT;
    } else if (filter_quality < 0.40f) {
        baro_bus = DIAG_BSP_BUS_ERROR;
    }

    return (imu_bus > baro_bus) ? imu_bus : baro_bus;
}

uint8_t fault_injection_worst_bsp_bus(
    uint32_t timestamp_ms,
    bool odom_fault_active,
    float filter_quality)
{
    if (timestamp_ms >= kFaultInjectSpiTimeoutMs) {
        return DIAG_BSP_BUS_TIMEOUT;
    }

    if (odom_fault_active) {
        return DIAG_BSP_BUS_ERROR;
    }

    if (timestamp_ms >= kFaultInjectGpsLossMs && timestamp_ms < kFaultInjectSpiTimeoutMs) {
        return DIAG_BSP_BUS_DMA_ACTIVE;
    }

    return pc_simulate_worst_bsp_bus_status(odom_fault_active, filter_quality);
}

void apply_safe_stop_air(
    SensorsSimulation *sensors,
    DeadReckoningFilter *nav,
    float hold_heading_deg,
    float dt_s)
{
    if (sensors == NULL || nav == NULL) {
        return;
    }

    float speed_mps = sensors->gps.speed_mps;
    speed_mps -= kSafeStopDecelMps2 * dt_s;
    if (speed_mps < 0.0f) {
        speed_mps = 0.0f;
    }

    sensors->gps.speed_mps = speed_mps;
    sensors->gps.course_deg = hold_heading_deg;
    sensors->imu.commanded_yaw_rate_radps = 0.0f;
    sensors->imu.commanded_forward_accel_mps2 =
        (speed_mps > 0.0f) ? -kSafeStopDecelMps2 : 0.0f;

    const float heading_rad = static_cast<float>(hold_heading_deg * (M_PI / 180.0));
    nav->state.heading_deg = hold_heading_deg;
    nav->state.velocity.x = speed_mps * std::cos(heading_rad);
    nav->state.velocity.y = speed_mps * std::sin(heading_rad);
}

void apply_safe_stop_submarine(DeadReckoningFilter *nav)
{
    if (nav == NULL) {
        return;
    }

    nav->state.velocity.z = 0.0f;
}

void waypoint_route_set_arrival_radius(StaticWaypointBuffer *route, uint32_t arrival_radius_m)
{
    if (route == NULL) {
        return;
    }

    for (size_t i = 0U; i < route->count; ++i) {
        const size_t idx = (route->head + i) % NAVICORE_MAX_WAYPOINTS;
        route->items[idx].arrival_radius_m = arrival_radius_m;
    }
}

void apply_health_contingency_air_state(
    SensorsSimulation *sensors,
    StaticWaypointBuffer *route,
    NavHealthMode health_mode,
    bool safe_stop_active)
{
    if (route == NULL) {
        return;
    }

    const uint32_t arrival_radius_m = contingency_arrival_radius_m(health_mode);

    waypoint_route_set_arrival_radius(route, arrival_radius_m);

    if (sensors != NULL && !safe_stop_active) {
        sensors->gps.speed_mps = contingency_cruise_speed_mps(health_mode);
    }
}

float submarine_contingency_pressure_rate(NavHealthMode health_mode)
{
    if (health_mode == HEALTH_DEGRADED) {
        return kSubmersionPressureRatePaS * (kDegradedCruiseSpeedMps / kCruiseSpeedMps);
    }

    return kSubmersionPressureRatePaS;
}

void log_health_contingency_transition(NavHealthMode health_mode, NavHealthMode *prev_health_mode, bool is_marine)
{
    if (prev_health_mode == NULL || health_mode == *prev_health_mode) {
        return;
    }

    if (health_mode == HEALTH_DEGRADED) {
        if (is_marine) {
            std::printf(
                ">>> CONTINGENCIA DEGRADADA (MAR): tasa presion %.1f Pa/s | radio WP %.1f m\n",
                submarine_contingency_pressure_rate(health_mode),
                kWaypointArrivalRadiusDegradedM);
        } else {
            std::printf(
                ">>> CONTINGENCIA DEGRADADA: velocidad objetivo %.1f m/s | radio WP %.1f m\n",
                kDegradedCruiseSpeedMps,
                kWaypointArrivalRadiusDegradedM);
        }
    } else if (health_mode == HEALTH_NOMINAL && *prev_health_mode == HEALTH_DEGRADED) {
        if (is_marine) {
            std::printf(
                ">>> CONTINGENCIA RESTAURADA (MAR): tasa presion %.1f Pa/s | radio WP %.1f m\n",
                submarine_contingency_pressure_rate(health_mode),
                kWaypointArrivalRadiusNominalM);
        } else {
            std::printf(
                ">>> CONTINGENCIA RESTAURADA: velocidad crucero %.1f m/s | radio WP %.1f m\n",
                kCruiseSpeedMps,
                kWaypointArrivalRadiusNominalM);
        }
    }

    *prev_health_mode = health_mode;
}

void apply_health_contingency_air(
    SensorsSimulation *sensors,
    StaticWaypointBuffer *route,
    NavHealthMode health_mode,
    bool safe_stop_active,
    NavHealthMode *prev_health_mode)
{
    (void)prev_health_mode;
    apply_health_contingency_air_state(sensors, route, health_mode, safe_stop_active);
}

void apply_health_contingency_submarine(
    StaticWaypointBuffer *route,
    NavHealthMode health_mode,
    bool safe_stop_active,
    NavHealthMode *prev_health_mode)
{
    (void)prev_health_mode;
    (void)safe_stop_active;

    if (route == NULL) {
        return;
    }

    const uint32_t arrival_radius_m = contingency_arrival_radius_m(health_mode);

    waypoint_route_set_arrival_radius(route, arrival_radius_m);
}

void navigation_cortex_tick(
    NavigationCortexState *cortex_state,
    DeadReckoningFilter *nav,
    SystemHealthMonitor *health,
    bool gps_fix_valid,
    bool skip_diagnostic_update,
    uint8_t filter_quality_u8,
    uint8_t worst_bsp_bus,
    uint32_t timestamp_ms,
    NavigationDecision *decision)
{
    if (cortex_state == NULL || nav == NULL || health == NULL || decision == NULL) {
        return;
    }

    health->shutdown_latched = power_manager_is_shutdown_latched();

    NavigationCortexInput input{};
    input.filter = nav;
    input.monitor = health;
    input.nav_state = &nav->state;
    input.gps_fix_valid = gps_fix_valid;
    input.skip_diagnostic_update = skip_diagnostic_update;
    input.filter_quality = filter_quality_u8;
    input.bsp_bus_status = worst_bsp_bus;

    navigation_cortex_step(cortex_state, &input, decision);
    TelemetryInterface *telemetry = TelemetryInterface::active();
    if (telemetry != NULL) {
        telemetry->emit_events(timestamp_ms, decision);
    }
}

Vector3D square_vehicle_start(Vector3D corner_origin)
{
    return vector3d_make(
        corner_origin.x,
        corner_origin.y - kSquareApproachLonOffsetDeg,
        corner_origin.z);
}

void init_mission_3d_route(
    StaticWaypointBuffer *route,
    Vector3D start,
    NavDomain domain)
{
    if (route == NULL) {
        return;
    }

    waypoint_buffer_init(route);

    const uint32_t arrival_radius_m = 20U;

    const Waypoint wp0 = waypoint_make(
        "M0",
        vector3d_make(start.x, start.y, 12.0f),
        domain,
        arrival_radius_m,
        kWaypointCornerSpeedMps);
    const Waypoint wp1 = waypoint_make(
        "M1",
        vector3d_make(start.x, start.y + kSquareLegStepDeg, 18.0f),
        domain,
        arrival_radius_m,
        kWaypointStraightSpeedMps);
    const Waypoint wp2 = waypoint_make(
        "M2",
        vector3d_make(start.x, start.y + (2.0f * kSquareLegStepDeg), 22.0f),
        domain,
        arrival_radius_m,
        kWaypointStraightSpeedMps);
    const Waypoint wp3 = waypoint_make(
        "M3",
        vector3d_make(start.x + kSquareLegStepDeg, start.y + (2.0f * kSquareLegStepDeg), 15.0f),
        domain,
        arrival_radius_m,
        kWaypointCornerSpeedMps);

    waypoint_buffer_push(route, wp0);
    waypoint_buffer_push(route, wp1);
    waypoint_buffer_push(route, wp2);
    waypoint_buffer_push(route, wp3);

    std::printf(
        "MISION: ruta 3D — %zu waypoints | look-ahead=%.1f m | tramos N/N/E | alt 12/18/22/15 m\n",
        kMission3dWaypointCount,
        kGuidanceLookAheadM);
}

void init_square_waypoint_route(
    StaticWaypointBuffer *route,
    Vector3D start,
    NavDomain domain)
{
    if (route == NULL) {
        return;
    }

    waypoint_buffer_init(route);

    const uint32_t arrival_radius_m = static_cast<uint32_t>(kSquareArrivalRadiusM);

    /*
     * Circuito cuadrado (vista desde el cielo, lat = norte, lon = este):
     *   WP0 -----> WP1
     *   ^            |
     *   |            v
     *   WP3 <----- WP2
     */
    const Waypoint wp0 = waypoint_make(
        "SQ0",
        start,
        domain,
        arrival_radius_m,
        kWaypointCornerSpeedMps);
    const Waypoint wp1 = waypoint_make(
        "SQ1",
        vector3d_make(start.x, start.y + kSquareLegStepDeg, start.z),
        domain,
        arrival_radius_m,
        kWaypointStraightSpeedMps);
    const Waypoint wp2 = waypoint_make(
        "SQ2",
        vector3d_make(start.x + kSquareLegStepDeg, start.y + kSquareLegStepDeg, start.z),
        domain,
        arrival_radius_m,
        kWaypointCornerSpeedMps);
    const Waypoint wp3 = waypoint_make(
        "SQ3",
        vector3d_make(start.x + kSquareLegStepDeg, start.y, start.z),
        domain,
        arrival_radius_m,
        kWaypointCornerSpeedMps);

    waypoint_buffer_push(route, wp0);
    waypoint_buffer_push(route, wp1);
    waypoint_buffer_push(route, wp2);
    waypoint_buffer_push(route, wp3);

    std::printf(
        "GUIADO: circuito cuadrado — %zu waypoints | look-ahead=%.1f m | radio aceptacion=%.1f m\n",
        kSquareWaypointCount,
        kGuidanceLookAheadM,
        kSquareArrivalRadiusM);
}

bool waypoint_buffer_at(
    const StaticWaypointBuffer *buffer,
    size_t index,
    Waypoint *out)
{
    if (buffer == NULL || out == NULL || index >= buffer->count) {
        return false;
    }

    *out = buffer->items[(buffer->head + index) % NAVICORE_MAX_WAYPOINTS];
    return true;
}

GuidanceErrors guidance_telemetry_errors(
    const StaticWaypointBuffer *route,
    size_t active_index,
    Vector3D position)
{
    return guidance_compute_leg_errors(route, active_index, position);
}

float heading_delta_deg(float from_deg, float to_deg)
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

float deg_to_rad(float deg)
{
    return deg * (static_cast<float>(M_PI) / 180.0f);
}

void closed_loop_pid_plant_init(ClosedLoopPidPlant *plant)
{
    if (plant == NULL) {
        return;
    }

    plant->speed_pid.init(0.85f, 0.18f, 0.10f, -3.5f, 3.5f);
    plant->yaw_pid.init(1.75f, 0.04f, 0.28f, -1.2f, 1.2f);
    plant->altitude_pid.init(0.65f, 0.14f, 0.09f, -2.5f, 2.5f);
    plant->telemetry = TelemetryPidSnapshot{};
}

float closed_loop_yaw_measurement_rad(const NavState *nav_state)
{
    if (nav_state == NULL) {
        return 0.0f;
    }

    const float speed_mps = navstate_speed_mps(nav_state);
    constexpr float kMinSpeedForCourseRadps = 0.35f;
    if (speed_mps > kMinSpeedForCourseRadps) {
        return atan2f(nav_state->velocity.y, nav_state->velocity.x);
    }

    return deg_to_rad(nav_state->heading_deg);
}

void apply_closed_loop_pid(
    ClosedLoopPidPlant *plant,
    SensorsSimulation *sensors,
    const NavState *nav_state,
    const GuidanceCommands *commands,
    float dt_s)
{
    if (plant == NULL || sensors == NULL || nav_state == NULL || commands == NULL || dt_s <= 0.0f) {
        return;
    }

    const float speed_meas = navstate_speed_mps(nav_state);
    const float yaw_meas = closed_loop_yaw_measurement_rad(nav_state);
    const float climb_meas = nav_state->velocity.z;

    const float forward_accel = plant->speed_pid.update(commands->desired_speed, speed_meas, dt_s);
    const float yaw_rate_cmd = plant->yaw_pid.update_yaw(commands->desired_heading, yaw_meas, dt_s);
    const float vertical_accel = plant->altitude_pid.update(commands->desired_climb, climb_meas, dt_s);

    sensors_simulation_apply_actuator_forces(
        sensors,
        forward_accel,
        yaw_rate_cmd,
        vertical_accel,
        dt_s);

    plant->telemetry.des_speed_mps = commands->desired_speed;
    plant->telemetry.des_heading_rad = commands->desired_heading;
    plant->telemetry.des_climb_mps = commands->desired_climb;
    plant->telemetry.pid_speed_out = forward_accel;
    plant->telemetry.pid_yaw_out = yaw_rate_cmd;
    plant->telemetry.pid_alt_out = vertical_accel;
    plant->telemetry.speed_meas_mps = speed_meas;
    plant->telemetry.yaw_meas_rad = yaw_meas;
    plant->telemetry.climb_meas_mps = climb_meas;
    plant->telemetry.forward_accel_mps2 = forward_accel;
    plant->telemetry.yaw_accel_radps2 = 0.0f;
    plant->telemetry.yaw_rate_cmd_radps = yaw_rate_cmd;
    plant->telemetry.vertical_accel_mps2 = vertical_accel;
    plant->telemetry.yaw_rate_radps = sensors->gps.yaw_rate_radps;
    plant->telemetry.active = true;
}

void apply_guidance_commands(
    SensorsSimulation *sensors,
    const GuidanceCommands *commands,
    const NavState *nav_state,
    float dt_s,
    float *synthetic_course_deg,
    ClosedLoopPidPlant *pid_plant)
{
    if (sensors == NULL || commands == NULL || nav_state == NULL || synthetic_course_deg == NULL) {
        return;
    }

    if (pid_plant != NULL) {
        apply_closed_loop_pid(pid_plant, sensors, nav_state, commands, dt_s);
        *synthetic_course_deg = navstate_normalize_heading(sensors->gps.course_deg);
        return;
    }

    sensors_simulation_apply_guidance_control(
        sensors,
        commands->desired_heading,
        commands->desired_speed,
        commands->desired_climb,
        nav_state->heading_deg,
        dt_s);

    const float heading_deg = commands->desired_heading * (180.0f / static_cast<float>(M_PI));
    *synthetic_course_deg = navstate_normalize_heading(heading_deg);
}

void guidance3d_step(
    Guidance3D *guidance,
    const NavState *nav_state,
    StaticWaypointBuffer *route,
    size_t *active_waypoint_index,
    uint32_t timestamp_ms,
    SensorsSimulation *sensors,
    float *synthetic_course_deg,
    float dt_s,
    bool enable_control,
    GuidanceErrors *telemetry_out)
{
    if (guidance == NULL || nav_state == NULL || route == NULL
        || active_waypoint_index == NULL || telemetry_out == NULL) {
        return;
    }

    if (*active_waypoint_index >= route->count) {
        *telemetry_out = GuidanceErrors{};
        return;
    }

    const GuidanceOutput output = guidance->compute(
        *nav_state,
        *route,
        *active_waypoint_index);

    *telemetry_out = output.track_errors;

    if (!output.valid) {
        return;
    }

    if (enable_control) {
        apply_guidance_commands(
            sensors,
            &output.commands,
            nav_state,
            dt_s,
            synthetic_course_deg,
            NULL);
    }

    if (!output.waypoint_completed) {
        return;
    }

    Waypoint completed_wp{};
    if (!waypoint_buffer_at(route, *active_waypoint_index + 1U, &completed_wp)) {
        if (!waypoint_buffer_at(route, *active_waypoint_index, &completed_wp)) {
            return;
        }
    }

    const size_t completed_index = *active_waypoint_index;
    const float t_s = static_cast<float>(timestamp_ms) * 0.001f;

    if (*active_waypoint_index + 1U < route->count) {
        ++(*active_waypoint_index);
        std::printf(
            "GUIADO: leg WP%zu completado @ t=%.2f s -> indice %zu\n",
            completed_index,
            t_s,
            *active_waypoint_index);
    } else {
        ++(*active_waypoint_index);
        std::printf(
            "GUIADO: ruta finalizada @ t=%.2f s (%zu/%zu)\n",
            t_s,
            route->count,
            route->count);
    }
}

void mission_guidance_step(
    Guidance3D *guidance,
    MissionController *mission,
    const NavState *nav_state,
    bool gps_fix_valid,
    uint8_t satellites,
    SensorsSimulation *sensors,
    uint32_t timestamp_ms,
    float dt_s,
    GuidanceErrors *telemetry_out,
    RuntimeHealth *runtime_health,
    MissionGuidanceSnapshot *snapshot_out,
    ClosedLoopPidPlant *pid_plant,
    bool ekf_calibrated,
    float cov_pos_n_m2,
    float cov_pos_e_m2,
    float cov_pos_d_m2,
    float gnss_nis,
    bool gnss_nis_rejected)
{
    if (guidance == NULL || mission == NULL || nav_state == NULL
        || sensors == NULL || telemetry_out == NULL || runtime_health == NULL) {
        return;
    }

    MissionInput input{};
    MissionOutput output{};

    if (snapshot_out != NULL) {
        snapshot_out->guidance_valid = false;
        snapshot_out->return_home_active = false;
        snapshot_out->active_waypoint_index = mission->active_waypoint_index;
    }

    input.dt_s = dt_s;
    input.nav_state = nav_state;
    input.runtime_health = runtime_health;
    input.guidance = guidance;
    input.gps_fix_valid = gps_fix_valid;
    input.satellites = satellites;
    input.estimate_quality = nav_state->confidence.estimate_quality;
    input.ekf_calibrated = ekf_calibrated;
    input.cov_pos_n_m2 = cov_pos_n_m2;
    input.cov_pos_e_m2 = cov_pos_e_m2;
    input.cov_pos_d_m2 = cov_pos_d_m2;
    input.gnss_nis = gnss_nis;
    input.gnss_nis_rejected = gnss_nis_rejected;
    input.route_loaded = mission->route.count >= 2U;

    static uint32_t ready_ticks = 0U;
    static bool mission_auto_arm_sent = false;
    if (mission_state(mission) == MISSION_STATE_READY && !mission_auto_arm_sent) {
        ++ready_ticks;
        if (ready_ticks >= kMissionAutoStartReadyTicks) {
            input.arm_system = true;
            mission_auto_arm_sent = true;
        }
    } else {
        ready_ticks = 0U;
    }

    input.arm_system = input.arm_system || mission->armed;

    const MissionState prev_state = mission_state(mission);
    mission_update(mission, &input, &output);

    if (snapshot_out != NULL) {
        snapshot_out->return_home_active = output.return_home_active;
        snapshot_out->active_waypoint_index = output.active_waypoint_index;
    }

    if (output.safe_mode) {
        const GuidanceCommands *commands = output.safe_commands_active
            ? &output.safe_commands
            : nullptr;
        GuidanceCommands zero{};
        const GuidanceCommands *applied = (commands != NULL) ? commands : &zero;
        float hold_heading_deg = nav_state->heading_deg;
        apply_guidance_commands(
            sensors,
            applied,
            nav_state,
            dt_s,
            &hold_heading_deg,
            pid_plant);
        *telemetry_out = GuidanceErrors{};
        if (prev_state != MISSION_STATE_SAFE_MODE) {
            std::printf(
                "MISION: SAFE_MODE (%s) @ t=%.2f s\n",
                mission_safe_cause_name(output.safe_cause),
                static_cast<float>(timestamp_ms) * 0.001f);
        }
        return;
    }

    if (!output.guidance_active || !output.guidance_valid) {
        *telemetry_out = GuidanceErrors{};
        return;
    }

    *telemetry_out = output.guidance.track_errors;

    if (snapshot_out != NULL) {
        snapshot_out->commands = output.guidance.commands;
        snapshot_out->guidance_valid = true;
    }

    float synthetic_course_deg = nav_state->heading_deg;
    apply_guidance_commands(
        sensors,
        &output.guidance.commands,
        nav_state,
        dt_s,
        &synthetic_course_deg,
        pid_plant);

    if ((timestamp_ms % 1000U) == 0U) {
        std::printf(
            "GUIADO: hdg=%.1f deg speed=%.2f m/s climb=%.2f m/s | xt=%.2f m along=%.1f m | WP%zu\n",
            synthetic_course_deg,
            output.guidance.commands.desired_speed,
            output.guidance.commands.desired_climb,
            output.guidance.track_errors.cross_track_signed_m,
            output.guidance.track_errors.along_track_m,
            output.active_waypoint_index);
    }

    if (output.guidance.route_completed && prev_state == MISSION_STATE_NAVIGATE) {
        std::printf(
            "MISION: ruta completada @ t=%.2f s -> RETURN_HOME\n",
            static_cast<float>(timestamp_ms) * 0.001f);
    }
}

void print_scenario_banner(SensorScenario scenario)
{
    std::printf("\n");
    std::printf("================================================================\n");

    switch (scenario) {
    case SCENARIO_CLEAN:
        std::printf(" ESCENARIO: Limpio (sin anomalias)\n");
        std::printf("  Velocidad: %.0f m/s | GPS e IMU sin inyeccion de fallos\n", kCruiseSpeedMps);
        break;
    case SCENARIO_GPS_LOSS:
        std::printf(" ESCENARIO: Perdida de GPS (Aire/Tierra)\n");
        std::printf(
            "  Velocidad: %.0f m/s | GPS invalido desde tick %u\n",
            kCruiseSpeedMps,
            SENSOR_FAULT_GPS_LOSS_START_TICK_DEFAULT);
        break;
    case SCENARIO_IMU_DRIFT:
        std::printf(" ESCENARIO: Deriva IMU acumulativa\n");
        std::printf("  Velocidad: %.0f m/s | Bias creciente en accel/gyro\n", kCruiseSpeedMps);
        break;
    case SCENARIO_ODOM_LOSS:
        std::printf(" ESCENARIO: Perdida de odometria (Aire/Tierra)\n");
        std::printf(
            "  Velocidad: %.0f m/s | GPS invalido y odometria=0 desde t=%.0f s\n",
            kCruiseSpeedMps,
            static_cast<float>(SENSOR_FAULT_ODOM_LOSS_START_TICK_DEFAULT * kStepMs) * 0.001f);
        break;
    default:
        std::printf(" ESCENARIO: Desconocido\n");
        break;
    }

    std::printf("================================================================\n");
    std::printf(
        "%-8s %-16s %-8s %-6s %-10s %s\n",
        "t[s]",
        "mode",
        "quality",
        "sats",
        "fix_age",
        "notas");
    std::printf(
        "----------------------------------------------------------------\n");
}

void high_demand_apply_artificial_wcet_delay(void)
{
#ifdef _WIN32
    Sleep(kHighDemandWcetArtificialDelayMs);
#else
    usleep(kHighDemandWcetArtificialDelayMs * 1000U);
#endif
}

void high_demand_build_radio_packet(
    size_t seq_index,
    const DeadReckoningFilter *nav,
    const StaticWaypointBuffer *route,
    RadioCommandPacket *packet_out)
{
    if (nav == NULL || route == NULL || packet_out == NULL) {
        return;
    }

    packet_out->magic = RADIO_CMD_MAGIC;
    packet_out->command_type = static_cast<uint8_t>(CMD_ADD_WAYPOINT);
    packet_out->sequence = static_cast<uint8_t>(seq_index & 0xFFU);

    const bool geom_violation_probe = ((seq_index % 25U) == 24U);
    const float lon_step_deg =
        geom_violation_probe ? kHighDemandGeomViolationStepDeg : kHighDemandRadioStepDeg;

    float base_lon_deg = nav->state.position.y;
    if (route->count > 0U) {
        const size_t last_index =
            (route->head + route->count - 1U) % NAVICORE_MAX_WAYPOINTS;
        base_lon_deg = route->items[last_index].position.y;
    }

    packet_out->pos_x = nav->state.position.x;
    packet_out->pos_y = base_lon_deg + lon_step_deg;
    packet_out->param = nav->state.position.z;
    packet_out->checksum = command_ingestor_compute_checksum(packet_out);
}

void high_demand_enqueue_radio_burst(
    uint32_t t_ms,
    const DeadReckoningFilter *nav,
    StaticWaypointBuffer *route)
{
    if (nav == NULL || route == NULL) {
        return;
    }

    StaticWaypointBuffer sim_route = *route;
    float sim_cruise_mps = kCruiseSpeedMps;
    size_t enqueued = 0U;

    std::printf(
        ">>> t=%.1fs: encolando rafaga x%zu RadioCommandPacket | max %u/tick\n",
        static_cast<float>(t_ms) * 0.001f,
        kHighDemandRadioBurstCount,
        NAVICORE_RADIO_MAX_PACKETS_PER_TICK);

    for (size_t i = 0U; i < kHighDemandRadioBurstCount; ++i) {
        RadioCommandPacket packet{};
        high_demand_build_radio_packet(i, nav, &sim_route, &packet);

        (void)command_ingestor_hw_enqueue(&packet);
        enqueued++;
        (void)command_ingestor_parse(&packet, &sim_route, &sim_cruise_mps, NULL);
    }

    std::printf(
        ">>> t=%.1fs: %zu paquetes en buffer HW (pendientes=%u dropped=%u)\n",
        static_cast<float>(t_ms) * 0.001f,
        enqueued,
        command_ingestor_hw_pending_count(),
        command_ingestor_hw_dropped_packets());
}

bool high_demand_apply_wcet_stress(SystemHealthMonitor *health, uint32_t t_ms)
{
    if (health == NULL) {
        return false;
    }

    const uint8_t health_before = health->health_score;
    const NavHealthMode mode_before = health->mode;

    time_guard_start();
    high_demand_apply_artificial_wcet_delay();
    uint32_t execution_ticks = time_guard_stop();
    if (execution_ticks <= TIME_GUARD_DEFAULT_MAX_TICKS) {
        execution_ticks = TIME_GUARD_DEFAULT_MAX_TICKS + 1U;
    }
    const bool within_budget = time_guard_validate(
        execution_ticks,
        TIME_GUARD_DEFAULT_MAX_TICKS,
        health);

    if (!within_budget) {
        std::printf(
            ">>> t=%.1fs: WCET violado | ticks=%u max=%u | penalizacion -%u | "
            "health %u -> %u | mode %s -> %s\n",
            static_cast<float>(t_ms) * 0.001f,
            execution_ticks,
            TIME_GUARD_DEFAULT_MAX_TICKS,
            TIME_GUARD_WCET_PENALTY,
            static_cast<unsigned>(health_before),
            static_cast<unsigned>(health->health_score),
            health_mode_name(mode_before),
            health_mode_name(health->mode));
    }

    return within_budget;
}

void run_high_demand_stress_test_scenario(TelemetryInterface *telemetry)
{
    const Vector3D corner_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const Vector3D origin = square_vehicle_start(corner_origin);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, SCENARIO_CLEAN, origin, kCruiseSpeedMps, kCruiseCourseDeg, 17U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    SystemHealthMonitor health{};
    power_manager_init();
    time_guard_init();
    command_ingestor_init();

    bool safe_stop_active = false;
    float safe_stop_heading_deg = kCruiseCourseDeg;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;
    NavigationCortexState cortex_state{};
    PowerState prev_power_state = POWER_PERFORMANCE;
    navigation_cortex_init(&cortex_state);
    bool radio_burst_enqueued = false;
    bool radio_burst_complete_logged = false;
    size_t radio_ingest_ok = 0U;
    size_t radio_ingest_fail = 0U;
    size_t radio_geometry_reject = 0U;

    StaticWaypointBuffer route{};
    Guidance3D pursuit_guidance(kGuidanceLookAheadM);
    size_t active_waypoint_index = 0U;
    init_square_waypoint_route(&route, corner_origin, NAVICORE_DOMAIN_AIR);

    float cruise_speed_mps = kCruiseSpeedMps;
    float synthetic_course_deg = kCruiseCourseDeg;
    const float dt_s = static_cast<float>(kStepMs) * 0.001f;

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: HIGH_DEMAND_STRESS_TEST\n");
    std::printf("  t=5s  rafaga x100 RadioCommandPacket (max %u/tick, parser + geometry guard)\n",
                NAVICORE_RADIO_MAX_PACKETS_PER_TICK);
    std::printf("  t=10s retraso WCET sostenido (-%u pts/tick) -> DEGRADED 8 m/s\n",
                TIME_GUARD_WCET_PENALTY);
    std::printf("  CRITICAL -> SAFE STOP -> POWER_SAFE_SHUTDOWN latched\n");
    std::printf("================================================================\n");

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.health = &health;
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_HIGH_DEMAND;
    bindings.temperature_c = kAmbientTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= kHighDemandDurationMs; t_ms += kStepMs) {
        apply_health_contingency_air(
            &sensors,
            &route,
            health.mode,
            safe_stop_active,
            &prev_health_mode);

        if (safe_stop_active) {
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        }

        ImuSample imu{};
        GpsSample gps{};

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        const uint32_t tick_index = t_ms / kStepMs;
        const uint8_t scenario_satellites = gps.fix_valid ? gps.satellites : 0U;

        dead_reckoning_update_imu(&nav, &imu, &health);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps, &health);
        }

        const bool odom_fault_active = dead_reckoning_has_odom_fault(&nav);
        const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
            nav.state.confidence.estimate_quality);
        const uint8_t worst_bsp_bus = pc_simulate_worst_bsp_bus_status(
            odom_fault_active,
            nav.state.confidence.estimate_quality);

        /*
         * Durante el estres WCET (t >= 10 s) no se llama a diagnostic_update para
         * que las penalizaciones de time_guard (-40) se acumulen de forma determinista.
         */
        NavigationDecision cortex_decision{};
        navigation_cortex_tick(
            &cortex_state,
            &nav,
            &health,
            gps.fix_valid,
            t_ms >= kHighDemandWcetStressMs,
            filter_quality_u8,
            worst_bsp_bus,
            t_ms,
            &cortex_decision);

        if (t_ms == kHighDemandRadioBurstMs && !radio_burst_enqueued) {
            high_demand_enqueue_radio_burst(t_ms, &nav, &route);
            radio_burst_enqueued = true;
        }

        CommandIngestContext ingest_ctx{
            &route,
            &cruise_speed_mps,
            &health,
        };
        CommandIngestTickStats ingest_stats{};
        (void)command_ingestor_process_queue(&ingest_ctx, &ingest_stats);

        radio_ingest_ok += ingest_stats.ingest_ok;
        radio_ingest_fail += ingest_stats.ingest_fail;
        radio_geometry_reject += ingest_stats.geometry_reject;

        if (radio_burst_enqueued &&
            !radio_burst_complete_logged &&
            !command_ingestor_hw_has_data()) {
            std::printf(
                ">>> t=%.1fs: rafaga drenada | ingest OK=%zu FAIL=%zu | "
                "geometry_reject=%zu | health=%u | WP=%zu\n",
                static_cast<float>(t_ms) * 0.001f,
                radio_ingest_ok,
                radio_ingest_fail,
                radio_geometry_reject,
                static_cast<unsigned>(health.health_score),
                route.count);
            radio_burst_complete_logged = true;
        }

        if (t_ms >= kHighDemandWcetStressMs) {
            if (t_ms == kHighDemandWcetStressMs) {
                std::printf(
                    ">>> t=10.0s (tick %u): inicio estres WCET sostenido (Time Guard)\n",
                    tick_index);
            }
            (void)high_demand_apply_wcet_stress(&health, t_ms);
            apply_health_contingency_air_state(
                &sensors,
                &route,
                health.mode,
                safe_stop_active);
        }

        log_health_contingency_transition(health.mode, &prev_health_mode, false);

        if (cortex_decision.requires_safe_stop) {
            if (!safe_stop_active) {
                safe_stop_active = true;
                safe_stop_heading_deg = nav.state.heading_deg;
                std::printf(
                    ">>> SAFE STOP en tick %u (t=%.1fs) | HEALTH_CRITICAL score=%u\n",
                    tick_index,
                    static_cast<float>(t_ms) * 0.001f,
                    static_cast<unsigned>(health.health_score));
            }
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        }

        const float speed_mps = navstate_speed_mps(&nav.state);
        const bool vehicle_stopped = speed_mps < kVehicleStoppedSpeedMps;
        power_manager_update(static_cast<SystemHealthMode>(health.mode), vehicle_stopped);
        health.shutdown_latched = power_manager_is_shutdown_latched();

        const PowerState current_power = power_manager_get_state();
        if (current_power == POWER_CONSERVATION && prev_power_state != POWER_CONSERVATION) {
            TelemetryInterface *telemetry = TelemetryInterface::active();
            if (telemetry != NULL) {
                telemetry->emit_event(t_ms, NAV_EVENT_POWER_CONSERVATION, health.health_score);
            }
        }
        prev_power_state = current_power;

        if (power_manager_get_state() == POWER_SAFE_SHUTDOWN &&
            power_manager_is_shutdown_latched() &&
            vehicle_stopped) {
            if ((t_ms % 1000U) == 0U ||
                (t_ms >= kHighDemandWcetStressMs && t_ms <= kHighDemandWcetStressMs + 3000U)) {
                std::printf(
                    ">>> t=%.1fs: POWER_SAFE_SHUTDOWN latched | speed=%.3f m/s | periph_mask=0x%08X\n",
                    static_cast<float>(t_ms) * 0.001f,
                    speed_mps,
                    power_manager_get_disabled_periph_mask());
            }
        }

        GuidanceErrors guidance{};
        guidance3d_step(
            &pursuit_guidance,
            &nav.state,
            &route,
            &active_waypoint_index,
            t_ms,
            &sensors,
            &synthetic_course_deg,
            dt_s,
            !safe_stop_active,
            &guidance);

        bindings.guidance = &guidance;
        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, MISSION_STATE_INIT);

        if ((t_ms % 1000U) == 0U) {
            std::printf(
                "[t=%5.1fs] mode=%-14s quality=%.3f health=%s(%u) power=%s shutdown=%u "
                "speed=%.2f wp_idx=%zu/%zu bsp=%u wcet_err=%u geom_err=%u\n",
                static_cast<float>(t_ms) * 0.001f,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
                health_mode_name(health.mode),
                static_cast<unsigned>(health.health_score),
                power_state_name(power_manager_get_state()),
                power_manager_is_shutdown_latched() ? 1U : 0U,
                speed_mps,
                active_waypoint_index,
                route.count,
                static_cast<unsigned>(worst_bsp_bus),
                static_cast<unsigned>(health.last_time_guard_error),
                static_cast<unsigned>(health.last_geometry_error));
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [%s]: health=%s(%u) power=%s shutdown=%u wp=%zu | "
        "radio OK=%zu FAIL=%zu geom_reject=%zu\n",
        kHighDemandScenarioName,
        health_mode_name(health.mode),
        static_cast<unsigned>(health.health_score),
        power_state_name(power_manager_get_state()),
        power_manager_is_shutdown_latched() ? 1U : 0U,
        route.count,
        radio_ingest_ok,
        radio_ingest_fail,
        radio_geometry_reject);
}

void run_fault_injection_scenario(TelemetryInterface *telemetry)
{
    const Vector3D corner_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const Vector3D origin = square_vehicle_start(corner_origin);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, SCENARIO_CLEAN, origin, kCruiseSpeedMps, kCruiseCourseDeg, 17U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    SystemHealthMonitor health{};
    power_manager_init();

    bool safe_stop_active = false;
    float safe_stop_heading_deg = kCruiseCourseDeg;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;
    NavigationCortexState cortex_state{};
    PowerState prev_power_state = POWER_PERFORMANCE;
    navigation_cortex_init(&cortex_state);
    bool radio_cmd_injected = false;
    size_t waypoint_count_before_radio = 0U;

    StaticWaypointBuffer route{};
    Guidance3D pursuit_guidance(kGuidanceLookAheadM);
    size_t active_waypoint_index = 0U;
    init_square_waypoint_route(&route, corner_origin, NAVICORE_DOMAIN_AIR);

    float cruise_speed_mps = kCruiseSpeedMps;
    float synthetic_course_deg = kCruiseCourseDeg;
    const float dt_s = static_cast<float>(kStepMs) * 0.001f;

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: Inyeccion automatizada de fallos (timeline)\n");
    std::printf("  t=5s  GPS -> DEGRADED | t=10s CMD_ADD_WAYPOINT | t=15s SPI -> CRITICAL\n");
    std::printf("  Parada segura -> POWER_SAFE_SHUTDOWN al detenerse\n");
    std::printf("================================================================\n");

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.health = &health;
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_FAULT_INJECTION;
    bindings.temperature_c = kAmbientTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= kFaultInjectDurationMs; t_ms += kStepMs) {
        apply_health_contingency_air(
            &sensors,
            &route,
            health.mode,
            safe_stop_active,
            &prev_health_mode);

        if (safe_stop_active) {
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        }

        ImuSample imu{};
        GpsSample gps{};

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        if (t_ms >= kFaultInjectGpsLossMs) {
            gps.fix_valid = false;
            gps.satellites = 0U;
        }

        const uint32_t tick_index = t_ms / kStepMs;
        const uint8_t scenario_satellites = gps.fix_valid ? gps.satellites : 0U;

        if (t_ms == kFaultInjectGpsLossMs) {
            std::printf(
                ">>> t=5.0s (tick %u): FALLO GPS forzado -> objetivo salud DEGRADED\n",
                tick_index);
        }

        dead_reckoning_update_imu(&nav, &imu, &health);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps, &health);
        }

        if (t_ms == kFaultInjectRadioCmdMs && !radio_cmd_injected) {
            waypoint_count_before_radio = route.count;

            RadioCommandPacket radio_cmd{};
            radio_cmd.magic = RADIO_CMD_MAGIC;
            radio_cmd.command_type = static_cast<uint8_t>(CMD_ADD_WAYPOINT);
            radio_cmd.sequence = 10U;
            radio_cmd.pos_x = nav.state.position.x;
            radio_cmd.pos_y = nav.state.position.y + kWaypointLonStepDeg;
            radio_cmd.param = nav.state.position.z;
            radio_cmd.checksum = command_ingestor_compute_checksum(&radio_cmd);

            const bool ingest_ok = command_ingestor_parse(
                &radio_cmd,
                &route,
                &cruise_speed_mps,
                &health);
            radio_cmd_injected = true;

            std::printf(
                ">>> t=10.0s (tick %u): RadioCommandPacket CMD_ADD_WAYPOINT | ingest=%s | WP count %zu -> %zu\n",
                tick_index,
                ingest_ok ? "OK" : "FAIL",
                waypoint_count_before_radio,
                route.count);
        }

        if (t_ms == kFaultInjectSpiTimeoutMs) {
            std::printf(
                ">>> t=15.0s (tick %u): Violacion SPI (TIMEOUT) -> objetivo salud CRITICAL\n",
                tick_index);
        }

        const bool odom_fault_active = dead_reckoning_has_odom_fault(&nav);
        const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
            nav.state.confidence.estimate_quality);
        const uint8_t worst_bsp_bus = fault_injection_worst_bsp_bus(
            t_ms,
            odom_fault_active,
            nav.state.confidence.estimate_quality);

        NavigationDecision cortex_decision{};
        navigation_cortex_tick(
            &cortex_state,
            &nav,
            &health,
            gps.fix_valid,
            false,
            filter_quality_u8,
            worst_bsp_bus,
            t_ms,
            &cortex_decision);
        log_health_contingency_transition(health.mode, &prev_health_mode, false);

        if (cortex_decision.requires_safe_stop && t_ms >= kFaultInjectSpiTimeoutMs) {
            if (!safe_stop_active) {
                safe_stop_active = true;
                safe_stop_heading_deg = nav.state.heading_deg;
                std::printf(
                    ">>> SAFE STOP en tick %u (t=%.1fs) | HEALTH_CRITICAL score=%u\n",
                    tick_index,
                    static_cast<float>(t_ms) * 0.001f,
                    static_cast<unsigned>(health.health_score));
            }
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        }

        const float speed_mps = navstate_speed_mps(&nav.state);
        const bool vehicle_stopped = speed_mps < kVehicleStoppedSpeedMps;
        power_manager_update(static_cast<SystemHealthMode>(health.mode), vehicle_stopped);
        health.shutdown_latched = power_manager_is_shutdown_latched();

        const PowerState current_power = power_manager_get_state();
        if (current_power == POWER_CONSERVATION && prev_power_state != POWER_CONSERVATION) {
            TelemetryInterface *telemetry = TelemetryInterface::active();
            if (telemetry != NULL) {
                telemetry->emit_event(t_ms, NAV_EVENT_POWER_CONSERVATION, health.health_score);
            }
        }
        prev_power_state = current_power;

        if (power_manager_get_state() == POWER_SAFE_SHUTDOWN &&
            power_manager_is_shutdown_latched() &&
            t_ms >= kFaultInjectSpiTimeoutMs &&
            vehicle_stopped) {
            if ((t_ms % 1000U) == 0U || t_ms == kFaultInjectSpiTimeoutMs + 5000U) {
                std::printf(
                    ">>> t=%.1fs: POWER_SAFE_SHUTDOWN latched | speed=%.3f m/s | periph_mask=0x%08X\n",
                    static_cast<float>(t_ms) * 0.001f,
                    speed_mps,
                    power_manager_get_disabled_periph_mask());
            }
        }

        GuidanceErrors guidance{};
        guidance3d_step(
            &pursuit_guidance,
            &nav.state,
            &route,
            &active_waypoint_index,
            t_ms,
            &sensors,
            &synthetic_course_deg,
            dt_s,
            !safe_stop_active,
            &guidance);

        bindings.guidance = &guidance;
        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, MISSION_STATE_INIT);

        if ((t_ms % 1000U) == 0U) {
            std::printf(
                "[t=%5.1fs] mode=%-14s quality=%.3f health=%s(%u) power=%s shutdown=%u speed=%.2f wp_idx=%zu/%zu bsp=%u\n",
                static_cast<float>(t_ms) * 0.001f,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
                health_mode_name(health.mode),
                static_cast<unsigned>(health.health_score),
                power_state_name(power_manager_get_state()),
                power_manager_is_shutdown_latched() ? 1U : 0U,
                speed_mps,
                active_waypoint_index,
                route.count,
                static_cast<unsigned>(worst_bsp_bus));
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [FAULT_INJECTION]: health=%s(%u) power=%s shutdown=%u wp=%zu radio=%s\n",
        health_mode_name(health.mode),
        static_cast<unsigned>(health.health_score),
        power_state_name(power_manager_get_state()),
        power_manager_is_shutdown_latched() ? 1U : 0U,
        route.count,
        radio_cmd_injected ? "injected" : "pending");
}

void run_sensor_scenario(TelemetryInterface *telemetry, SensorScenario scenario)
{
    const Vector3D corner_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const Vector3D origin = square_vehicle_start(corner_origin);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, scenario, origin, kCruiseSpeedMps, kCruiseCourseDeg, 11U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    SystemHealthMonitor health{};
    bool safe_stop_active = false;
    float safe_stop_heading_deg = kCruiseCourseDeg;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;
    NavigationCortexState cortex_state{};
    navigation_cortex_init(&cortex_state);

    StaticWaypointBuffer route{};
    Guidance3D pursuit_guidance(kGuidanceLookAheadM);
    size_t active_waypoint_index = 0U;
    init_square_waypoint_route(&route, corner_origin, NAVICORE_DOMAIN_AIR);

    print_scenario_banner(scenario);

    constexpr uint32_t kDurationMs = 20000U;
    bool prev_gps_valid = true;
    bool prev_odom_fault = false;
    float synthetic_course_deg = kCruiseCourseDeg;
    const float dt_s = static_cast<float>(kStepMs) * 0.001f;

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.health = &health;
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = telemetry_scenario_id_from_sensor(scenario);
    bindings.temperature_c = kAmbientTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= kDurationMs; t_ms += kStepMs) {
        apply_health_contingency_air(
            &sensors,
            &route,
            health.mode,
            safe_stop_active,
            &prev_health_mode);

        if (safe_stop_active) {
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        }

        ImuSample imu;
        GpsSample gps;

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        const uint32_t tick_index = sensors.faults.tick_index - 1U;
        const uint8_t scenario_satellites = gps.fix_valid ? gps.satellites : 0U;

        if (scenario == SCENARIO_GPS_LOSS || scenario == SCENARIO_ODOM_LOSS) {
            if (gps.fix_valid && !prev_gps_valid) {
                std::printf(
                    ">>> t=%.1fs (tick %u): GPS RECUPERADO (%u satelites)\n",
                    static_cast<float>(t_ms) * 0.001f,
                    tick_index,
                    scenario_satellites);
            } else if (!gps.fix_valid && prev_gps_valid) {
                std::printf(
                    ">>> t=%.1fs (tick %u): PERDIDA DE GPS (0 satelites)\n",
                    static_cast<float>(t_ms) * 0.001f,
                    tick_index);
            }
        } else if (scenario == SCENARIO_IMU_DRIFT && tick_index == 0U) {
            std::printf(">>> t=%.1fs (tick %u): inicio deriva IMU acumulativa\n",
                        static_cast<float>(t_ms) * 0.001f,
                        tick_index);
        }

        prev_gps_valid = gps.fix_valid;

        dead_reckoning_update_imu(&nav, &imu, &health);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps, &health);
        }

        if (scenario == SCENARIO_ODOM_LOSS) {
            float wheel_speed_mps = 0.0f;
            if (sensors_simulation_read_wheel_odometry(&sensors, &wheel_speed_mps)) {
                dead_reckoning_update_wheel_odometry(&nav, wheel_speed_mps, false, t_ms);
            }
        }

        const bool odom_fault_active = dead_reckoning_has_odom_fault(&nav);
        if (scenario == SCENARIO_ODOM_LOSS) {
            if (odom_fault_active && !prev_odom_fault) {
                std::printf(
                    ">>> t=%.1fs (tick %u): FALLO ODOMETRIA detectado (consistencia cinematica)\n",
                    static_cast<float>(t_ms) * 0.001f,
                    tick_index);
            } else if (!odom_fault_active && prev_odom_fault) {
                std::printf(
                    ">>> t=%.1fs (tick %u): ODOMETRIA recuperada\n",
                    static_cast<float>(t_ms) * 0.001f,
                    tick_index);
            }
        }
        prev_odom_fault = odom_fault_active;

        const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
            nav.state.confidence.estimate_quality);
        const uint8_t worst_bsp_bus = pc_simulate_worst_bsp_bus_status(
            odom_fault_active,
            nav.state.confidence.estimate_quality);
        NavigationDecision cortex_decision{};
        navigation_cortex_tick(
            &cortex_state,
            &nav,
            &health,
            gps.fix_valid,
            false,
            filter_quality_u8,
            worst_bsp_bus,
            t_ms,
            &cortex_decision);
        log_health_contingency_transition(health.mode, &prev_health_mode, false);

        if (cortex_decision.requires_safe_stop) {
            if (!safe_stop_active) {
                safe_stop_active = true;
                safe_stop_heading_deg = nav.state.heading_deg;
                std::printf(
                    ">>> SAFE STOP activado en tick %u (t=%.1fs) | HEALTH_CRITICAL score=%u | rumbo=%.1f deg\n",
                    tick_index,
                    static_cast<float>(t_ms) * 0.001f,
                    static_cast<unsigned>(health.health_score),
                    safe_stop_heading_deg);
            }
            apply_safe_stop_air(&sensors, &nav, safe_stop_heading_deg, dt_s);
        } else if (safe_stop_active && navstate_speed_mps(&nav.state) <= 0.01f) {
            safe_stop_active = false;
            std::printf(
                ">>> SAFE STOP liberado en tick %u (t=%.1fs) | reanudacion modo %s\n",
                tick_index,
                static_cast<float>(t_ms) * 0.001f,
                health_mode_name(health.mode));
            apply_health_contingency_air_state(&sensors, &route, health.mode, safe_stop_active);
        }

        GuidanceErrors guidance{};
        guidance3d_step(
            &pursuit_guidance,
            &nav.state,
            &route,
            &active_waypoint_index,
            t_ms,
            &sensors,
            &synthetic_course_deg,
            dt_s,
            !safe_stop_active,
            &guidance);

        bindings.guidance = &guidance;
        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, MISSION_STATE_INIT);

        if ((t_ms % 1000U) == 0U) {
            const char *note = "";
            if (scenario == SCENARIO_GPS_LOSS
                && tick_index == SENSOR_FAULT_GPS_LOSS_START_TICK_DEFAULT) {
                note = "<-- inicio outage (tick 3)";
            } else if (scenario == SCENARIO_ODOM_LOSS
                && tick_index == SENSOR_FAULT_ODOM_LOSS_START_TICK_DEFAULT) {
                note = "<-- inicio outage GPS + odometria=0";
            } else if (scenario == SCENARIO_IMU_DRIFT && t_ms == 5000U) {
                note = "<-- deriva visible en IMU";
            }

            const float t_s = static_cast<float>(t_ms) * 0.001f;
            std::printf(
                "[t=%5.1fs] tick=%-4u mode=%-14s quality=%.3f health=%s(%u) scenario_sats=%u fix_valid=%s fix_age=%u ms | "
                "heading=%.1f speed=%.2f m/s\n",
                t_s,
                tick_index,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
                health_mode_name(health.mode),
                static_cast<unsigned>(health.health_score),
                scenario_satellites,
                gps.fix_valid ? "yes" : "no",
                nav.state.confidence.fix_age_ms,
                nav.state.heading_deg,
                navstate_speed_mps(&nav.state));

            if (note[0] != '\0') {
                std::printf("         %s\n", note);
            }
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [%s]: mode=%s quality=%.3f health=%s(%u) odom_fault=%u safe_stop=%s\n",
        sensor_scenario_name(scenario),
        nav_mode_name(nav.state.mode),
        nav.state.confidence.estimate_quality,
        health_mode_name(health.mode),
        static_cast<unsigned>(health.health_score),
        dead_reckoning_has_odom_fault(&nav) ? 1U : 0U,
        safe_stop_active ? "yes" : "no");
}

void run_scenario_submarine(TelemetryInterface *telemetry)
{
    const Vector3D surface = vector3d_make(41.3900f, 2.1750f, kSurfacePressurePa);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, surface, NAVICORE_DOMAIN_SEA);

    SystemHealthMonitor health{};
    bool safe_stop_active = false;
    float frozen_pressure_pa = kSurfacePressurePa;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;
    NavigationCortexState cortex_state{};
    navigation_cortex_init(&cortex_state);
    float active_pressure_rate_pa_s = kSubmersionPressureRatePaS;

    StaticWaypointBuffer route{};
    size_t active_waypoint_index = 0U;
    init_square_waypoint_route(&route, surface, NAVICORE_DOMAIN_SEA);

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO 2: Inmersion Submarina\n");
    std::printf("  GPS: OFF al entrar al agua | Presion: +%.0f Pa/s\n", kSubmersionPressureRatePaS);
    std::printf("================================================================\n");
    std::printf(
        "%-8s %-16s %-14s %-14s %-10s %s\n",
        "t[s]",
        "mode",
        "pos.z [Pa]",
        "vel.z [Pa/s]",
        "expected",
        "notas");
    std::printf(
        "----------------------------------------------------------------\n");
    std::printf(">>> t=0.0s: INMERSION — GPS desactivado, dominio MAR activo\n");

    constexpr uint32_t kDurationMs = 10000U;

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.health = &health;
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_SUBMARINE;
    bindings.temperature_c = kSubmarineTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= kDurationMs; t_ms += kStepMs) {
        apply_health_contingency_submarine(
            &route,
            health.mode,
            safe_stop_active,
            &prev_health_mode);
        active_pressure_rate_pa_s = submarine_contingency_pressure_rate(health.mode);

        ImuSample imu;
        PressureSample pressure{};

        ImuSimulator imu_sim;
        imu_simulator_init(&imu_sim, 21U);
        imu_simulator_read(&imu_sim, t_ms, &imu);

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        if (safe_stop_active) {
            pressure.pressure_pa = frozen_pressure_pa;
        } else {
            pressure.pressure_pa = kSurfacePressurePa + (active_pressure_rate_pa_s * t_s);
        }
        pressure.temperature_c = 10.0f;
        pressure.timestamp_ms = t_ms;
        pressure.valid = true;

        dead_reckoning_update_imu(&nav, &imu, &health);
        dead_reckoning_update_pressure(&nav, &pressure, kSurfacePressurePa);

        const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
            nav.state.confidence.estimate_quality);
        const uint8_t worst_bsp_bus = pc_simulate_worst_bsp_bus_status(false, nav.state.confidence.estimate_quality);
        NavigationDecision cortex_decision{};
        navigation_cortex_tick(
            &cortex_state,
            &nav,
            &health,
            false,
            false,
            filter_quality_u8,
            worst_bsp_bus,
            t_ms,
            &cortex_decision);
        log_health_contingency_transition(health.mode, &prev_health_mode, true);

        if (cortex_decision.requires_safe_stop) {
            if (!safe_stop_active) {
                safe_stop_active = true;
                frozen_pressure_pa = pressure.pressure_pa;
                const uint32_t tick_index = t_ms / kStepMs;
                std::printf(
                    ">>> SAFE STOP activado en tick %u (t=%.1fs) | HEALTH_CRITICAL score=%u | inmersion detenida\n",
                    tick_index,
                    t_s,
                    static_cast<unsigned>(health.health_score));
            }
            apply_safe_stop_submarine(&nav);
            pressure.pressure_pa = frozen_pressure_pa;
            nav.state.position.z = frozen_pressure_pa;
        }

        const GuidanceErrors guidance = guidance_telemetry_errors(
            &route,
            active_waypoint_index,
            nav.state.position);

        bindings.guidance = &guidance;
        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, MISSION_STATE_INIT);

        if ((t_ms % 1000U) == 0U) {
            const float expected_pressure_pa = kSurfacePressurePa + (kSubmersionPressureRatePaS * t_s);
            const float pressure_error_pa = std::fabs(nav.state.position.z - expected_pressure_pa);
            const float velocity_error_pa_s = std::fabs(nav.state.velocity.z - kSubmersionPressureRatePaS);

            std::printf(
                "[t=%5.1fs] mode=%-14s pos.z=%12.1f vel.z=%10.1f expected=%10.1f Pa/s",
                t_s,
                nav_mode_name(nav.state.mode),
                nav.state.position.z,
                nav.state.velocity.z,
                kSubmersionPressureRatePaS);

            if (t_ms == 0U) {
                std::printf(" | vel.z N/A en primer tick");
            } else if (velocity_error_pa_s < 1.0f && pressure_error_pa < 1.0f) {
                std::printf(" | OK");
            } else {
                std::printf(" | delta_p=%.1f delta_v=%.1f", pressure_error_pa, velocity_error_pa_s);
            }
            std::printf("\n");
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado: pos.z=%.1f Pa (esperado %.1f) | vel.z=%.1f Pa/s (esperado %.1f) | health=%s(%u) safe_stop=%s\n",
        nav.state.position.z,
        kSurfacePressurePa + (kSubmersionPressureRatePaS * 10.0f),
        nav.state.velocity.z,
        kSubmersionPressureRatePaS,
        health_mode_name(health.mode),
        static_cast<unsigned>(health.health_score),
        safe_stop_active ? "yes" : "no");
}

void run_mission_clean_scenario(TelemetryInterface *telemetry)
{
    const Vector3D corner_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const Vector3D start_pos = square_vehicle_start(corner_origin);

    /*
     * SCENARIO_CLEAN: sin latencias WCET artificiales, sin overflows UART
     * inyectados ni ráfagas de ruido/fallo GPS (sensor SCENARIO_CLEAN).
     */
    SensorsSimulation sensors{};
    sensors_simulation_init(&sensors, SCENARIO_CLEAN, start_pos, kCruiseSpeedMps, 90.0f, 23U);

    ClosedLoopPidPlant pid_plant{};
    closed_loop_pid_plant_init(&pid_plant);

    DeadReckoningFilter nav{};
    dead_reckoning_init(&nav, start_pos, NAVICORE_DOMAIN_AIR);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;

    SystemHealthMonitor health{};
    MissionController mission{};
    Guidance3D guidance(kGuidanceLookAheadM);
    RuntimeHealth runtime_health{};

    GuidanceProfile profile = guidance_profile_default();
    profile.cruise_speed_mps = kCruiseSpeedMps;
    /* Target PC/sim: vehiculo terrestre con detencion en HOME. */
    profile.require_terminal_speed_at_home = true;
    profile.terminal_speed_mps = kVehicleStoppedSpeedMps;
    guidance.set_profile(profile);

    StaticWaypointBuffer route{};
    init_mission_3d_route(&route, corner_origin, NAVICORE_DOMAIN_AIR);
    mission_controller_init(&mission);
    mission_controller_set_route(&mission, &route);
    mission.config.require_terminal_speed_at_home = true;
    mission.config.terminal_speed_mps = kVehicleStoppedSpeedMps;

    const float dt_s = kEkfDtS;
    const uint32_t max_duration_ms = kMissionCleanMaxTicks * kEkfStepMs;
    MissionState last_mission_state = MISSION_STATE_INIT;

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: MISION 3D — SCENARIO_CLEAN (Caja Negra EKF @ 100 Hz)\n");
    std::printf("  Lazo cerrado: 3x PID (velocidad, rumbo, altitud) + dinamica con inercia\n");
    std::printf("  Sin fallos: sin WCET artificial, UART stress ni GPS loss\n");
    std::printf("  FSM: INIT -> WAIT_GPS -> READY -> NAVIGATE -> RETURN_HOME -> SAFE_MODE\n");
    std::printf("  %zu waypoints 3D | nominal %u ticks (%u s) | max %u ticks (%u s)\n",
                kMission3dWaypointCount,
                kMissionCleanNominalTicks,
                (kMissionCleanNominalTicks * kEkfStepMs) / 1000U,
                kMissionCleanMaxTicks,
                max_duration_ms / 1000U);
    std::printf("================================================================\n");

    uint32_t ticks_logged = 0U;

    GpsDeadReckoningState gps_dr{};

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.health = &health;
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_CLEAN;
    bindings.temperature_c = kAmbientTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= max_duration_ms; t_ms += kEkfStepMs) {
        const auto tick_start = std::chrono::steady_clock::now();

        ImuSample imu{};
        GpsSample gps{};

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        const GpsDeadReckoningEval gps_dr_eval = gps_dead_reckoning_step(
            &gps_dr,
            &gps,
            t_ms,
            gps.fix_valid);

        dead_reckoning_update_imu(&nav, &imu, &health);
        if (gps_dr_eval.allow_gnss_update) {
            dead_reckoning_update_gps(&nav, &gps, &health);
        }
        if (gps_dr_eval.dead_reckoning_active) {
            nav.state.mode = NAV_MODE_DEAD_RECKONING;
        }

        bool gnss_update_this_cycle = false;
        float tick_nis = 0.0f;
        float tick_innov_ned[3] = {0.0f, 0.0f, 0.0f};

        bool gnss_nis_rejected = false;
        float cov_pos_n_m2 = 999.0f;
        float cov_pos_e_m2 = 999.0f;
        float cov_pos_d_m2 = 999.0f;

        if (gps.fix_valid && !ekf_seeded) {
            const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ekf_seeded = true;
        }
        if (ekf_seeded && imu.valid) {
            ekf.predict(imu, kEkfDtS);
            cov_pos_n_m2 = ins_ekf_get_covariance_flat(&ekf, 0U);
            cov_pos_e_m2 = ins_ekf_get_covariance_flat(&ekf, 16U);
            cov_pos_d_m2 = ins_ekf_get_covariance_flat(&ekf, 32U);
            if (gps_dr_eval.allow_gnss_update) {
                gnss_update_this_cycle = true;
                const bool accepted = ekf.update_gnss(gps, &tick_nis);
                gnss_nis_rejected = !accepted;
                ins_ekf_get_gnss_innovation(&ekf, tick_innov_ned);
                if (accepted) {
                    ins_ekf_clear_outlier_flag(&ekf);
                    ++ekf.gnss_accept_count;
                    ekf.last_gnss_accept_ms = gps.timestamp_ms;
                } else {
                    ++ekf.gnss_reject_count;
                }
            }
        }

        MissionGuidanceSnapshot guidance_snapshot{};
        GuidanceErrors guidance_errors{};
        mission_guidance_step(
            &guidance,
            &mission,
            &nav.state,
            gps_dr_eval.allow_gnss_update,
            gps.satellites,
            &sensors,
            t_ms,
            dt_s,
            &guidance_errors,
            &runtime_health,
            &guidance_snapshot,
            &pid_plant,
            ekf_seeded,
            cov_pos_n_m2,
            cov_pos_e_m2,
            cov_pos_d_m2,
            tick_nis,
            gnss_nis_rejected);

        const MissionState mission_state = mission_controller_state(&mission);
        if (mission_state != last_mission_state) {
            std::printf(
                "MISION: transicion %s -> %s @ t=%.2f s\n",
                mission_state_name(last_mission_state),
                mission_state_name(mission_state),
                static_cast<float>(t_ms) * 0.001f);
            last_mission_state = mission_state;
        }

        const auto tick_end = std::chrono::steady_clock::now();
        const uint32_t loop_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(tick_end - tick_start).count());
        if (loop_us > runtime_health.max_loop_us) {
            runtime_health.max_loop_us = loop_us;
        }

        const uint8_t health_score = health.health_score;
        const NavHealthMode health_mode = health.mode;
        (void)health_score;
        (void)health_mode;

        bindings.ekf = ekf_seeded ? &ekf : NULL;
        bindings.pid = pid_plant.telemetry.active ? &pid_plant.telemetry : NULL;
        bindings.guidance = &guidance_errors;
        ekf_tick.gnss_update_this_cycle = gnss_update_this_cycle;
        ekf_tick.nis = tick_nis;
        ekf_tick.innov_ned[0] = tick_innov_ned[0];
        ekf_tick.innov_ned[1] = tick_innov_ned[1];
        ekf_tick.innov_ned[2] = tick_innov_ned[2];

        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, mission_state);
        ++ticks_logged;

        if ((ticks_logged % 1000U) == 0U) {
            telemetry->flush();
        }

        if (mission_state == MISSION_STATE_SAFE_MODE) {
            std::printf(
                "MISION: SAFE_MODE alcanzado tras %u ticks (t=%.2f s)\n",
                ticks_logged,
                static_cast<float>(t_ms) * 0.001f);
            break;
        }
    }

    telemetry->flush();

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [MISION_CLEAN]: estado=%s | ticks=%u | pos=(%.6f,%.6f,%.1f) | "
        "home_valid=%u | ekf_accepts=%u rejects=%u\n",
        mission_state_name(mission_controller_state(&mission)),
        ticks_logged,
        nav.state.position.x,
        nav.state.position.y,
        nav.state.position.z,
        mission.home_valid ? 1U : 0U,
        ekf_seeded ? ins_ekf_gnss_accept_count(&ekf) : 0U,
        ekf_seeded ? ins_ekf_gnss_reject_count(&ekf) : 0U);
}

void run_replay_scenario(TelemetryInterface *telemetry, const char *replay_csv_path)
{
    InertialReplayLog replay{};
    if (!inertial_replay_load(&replay, replay_csv_path)) {
        return;
    }

    InsEkfFilter ekf{};
    bool ekf_seeded = false;

    DeadReckoningFilter nav{};
    const Vector3D default_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    dead_reckoning_init(&nav, default_origin, NAVICORE_DOMAIN_AIR);

    const float dt_s = kEkfDtS;
    const uint32_t max_duration_ms = inertial_replay_duration_ms(&replay);

    uint32_t ticks_logged = 0U;
    uint32_t predict_ticks = 0U;
    uint32_t gnss_update_ticks = 0U;
    uint32_t gnss_skip_ticks = 0U;
    uint32_t imu_gap_ticks = 0U;

    GpsDeadReckoningState gps_dr{};

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: REPLAY SiL — Log inercial CSV @ 100 Hz\n");
    std::printf("  Entrada:  %s\n", replay_csv_path);
    std::printf("  Salida:   %s\n", kTelemetryCsvPath);
    std::printf("  Muestras: %zu | duracion=%u ms (%.2f s)\n",
                inertial_replay_row_count(&replay),
                max_duration_ms,
                static_cast<float>(max_duration_ms) * 0.001f);
    std::printf("  Lazo: predict(IMU) + update_gnss(GPS) cuando hay fix\n");
    std::printf("================================================================\n");

    TelemetryEkfTick ekf_tick{};
    TelemetryBindings bindings{};
    bindings.ekf_tick = &ekf_tick;
    bindings.scenario_id = TELEM_SCENARIO_REPLAY;
    bindings.temperature_c = kAmbientTemperatureC;
    telemetry->bind_sources(&bindings);

    for (uint32_t t_ms = 0U; t_ms <= max_duration_ms; t_ms += kEkfStepMs) {
        ImuSample imu{};
        GpsSample gps{};
        bool has_imu = false;
        bool has_gnss = false;

        if (!inertial_replay_sample_at(&replay, t_ms, &imu, &gps, &has_imu, &has_gnss)) {
            break;
        }

        if (!has_imu) {
            ++imu_gap_ticks;
            continue;
        }

        const GpsDeadReckoningEval gps_dr_eval = gps_dead_reckoning_step(
            &gps_dr,
            &gps,
            t_ms,
            has_gnss);

        if (has_gnss && !ekf_seeded) {
            float yaw_rad = 0.0f;
            if (gps.course_deg != 0.0f) {
                yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            }
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            dead_reckoning_init(&nav, gps.position, NAVICORE_DOMAIN_AIR);
            ekf_seeded = true;
            std::printf(
                "REPLAY: EKF inicializado @ t=%.3f s | ref=(%.6f, %.6f, %.1f m)\n",
                static_cast<float>(t_ms) * 0.001f,
                gps.position.x,
                gps.position.y,
                gps.position.z);
        }

        bool gnss_update_this_cycle = false;
        float tick_nis = 0.0f;
        float tick_innov_ned[3] = {0.0f, 0.0f, 0.0f};
        bool gnss_nis_rejected = false;

        if (ekf_seeded && imu.valid) {
            ekf.predict(imu, dt_s);
            ++predict_ticks;

            if (gps_dr_eval.allow_gnss_update) {
                gnss_update_this_cycle = true;
                const bool accepted = ekf.update_gnss(gps, &tick_nis);
                gnss_nis_rejected = !accepted;
                ins_ekf_get_gnss_innovation(&ekf, tick_innov_ned);
                if (accepted) {
                    ins_ekf_clear_outlier_flag(&ekf);
                    ++ekf.gnss_accept_count;
                    ekf.last_gnss_accept_ms = gps.timestamp_ms;
                    ++gnss_update_ticks;
                } else {
                    ++ekf.gnss_reject_count;
                }
            } else {
                ++gnss_skip_ticks;
            }

            ins_ekf_export_nav_state(&ekf, &nav.state, t_ms, &gps);
            if (gps_dr_eval.dead_reckoning_active) {
                nav.state.mode = NAV_MODE_DEAD_RECKONING;
            }
        }

        bindings.ekf = ekf_seeded ? &ekf : NULL;
        ekf_tick.gnss_update_this_cycle = gnss_update_this_cycle;
        ekf_tick.nis = tick_nis;
        ekf_tick.innov_ned[0] = tick_innov_ned[0];
        ekf_tick.innov_ned[1] = tick_innov_ned[1];
        ekf_tick.innov_ned[2] = tick_innov_ned[2];

        nav.state.timestamp_ms = t_ms;
        telemetry->broadcast(nav.state, MISSION_STATE_INIT);
        ++ticks_logged;

        if ((ticks_logged % 1000U) == 0U) {
            telemetry->flush();
        }

        if ((t_ms % 2000U) == 0U && ekf_seeded) {
            std::printf(
                "[t=%5.1fs] predict=%u gnss_ok=%u gnss_skip=%u nis=%.2f pos=(%.2f,%.2f,%.2f) m (NED)\n",
                static_cast<float>(t_ms) * 0.001f,
                predict_ticks,
                gnss_update_ticks,
                gnss_skip_ticks,
                gnss_update_this_cycle ? tick_nis : 0.0f,
                nav.state.position.x,
                nav.state.position.y,
                nav.state.position.z);
        }
    }

    telemetry->flush();
    inertial_replay_free(&replay);

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [REPLAY]: ticks=%u | predict=%u | gnss_updates=%u | "
        "gnss_skipped=%u | imu_gaps=%u | ekf_accepts=%u rejects=%u\n",
        ticks_logged,
        predict_ticks,
        gnss_update_ticks,
        gnss_skip_ticks,
        imu_gap_ticks,
        ekf_seeded ? ins_ekf_gnss_accept_count(&ekf) : 0U,
        ekf_seeded ? ins_ekf_gnss_reject_count(&ekf) : 0U);
}

} // namespace

int main(int argc, char *argv[])
{
    bool enable_udp = true;
    SimPrimaryScenario scenario = kCompileTimeSimScenario;
    const char *replay_csv_path = NULL;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-udp") == 0) {
            enable_udp = false;
        } else if (std::strcmp(argv[i], "--stress") == 0
                   || std::strcmp(argv[i], "--high-stress") == 0) {
            scenario = SimPrimaryScenario::SCENARIO_HIGH_STRESS;
        } else if (std::strcmp(argv[i], "--clean") == 0) {
            scenario = SimPrimaryScenario::SCENARIO_CLEAN;
        } else if (std::strcmp(argv[i], "--replay") == 0) {
            if (i + 1 >= argc) {
                std::printf("ERROR: --replay requiere la ruta al archivo CSV\n");
                std::printf("Uso: NaviCore3D_Sim --replay <ruta_archivo.csv> [--no-udp]\n");
                return 1;
            }
            replay_csv_path = argv[i + 1];
            ++i;
        }
    }

    std::printf("NaviCore-3D — Simulador PC (Fase 2: Guiado 3D + Mision)\n");
    if (replay_csv_path != NULL) {
        std::printf("Modo: REPLAY SiL (%s)\n", replay_csv_path);
    } else {
        std::printf(
            "Perfil de simulacion: %s\n",
            sim_primary_scenario_name(scenario));
    }

    TelemetryConfig telemetry_config{};
    telemetry_config.enable_simulator_channel = true;
    telemetry_config.enable_unity_channel = enable_udp;
    telemetry_config.enable_logger_channel = true;
    telemetry_config.logger_csv_path = kTelemetryCsvPath;
    telemetry_config.unity_host = kTelemetryUnityHost;
    telemetry_config.unity_port = UNITY_TELEMETRY_DEFAULT_PORT;
    telemetry_config.sim_console_interval_ms = TELEMETRY_SIM_CONSOLE_INTERVAL_MS;

    TelemetryInterface telemetry;
    if (!telemetry.initialize(telemetry_config)) {
        std::printf("ERROR: no se pudo inicializar telemetria (%s)\n", kTelemetryCsvPath);
        return 1;
    }

    TelemetryInterface::set_active(&telemetry);

    if (!enable_udp) {
        std::printf("Canal Unity UDP: deshabilitado (--no-udp)\n");
    }

    if (replay_csv_path != NULL) {
        run_replay_scenario(&telemetry, replay_csv_path);
    } else if (scenario == SimPrimaryScenario::SCENARIO_CLEAN) {
        run_mission_clean_scenario(&telemetry);
    } else {
        run_high_demand_stress_test_scenario(&telemetry);
    }

    telemetry.flush();
    telemetry.log_stats();
    telemetry.shutdown();
    TelemetryInterface::set_active(NULL);

    std::printf("\nSimulacion completada. Gemelo Digital: %s\n", kTelemetryCsvPath);
    std::printf("Visualizar CSV:     python tools/visualizer.py\n");
    std::printf("Visualizar remoto:  python tools/remote_visualizer.py\n");
    return 0;
}
