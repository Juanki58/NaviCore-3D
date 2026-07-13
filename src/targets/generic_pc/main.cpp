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
#include "flight_recorder.hpp"
#include "waypoint.hpp"
#include "sensors_sim.hpp"
#include "diagnostic.hpp"
#include "command_ingestor.hpp"
#include "navigation_cortex.hpp"
#include "power_state_machine.hpp"
#include "time_guard.hpp"
#include "telemetry_udp_sender.hpp"
#include "telemetry_udp.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr uint32_t kStepMs = 100U;
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

constexpr const char *kTelemetryCsvPath = "docs/telemetria_navicore.csv";
constexpr const char *kTelemetryUdpHost = "127.0.0.1";
constexpr int kTelemetryUdpPort = 5005;
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
constexpr uint32_t kMissionCleanDurationMs = 60000U;
constexpr uint32_t kMissionAutoStartReadyTicks = 3U;
constexpr float kWaypointArrivalRadiusNominalM = 5.0f;
constexpr float kWaypointArrivalRadiusDegradedM = 15.0f;

struct MissionGuidanceSnapshot {
    GuidanceCommands commands;
    bool guidance_valid;
    bool return_home_active;
    size_t active_waypoint_index;
};

struct FlightRecorderExtras {
    const ImuSample *imu;
    const GpsSample *gps;
    const InsEkfFilter *ekf;
    bool gnss_update_accepted;
    const GuidanceCommands *commands;
    bool guidance_commands_valid;
    MissionState mission_state;
    bool return_home_active;
    size_t active_waypoint_index;
    const RuntimeHealth *runtime_health;
    uint32_t loop_us;
};

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

bool telemetry_ensure_docs_dir()
{
#ifdef _WIN32
    return _mkdir("docs") == 0 || errno == EEXIST;
#else
    return mkdir("docs", 0755) == 0 || errno == EEXIST;
#endif
}

FILE *telemetry_open(const char *path)
{
    telemetry_ensure_docs_dir();
    return std::fopen(path, "w");
}

void telemetry_write_header(FILE *file)
{
    flight_recorder_write_csv_header(file);
}

void telemetry_record_tick(FILE *file, const FlightRecorderTickInput *input)
{
    if (file == NULL || input == NULL || input->nav_state == NULL) {
        return;
    }

    FlightRecorderSample sample{};
    if (!flight_recorder_capture(&sample, input)) {
        return;
    }

    flight_recorder_write_csv_row(file, &sample);

    const uint16_t dropped_packets_udp = static_cast<uint16_t>(
        input->radio_dropped_packets > 16383U ? 16383U : input->radio_dropped_packets);
    telemetry_udp_send(
        input->timestamp_ms,
        input->nav_state->position.x,
        input->nav_state->position.y,
        input->nav_state->position.z,
        sample.cross_track_m,
        sample.along_track_m,
        input->health_score,
        static_cast<uint8_t>(input->health_mode),
        dropped_packets_udp,
        input->scenario_id,
        static_cast<uint8_t>(input->nav_state->mode),
        input->temperature_c);
}

void telemetry_write_row(
    FILE *file,
    uint32_t timestamp_ms,
    const char *scenario,
    const NavState *state,
    uint8_t scenario_satellites,
    const GuidanceErrors *guidance,
    uint8_t odom_fault,
    uint8_t health_score,
    NavHealthMode health_mode,
    PowerState power_state,
    bool shutdown_latched,
    size_t waypoint_count,
    uint8_t bsp_bus_status,
    uint32_t radio_dropped_packets,
    uint8_t scenario_id,
    float temperature_c,
    const FlightRecorderExtras *extras)
{
    if (state == NULL || scenario == NULL) {
        return;
    }

    FlightRecorderTickInput input{};
    input.timestamp_ms = timestamp_ms;
    input.scenario_name = scenario;
    input.scenario_id = scenario_id;
    input.nav_state = state;
    input.guidance_errors = guidance;
    input.health_score = health_score;
    input.health_mode = health_mode;
    input.power_state = static_cast<uint8_t>(power_state);
    input.shutdown_latched = shutdown_latched;
    input.odom_fault = odom_fault;
    input.waypoint_count = waypoint_count;
    input.bsp_bus_status = bsp_bus_status;
    input.radio_dropped_packets = radio_dropped_packets;
    input.temperature_c = temperature_c;

    if (extras != NULL) {
        input.imu = extras->imu;
        input.gps = extras->gps;
        input.ekf = extras->ekf;
        input.gnss_update_accepted = extras->gnss_update_accepted;
        input.guidance_commands = extras->commands;
        input.guidance_commands_valid = extras->guidance_commands_valid;
        input.mission_state = extras->mission_state;
        input.return_home_active = extras->return_home_active;
        input.active_waypoint_index = extras->active_waypoint_index;
        input.runtime_health = extras->runtime_health;
        input.loop_us = extras->loop_us;
    }

    telemetry_record_tick(file, &input);
}

void telemetry_udp_emit_events(uint32_t timestamp_ms, const NavigationDecision *decision)
{
    if (decision == NULL) {
        return;
    }

    for (uint8_t i = 0; i < decision->event_count; ++i) {
        telemetry_udp_send_event(timestamp_ms, decision->events[i].id, decision->events[i].param);
    }
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
    telemetry_udp_emit_events(timestamp_ms, decision);
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

    const uint32_t arrival_radius_m = 15U;

    const Waypoint wp0 = waypoint_make(
        "M0",
        vector3d_make(start.x, start.y, 12.0f),
        domain,
        arrival_radius_m);
    const Waypoint wp1 = waypoint_make(
        "M1",
        vector3d_make(start.x, start.y + kSquareLegStepDeg, 18.0f),
        domain,
        arrival_radius_m);
    const Waypoint wp2 = waypoint_make(
        "M2",
        vector3d_make(start.x + kSquareLegStepDeg, start.y + kSquareLegStepDeg, 22.0f),
        domain,
        arrival_radius_m);
    const Waypoint wp3 = waypoint_make(
        "M3",
        vector3d_make(start.x + kSquareLegStepDeg, start.y, 15.0f),
        domain,
        arrival_radius_m);

    waypoint_buffer_push(route, wp0);
    waypoint_buffer_push(route, wp1);
    waypoint_buffer_push(route, wp2);
    waypoint_buffer_push(route, wp3);

    std::printf(
        "MISION: ruta 3D — %zu waypoints | look-ahead=%.1f m | altitudes 12/18/22/15 m\n",
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
    const Waypoint wp0 = waypoint_make("SQ0", start, domain, arrival_radius_m);
    const Waypoint wp1 = waypoint_make(
        "SQ1",
        vector3d_make(start.x, start.y + kSquareLegStepDeg, start.z),
        domain,
        arrival_radius_m);
    const Waypoint wp2 = waypoint_make(
        "SQ2",
        vector3d_make(start.x + kSquareLegStepDeg, start.y + kSquareLegStepDeg, start.z),
        domain,
        arrival_radius_m);
    const Waypoint wp3 = waypoint_make(
        "SQ3",
        vector3d_make(start.x + kSquareLegStepDeg, start.y, start.z),
        domain,
        arrival_radius_m);

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

void apply_guidance_commands(
    SensorsSimulation *sensors,
    const GuidanceCommands *commands,
    float current_heading_deg,
    float dt_s,
    float *synthetic_course_deg)
{
    if (sensors == NULL || commands == NULL || synthetic_course_deg == NULL) {
        return;
    }

    sensors_simulation_apply_guidance_control(
        sensors,
        commands->desired_heading,
        commands->desired_speed,
        commands->desired_climb,
        current_heading_deg,
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
            nav_state->heading_deg,
            dt_s,
            synthetic_course_deg);
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
    MissionGuidanceSnapshot *snapshot_out)
{
    if (guidance == NULL || mission == NULL || nav_state == NULL
        || sensors == NULL || telemetry_out == NULL || runtime_health == NULL) {
        return;
    }

    MissionTickInput input{};
    MissionTickOutput output{};

    if (snapshot_out != NULL) {
        snapshot_out->guidance_valid = false;
        snapshot_out->return_home_active = false;
        snapshot_out->active_waypoint_index = mission->active_waypoint_index;
    }

    input.nav_state = nav_state;
    input.runtime_health = runtime_health;
    input.gps_fix_valid = gps_fix_valid;
    input.satellites = satellites;
    input.estimate_quality = nav_state->confidence.estimate_quality;
    input.start_signal = false;
    input.timestamp_ms = timestamp_ms;

    static uint32_t ready_ticks = 0U;
    static bool mission_auto_start_sent = false;
    if (mission_controller_state(mission) == MissionState::READY && !mission_auto_start_sent) {
        ++ready_ticks;
        if (ready_ticks >= kMissionAutoStartReadyTicks) {
            input.start_signal = true;
            mission_auto_start_sent = true;
        }
    } else {
        ready_ticks = 0U;
    }

    (void)mission_controller_tick(mission, &input, &output);

    if (snapshot_out != NULL) {
        snapshot_out->return_home_active = output.return_home_active;
        snapshot_out->active_waypoint_index = output.active_waypoint_index;
    }

    if (output.safe_mode) {
        GuidanceCommands zero{};
        float hold_heading_deg = nav_state->heading_deg;
        apply_guidance_commands(sensors, &zero, nav_state->heading_deg, dt_s, &hold_heading_deg);
        *telemetry_out = GuidanceErrors{};
        return;
    }

    if (!output.guidance_active || output.active_route == NULL) {
        *telemetry_out = GuidanceErrors{};
        return;
    }

    const GuidanceOutput guidance_out = output.return_home_active
        ? guidance_compute_homing(*nav_state, mission->home, guidance_profile_default())
        : guidance->compute(
            *nav_state,
            *output.active_route,
            output.active_waypoint_index);

    *telemetry_out = guidance_out.track_errors;

    if (snapshot_out != NULL) {
        snapshot_out->commands = guidance_out.commands;
        snapshot_out->guidance_valid = guidance_out.valid;
    }

    if (!guidance_out.valid) {
        return;
    }

    float synthetic_course_deg = nav_state->heading_deg;
    apply_guidance_commands(
        sensors,
        &guidance_out.commands,
        nav_state->heading_deg,
        dt_s,
        &synthetic_course_deg);

    if ((timestamp_ms % 1000U) == 0U) {
        std::printf(
            "GUIADO: hdg=%.1f deg speed=%.2f m/s climb=%.2f m/s | xt=%.2f m along=%.1f m\n",
            synthetic_course_deg,
            guidance_out.commands.desired_speed,
            guidance_out.commands.desired_climb,
            guidance_out.track_errors.cross_track_signed_m,
            guidance_out.track_errors.along_track_m);
    }

    if (guidance_out.waypoint_completed && output.return_home_active) {
        mission->state = MissionState::READY;
        mission->start_requested = false;
        std::printf(
            "MISION: HOME alcanzado @ t=%.2f s -> READY\n",
            static_cast<float>(timestamp_ms) * 0.001f);
        return;
    }

    if (!guidance_out.waypoint_completed || mission->return_home_requested) {
        return;
    }

    mission_controller_on_waypoint_completed(mission);
    std::printf(
        "MISION: WP%zu completado @ t=%.2f s | estado=%s\n",
        output.active_waypoint_index,
        static_cast<float>(timestamp_ms) * 0.001f,
        mission_state_name(mission_controller_state(mission)));
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

void run_high_demand_stress_test_scenario(FILE *telemetry_file)
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
            telemetry_udp_send_event(t_ms, NAV_EVENT_POWER_CONSERVATION, health.health_score);
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

        telemetry_write_row(
            telemetry_file,
            t_ms,
            kHighDemandScenarioName,
            &nav.state,
            scenario_satellites,
            &guidance,
            odom_fault_active ? 1U : 0U,
            health.health_score,
            health.mode,
            power_manager_get_state(),
            power_manager_is_shutdown_latched(),
            route.count,
            worst_bsp_bus,
            command_ingestor_hw_dropped_packets(),
            TELEM_SCENARIO_HIGH_DEMAND,
            kAmbientTemperatureC,
            NULL);

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

void run_fault_injection_scenario(FILE *telemetry_file)
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
            telemetry_udp_send_event(t_ms, NAV_EVENT_POWER_CONSERVATION, health.health_score);
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

        telemetry_write_row(
            telemetry_file,
            t_ms,
            "FAULT_INJECTION",
            &nav.state,
            scenario_satellites,
            &guidance,
            odom_fault_active ? 1U : 0U,
            health.health_score,
            health.mode,
            power_manager_get_state(),
            power_manager_is_shutdown_latched(),
            route.count,
            worst_bsp_bus,
            command_ingestor_hw_dropped_packets(),
            TELEM_SCENARIO_FAULT_INJECTION,
            kAmbientTemperatureC,
            NULL);

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

void run_sensor_scenario(FILE *telemetry_file, SensorScenario scenario)
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

        telemetry_write_row(
            telemetry_file,
            t_ms,
            sensor_scenario_name(scenario),
            &nav.state,
            scenario_satellites,
            &guidance,
            odom_fault_active ? 1U : 0U,
            health.health_score,
            health.mode,
            POWER_PERFORMANCE,
            false,
            route.count,
            worst_bsp_bus,
            command_ingestor_hw_dropped_packets(),
            telemetry_scenario_id_from_sensor(scenario),
            kAmbientTemperatureC,
            NULL);

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

void run_scenario_submarine(FILE *telemetry_file)
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

        telemetry_write_row(
            telemetry_file,
            t_ms,
            "SUBMARINE",
            &nav.state,
            0U,
            &guidance,
            0U,
            health.health_score,
            health.mode,
            POWER_PERFORMANCE,
            false,
            route.count,
            worst_bsp_bus,
            command_ingestor_hw_dropped_packets(),
            TELEM_SCENARIO_SUBMARINE,
            kSubmarineTemperatureC,
            NULL);

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

void run_mission_clean_scenario(FILE *telemetry_file)
{
    const Vector3D corner_origin = vector3d_make(41.3874f, 2.1686f, 12.0f);
    const Vector3D origin = corner_origin;

    SensorsSimulation sensors{};
    sensors_simulation_init(&sensors, SCENARIO_CLEAN, origin, kCruiseSpeedMps, 90.0f, 23U);

    DeadReckoningFilter nav{};
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    InsEkfFilter ekf{};
    bool ekf_seeded = false;

    SystemHealthMonitor health{};
    MissionController mission{};
    Guidance3D guidance(kGuidanceLookAheadM);
    RuntimeHealth runtime_health{};

    GuidanceProfile profile = guidance_profile_default();
    guidance.set_profile(profile);

    StaticWaypointBuffer route{};
    init_mission_3d_route(&route, corner_origin, NAVICORE_DOMAIN_AIR);
    mission_controller_init(&mission);
    mission_controller_set_route(&mission, &route);

    const float dt_s = static_cast<float>(kStepMs) * 0.001f;

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO: MISION 3D — SCENARIO_CLEAN (FlightRecorder + shadow EKF)\n");
    std::printf("  FSM: INIT -> WAIT_GPS -> READY -> NAVIGATE -> RETURN_HOME\n");
    std::printf("  %zu waypoints 3D | Guiado: cross-track + heading + speed + climb\n",
                kMission3dWaypointCount);
    std::printf("================================================================\n");

    for (uint32_t t_ms = 0U; t_ms <= kMissionCleanDurationMs; t_ms += kStepMs) {
        const auto tick_start = std::chrono::steady_clock::now();

        ImuSample imu{};
        GpsSample gps{};

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        dead_reckoning_update_imu(&nav, &imu, &health);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps, &health);
        }

        bool gnss_update_accepted = false;
        if (gps.fix_valid && !ekf_seeded) {
            const float yaw_rad = static_cast<float>(gps.course_deg * M_PI / 180.0);
            ins_ekf_init(&ekf, gps.position, yaw_rad, NAVICORE_DOMAIN_AIR);
            ekf_seeded = true;
        }
        if (ekf_seeded) {
            ins_ekf_predict(&ekf, &imu);
            if (gps.fix_valid) {
                gnss_update_accepted = ins_ekf_update_gnss(&ekf, &gps);
            }
        }

        MissionGuidanceSnapshot guidance_snapshot{};
        GuidanceErrors guidance_errors{};
        mission_guidance_step(
            &guidance,
            &mission,
            &nav.state,
            gps.fix_valid,
            gps.satellites,
            &sensors,
            t_ms,
            dt_s,
            &guidance_errors,
            &runtime_health,
            &guidance_snapshot);

        const auto tick_end = std::chrono::steady_clock::now();
        const uint32_t loop_us = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(tick_end - tick_start).count());
        if (loop_us > runtime_health.max_loop_us) {
            runtime_health.max_loop_us = loop_us;
        }

        const bool ekf_outlier = ekf_seeded && ins_ekf_outlier_detected(&ekf);
        const uint8_t health_score = health.health_score;
        const NavHealthMode health_mode = health.mode;

        FlightRecorderExtras extras{};
        extras.imu = &imu;
        extras.gps = &gps;
        extras.ekf = ekf_seeded ? &ekf : NULL;
        extras.gnss_update_accepted = gnss_update_accepted;
        extras.commands = guidance_snapshot.guidance_valid ? &guidance_snapshot.commands : NULL;
        extras.guidance_commands_valid = guidance_snapshot.guidance_valid;
        extras.mission_state = mission_controller_state(&mission);
        extras.return_home_active = guidance_snapshot.return_home_active;
        extras.active_waypoint_index = guidance_snapshot.active_waypoint_index;
        extras.runtime_health = &runtime_health;
        extras.loop_us = loop_us;

        telemetry_write_row(
            telemetry_file,
            t_ms,
            sensor_scenario_name(SCENARIO_CLEAN),
            &nav.state,
            gps.fix_valid ? gps.satellites : 0U,
            &guidance_errors,
            0U,
            health_score,
            health_mode,
            POWER_PERFORMANCE,
            false,
            route.count,
            DIAG_BSP_BUS_IDLE,
            0U,
            TELEM_SCENARIO_CLEAN,
            kAmbientTemperatureC,
            &extras);

        if ((t_ms % 2000U) == 0U) {
            std::printf(
                "[t=%5.1fs] mission=%s mode=%-14s pos=(%.6f,%.6f,%.1f) speed=%.2f nis=%.2f loop=%uus\n",
                static_cast<float>(t_ms) * 0.001f,
                mission_state_name(mission_controller_state(&mission)),
                nav_mode_name(nav.state.mode),
                nav.state.position.x,
                nav.state.position.y,
                nav.state.position.z,
                navstate_speed_mps(&nav.state),
                ekf_seeded ? ins_ekf_last_nis(&ekf) : 0.0f,
                loop_us);
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf(
        "Resultado [MISION_CLEAN]: estado=%s | pos=(%.6f,%.6f,%.1f) | home_valid=%u | ekf_accepts=%u rejects=%u\n",
        mission_state_name(mission_controller_state(&mission)),
        nav.state.position.x,
        nav.state.position.y,
        nav.state.position.z,
        mission.home_valid ? 1U : 0U,
        ekf_seeded ? ins_ekf_gnss_accept_count(&ekf) : 0U,
        ekf_seeded ? ins_ekf_gnss_reject_count(&ekf) : 0U);
}

} // namespace

int main(int argc, char *argv[])
{
    bool enable_udp = true;
    bool run_stress = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-udp") == 0) {
            enable_udp = false;
        }
        if (std::strcmp(argv[i], "--stress") == 0) {
            run_stress = true;
        }
    }

    std::printf("NaviCore-3D — Simulador PC (Fase 2: Guiado 3D + Mision)\n");
    std::printf("Escenario principal: MISION_CLEAN (SCENARIO_CLEAN)\n");

    FILE *telemetry_file = telemetry_open(kTelemetryCsvPath);
    if (telemetry_file == NULL) {
        std::printf("ERROR: no se pudo crear %s\n", kTelemetryCsvPath);
        return 1;
    }

    telemetry_write_header(telemetry_file);
    std::printf("Telemetria CSV: %s\n", kTelemetryCsvPath);

    if (enable_udp) {
        telemetry_udp_init(kTelemetryUdpHost, kTelemetryUdpPort);
    } else {
        std::printf("Telemetria UDP: deshabilitada (--no-udp)\n");
    }

    run_mission_clean_scenario(telemetry_file);

    if (run_stress) {
        run_high_demand_stress_test_scenario(telemetry_file);
    }

    std::fclose(telemetry_file);
    telemetry_udp_log_stats();

    std::printf("\nSimulacion completada. Gemelo Digital: %s\n", kTelemetryCsvPath);
    std::printf("Visualizar CSV:     python tools/visualizer.py\n");
    std::printf("Visualizar remoto:  python tools/remote_visualizer.py\n");
    return 0;
}
