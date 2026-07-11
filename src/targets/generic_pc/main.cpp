#include <cstdio>
#include <cstdint>
#include <cmath>
#include <cerrno>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

#include "vector3d.h"
#include "NavState.h"
#include "fusion.hpp"
#include "guidance.hpp"
#include "waypoint.hpp"
#include "sensors_sim.hpp"
#include "diagnostic.hpp"
#include "command_ingest.hpp"
#include "power_state_machine.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr uint32_t kStepMs = 100U;
constexpr float kCruiseSpeedMps = 15.0f;
constexpr float kDegradedCruiseSpeedMps = 8.0f;
constexpr float kCruiseCourseDeg = 90.0f;
constexpr float kKpHeading = 0.5f;
constexpr float kSurfacePressurePa = 101325.0f;
constexpr float kSubmersionPressureRatePaS = 10000.0f;
constexpr float kSafeStopDecelMps2 = 3.0f;
constexpr float kVehicleStoppedSpeedMps = 0.05f;

constexpr uint32_t kFaultInjectGpsLossMs = 5000U;
constexpr uint32_t kFaultInjectRadioCmdMs = 10000U;
constexpr uint32_t kFaultInjectSpiTimeoutMs = 15000U;
constexpr uint32_t kFaultInjectDurationMs = 30000U;
constexpr const char *kTelemetryCsvPath = "docs/telemetria_navicore.csv";

constexpr SensorScenario kSelectedScenario = SCENARIO_ODOM_LOSS;

constexpr float kWaypointLonStepDeg = 0.00018f;
constexpr float kWaypointArrivalRadiusNominalM = 5.0f;
constexpr float kWaypointArrivalRadiusDegradedM = 15.0f;

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
    if (file == NULL) {
        return;
    }

    std::fprintf(
        file,
        "Timestamp_ms,Escenario,Modo,Calidad,Satelites,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Rumbo,CrossTrack_m,AlongTrack_m,OdomFault,HealthScore,HealthMode,PowerState,ShutdownLatched,WaypointCount,BspBus\n");
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
    uint8_t bsp_bus_status)
{
    if (file == NULL || state == NULL || scenario == NULL) {
        return;
    }

    const float cross_track_m = (guidance != NULL) ? guidance->cross_track_m : 0.0f;
    const float along_track_m = (guidance != NULL) ? guidance->along_track_m : 0.0f;

    std::fprintf(
        file,
        "%u,%s,%s,%.6f,%u,%.8f,%.8f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%u,%u,%s,%s,%u,%zu,%u\n",
        timestamp_ms,
        scenario,
        nav_mode_name(state->mode),
        state->confidence.estimate_quality,
        static_cast<unsigned>(scenario_satellites),
        state->position.x,
        state->position.y,
        state->position.z,
        state->velocity.x,
        state->velocity.y,
        state->velocity.z,
        state->heading_deg,
        cross_track_m,
        along_track_m,
        static_cast<unsigned>(odom_fault),
        static_cast<unsigned>(health_score),
        health_mode_name(health_mode),
        power_state_name(power_state),
        shutdown_latched ? 1U : 0U,
        waypoint_count,
        static_cast<unsigned>(bsp_bus_status));
}

void init_test_waypoint_route(
    StaticWaypointBuffer *route,
    Waypoint *leg_origin,
    Vector3D start,
    NavDomain domain)
{
    if (route == NULL || leg_origin == NULL) {
        return;
    }

    waypoint_buffer_init(route);

    const uint32_t nominal_radius_m = contingency_arrival_radius_m(HEALTH_NOMINAL);

    const Waypoint wp0 = waypoint_make("WP0", start, domain, nominal_radius_m);
    const Waypoint wp1 = waypoint_make(
        "WP1",
        vector3d_make(start.x, start.y + kWaypointLonStepDeg, start.z),
        domain,
        nominal_radius_m);
    const Waypoint wp2 = waypoint_make(
        "WP2",
        vector3d_make(start.x, start.y + (2.0f * kWaypointLonStepDeg), start.z),
        domain,
        nominal_radius_m);

    waypoint_buffer_push(route, wp0);
    waypoint_buffer_push(route, wp1);
    waypoint_buffer_push(route, wp2);
    *leg_origin = wp0;
}

GuidanceErrors guidance_tick_update_route(
    StaticWaypointBuffer *route,
    Waypoint *leg_origin,
    Vector3D position)
{
    GuidanceErrors errors{};

    if (route == NULL || leg_origin == NULL || route->count < 2U) {
        return errors;
    }

    const Waypoint leg_dest = route->items[(route->head + 1U) % NAVICORE_MAX_WAYPOINTS];
    errors = guidance_compute_errors(position, *leg_origin, leg_dest);

    if (errors.along_track_m <= static_cast<float>(leg_dest.arrival_radius_m)) {
        Waypoint passed{};
        if (waypoint_buffer_pop(route, &passed)) {
            *leg_origin = passed;
        }
    }

    return errors;
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

void apply_closed_loop_heading_control(
    SensorsSimulation *sensors,
    float prev_cross_track_m,
    float *synthetic_course_deg,
    float dt_s)
{
    if (sensors == NULL || synthetic_course_deg == NULL) {
        return;
    }

    const float prev_course_deg = *synthetic_course_deg;
    const float heading_correction_deg = -kKpHeading * prev_cross_track_m;
    const float corrected_course_deg = navstate_normalize_heading(
        kCruiseCourseDeg + heading_correction_deg);

    const float delta_heading_deg = heading_delta_deg(prev_course_deg, corrected_course_deg);
    const float yaw_rate_radps = (dt_s > 0.0f) ? ((delta_heading_deg * (M_PI / 180.0f)) / dt_s) : 0.0f;

    sensors_simulation_apply_heading_control(sensors, corrected_course_deg, yaw_rate_radps);
    *synthetic_course_deg = corrected_course_deg;
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

void run_fault_injection_scenario(FILE *telemetry_file)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, SCENARIO_CLEAN, origin, kCruiseSpeedMps, kCruiseCourseDeg, 17U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    SystemHealthMonitor health{};
    power_manager_init();

    bool safe_stop_active = false;
    float safe_stop_heading_deg = kCruiseCourseDeg;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;
    bool radio_cmd_injected = false;
    size_t waypoint_count_before_radio = 0U;

    StaticWaypointBuffer route{};
    Waypoint leg_origin{};
    init_test_waypoint_route(&route, &leg_origin, origin, NAVICORE_DOMAIN_AIR);

    float cruise_speed_mps = kCruiseSpeedMps;
    float prev_cross_track_m = 0.0f;
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
        } else {
            apply_closed_loop_heading_control(
                &sensors,
                prev_cross_track_m,
                &synthetic_course_deg,
                dt_s);
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
            radio_cmd.checksum = command_ingest_compute_checksum(&radio_cmd);

            const bool ingest_ok = command_ingest_parse(&radio_cmd, &route, &cruise_speed_mps);
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

        diagnostic_update(&health, filter_quality_u8, worst_bsp_bus);
        log_health_contingency_transition(health.mode, &prev_health_mode, false);

        if (diagnostic_requires_safe_stop(&health) && t_ms >= kFaultInjectSpiTimeoutMs) {
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

        const GuidanceErrors guidance = guidance_tick_update_route(
            &route,
            &leg_origin,
            nav.state.position);

        prev_cross_track_m = guidance.cross_track_m;

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
            worst_bsp_bus);

        if ((t_ms % 1000U) == 0U) {
            std::printf(
                "[t=%5.1fs] mode=%-14s quality=%.3f health=%s(%u) power=%s shutdown=%u speed=%.2f wp=%zu bsp=%u\n",
                static_cast<float>(t_ms) * 0.001f,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
                health_mode_name(health.mode),
                static_cast<unsigned>(health.health_score),
                power_state_name(power_manager_get_state()),
                power_manager_is_shutdown_latched() ? 1U : 0U,
                speed_mps,
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
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, scenario, origin, kCruiseSpeedMps, kCruiseCourseDeg, 11U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    SystemHealthMonitor health{};
    bool safe_stop_active = false;
    float safe_stop_heading_deg = kCruiseCourseDeg;
    NavHealthMode prev_health_mode = HEALTH_NOMINAL;

    StaticWaypointBuffer route{};
    Waypoint leg_origin{};
    init_test_waypoint_route(&route, &leg_origin, origin, NAVICORE_DOMAIN_AIR);

    print_scenario_banner(scenario);

    constexpr uint32_t kDurationMs = 20000U;
    bool prev_gps_valid = true;
    bool prev_odom_fault = false;
    float prev_cross_track_m = 0.0f;
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
        } else {
            apply_closed_loop_heading_control(
                &sensors,
                prev_cross_track_m,
                &synthetic_course_deg,
                dt_s);
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
        diagnostic_update(&health, filter_quality_u8, worst_bsp_bus);
        log_health_contingency_transition(health.mode, &prev_health_mode, false);

        if (diagnostic_requires_safe_stop(&health)) {
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

        const GuidanceErrors guidance = guidance_tick_update_route(
            &route,
            &leg_origin,
            nav.state.position);

        prev_cross_track_m = guidance.cross_track_m;

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
            worst_bsp_bus);

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
    float active_pressure_rate_pa_s = kSubmersionPressureRatePaS;

    StaticWaypointBuffer route{};
    Waypoint leg_origin{};
    init_test_waypoint_route(&route, &leg_origin, surface, NAVICORE_DOMAIN_SEA);

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
        diagnostic_update(&health, filter_quality_u8, worst_bsp_bus);
        log_health_contingency_transition(health.mode, &prev_health_mode, true);

        if (diagnostic_requires_safe_stop(&health)) {
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

        const GuidanceErrors guidance = guidance_tick_update_route(
            &route,
            &leg_origin,
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
            worst_bsp_bus);

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

} // namespace

int main()
{
    std::printf("NaviCore-3D — Simulador de estres PC\n");
    std::printf("Escenario principal: inyeccion automatizada de fallos (FAULT_INJECTION)\n");

    FILE *telemetry_file = telemetry_open(kTelemetryCsvPath);
    if (telemetry_file == NULL) {
        std::printf("ERROR: no se pudo crear %s\n", kTelemetryCsvPath);
        return 1;
    }

    telemetry_write_header(telemetry_file);
    std::printf("Telemetria CSV: %s\n", kTelemetryCsvPath);

    run_fault_injection_scenario(telemetry_file);

    std::fclose(telemetry_file);

    std::printf("\nSimulacion completada. Gemelo Digital: %s\n", kTelemetryCsvPath);
    std::printf("Visualizar: python tools/visualizer.py\n");
    return 0;
}
