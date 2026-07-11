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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

constexpr uint32_t kStepMs = 100U;
constexpr float kCruiseSpeedMps = 15.0f;
constexpr float kCruiseCourseDeg = 90.0f;
constexpr float kKpHeading = 0.5f;
constexpr float kSurfacePressurePa = 101325.0f;
constexpr float kSubmersionPressureRatePaS = 10000.0f;
constexpr const char *kTelemetryCsvPath = "docs/telemetria_navicore.csv";

constexpr SensorScenario kSelectedScenario = SCENARIO_GPS_LOSS;

constexpr float kWaypointLonStepDeg = 0.00018f;
constexpr uint32_t kWaypointArrivalRadiusM = 15U;

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
        "Timestamp_ms,Escenario,Modo,Calidad,Satelites,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Rumbo,CrossTrack_m,AlongTrack_m\n");
}

void telemetry_write_row(
    FILE *file,
    uint32_t timestamp_ms,
    const char *scenario,
    const NavState *state,
    uint8_t scenario_satellites,
    const GuidanceErrors *guidance)
{
    if (file == NULL || state == NULL || scenario == NULL) {
        return;
    }

    const float cross_track_m = (guidance != NULL) ? guidance->cross_track_m : 0.0f;
    const float along_track_m = (guidance != NULL) ? guidance->along_track_m : 0.0f;

    std::fprintf(
        file,
        "%u,%s,%s,%.6f,%u,%.8f,%.8f,%.4f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
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
        along_track_m);
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

    const Waypoint wp0 = waypoint_make("WP0", start, domain, kWaypointArrivalRadiusM);
    const Waypoint wp1 = waypoint_make(
        "WP1",
        vector3d_make(start.x, start.y + kWaypointLonStepDeg, start.z),
        domain,
        kWaypointArrivalRadiusM);
    const Waypoint wp2 = waypoint_make(
        "WP2",
        vector3d_make(start.x, start.y + (2.0f * kWaypointLonStepDeg), start.z),
        domain,
        kWaypointArrivalRadiusM);

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

void run_sensor_scenario(FILE *telemetry_file, SensorScenario scenario)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    SensorsSimulation sensors;
    sensors_simulation_init(&sensors, scenario, origin, kCruiseSpeedMps, kCruiseCourseDeg, 11U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    StaticWaypointBuffer route{};
    Waypoint leg_origin{};
    init_test_waypoint_route(&route, &leg_origin, origin, NAVICORE_DOMAIN_AIR);

    print_scenario_banner(scenario);

    constexpr uint32_t kDurationMs = 20000U;
    bool prev_gps_valid = true;
    float prev_cross_track_m = 0.0f;
    float synthetic_course_deg = kCruiseCourseDeg;
    const float dt_s = static_cast<float>(kStepMs) * 0.001f;

    for (uint32_t t_ms = 0U; t_ms <= kDurationMs; t_ms += kStepMs) {
        apply_closed_loop_heading_control(
            &sensors,
            prev_cross_track_m,
            &synthetic_course_deg,
            dt_s);

        ImuSample imu;
        GpsSample gps;

        if (!sensors_simulation_tick(&sensors, t_ms, &imu, &gps)) {
            continue;
        }

        const uint32_t tick_index = sensors.faults.tick_index - 1U;
        const uint8_t scenario_satellites = gps.fix_valid ? gps.satellites : 0U;

        if (scenario == SCENARIO_GPS_LOSS) {
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

        dead_reckoning_update_imu(&nav, &imu);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps);
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
            &guidance);

        if ((t_ms % 1000U) == 0U) {
            const char *note = "";
            if (scenario == SCENARIO_GPS_LOSS
                && tick_index == SENSOR_FAULT_GPS_LOSS_START_TICK_DEFAULT) {
                note = "<-- inicio outage (tick 3)";
            } else if (scenario == SCENARIO_IMU_DRIFT && t_ms == 5000U) {
                note = "<-- deriva visible en IMU";
            }

            const float t_s = static_cast<float>(t_ms) * 0.001f;
            std::printf(
                "[t=%5.1fs] tick=%-4u mode=%-14s quality=%.3f scenario_sats=%u fix_valid=%s fix_age=%u ms | "
                "heading=%.1f speed=%.2f m/s\n",
                t_s,
                tick_index,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
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
        "Resultado [%s]: mode=%s quality=%.3f\n",
        sensor_scenario_name(scenario),
        nav_mode_name(nav.state.mode),
        nav.state.confidence.estimate_quality);
}

void run_scenario_submarine(FILE *telemetry_file)
{
    const Vector3D surface = vector3d_make(41.3900f, 2.1750f, kSurfacePressurePa);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, surface, NAVICORE_DOMAIN_SEA);

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
        ImuSample imu;
        PressureSample pressure{};

        ImuSimulator imu_sim;
        imu_simulator_init(&imu_sim, 21U);
        imu_simulator_read(&imu_sim, t_ms, &imu);

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        pressure.pressure_pa = kSurfacePressurePa + (kSubmersionPressureRatePaS * t_s);
        pressure.temperature_c = 10.0f;
        pressure.timestamp_ms = t_ms;
        pressure.valid = true;

        dead_reckoning_update_imu(&nav, &imu);
        dead_reckoning_update_pressure(&nav, &pressure, kSurfacePressurePa);

        const GuidanceErrors guidance = guidance_tick_update_route(
            &route,
            &leg_origin,
            nav.state.position);

        telemetry_write_row(telemetry_file, t_ms, "SUBMARINE", &nav.state, 0U, &guidance);

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
        "Resultado: pos.z=%.1f Pa (esperado %.1f) | vel.z=%.1f Pa/s (esperado %.1f)\n",
        nav.state.position.z,
        kSurfacePressurePa + (kSubmersionPressureRatePaS * 10.0f),
        nav.state.velocity.z,
        kSubmersionPressureRatePaS);
}

} // namespace

int main()
{
    std::printf("NaviCore-3D — Simulador de estres PC\n");
    std::printf("Escenario seleccionado: %s\n", sensor_scenario_name(kSelectedScenario));

    FILE *telemetry_file = telemetry_open(kTelemetryCsvPath);
    if (telemetry_file == NULL) {
        std::printf("ERROR: no se pudo crear %s\n", kTelemetryCsvPath);
        return 1;
    }

    telemetry_write_header(telemetry_file);
    std::printf("Telemetria CSV: %s\n", kTelemetryCsvPath);

    run_sensor_scenario(telemetry_file, kSelectedScenario);
    run_scenario_submarine(telemetry_file);

    std::fclose(telemetry_file);

    std::printf("\nSimulacion completada. Gemelo Digital: %s\n", kTelemetryCsvPath);
    return 0;
}
