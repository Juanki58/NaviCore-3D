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
#include "dead_reckoning.h"
#include "gps.h"
#include "imu.h"
#include "pressure.h"

namespace {

constexpr uint32_t kStepMs = 100U;
constexpr float kCruiseSpeedMps = 15.0f;
constexpr float kCruiseCourseDeg = 90.0f;
constexpr float kSurfacePressurePa = 101325.0f;
constexpr float kSubmersionPressureRatePaS = 10000.0f;
constexpr const char *kTelemetryCsvPath = "../docs/telemetria_navicore.csv";

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
    return _mkdir("../docs") == 0 || errno == EEXIST;
#else
    return mkdir("../docs", 0755) == 0 || errno == EEXIST;
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
        "Timestamp_ms,Escenario,Modo,Calidad,Satelites,Pos_X,Pos_Y,Pos_Z,Vel_X,Vel_Y,Vel_Z,Rumbo\n");
}

void telemetry_write_row(
    FILE *file,
    uint32_t timestamp_ms,
    const char *scenario,
    const NavState *state,
    uint8_t scenario_satellites)
{
    if (file == NULL || state == NULL || scenario == NULL) {
        return;
    }

    std::fprintf(
        file,
        "%u,%s,%s,%.6f,%u,%.8f,%.8f,%.4f,%.6f,%.6f,%.6f,%.4f\n",
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
        state->heading_deg);
}

void fill_cruise_imu(ImuSample *imu, uint32_t timestamp_ms)
{
    imu->accel_mps2[0] = 0.0f;
    imu->accel_mps2[1] = 0.0f;
    imu->accel_mps2[2] = 0.0f;
    imu->gyro_radps[0] = 0.0f;
    imu->gyro_radps[1] = 0.0f;
    imu->gyro_radps[2] = 0.0f;
    imu->mag_ut[0] = 22.0f;
    imu->mag_ut[1] = 5.0f;
    imu->mag_ut[2] = 42.0f;
    imu->timestamp_ms = timestamp_ms;
    imu->valid = true;
}

void apply_gps_scenario(GpsSample *gps, uint32_t t_ms, bool gps_available)
{
    if (!gps_available) {
        gps->satellites = 0U;
        gps->fix_valid = false;
        return;
    }

    gps->satellites = 8U;
    gps->fix_valid = true;
    gps->speed_mps = kCruiseSpeedMps;
    gps->course_deg = kCruiseCourseDeg;
    gps->timestamp_ms = t_ms;
}

bool gps_available_at(uint32_t t_ms)
{
    constexpr uint32_t kOutageStartMs = 5000U;
    constexpr uint32_t kOutageEndMs = 15000U;
    return t_ms < kOutageStartMs || t_ms >= kOutageEndMs;
}

void run_scenario_gps_loss(FILE *telemetry_file)
{
    const Vector3D origin = vector3d_make(41.3874f, 2.1686f, 12.0f);

    GpsSimulator gps_sim;
    gps_simulator_init(&gps_sim, origin, kCruiseSpeedMps, kCruiseCourseDeg, 11U);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, origin, NAVICORE_DOMAIN_AIR);

    std::printf("\n");
    std::printf("================================================================\n");
    std::printf(" ESCENARIO 1: Perdida de GPS (Aire/Tierra)\n");
    std::printf("  Velocidad: %.0f m/s | Satelites: 8 (normal) -> 0 (s 5..15)\n", kCruiseSpeedMps);
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

    constexpr uint32_t kDurationMs = 20000U;
    bool prev_gps_available = true;

    for (uint32_t t_ms = 0U; t_ms <= kDurationMs; t_ms += kStepMs) {
        ImuSample imu;
        GpsSample gps;

        fill_cruise_imu(&imu, t_ms);
        gps_simulator_read(&gps_sim, t_ms, &gps);

        const bool gps_available = gps_available_at(t_ms);
        apply_gps_scenario(&gps, t_ms, gps_available);
        const uint8_t scenario_satellites = gps_available ? 8U : 0U;

        if (gps_available && !prev_gps_available) {
            std::printf(">>> t=%.1fs: GPS RECUPERADO (8 satelites)\n", static_cast<float>(t_ms) * 0.001f);
        } else if (!gps_available && prev_gps_available) {
            std::printf(">>> t=%.1fs: PERDIDA DE GPS (0 satelites, 10 s sin fix)\n", static_cast<float>(t_ms) * 0.001f);
        }
        prev_gps_available = gps_available;

        dead_reckoning_update_imu(&nav, &imu);
        if (gps.fix_valid) {
            dead_reckoning_update_gps(&nav, &gps);
        }

        telemetry_write_row(telemetry_file, t_ms, "GPS_LOSS", &nav.state, scenario_satellites);

        if ((t_ms % 1000U) == 0U) {
            const char *note = "";
            if (t_ms == 5000U) {
                note = "<-- inicio outage";
            } else if (t_ms == 15000U) {
                note = "<-- fin outage";
            }

            const float t_s = static_cast<float>(t_ms) * 0.001f;
            std::printf(
                "[t=%5.1fs] mode=%-14s quality=%.3f scenario_sats=%u fix_valid=%s fix_age=%u ms | "
                "heading=%.1f speed=%.2f m/s\n",
                t_s,
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality,
                scenario_satellites,
                gps_available ? "yes" : "no",
                nav.state.confidence.fix_age_ms,
                nav.state.heading_deg,
                navstate_speed_mps(&nav.state));

            if (note[0] != '\0') {
                std::printf("         %s\n", note);
            }
        }
    }

    std::printf("----------------------------------------------------------------\n");
    std::printf("Resultado: tras 10 s sin GPS, mode=%s quality=%.3f (degradacion visible)\n",
                nav_mode_name(nav.state.mode),
                nav.state.confidence.estimate_quality);
}

void run_scenario_submarine(FILE *telemetry_file)
{
    const Vector3D surface = vector3d_make(41.3900f, 2.1750f, kSurfacePressurePa);

    DeadReckoningFilter nav;
    dead_reckoning_init(&nav, surface, NAVICORE_DOMAIN_SEA);

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

        fill_cruise_imu(&imu, t_ms);

        const float t_s = static_cast<float>(t_ms) * 0.001f;
        pressure.pressure_pa = kSurfacePressurePa + (kSubmersionPressureRatePaS * t_s);
        pressure.temperature_c = 10.0f;
        pressure.timestamp_ms = t_ms;
        pressure.valid = true;

        dead_reckoning_update_imu(&nav, &imu);
        dead_reckoning_update_pressure(&nav, &pressure, kSurfacePressurePa);

        telemetry_write_row(telemetry_file, t_ms, "SUBMARINE", &nav.state, 0U);

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
    std::printf("Ejecutando escenarios secuenciales...\n");

    FILE *telemetry_file = telemetry_open(kTelemetryCsvPath);
    if (telemetry_file == NULL) {
        std::printf("ERROR: no se pudo crear %s\n", kTelemetryCsvPath);
        return 1;
    }

    telemetry_write_header(telemetry_file);
    std::printf("Telemetria CSV: %s\n", kTelemetryCsvPath);

    run_scenario_gps_loss(telemetry_file);
    run_scenario_submarine(telemetry_file);

    std::fclose(telemetry_file);

    std::printf("\nSimulacion completada. Gemelo Digital: %s\n", kTelemetryCsvPath);
    return 0;
}
