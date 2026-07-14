#pragma once

#include "NavState.h"
#include "guidance.hpp"
#include "runtime_health.hpp"
#include "waypoint.hpp"

#include <stdbool.h>
#include <stdint.h>

/*
 * HFSM de mision y seguridad — agnostica de plataforma, zero-heap, float/FPU.
 *
 * Emite consignas cinematicas (GuidanceCommands) o consignas de seguridad en
 * SAFE_MODE. La capa target traduce a actuadores (freno, timon, motor, VTOL…).
 */

typedef enum : uint8_t {
    MISSION_STATE_INIT = 0,
    MISSION_STATE_WAIT_GPS = 1,
    MISSION_STATE_READY = 2,
    MISSION_STATE_NAVIGATE = 3,
    MISSION_STATE_RETURN_HOME = 4,
    MISSION_STATE_SAFE_MODE = 5,
} MissionState;

typedef enum : uint8_t {
    MISSION_SAFE_CAUSE_NONE = 0,
    MISSION_SAFE_CAUSE_GNSS_LOSS = 1,
    MISSION_SAFE_CAUSE_NIS_REJECT = 2,
    MISSION_SAFE_CAUSE_LOOP_OVERRUN = 3,
    MISSION_SAFE_CAUSE_RUNTIME_FAULT = 4,
    MISSION_SAFE_CAUSE_MANUAL = 5,
} MissionSafeModeCause;

#ifndef NAVICORE_MISSION_GPS_STABLE_TICKS
#define NAVICORE_MISSION_GPS_STABLE_TICKS 10U
#endif

#ifndef NAVICORE_MISSION_GPS_MIN_QUALITY
#define NAVICORE_MISSION_GPS_MIN_QUALITY 0.55f
#endif

#ifndef NAVICORE_MISSION_GPS_MIN_SATELLITES
#define NAVICORE_MISSION_GPS_MIN_SATELLITES 6U
#endif

#ifndef NAVICORE_MISSION_COV_POS_MAX_M2
#define NAVICORE_MISSION_COV_POS_MAX_M2 4.0f
#endif

#ifndef NAVICORE_MISSION_GNSS_LOSS_TIMEOUT_S
#define NAVICORE_MISSION_GNSS_LOSS_TIMEOUT_S 3.0f
#endif

#ifndef NAVICORE_MISSION_NIS_REJECT_STREAK_MAX
#define NAVICORE_MISSION_NIS_REJECT_STREAK_MAX 10U
#endif

#ifndef NAVICORE_MISSION_NIS_CRITICAL
#define NAVICORE_MISSION_NIS_CRITICAL 11.345f
#endif

#ifndef NAVICORE_MISSION_SAFE_VERTICAL_RATE_MPS
#define NAVICORE_MISSION_SAFE_VERTICAL_RATE_MPS 0.0f
#endif

#ifndef NAVICORE_MISSION_SAFE_MISSED_TICKS
#define NAVICORE_MISSION_SAFE_MISSED_TICKS 5U
#endif

#ifndef NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX
#define NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX 3U
#endif

typedef struct {
    float cov_pos_max_m2;
    float gnss_loss_timeout_s;
    uint32_t nis_reject_streak_max;
    float nis_critical;
    float safe_vertical_rate_mps;
    uint8_t gps_min_satellites;
    float gps_min_quality;
    uint32_t gps_stable_ticks;
    bool require_terminal_speed_at_home;
    float terminal_speed_mps;
} MissionConfig;

typedef struct {
    MissionState state;
    Vector3D home;
    bool home_valid;
    bool armed;
    bool return_home_requested;
    uint32_t gps_stable_streak;
    size_t active_waypoint_index;
    float gnss_loss_timer_s;
    uint32_t nis_reject_streak;
    MissionSafeModeCause safe_cause;
    MissionConfig config;
    StaticWaypointBuffer route;
    StaticWaypointBuffer return_route;
} MissionController;

typedef struct {
    float dt_s;
    const NavState *nav_state;
    const RuntimeHealth *runtime_health;
    Guidance3D *guidance;
    bool gps_fix_valid;
    uint8_t satellites;
    float estimate_quality;
    bool ekf_calibrated;
    float cov_pos_n_m2;
    float cov_pos_e_m2;
    float cov_pos_d_m2;
    float gnss_nis;
    bool gnss_nis_rejected;
    bool arm_system;
    bool route_loaded;
} MissionInput;

typedef struct {
    MissionState state;
    bool guidance_active;
    bool guidance_valid;
    bool control_outputs_enabled;
    bool safe_mode;
    bool return_home_active;
    MissionSafeModeCause safe_cause;
    const StaticWaypointBuffer *active_route;
    size_t active_waypoint_index;
    GuidanceOutput guidance;
    GuidanceCommands safe_commands;
    bool safe_commands_active;
} MissionOutput;

/* Retrocompatibilidad con integraciones previas */
typedef MissionInput MissionTickInput;
typedef MissionOutput MissionTickOutput;

void mission_config_default(MissionConfig *config);
void mission_init(MissionController *controller);
void mission_set_route(MissionController *controller, const StaticWaypointBuffer *route);
void mission_arm_system(MissionController *controller, bool arm_system);
void mission_request_return_home(MissionController *controller);

MissionState mission_state(const MissionController *controller);
const char *mission_state_name(MissionState state);
const char *mission_safe_cause_name(MissionSafeModeCause cause);

void mission_fill_safe_mode_commands(
    const NavState *nav_state,
    const MissionConfig *config,
    GuidanceCommands *commands_out);

bool mission_runtime_health_critical(const RuntimeHealth *health);

/*
 * Actualizacion sincrona @ 100 Hz (input->dt_s tipicamente 0.01f).
 */
void mission_update(
    MissionController *controller,
    const MissionInput *input,
    MissionOutput *output);

void mission_on_waypoint_completed(MissionController *controller);

/* Alias retrocompatible */
void mission_controller_init(MissionController *controller);
void mission_controller_set_route(MissionController *controller, const StaticWaypointBuffer *route);
void mission_controller_request_start(MissionController *controller);
void mission_controller_request_return_home(MissionController *controller);
MissionState mission_controller_state(const MissionController *controller);
bool mission_controller_tick(
    MissionController *controller,
    const MissionTickInput *input,
    MissionTickOutput *output);
void mission_controller_on_waypoint_completed(MissionController *controller);
