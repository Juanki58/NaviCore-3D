#pragma once

#include "NavState.h"
#include "runtime_health.hpp"
#include "waypoint.hpp"

#include <stdbool.h>
#include <stdint.h>

enum class MissionState : uint8_t {
    INIT = 0,
    WAIT_GPS = 1,
    READY = 2,
    NAVIGATE = 3,
    RETURN_HOME = 4,
    SAFE_MODE = 5,
};

#ifndef NAVICORE_MISSION_INIT_TICKS
#define NAVICORE_MISSION_INIT_TICKS 5U
#endif

#ifndef NAVICORE_MISSION_GPS_STABLE_TICKS
#define NAVICORE_MISSION_GPS_STABLE_TICKS 10U
#endif

#ifndef NAVICORE_MISSION_GPS_MIN_QUALITY
#define NAVICORE_MISSION_GPS_MIN_QUALITY 0.55f
#endif

#ifndef NAVICORE_MISSION_GPS_MIN_SATELLITES
#define NAVICORE_MISSION_GPS_MIN_SATELLITES 6U
#endif

#ifndef NAVICORE_MISSION_SAFE_MISSED_TICKS
#define NAVICORE_MISSION_SAFE_MISSED_TICKS 5U
#endif

#ifndef NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX
#define NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX 3U
#endif

#ifndef NAVICORE_MISSION_HOME_ARRIVAL_RADIUS_M
#define NAVICORE_MISSION_HOME_ARRIVAL_RADIUS_M 5.0f
#endif

struct MissionController {
    MissionState state;
    Vector3D home;
    bool home_valid;
    bool start_requested;
    bool return_home_requested;
    uint32_t state_tick_count;
    uint32_t gps_stable_streak;
    size_t active_waypoint_index;
    StaticWaypointBuffer route;
    StaticWaypointBuffer return_route;
};

struct MissionTickInput {
    const NavState *nav_state;
    const RuntimeHealth *runtime_health;
    bool gps_fix_valid;
    uint8_t satellites;
    float estimate_quality;
    bool start_signal;
    uint32_t timestamp_ms;
};

struct MissionTickOutput {
    MissionState state;
    bool guidance_active;
    bool safe_mode;
    bool return_home_active;
    const StaticWaypointBuffer *active_route;
    size_t active_waypoint_index;
};

void mission_controller_init(MissionController *controller);
void mission_controller_set_route(MissionController *controller, const StaticWaypointBuffer *route);
void mission_controller_request_start(MissionController *controller);
void mission_controller_request_return_home(MissionController *controller);
MissionState mission_controller_state(const MissionController *controller);
const char *mission_state_name(MissionState state);

bool mission_controller_tick(
    MissionController *controller,
    const MissionTickInput *input,
    MissionTickOutput *output);

void mission_controller_on_waypoint_completed(MissionController *controller);

bool mission_runtime_health_critical(const RuntimeHealth *health);
