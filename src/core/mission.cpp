#include "mission.hpp"

#include "waypoint.hpp"

#include <math.h>
#include <string.h>

static void mission_copy_route(StaticWaypointBuffer *dst, const StaticWaypointBuffer *src)
{
    if (dst == NULL || src == NULL) {
        return;
    }

    memcpy(dst, src, sizeof(StaticWaypointBuffer));
}

static float mission_horizontal_distance_m(Vector3D a, Vector3D b)
{
    const float dlat_m = (b.x - a.x) * 111132.954f;
    const float mean_lat_rad = (a.x + b.x) * 0.5f * (3.14159265358979323846f / 180.0f);
    const float dlon_m = (b.y - a.y) * 111132.954f * cosf(mean_lat_rad);
    return sqrtf((dlat_m * dlat_m) + (dlon_m * dlon_m));
}

static void mission_build_return_route(
    MissionController *controller,
    const NavState *nav_state)
{
    if (controller == NULL || nav_state == NULL || !controller->home_valid) {
        return;
    }

    waypoint_buffer_init(&controller->return_route);

    const uint32_t arrival_radius_m =
        static_cast<uint32_t>(NAVICORE_MISSION_HOME_ARRIVAL_RADIUS_M);

    const Waypoint start_wp = waypoint_make(
        "RTN0",
        nav_state->position,
        nav_state->domain,
        arrival_radius_m);
    const Waypoint home_wp = waypoint_make(
        "HOME",
        controller->home,
        nav_state->domain,
        arrival_radius_m);

    waypoint_buffer_push(&controller->return_route, start_wp);
    waypoint_buffer_push(&controller->return_route, home_wp);
}

static void mission_advance_waypoint(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    const StaticWaypointBuffer *route = (controller->state == MissionState::RETURN_HOME)
        ? &controller->return_route
        : &controller->route;

    if (controller->active_waypoint_index + 1U < route->count) {
        ++controller->active_waypoint_index;
    }
}

bool mission_runtime_health_critical(const RuntimeHealth *health)
{
    if (health == NULL) {
        return false;
    }

    if (health->missed_ticks > NAVICORE_MISSION_SAFE_MISSED_TICKS) {
        return true;
    }

    if (health->uart0_overflows >= NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX) {
        return true;
    }

    if (health->uart1_overflows >= NAVICORE_MISSION_SAFE_UART_OVERFLOW_MAX) {
        return true;
    }

    return false;
}

void mission_controller_init(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(MissionController));
    controller->state = MissionState::INIT;
    waypoint_buffer_init(&controller->route);
    waypoint_buffer_init(&controller->return_route);
}

void mission_controller_set_route(MissionController *controller, const StaticWaypointBuffer *route)
{
    if (controller == NULL || route == NULL) {
        return;
    }

    mission_copy_route(&controller->route, route);
    controller->active_waypoint_index = 0U;
}

void mission_controller_request_start(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->start_requested = true;
}

void mission_controller_request_return_home(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->return_home_requested = true;
}

MissionState mission_controller_state(const MissionController *controller)
{
    if (controller == NULL) {
        return MissionState::INIT;
    }

    return controller->state;
}

const char *mission_state_name(MissionState state)
{
    switch (state) {
    case MissionState::INIT:
        return "INIT";
    case MissionState::WAIT_GPS:
        return "WAIT_GPS";
    case MissionState::READY:
        return "READY";
    case MissionState::NAVIGATE:
        return "NAVIGATE";
    case MissionState::RETURN_HOME:
        return "RETURN_HOME";
    case MissionState::SAFE_MODE:
        return "SAFE_MODE";
    default:
        return "UNKNOWN";
    }
}

bool mission_controller_tick(
    MissionController *controller,
    const MissionTickInput *input,
    MissionTickOutput *output)
{
    if (controller == NULL || input == NULL || output == NULL) {
        return false;
    }

    memset(output, 0, sizeof(MissionTickOutput));
    output->state = controller->state;
    output->active_route = &controller->route;
    output->active_waypoint_index = controller->active_waypoint_index;

    if (mission_runtime_health_critical(input->runtime_health)) {
        controller->state = MissionState::SAFE_MODE;
    }

    switch (controller->state) {
    case MissionState::INIT:
        ++controller->state_tick_count;
        if (controller->state_tick_count >= NAVICORE_MISSION_INIT_TICKS) {
            controller->state = MissionState::WAIT_GPS;
            controller->state_tick_count = 0U;
        }
        break;

    case MissionState::WAIT_GPS: {
        const bool gps_ok = input->gps_fix_valid
            && (input->satellites >= NAVICORE_MISSION_GPS_MIN_SATELLITES)
            && (input->estimate_quality >= NAVICORE_MISSION_GPS_MIN_QUALITY);

        if (gps_ok && input->nav_state != NULL) {
            if (controller->gps_stable_streak < 0xFFFFU) {
                ++controller->gps_stable_streak;
            }
        } else {
            controller->gps_stable_streak = 0U;
        }

        if (controller->gps_stable_streak >= NAVICORE_MISSION_GPS_STABLE_TICKS
            && input->nav_state != NULL) {
            controller->home = input->nav_state->position;
            controller->home_valid = true;
            controller->state = MissionState::READY;
            controller->state_tick_count = 0U;
        }
        break;
    }

    case MissionState::READY:
        if (controller->start_requested || input->start_signal) {
            controller->state = MissionState::NAVIGATE;
            controller->active_waypoint_index = 0U;
            controller->start_requested = false;
        }
        break;

    case MissionState::NAVIGATE:
        output->guidance_active = true;
        if (controller->return_home_requested && controller->home_valid && input->nav_state != NULL) {
            mission_build_return_route(controller, input->nav_state);
            controller->state = MissionState::RETURN_HOME;
            controller->active_waypoint_index = 0U;
            controller->return_home_requested = false;
        }
        break;

    case MissionState::RETURN_HOME:
        output->guidance_active = true;
        output->return_home_active = true;
        output->active_route = &controller->return_route;
        output->active_waypoint_index = controller->active_waypoint_index;

        if (controller->return_route.count >= 2U && input->nav_state != NULL) {
            controller->return_route.items[controller->return_route.head].position =
                input->nav_state->position;
        }

        if (controller->home_valid && input->nav_state != NULL) {
            const float dist_home_m = mission_horizontal_distance_m(
                input->nav_state->position,
                controller->home);
            if (dist_home_m <= NAVICORE_MISSION_HOME_ARRIVAL_RADIUS_M) {
                controller->state = MissionState::READY;
                controller->start_requested = false;
            }
        }
        break;

    case MissionState::SAFE_MODE:
        output->safe_mode = true;
        break;

    default:
        break;
    }

    output->state = controller->state;
    return true;
}

void mission_controller_on_waypoint_completed(MissionController *controller)
{
    if (controller == NULL || controller->return_home_requested) {
        return;
    }

    if (controller->state == MissionState::NAVIGATE) {
        if (controller->route.count < 2U) {
            return;
        }

        const size_t last_leg_index = controller->route.count - 2U;
        if (controller->active_waypoint_index >= last_leg_index) {
            controller->return_home_requested = true;
        } else if (controller->active_waypoint_index + 1U < controller->route.count) {
            ++controller->active_waypoint_index;
        }
        return;
    }

    if (controller->state == MissionState::RETURN_HOME) {
        mission_advance_waypoint(controller);
    }
}
