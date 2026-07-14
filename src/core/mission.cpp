#include "mission.hpp"

#include "NavState.h"
#include "guidance.hpp"
#include "waypoint.hpp"

#include <math.h>
#include <string.h>

#ifndef NAVICORE_MISSION_PI_F
#define NAVICORE_MISSION_PI_F 3.14159265358979323846f
#endif

static float mission_deg_to_rad(float deg)
{
    return deg * (NAVICORE_MISSION_PI_F / 180.0f);
}

static void mission_copy_route(StaticWaypointBuffer *dst, const StaticWaypointBuffer *src)
{
    if (dst == NULL || src == NULL) {
        return;
    }

    memcpy(dst, src, sizeof(StaticWaypointBuffer));
}

static bool mission_covariance_ready(
    float cov_pos_n_m2,
    float cov_pos_e_m2,
    float cov_pos_d_m2,
    float cov_pos_max_m2)
{
    return (cov_pos_n_m2 <= cov_pos_max_m2)
        && (cov_pos_e_m2 <= cov_pos_max_m2)
        && (cov_pos_d_m2 <= cov_pos_max_m2);
}

static bool mission_route_is_loaded(const MissionController *controller, bool route_loaded_input)
{
    if (controller == NULL) {
        return false;
    }

    if (route_loaded_input) {
        return true;
    }

    return controller->route.count >= 2U;
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
        static_cast<uint32_t>(NAVICORE_GUIDANCE_HOME_ARRIVAL_RADIUS_M);

    const Waypoint start_wp = waypoint_make(
        "RTN0",
        nav_state->position,
        nav_state->domain,
        arrival_radius_m,
        NAVICORE_WAYPOINT_DEFAULT_TRANSIT_SPEED_MPS);
    const Waypoint home_wp = waypoint_make(
        "HOME",
        controller->home,
        nav_state->domain,
        arrival_radius_m,
        NAVICORE_WAYPOINT_DEFAULT_TERMINAL_SPEED_MPS);

    waypoint_buffer_push(&controller->return_route, start_wp);
    waypoint_buffer_push(&controller->return_route, home_wp);
}

static void mission_begin_return_home(
    MissionController *controller,
    const NavState *nav_state)
{
    if (controller == NULL || nav_state == NULL || !controller->home_valid) {
        return;
    }

    mission_build_return_route(controller, nav_state);
    controller->state = MISSION_STATE_RETURN_HOME;
    controller->active_waypoint_index = 0U;
    controller->return_home_requested = false;
}

static void mission_enter_safe_mode(
    MissionController *controller,
    MissionOutput *output,
    MissionSafeModeCause cause)
{
    if (controller == NULL || output == NULL) {
        return;
    }

    controller->state = MISSION_STATE_SAFE_MODE;
    controller->armed = false;
    controller->return_home_requested = false;
    controller->safe_cause = cause;
    controller->gnss_loss_timer_s = 0.0f;
    controller->nis_reject_streak = 0U;

    output->state = MISSION_STATE_SAFE_MODE;
    output->safe_mode = true;
    output->safe_cause = cause;
    output->guidance_active = false;
    output->guidance_valid = false;
    output->safe_commands_active = true;
}

static bool mission_terminal_home_satisfied(
    const GuidanceOutput &guidance,
    const NavState *nav_state,
    const MissionConfig *config)
{
    if (!guidance.waypoint_completed || nav_state == NULL || config == NULL) {
        return false;
    }

    if (!config->require_terminal_speed_at_home) {
        return true;
    }

    return navstate_speed_mps(nav_state) <= config->terminal_speed_mps;
}

static void mission_update_guidance(
    MissionController *controller,
    const MissionInput *input,
    MissionOutput *output)
{
    if (controller == NULL || input == NULL || output == NULL
        || input->guidance == NULL || input->nav_state == NULL
        || !output->guidance_active) {
        return;
    }

    GuidanceProfile profile = input->guidance->get_profile();
    profile.require_terminal_speed_at_home = controller->config.require_terminal_speed_at_home;
    profile.terminal_speed_mps = controller->config.terminal_speed_mps;

    if (output->return_home_active && controller->home_valid) {
        output->guidance = guidance_compute_homing(
            *input->nav_state,
            controller->home,
            profile);
        output->guidance_valid = output->guidance.valid;
        controller->active_waypoint_index = 0U;
        output->active_waypoint_index = 0U;

        if (mission_terminal_home_satisfied(
                output->guidance,
                input->nav_state,
                &controller->config)) {
            mission_enter_safe_mode(controller, output, MISSION_SAFE_CAUSE_NONE);
        }
        return;
    }

    output->guidance = input->guidance->compute(*input->nav_state);
    output->guidance_valid = output->guidance.valid;
    controller->active_waypoint_index = input->guidance->active_waypoint_index();
    output->active_waypoint_index = controller->active_waypoint_index;

    if (!output->guidance.route_completed) {
        return;
    }

    if (controller->state == MISSION_STATE_NAVIGATE && controller->home_valid) {
        mission_begin_return_home(controller, input->nav_state);
        output->state = MISSION_STATE_RETURN_HOME;
        output->return_home_active = true;
        output->guidance_active = true;
        output->active_route = &controller->return_route;
        output->active_waypoint_index = 0U;

        output->guidance = guidance_compute_homing(
            *input->nav_state,
            controller->home,
            profile);
        output->guidance_valid = output->guidance.valid;
    }
}

static bool mission_evaluate_safety(
    MissionController *controller,
    const MissionInput *input,
    MissionOutput *output)
{
    if (controller == NULL || input == NULL || output == NULL) {
        return false;
    }

    if (controller->state == MISSION_STATE_SAFE_MODE) {
        return true;
    }

    if (mission_runtime_health_critical(input->runtime_health)) {
        mission_enter_safe_mode(controller, output, MISSION_SAFE_CAUSE_RUNTIME_FAULT);
        return true;
    }

    if (input->runtime_health != NULL && input->runtime_health->loop_budget_exceeded > 0U) {
        mission_enter_safe_mode(controller, output, MISSION_SAFE_CAUSE_LOOP_OVERRUN);
        return true;
    }

    const bool gnss_loss_monitor_active =
        (controller->state == MISSION_STATE_READY)
        || (controller->state == MISSION_STATE_NAVIGATE)
        || (controller->state == MISSION_STATE_RETURN_HOME);

    if (gnss_loss_monitor_active) {
        if (input->gps_fix_valid) {
            controller->gnss_loss_timer_s = 0.0f;
        } else if (input->dt_s > 0.0f) {
            controller->gnss_loss_timer_s += input->dt_s;
            if (controller->gnss_loss_timer_s >= controller->config.gnss_loss_timeout_s) {
                mission_enter_safe_mode(controller, output, MISSION_SAFE_CAUSE_GNSS_LOSS);
                return true;
            }
        }
    } else {
        controller->gnss_loss_timer_s = 0.0f;
    }

    if (input->gnss_nis_rejected
        && input->gnss_nis >= controller->config.nis_critical) {
        if (controller->nis_reject_streak < 0xFFFFFFFFU) {
            ++controller->nis_reject_streak;
        }
    } else {
        controller->nis_reject_streak = 0U;
    }

    if (controller->nis_reject_streak >= controller->config.nis_reject_streak_max) {
        mission_enter_safe_mode(controller, output, MISSION_SAFE_CAUSE_NIS_REJECT);
        return true;
    }

    return false;
}

void mission_config_default(MissionConfig *config)
{
    if (config == NULL) {
        return;
    }

    memset(config, 0, sizeof(MissionConfig));
    config->cov_pos_max_m2 = NAVICORE_MISSION_COV_POS_MAX_M2;
    config->gnss_loss_timeout_s = NAVICORE_MISSION_GNSS_LOSS_TIMEOUT_S;
    config->nis_reject_streak_max = NAVICORE_MISSION_NIS_REJECT_STREAK_MAX;
    config->nis_critical = NAVICORE_MISSION_NIS_CRITICAL;
    config->safe_vertical_rate_mps = NAVICORE_MISSION_SAFE_VERTICAL_RATE_MPS;
    config->gps_min_satellites = NAVICORE_MISSION_GPS_MIN_SATELLITES;
    config->gps_min_quality = NAVICORE_MISSION_GPS_MIN_QUALITY;
    config->gps_stable_ticks = NAVICORE_MISSION_GPS_STABLE_TICKS;
    config->require_terminal_speed_at_home = false;
    config->terminal_speed_mps = NAVICORE_GUIDANCE_TERMINAL_SPEED_MPS;
}

void mission_fill_safe_mode_commands(
    const NavState *nav_state,
    const MissionConfig *config,
    GuidanceCommands *commands_out)
{
    if (commands_out == NULL) {
        return;
    }

    memset(commands_out, 0, sizeof(GuidanceCommands));

    if (nav_state != NULL) {
        commands_out->desired_heading = mission_deg_to_rad(nav_state->heading_deg);
    }

    commands_out->desired_speed = 0.0f;

    if (nav_state != NULL && config != NULL) {
        commands_out->desired_climb = config->safe_vertical_rate_mps;
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

void mission_init(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    memset(controller, 0, sizeof(MissionController));
    controller->state = MISSION_STATE_INIT;
    mission_config_default(&controller->config);
    waypoint_buffer_init(&controller->route);
    waypoint_buffer_init(&controller->return_route);
}

void mission_set_route(MissionController *controller, const StaticWaypointBuffer *route)
{
    if (controller == NULL || route == NULL) {
        return;
    }

    mission_copy_route(&controller->route, route);
    controller->active_waypoint_index = 0U;
}

void mission_arm_system(MissionController *controller, bool arm_system)
{
    if (controller == NULL) {
        return;
    }

    controller->armed = arm_system;
}

void mission_request_return_home(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    controller->return_home_requested = true;
}

MissionState mission_state(const MissionController *controller)
{
    if (controller == NULL) {
        return MISSION_STATE_INIT;
    }

    return controller->state;
}

const char *mission_state_name(MissionState state)
{
    switch (state) {
    case MISSION_STATE_INIT:
        return "INIT";
    case MISSION_STATE_WAIT_GPS:
        return "WAIT_GPS";
    case MISSION_STATE_READY:
        return "READY";
    case MISSION_STATE_NAVIGATE:
        return "NAVIGATE";
    case MISSION_STATE_RETURN_HOME:
        return "RETURN_HOME";
    case MISSION_STATE_SAFE_MODE:
        return "SAFE_MODE";
    default:
        return "UNKNOWN";
    }
}

const char *mission_safe_cause_name(MissionSafeModeCause cause)
{
    switch (cause) {
    case MISSION_SAFE_CAUSE_NONE:
        return "NONE";
    case MISSION_SAFE_CAUSE_GNSS_LOSS:
        return "GNSS_LOSS";
    case MISSION_SAFE_CAUSE_NIS_REJECT:
        return "NIS_REJECT";
    case MISSION_SAFE_CAUSE_LOOP_OVERRUN:
        return "LOOP_OVERRUN";
    case MISSION_SAFE_CAUSE_RUNTIME_FAULT:
        return "RUNTIME_FAULT";
    case MISSION_SAFE_CAUSE_MANUAL:
        return "MANUAL";
    default:
        return "UNKNOWN";
    }
}

void mission_update(
    MissionController *controller,
    const MissionInput *input,
    MissionOutput *output)
{
    if (controller == NULL || input == NULL || output == NULL) {
        return;
    }

    memset(output, 0, sizeof(MissionOutput));
    output->state = controller->state;
    output->control_outputs_enabled = true;
    output->active_route = &controller->route;
    output->active_waypoint_index = controller->active_waypoint_index;
    output->safe_cause = controller->safe_cause;

    if (mission_evaluate_safety(controller, input, output)) {
        if (input->nav_state != NULL) {
            mission_fill_safe_mode_commands(
                input->nav_state,
                &controller->config,
                &output->safe_commands);
        }
        output->safe_commands_active = true;
        output->control_outputs_enabled = true;
        output->state = controller->state;
        return;
    }

    switch (controller->state) {
    case MISSION_STATE_INIT:
        if (input->ekf_calibrated) {
            controller->state = MISSION_STATE_WAIT_GPS;
        }
        break;

    case MISSION_STATE_WAIT_GPS: {
        const bool gps_ok = input->gps_fix_valid
            && (input->satellites >= controller->config.gps_min_satellites)
            && (input->estimate_quality >= controller->config.gps_min_quality);

        const bool cov_ok = mission_covariance_ready(
            input->cov_pos_n_m2,
            input->cov_pos_e_m2,
            input->cov_pos_d_m2,
            controller->config.cov_pos_max_m2);

        if (gps_ok && cov_ok && input->nav_state != NULL) {
            if (controller->gps_stable_streak < 0xFFFFU) {
                ++controller->gps_stable_streak;
            }
        } else {
            controller->gps_stable_streak = 0U;
        }

        if (controller->gps_stable_streak >= controller->config.gps_stable_ticks
            && input->nav_state != NULL) {
            controller->home = input->nav_state->position;
            controller->home_valid = true;
            controller->state = MISSION_STATE_READY;
            controller->gps_stable_streak = 0U;
        }
        break;
    }

    case MISSION_STATE_READY:
        if ((controller->armed || input->arm_system)
            && mission_route_is_loaded(controller, input->route_loaded)) {
            controller->state = MISSION_STATE_NAVIGATE;
            controller->active_waypoint_index = 0U;
            controller->armed = true;
            if (input->guidance != NULL) {
                input->guidance->set_route(controller->route);
            }
        }
        break;

    case MISSION_STATE_NAVIGATE:
        output->guidance_active = true;
        if (controller->return_home_requested && controller->home_valid && input->nav_state != NULL) {
            mission_begin_return_home(controller, input->nav_state);
        }
        break;

    case MISSION_STATE_RETURN_HOME:
        output->guidance_active = true;
        output->return_home_active = true;
        output->active_route = &controller->return_route;
        output->active_waypoint_index = controller->active_waypoint_index;
        break;

    case MISSION_STATE_SAFE_MODE:
        output->safe_mode = true;
        output->safe_commands_active = true;
        if (input->nav_state != NULL) {
            mission_fill_safe_mode_commands(
                input->nav_state,
                &controller->config,
                &output->safe_commands);
        }
        output->state = controller->state;
        return;

    default:
        break;
    }

    output->state = controller->state;

    if (output->return_home_active) {
        output->active_route = &controller->return_route;
    }

    mission_update_guidance(controller, input, output);

    if (output->safe_mode) {
        output->safe_commands_active = true;
        if (input->nav_state != NULL) {
            mission_fill_safe_mode_commands(
                input->nav_state,
                &controller->config,
                &output->safe_commands);
        }
    }

    output->state = controller->state;
}

void mission_on_waypoint_completed(MissionController *controller)
{
    if (controller == NULL) {
        return;
    }

    if (controller->state == MISSION_STATE_RETURN_HOME) {
        if (controller->active_waypoint_index + 1U < controller->return_route.count) {
            ++controller->active_waypoint_index;
        }
    }
}

void mission_controller_init(MissionController *controller)
{
    mission_init(controller);
}

void mission_controller_set_route(MissionController *controller, const StaticWaypointBuffer *route)
{
    mission_set_route(controller, route);
}

void mission_controller_request_start(MissionController *controller)
{
    mission_arm_system(controller, true);
}

void mission_controller_request_return_home(MissionController *controller)
{
    mission_request_return_home(controller);
}

MissionState mission_controller_state(const MissionController *controller)
{
    return mission_state(controller);
}

bool mission_controller_tick(
    MissionController *controller,
    const MissionTickInput *input,
    MissionTickOutput *output)
{
    if (controller == NULL || input == NULL || output == NULL) {
        return false;
    }

    MissionInput wrapped = *input;
    if (!wrapped.route_loaded) {
        wrapped.route_loaded = controller->route.count >= 2U;
    }

    mission_update(controller, &wrapped, output);
    return true;
}

void mission_controller_on_waypoint_completed(MissionController *controller)
{
    mission_on_waypoint_completed(controller);
}
