#include "navigation_cortex.hpp"

#include "recovery_guard.hpp"

namespace {

void push_event(NavigationDecision *decision, uint8_t id, uint8_t param)
{
    if (decision == NULL || decision->event_count >= NAV_CORTEX_MAX_EVENTS_PER_TICK) {
        return;
    }

    decision->events[decision->event_count].id = id;
    decision->events[decision->event_count].param = param;
    ++decision->event_count;
}

void update_health_ema(NavigationCortexState *state, uint8_t health_score)
{
    if (state == NULL) {
        return;
    }

    if (state->health_ema == 0U) {
        state->health_ema = health_score;
        return;
    }

    state->health_ema =
        static_cast<uint8_t>((static_cast<uint16_t>(state->health_ema) * 7U + health_score) / 8U);
}

} // namespace

void navigation_cortex_init(NavigationCortexState *state)
{
    if (state == NULL) {
        return;
    }

    state->prev_health = HEALTH_NOMINAL;
    state->prev_nav_mode = NAV_MODE_INITIALIZING;
    state->prev_gps_valid = true;
    state->prev_safe_stop = false;
    state->health_ema = DIAG_HEALTH_SCORE_MAX;
}

void navigation_cortex_step(
    NavigationCortexState *state,
    const NavigationCortexInput *input,
    NavigationDecision *decision)
{
    if (state == NULL || input == NULL || decision == NULL ||
        input->filter == NULL || input->monitor == NULL || input->nav_state == NULL) {
        return;
    }

    decision->requires_safe_stop = false;
    decision->hot_restart = false;
    decision->predictive_degrade = false;
    decision->telemetry_tier = NAV_CORTEX_TELEMETRY_FULL;
    decision->contingency_flags = NAV_CORTEX_CONTINGENCY_NONE;
    decision->event_count = 0U;

    if (!input->skip_diagnostic_update) {
        diagnostic_update(input->monitor, input->filter_quality, input->bsp_bus_status);
    }

    const bool recovered = recovery_guard_step(
        input->filter,
        input->monitor,
        input->monitor->last_divergence_innovation_sq);

    if (recovered) {
        decision->hot_restart = true;
        push_event(decision, NAV_EVENT_HOT_RESTART, input->monitor->health_score);
    }

    update_health_ema(state, input->monitor->health_score);

    if (input->monitor->last_time_guard_error == TIME_GUARD_ERROR_WCET) {
        push_event(decision, NAV_EVENT_WCET_VIOLATION, input->monitor->health_score);
        decision->contingency_flags |= NAV_CORTEX_CONTINGENCY_WIDEN_WAYPOINT;
    }

    if (input->monitor->mode != state->prev_health) {
        if (input->monitor->mode == HEALTH_DEGRADED) {
            push_event(decision, NAV_EVENT_HEALTH_DEGRADED, input->monitor->health_score);
            decision->telemetry_tier = NAV_CORTEX_TELEMETRY_FULL;
        } else if (input->monitor->mode == HEALTH_CRITICAL) {
            push_event(decision, NAV_EVENT_HEALTH_CRITICAL, input->monitor->health_score);
            decision->telemetry_tier = NAV_CORTEX_TELEMETRY_EVENT_ONLY;
        } else if (input->monitor->mode == HEALTH_NOMINAL) {
            push_event(decision, NAV_EVENT_HEALTH_NOMINAL, input->monitor->health_score);
        }
    }

    if (input->gps_fix_valid != state->prev_gps_valid) {
        if (!input->gps_fix_valid) {
            push_event(decision, NAV_EVENT_GPS_LOST, input->nav_state->confidence.satellites);
        } else {
            push_event(decision, NAV_EVENT_GPS_RESTORED, input->nav_state->confidence.satellites);
        }
    }

    if (state->health_ema >= DIAG_HEALTH_SCORE_DEGRADED_MIN &&
        input->monitor->health_score < DIAG_HEALTH_SCORE_DEGRADED_MIN &&
        input->monitor->health_score + 5U < state->health_ema) {
        decision->predictive_degrade = true;
        push_event(decision, NAV_EVENT_PREDICTIVE_DEGRADE, state->health_ema);
        decision->contingency_flags |= NAV_CORTEX_CONTINGENCY_WIDEN_WAYPOINT;
    }

    decision->requires_safe_stop = diagnostic_requires_safe_stop(input->monitor);

    if (decision->requires_safe_stop && !state->prev_safe_stop) {
        push_event(decision, NAV_EVENT_SAFE_STOP, input->monitor->health_score);
        decision->telemetry_tier = NAV_CORTEX_TELEMETRY_EVENT_ONLY;
    }

    if (input->monitor->mode == HEALTH_DEGRADED) {
        decision->contingency_flags |= NAV_CORTEX_CONTINGENCY_WIDEN_WAYPOINT;
    }

    if (input->monitor->shutdown_latched) {
        decision->telemetry_tier = NAV_CORTEX_TELEMETRY_SILENT;
    }

    state->prev_health = input->monitor->mode;
    state->prev_nav_mode = input->nav_state->mode;
    state->prev_gps_valid = input->gps_fix_valid;
    state->prev_safe_stop = decision->requires_safe_stop;
}
