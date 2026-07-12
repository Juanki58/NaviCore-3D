#include "pico_nav_tick.hpp"

#include "../ambiq_apollo/power_state_machine.hpp"
#include "telemetry_udp_lwip.hpp"

void pico_navigation_cortex_tick(
    NavigationCortexState *cortex_state,
    DeadReckoningFilter *nav,
    SystemHealthMonitor *health,
    bool gps_fix_valid,
    uint8_t filter_quality_u8,
    uint8_t worst_bsp_bus,
    uint32_t timestamp_ms,
    NavigationDecision *decision)
{
    if (cortex_state == nullptr || nav == nullptr || health == nullptr || decision == nullptr) {
        return;
    }

    health->shutdown_latched = power_manager_is_shutdown_latched();

    NavigationCortexInput input{};
    input.filter = nav;
    input.monitor = health;
    input.nav_state = &nav->state;
    input.gps_fix_valid = gps_fix_valid;
    input.skip_diagnostic_update = false;
    input.filter_quality = filter_quality_u8;
    input.bsp_bus_status = worst_bsp_bus;

    navigation_cortex_step(cortex_state, &input, decision);

    for (uint8_t i = 0U; i < decision->event_count; ++i) {
        telemetry_udp_lwip_send_event(
            timestamp_ms,
            decision->events[i].id,
            decision->events[i].param);
    }
}
