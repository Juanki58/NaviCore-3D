/**
 * @file main.cpp
 * @brief NaviCore-3D en Raspberry Pi Pico W — bucle determinista @ 100 Hz + telemetria UDP
 */
#include "bsp_sensors_stub.hpp"
#include "pico_nav_tick.hpp"
#include "telemetry_udp_lwip.hpp"

#include "../../core/fusion.hpp"
#include "../../core/navigation_cortex.hpp"
#include "../ambiq_apollo/power_state_machine.hpp"
#include "../generic_pc/telemetry_udp.hpp"

#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#define WIFI_SSID "TU_WIFI_DE_CASA"
#define WIFI_PASSWORD "TU_CONTRASEÑA"
#define HOST_IP "192.168.1.100"
#define UDP_PORT 5005
#endif

namespace {

constexpr uint32_t kTickPeriodMs = 10U;
constexpr uint32_t kTelemetryDecimate = 10U;
constexpr float kVehicleStoppedSpeedMps = 0.05f;

volatile uint32_t g_tick_ready_count = 0U;

bool repeating_timer_callback(struct repeating_timer *timer)
{
    (void)timer;
    __atomic_fetch_add(&g_tick_ready_count, 1U, __ATOMIC_RELEASE);
    return true;
}

} /* namespace */

int main()
{
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Error: fallo al inicializar Wi-Fi\n");
        return -1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando a %s...\n", WIFI_SSID);

    if (cyw43_arch_wifi_connect_blocking(
            WIFI_SSID,
            WIFI_PASSWORD,
            CYW43_AUTH_WPA2_AES_PSK,
            30000)) {
        printf("Error: no se pudo conectar al Wi-Fi\n");
        cyw43_arch_deinit();
        return -1;
    }

    printf(
        "Conectado. IP local: %s\n",
        ip4addr_ntoa(netif_ip4_addr(cyw43_arch_lwip_netif())));

    if (!telemetry_udp_lwip_init(HOST_IP, static_cast<uint16_t>(UDP_PORT))) {
        printf("Error: telemetria UDP no configurada (HOST_IP=%s)\n", HOST_IP);
        cyw43_arch_deinit();
        return -1;
    }

    if (!pico_bsp_sensors_init()) {
        printf("Error: BSP de sensores\n");
        cyw43_arch_deinit();
        return -1;
    }

    static DeadReckoningFilter nav_filter{};
    const Vector3D origin = vector3d_make(41.2606f, 1.6769f, 12.0f);
    dead_reckoning_init(&nav_filter, origin, NAVICORE_DOMAIN_AIR);

    static SystemHealthMonitor health_monitor{};
    static NavigationCortexState cortex_state{};
    static NavigationDecision decision{};

    power_manager_init();
    navigation_cortex_init(&cortex_state);

    struct repeating_timer timer{};
    add_repeating_timer_ms(-static_cast<int64_t>(kTickPeriodMs), repeating_timer_callback, nullptr, &timer);

    printf("NavigationCortex @ %u Hz — telemetria cada %u ms\n", 1000U / kTickPeriodMs, kTickPeriodMs * kTelemetryDecimate);

    uint32_t tick_index = 0U;

    while (true) {
        const uint32_t pending = __atomic_exchange_n(&g_tick_ready_count, 0U, __ATOMIC_ACQUIRE);
        for (uint32_t i = 0U; i < pending; ++i) {
            const uint32_t timestamp_ms = tick_index * kTickPeriodMs;
            bool gps_fix_valid = false;

            (void)pico_bsp_sensors_tick(&nav_filter, timestamp_ms, &gps_fix_valid);

            const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
                nav_filter.state.confidence.estimate_quality);

            pico_navigation_cortex_tick(
                &cortex_state,
                &nav_filter,
                &health_monitor,
                gps_fix_valid,
                filter_quality_u8,
                DIAG_BSP_BUS_IDLE,
                timestamp_ms,
                &decision);

            const float speed_mps = navstate_speed_mps(&nav_filter.state);
            power_manager_update(
                static_cast<SystemHealthMode>(health_monitor.mode),
                speed_mps < kVehicleStoppedSpeedMps);

            if ((tick_index % kTelemetryDecimate) == 0U) {
                const NavState *state = &nav_filter.state;
                telemetry_udp_lwip_send(
                    timestamp_ms,
                    state->position.x,
                    state->position.y,
                    state->position.z,
                    0.0f,
                    0.0f,
                    health_monitor.health_score,
                    static_cast<uint8_t>(health_monitor.mode),
                    0U,
                    TELEM_SCENARIO_CLEAN,
                    static_cast<uint8_t>(state->mode),
                    25.0f);
            }

            tick_index++;
        }

        cyw43_arch_poll();
    }

    return 0;
}
