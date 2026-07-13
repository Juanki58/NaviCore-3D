/**
 * @file main.cpp
 * @brief NaviCore-3D en Raspberry Pi Pico 2 W — laboratorio Comarruga @ 100 Hz
 */
#include "bsp_sensors.hpp"
#include "hw_config.hpp"

#include "core/fusion.hpp"
#include "core/navigation_cortex.hpp"
#include "core/vector3d.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
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

volatile bool g_tick_ready = false;

bool repeating_timer_callback(struct repeating_timer *timer)
{
    (void)timer;
    g_tick_ready = true;
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
        ip4addr_ntoa(netif_ip4_addr(cyw43_state.netif[0])));

    if (!pico2_bsp_sensors_init()) {
        printf("Error: BSP sensores Comarruga\n");
        cyw43_arch_deinit();
        return -1;
    }

    DeadReckoningFilter nav_filter{};
    dead_reckoning_init(&nav_filter, vector3d_zero(), NAVICORE_DOMAIN_AIR);

    struct repeating_timer timer;
    add_repeating_timer_ms(-static_cast<int64_t>(PICO2_NAV_TICK_MS), repeating_timer_callback, nullptr, &timer);

    watchdog_enable(PICO2_WDT_TIMEOUT_MS, true);

    gpio_init(PICO2_GPIO_BENCHMARK);
    gpio_set_dir(PICO2_GPIO_BENCHMARK, GPIO_OUT);
    gpio_put(PICO2_GPIO_BENCHMARK, 0);

    printf(
        "NavigationCortex @ %u Hz — WDT %u ms — WCET GP%u — hardware Comarruga validado\n",
        1000U / PICO2_NAV_TICK_MS,
        PICO2_WDT_TIMEOUT_MS,
        PICO2_GPIO_BENCHMARK);

    uint32_t tick_count = 0U;
    while (true) {
        pico2_bsp_sensors_rx_pump();
        watchdog_update();

        if (g_tick_ready) {
            g_tick_ready = false;
            const uint32_t timestamp_ms = tick_count * PICO2_NAV_TICK_MS;
            ++tick_count;

            bool gps_fix_valid = false;
            gpio_put(PICO2_GPIO_BENCHMARK, 1);
            pico2_bsp_sensors_tick(&nav_filter, timestamp_ms, &gps_fix_valid);
            gpio_put(PICO2_GPIO_BENCHMARK, 0);

            (void)HOST_IP;
            (void)UDP_PORT;
            (void)gps_fix_valid;
        }

        pico2_bsp_sensors_housekeeping(tick_count);

        /* Periférico blando: SPI CYW43439 + lwIP; sin timeout SDK acotado. */
        cyw43_arch_poll();

        if (!g_tick_ready && pico2_bsp_sensors_can_sleep()) {
            __wfi();
        }
    }

    return 0;
}
