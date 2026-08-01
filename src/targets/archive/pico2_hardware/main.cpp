/**
 * @file main.cpp
 * @brief NaviCore-3D en Raspberry Pi Pico 2 W — laboratorio Comarruga @ 100 Hz
 */
#include "bsp_sensors.hpp"
#include "bsp_power.hpp"
#include "bsp_ext_wdt.hpp"
#include "health_monitor.hpp"
#include "hw_config.hpp"
#include "loop_metrics.hpp"
#include "nav_state_udp.hpp"
#include "safe_log.hpp"
#include "task_monitor.hpp"

#include "core/ins_ekf.hpp"
#include "core/navigation_state.hpp"
#include "core/NavState.h"
#include "core/navigation_cortex.hpp"
#include "core/vector3d.h"

#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "pico/cyw43_arch.h"
#include "pico/stdio.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include <atomic>

#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#define WIFI_SSID "TU_WIFI_DE_CASA"
#define WIFI_PASSWORD "TU_CONTRASEÑA"
#define HOST_IP "192.168.1.100"
#define UDP_PORT 5005
#endif

namespace {

/* Contador atómico: la ISR solo incrementa; un bool pierde ticks si el bucle > 10 ms. */
std::atomic<uint32_t> g_tick_ready{0U};

bool repeating_timer_callback(struct repeating_timer *timer)
{
    (void)timer;
    g_tick_ready.fetch_add(1U, std::memory_order_release);
    return true;
}

} /* namespace */

static_assert(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS == 0, "USB stdio debe ser no bloqueante");

int main()
{
    stdio_usb_init();
    stdio_set_translate_crlf(&stdio_usb, false);
    safe_log_init();
    loop_metrics_init();
    health_monitor_init();
    task_monitor_init();

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

    if (!pico2_nav_state_udp_init(HOST_IP, UDP_PORT)) {
        printf("Aviso: NavigationState UDP no disponible (%s:%u)\n", HOST_IP, UDP_PORT);
    } else {
        printf("NavigationState UDP -> %s:%u (64 B)\n", HOST_IP, UDP_PORT);
    }

    if (!pico2_bsp_sensors_init()) {
        printf("Error: BSP sensores Comarruga\n");
        cyw43_arch_deinit();
        return -1;
    }

    InsEkfFilter ins_filter{};
    NavState nav_state = navstate_zero(NAVICORE_DOMAIN_AIR);

    struct repeating_timer timer;
    add_repeating_timer_ms(-static_cast<int64_t>(PICO2_NAV_TICK_MS), repeating_timer_callback, nullptr, &timer);

    watchdog_enable(PICO2_WDT_TIMEOUT_MS, true);
    if (pico2_bsp_ext_wdt_init()) {
        printf("External HW WDT supervisor armed on GP%u (kick end-of-loop only)\n", PICO2_EXT_WDT_GPIO);
    } else {
        printf(
            "MCU on-chip WDT %u ms armed; external supervisor disabled "
            "(set PICO2_EXT_WDT_ENABLE=1 + TPL5010/MAX6822 on GP%u)\n",
            PICO2_WDT_TIMEOUT_MS,
            PICO2_EXT_WDT_GPIO);
    }

    gpio_init(PICO2_GPIO_BENCHMARK);
    gpio_set_dir(PICO2_GPIO_BENCHMARK, GPIO_OUT);
    gpio_put(PICO2_GPIO_BENCHMARK, 0);

    gpio_init(PICO2_GPIO_BENCHMARK_WIFI);
    gpio_set_dir(PICO2_GPIO_BENCHMARK_WIFI, GPIO_OUT);
    gpio_put(PICO2_GPIO_BENCHMARK_WIFI, 0);

    printf(
        "NavigationCortex @ %u Hz — WDT %u ms — WCET GP%u tick GP%u wifi — hardware Comarruga validado\n",
        1000U / PICO2_NAV_TICK_MS,
        PICO2_WDT_TIMEOUT_MS,
        PICO2_GPIO_BENCHMARK,
        PICO2_GPIO_BENCHMARK_WIFI);

    uint32_t tick_count = 0U;
    while (true) {
        const uint64_t loop_start_us = time_us_64();
        uint64_t phase_start_us = loop_start_us;

        if (g_tick_ready.load(std::memory_order_acquire) > 0U) {
            health_monitor_check_task_deadline(
                TaskId::Loop,
                PICO2_LOOP_MAX_IDLE_US,
                "loop");
        }

        health_monitor_check_task_deadline(
            TaskId::RxPump,
            PICO2_RX_PUMP_MAX_IDLE_US,
            "rx_pump");

        pico2_bsp_sensors_rx_pump();
        task_monitor_record(TaskId::RxPump, tick_count);
        loop_metrics_record_rx_pump_us(static_cast<uint32_t>(time_us_64() - phase_start_us));
        phase_start_us = time_us_64();

        const uint32_t pending_ticks = g_tick_ready.fetch_sub(1U, std::memory_order_acq_rel);
        if (pending_ticks == 0U) {
            g_tick_ready.fetch_add(1U, std::memory_order_release);
        } else {
            loop_metrics_record_tick_backlog(pending_ticks);

            const uint32_t timestamp_ms = tick_count * PICO2_NAV_TICK_MS;
            ++tick_count;

            if (health_monitor_nav_update_allowed(pending_ticks)) {
                health_monitor_check_task_deadline(
                    TaskId::NavTick,
                    PICO2_NAV_TICK_MAX_IDLE_US,
                    "nav_tick");

                bool gps_fix_valid = false;
                gpio_put(PICO2_GPIO_BENCHMARK, 1);
                pico2_bsp_sensors_tick(&ins_filter, &nav_state, timestamp_ms, &gps_fix_valid);
                gpio_put(PICO2_GPIO_BENCHMARK, 0);
                task_monitor_record(TaskId::NavTick, tick_count);

                if (ins_filter.initialized && pico2_nav_state_udp_is_ready()) {
                    uint32_t nav_flags = NAV_STATE_FLAG_EKF_VALID;
                    if (gps_fix_valid) {
                        nav_flags |= NAV_STATE_FLAG_GPS_FIX;
                    }
                    if (ins_ekf_nhc_enabled(&ins_filter)) {
                        nav_flags |= NAV_STATE_FLAG_NHC_ENABLED;
                    }
                    if (ins_ekf_outlier_detected(&ins_filter)) {
                        nav_flags |= NAV_STATE_FLAG_GNSS_OUTLIER;
                    }

                    NavigationState nav_packet{};
                    if (ins_ekf_pack_navigation_state(
                            &ins_filter,
                            timestamp_ms,
                            nav_flags,
                            &nav_packet)) {
                        (void)pico2_nav_state_udp_send(&nav_packet);
                    }
                }
            }

            loop_metrics_record_tick_us(static_cast<uint32_t>(time_us_64() - phase_start_us));
            phase_start_us = time_us_64();
        }

        health_monitor_check_task_deadline(
            TaskId::Housekeeping,
            PICO2_HOUSEKEEPING_MAX_IDLE_US,
            "housekeeping");
        pico2_bsp_sensors_housekeeping(tick_count);
        task_monitor_record(TaskId::Housekeeping, tick_count);
        loop_metrics_record_housekeeping_us(static_cast<uint32_t>(time_us_64() - phase_start_us));
        phase_start_us = time_us_64();

        const uint32_t loop_elapsed_us = static_cast<uint32_t>(time_us_64() - loop_start_us);
        const bool wifi_budget_ok = (loop_elapsed_us < PICO2_LOOP_BUDGET_US)
            && ((PICO2_LOOP_BUDGET_US - loop_elapsed_us) >= PICO2_WIFI_MIN_REMAINING_US);
        if (health_monitor_wifi_poll_allowed() && wifi_budget_ok) {
            health_monitor_check_task_deadline(
                TaskId::Wifi,
                PICO2_WIFI_MAX_IDLE_US,
                "wifi");

            gpio_put(PICO2_GPIO_BENCHMARK_WIFI, 1);
            cyw43_arch_poll();
            gpio_put(PICO2_GPIO_BENCHMARK_WIFI, 0);
            task_monitor_record(TaskId::Wifi, tick_count);
            loop_metrics_record_wifi_us(static_cast<uint32_t>(time_us_64() - phase_start_us));
        } else {
            loop_metrics_add_wifi_skipped();
        }
        phase_start_us = time_us_64();

        loop_metrics_sync_uart_overflows(
            pico2_bsp_uart_get_overflow_count(PICO2_UART_ID_IMU),
            pico2_bsp_uart_get_overflow_count(PICO2_UART_ID_GNSS));

        safe_log_flush_pending();
        task_monitor_record(TaskId::Logging, tick_count);
        loop_metrics_record_logging_us(static_cast<uint32_t>(time_us_64() - phase_start_us));

        const uint64_t loop_elapsed = time_us_64() - loop_start_us;
        loop_metrics_on_loop_complete(loop_elapsed);
        task_monitor_record(TaskId::Loop, tick_count);

        SensorConfidenceFlags confidence{};
        pico2_bsp_sensors_get_confidence_flags(&confidence);
        health_monitor_on_loop_complete(
            static_cast<uint32_t>(loop_elapsed),
            tick_count * PICO2_NAV_TICK_MS,
            confidence.imu_degraded,
            confidence.gnss_degraded,
            pico2_bsp_power_is_offline());

        loop_metrics_report_due();

        /* On-chip RP2350 WDT + optional external supervisor (independent die).
         * Kick ONLY here — never from I2C recovery — so a hung path cannot pet. */
        watchdog_update();
        pico2_bsp_ext_wdt_kick();

        if (g_tick_ready.load(std::memory_order_acquire) == 0U
            && pico2_bsp_sensors_can_sleep()) {
            __wfi();
        }
    }

    return 0;
}
