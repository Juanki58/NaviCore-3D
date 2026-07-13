/**
 * @file power_state_machine.cpp
 * @brief Maquina de estados de energia — stub host para NaviCore3D_Sim
 */
#include "power_state_machine.hpp"

#include <stddef.h>

namespace {

void host_enter_deep_sleep_stub(void)
{
    /* Stub host: en silicio embebido, delegar a HAL de deep sleep del target. */
}

} /* namespace */

static struct {
    PowerState current;
    PowerState previous;
    bool shutdown_latched;
    bool uart_silenced;
    uint32_t disabled_periph_mask;
    void *uart_handle;
} g_power_ctx{};

static bool power_hal_disable_peripheral(uint32_t periph_bit)
{
    if ((g_power_ctx.disabled_periph_mask & periph_bit) != 0U) {
        return true;
    }

    g_power_ctx.disabled_periph_mask |= periph_bit;
    return true;
}

static void power_hal_uart_silence(void)
{
    if (g_power_ctx.uart_silenced) {
        return;
    }

    (void)g_power_ctx.uart_handle;
    g_power_ctx.uart_silenced = true;
    (void)power_hal_disable_peripheral(POWER_PERIPH_UART0);
}

static void power_hal_force_deep_sleep(void)
{
    host_enter_deep_sleep_stub();
}

static void power_manager_on_enter_conservation(void)
{
    (void)power_hal_disable_peripheral(POWER_PERIPH_GPIO);
    power_hal_uart_silence();
}

static void power_manager_on_enter_performance(void)
{
}

static void power_manager_on_enter_safe_shutdown(void)
{
    g_power_ctx.shutdown_latched = true;

    power_hal_uart_silence();
    (void)power_hal_disable_peripheral(POWER_PERIPH_IOM0);
    (void)power_hal_disable_peripheral(POWER_PERIPH_IOM1);
    (void)power_hal_disable_peripheral(POWER_PERIPH_GPIO);

    power_hal_force_deep_sleep();
}

static void power_manager_apply_state_entry(PowerState new_state)
{
    if (new_state == g_power_ctx.previous) {
        return;
    }

    switch (new_state) {
    case POWER_PERFORMANCE:
        power_manager_on_enter_performance();
        break;
    case POWER_CONSERVATION:
        power_manager_on_enter_conservation();
        break;
    case POWER_SAFE_SHUTDOWN:
        power_manager_on_enter_safe_shutdown();
        break;
    default:
        break;
    }

    g_power_ctx.previous = new_state;
}

static PowerState power_manager_resolve_target(
    SystemHealthMode health_mode,
    bool vehicle_stopped)
{
    if (g_power_ctx.shutdown_latched) {
        return POWER_SAFE_SHUTDOWN;
    }

    if (health_mode == HEALTH_CRITICAL && vehicle_stopped) {
        return POWER_SAFE_SHUTDOWN;
    }

    if (health_mode == HEALTH_DEGRADED ||
        (health_mode == HEALTH_CRITICAL && !vehicle_stopped)) {
        return POWER_CONSERVATION;
    }

    return POWER_PERFORMANCE;
}

void power_manager_init(void)
{
    g_power_ctx.current = POWER_PERFORMANCE;
    g_power_ctx.previous = POWER_PERFORMANCE;
    g_power_ctx.shutdown_latched = false;
    g_power_ctx.uart_silenced = false;
    g_power_ctx.disabled_periph_mask = 0U;
    g_power_ctx.uart_handle = NULL;
}

void power_manager_update(SystemHealthMode health_mode, bool vehicle_stopped)
{
    const PowerState target = power_manager_resolve_target(health_mode, vehicle_stopped);

    if (target != g_power_ctx.current) {
        g_power_ctx.current = target;
        power_manager_apply_state_entry(target);
    }

    if (g_power_ctx.current == POWER_SAFE_SHUTDOWN) {
        power_hal_force_deep_sleep();
    }
}

PowerState power_manager_get_state(void)
{
    return g_power_ctx.current;
}

bool power_manager_is_shutdown_latched(void)
{
    return g_power_ctx.shutdown_latched;
}

bool power_manager_is_uart_silenced(void)
{
    return g_power_ctx.uart_silenced;
}

uint32_t power_manager_get_disabled_periph_mask(void)
{
    return g_power_ctx.disabled_periph_mask;
}
