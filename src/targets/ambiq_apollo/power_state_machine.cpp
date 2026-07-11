/**
 * @file power_state_machine.cpp
 * @brief Maquina de estados de energia — shims HAL Ambiq, memoria estatica, cero heap
 */
#include "power_state_machine.hpp"

#include "ambiq_system.hpp"
#include "drivers/ambiq_driver_config.hpp"

#include <stddef.h>

/*
 * Shims minimos del SDK Ambiq Apollo4.
 * En silicio: enlazar am_hal_pwrctrl.h, am_hal_uart.h, am_hal_sysctrl.h
 * y eliminar estos stubs.
 */
#define POWER_HAL_STATUS_OK    0U
#define POWER_HAL_STATUS_FAIL  1U

#define AM_HAL_SYSCTRL_SLEEP_DEEP 2U

extern "C" uint32_t am_hal_pwrctrl_periph_disable(uint32_t ui32PeriphEnable)
{
    (void)ui32PeriphEnable;
    /*
     * TODO(Ambiq): return am_hal_pwrctrl_periph_disable(ui32PeriphEnable);
     * Stub host: acepta la solicitud sin error.
     */
    return POWER_HAL_STATUS_OK;
}

extern "C" uint32_t am_hal_uart_tx_disable(void *pHandle)
{
    (void)pHandle;
    /*
     * TODO(Ambiq): am_hal_uart_tx_disable(pHandle);
     * Stub host: silencia la linea TX del UART de telemetria.
     */
    return POWER_HAL_STATUS_OK;
}

extern "C" void am_hal_sysctrl_sleep(uint32_t ui32SleepMode)
{
    (void)ui32SleepMode;
    /*
     * TODO(Ambiq): am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
     * Stub host: delega al shim de sistema existente.
     */
    Ambiq_MCU_Enter_DeepSleep();
}

/** Contexto global estatico — unica instancia, sin malloc. */
static struct {
    PowerState current;
    PowerState previous;
    bool shutdown_latched;
    bool uart_silenced;
    uint32_t disabled_periph_mask;
    void *uart_handle;
} g_power_ctx{};

static uint32_t power_hal_map_periph_bit_to_ambiq_enable(uint32_t periph_bit)
{
    /*
     * TODO(Ambiq): mapear POWER_PERIPH_* -> AM_HAL_PWRCTRL_PERIPH_* del SDK.
     * Stub host: devuelve el bit tal cual como identificador opaco.
     */
    return periph_bit;
}

static bool power_hal_disable_peripheral(uint32_t periph_bit)
{
    if ((g_power_ctx.disabled_periph_mask & periph_bit) != 0U) {
        return true;
    }

    const uint32_t status = am_hal_pwrctrl_periph_disable(
        power_hal_map_periph_bit_to_ambiq_enable(periph_bit));

    if (status != POWER_HAL_STATUS_OK) {
        return false;
    }

    g_power_ctx.disabled_periph_mask |= periph_bit;
    return true;
}

static void power_hal_uart_silence(void)
{
    if (g_power_ctx.uart_silenced) {
        return;
    }

    /*
     * TODO(Ambiq): g_power_ctx.uart_handle = handle de AMBIQ_UART_TELEM_INSTANCE.
     */
    (void)AMBIQ_UART_TELEM_INSTANCE;
    (void)AMBIQ_UART_TELEM_BAUD;

    const uint32_t status = am_hal_uart_tx_disable(g_power_ctx.uart_handle);
    if (status == POWER_HAL_STATUS_OK) {
        g_power_ctx.uart_silenced = true;
        (void)power_hal_disable_peripheral(POWER_PERIPH_UART0);
    }
}

static void power_hal_force_deep_sleep(void)
{
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
}

static void power_manager_on_enter_conservation(void)
{
    /*
     * Conservacion: apagar GNSS (GPIO/interrupt) y UART de telemetria;
     * mantener IOM SPI para IMU/barometro criticos de navegacion.
     */
    (void)power_hal_disable_peripheral(POWER_PERIPH_GPIO);
    power_hal_uart_silence();
}

static void power_manager_on_enter_performance(void)
{
    /*
     * Rendimiento pleno: no re-habilita perifericos ya latcheados en shutdown.
     * En silicio real, am_hal_pwrctrl_periph_enable() iria aqui si no hay latch.
     */
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
