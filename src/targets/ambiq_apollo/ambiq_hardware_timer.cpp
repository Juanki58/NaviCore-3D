/**
 * @file ambiq_hardware_timer.cpp
 * @brief STIMER AmbiqSuite @ 32.768 kHz, tick periodico 100 ms, deep sleep sincronizado
 */
#include "ambiq_system.hpp"

#include "drivers/ambiq_driver_config.hpp"

#include <stdint.h>

volatile bool g_hardware_tick_ready = false;

#ifdef NAVICORE_AMBIQ_SDK

#include "am_hal_stimer.h"
#include "am_hal_sysctrl.h"
#include "am_mcu_apollo.h"

static void ambiq_hardware_timer_configure_nvic(void)
{
    NVIC_ClearPendingIRQ(STIMER_CMPR0_IRQn);
    NVIC_SetPriority(STIMER_CMPR0_IRQn, AM_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(STIMER_CMPR0_IRQn);
}

static void ambiq_hardware_timer_configure_stimer(void)
{
    uint32_t cfg = am_hal_stimer_config(AM_HAL_STIMER_CFG_CLEAR | AM_HAL_STIMER_CFG_FREEZE);

    cfg = am_hal_stimer_config(
        (cfg & ~(AM_HAL_STIMER_CFG_FREEZE | STIMER_STCFG_CLKSEL_Msk))
        | AM_HAL_STIMER_XTAL_32KHZ
        | AM_HAL_STIMER_CFG_COMPARE_A_ENABLE);

    (void)cfg;

    ambiq_hardware_timer_configure_nvic();

    (void)am_hal_stimer_int_enable(AM_HAL_STIMER_INT_COMPAREA);
    (void)am_hal_stimer_compare_delta_set(
        AMBIQ_STIMER_COMPARE_INSTANCE,
        AMBIQ_STIMER_TICK_CYCLES);
}

extern "C" void am_stimer_isr(void)
{
    const uint32_t int_status = am_hal_stimer_int_status_get(false);

    if ((int_status & AM_HAL_STIMER_INT_COMPAREA) != 0U) {
        (void)am_hal_stimer_int_clear(AM_HAL_STIMER_INT_COMPAREA);
        g_hardware_tick_ready = true;
        (void)am_hal_stimer_compare_delta_set(
            AMBIQ_STIMER_COMPARE_INSTANCE,
            AMBIQ_STIMER_TICK_CYCLES);
    }
}

#else /* !NAVICORE_AMBIQ_SDK */

static void ambiq_hardware_timer_configure_stimer(void)
{
    /*
     * Stub host/sin SDK: sin periferico STIMER real.
     * El contador monotono se expone via am_hal_stimer_counter_get().
     */
}

extern "C" uint32_t am_hal_stimer_counter_get(void)
{
    static uint32_t s_stub_counter = 0U;
    s_stub_counter++;
    return s_stub_counter;
}

#endif /* NAVICORE_AMBIQ_SDK */

extern "C" void Ambiq_Hardware_Timer_Init(void)
{
    g_hardware_tick_ready = false;
    ambiq_hardware_timer_configure_stimer();
}

extern "C" void Ambiq_Hardware_Timer_WaitNextTick(void)
{
#ifdef NAVICORE_AMBIQ_SDK
    g_hardware_tick_ready = false;

    while (!g_hardware_tick_ready) {
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
    }

    g_hardware_tick_ready = false;
#else
    /*
     * Host/stub: avance determinista sin bloqueo ni deep sleep real.
     */
    (void)am_hal_stimer_counter_get();
#endif
}
