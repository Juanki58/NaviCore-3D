/**
 * @file ambiq_system_stub.cpp
 * @brief Stubs sistema Apollo — clock, STimer, deep sleep
 */
#include "ambiq_system.hpp"

static uint32_t g_tick_index = 0U;

extern "C" void Ambiq_LowPower_SystemInit(void)
{
    /* TODO(Ambiq): am_hal_pwrctrl_lowpower_init(); am_hal_cachectrl_config(); FPU enable */
    g_tick_index = 0U;
}

extern "C" void Ambiq_Hardware_Timer_WaitNextTick(void)
{
    /*
     * TODO(Ambiq): WFI hasta interrupcion STimer @ AMBIQ_TICK_INTERVAL_MS
     * Stub host: retorno inmediato.
     */
}

extern "C" void Ambiq_MCU_Enter_DeepSleep(void)
{
    /* TODO(Ambiq): am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); */
}

uint32_t Ambiq_System_GetTickIndex(void)
{
    return g_tick_index;
}

void Ambiq_System_AdvanceTick(void)
{
    g_tick_index++;
}
