/**
 * @file ambiq_system_stub.cpp
 * @brief Stubs sistema Apollo — init de bajo consumo y deep sleep
 */
#include "ambiq_system.hpp"

#ifdef NAVICORE_AMBIQ_SDK
#include "am_hal_sysctrl.h"
#endif

static uint32_t g_tick_index = 0U;

extern "C" void Ambiq_LowPower_SystemInit(void)
{
    /* TODO(Ambiq): am_hal_pwrctrl_lowpower_init(); am_hal_cachectrl_config(); FPU enable */
    g_tick_index = 0U;
    Ambiq_Hardware_Timer_Init();
}

extern "C" void Ambiq_MCU_Enter_DeepSleep(void)
{
#ifdef NAVICORE_AMBIQ_SDK
    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
#else
    /* TODO(Ambiq): am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP); */
#endif
}

uint32_t Ambiq_System_GetTickIndex(void)
{
    return g_tick_index;
}

void Ambiq_System_AdvanceTick(void)
{
    g_tick_index++;
}
