/**
 * @file ambiq_system_stub.cpp
 * @brief Stubs sistema Apollo — clock, STimer, deep sleep
 */
#include "ambiq_system.hpp"

#include "drivers/ambiq_dma.hpp"
#include "drivers/ambiq_driver_config.hpp"
#include "drivers/ambiq_gpio_gnss.hpp"
#include "drivers/ambiq_power_monitor.hpp"
#include "drivers/ambiq_spi_imu.hpp"
#include "drivers/ambiq_uart_telemetry.hpp"

static uint32_t g_tick_index = 0U;

static void ambiq_drivers_init(void)
{
    ambiq_dma_init();
    ambiq_spi_imu_init();
    ambiq_gpio_gnss_init();
    ambiq_uart_telemetry_init();
    ambiq_power_monitor_init();
}

extern "C" void Ambiq_LowPower_SystemInit(void)
{
    /* TODO(Ambiq): am_hal_pwrctrl_lowpower_init(); am_hal_cachectrl_config(); FPU enable */
    ambiq_drivers_init();
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
