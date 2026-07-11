/**
 * @file main_ambiq.cpp
 * @brief Orquestador del ciclo de navegacion determinista para Ambiq Apollo
 */
#include "../../core/fusion.hpp"
#include "ambiq_system.hpp"
#include "bsp_sensors.hpp"

int main(void)
{
    Ambiq_LowPower_SystemInit();

    static DeadReckoningFilter nav_filter;
    dead_reckoning_init(&nav_filter, vector3d_zero(), NAVICORE_DOMAIN_AIR);

    if (!bsp_sensors_init()) {
        return 1;
    }

    PowerMetrics current_tick_power{};

    while (1) {
        Ambiq_Hardware_Timer_WaitNextTick();

        (void)bsp_sensors_orchestrate_tick(&nav_filter);
        Ambiq_BSP_GetPowerMetrics(&current_tick_power);

        Ambiq_System_AdvanceTick();
        Ambiq_MCU_Enter_DeepSleep();
    }

    return 0;
}
