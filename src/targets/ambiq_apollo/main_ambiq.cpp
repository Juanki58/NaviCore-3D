/**
 * @file main_ambiq.cpp
 * @brief Orquestador del ciclo de navegacion determinista para Ambiq Apollo
 */
#include "../../core/fusion.hpp"
#include "ambiq_system.hpp"
#include "bsp_sensors.hpp"
#include "drivers/ambiq_driver_config.hpp"

int main(void)
{
    Ambiq_LowPower_SystemInit();

    static DeadReckoningFilter nav_filter;
    static bool nav_initialized = false;

    if (!nav_initialized) {
        dead_reckoning_init(&nav_filter, vector3d_zero(), NAVICORE_DOMAIN_AIR);
        nav_initialized = true;
    }

    NaviCore::IMUMeasurement current_imu{};
    NaviCore::GNSSMeasurement current_gnss{};
    PowerMetrics current_tick_power{};

    while (1) {
        Ambiq_Hardware_Timer_WaitNextTick();

        const uint32_t timestamp_ms = Ambiq_System_GetTickIndex() * AMBIQ_TICK_INTERVAL_MS;

        Ambiq_BSP_ReadIMU(&current_imu);
        Ambiq_BSP_ReadGNSS(&current_gnss);

        current_imu.timestamp_ms = timestamp_ms;
        current_gnss.timestamp_ms = timestamp_ms;

        dead_reckoning_update_imu(&nav_filter, &current_imu);

        if (current_gnss.fix_valid) {
            dead_reckoning_update_gps(&nav_filter, &current_gnss);
        }

        nav_filter.state.timestamp_ms = timestamp_ms;

        Ambiq_BSP_TransmitState(&nav_filter.state);
        Ambiq_BSP_GetPowerMetrics(&current_tick_power);

        Ambiq_System_AdvanceTick();
        Ambiq_MCU_Enter_DeepSleep();
    }

    return 0;
}
