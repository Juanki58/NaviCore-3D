/**
 * @file main_ambiq.cpp
 * @brief Orquestador del ciclo de navegacion determinista para Ambiq Apollo
 */
#include "../../core/diagnostic.hpp"
#include "../../core/fusion.hpp"
#include "../../core/NavState.h"
#include "ambiq_system.hpp"
#include "bsp_sensors.hpp"
#include "power_state_machine.hpp"

namespace {

constexpr float kVehicleStoppedSpeedMps = 0.05f;

static uint8_t ambiq_worst_bsp_bus_status(const BspSensorsBusStatus *status)
{
    if (status == NULL) {
        return DIAG_BSP_BUS_IDLE;
    }

    const uint8_t imu = (uint8_t)status->imu;
    const uint8_t baro = (uint8_t)status->baro;
    return (imu >= baro) ? imu : baro;
}

} /* namespace */

int main(void)
{
    Ambiq_LowPower_SystemInit();

    static DeadReckoningFilter nav_filter;
    dead_reckoning_init(&nav_filter, vector3d_zero(), NAVICORE_DOMAIN_AIR);

    if (!bsp_sensors_init()) {
        return 1;
    }

    power_manager_init();

    static SystemHealthMonitor health_monitor{};
    PowerMetrics current_tick_power{};

    while (1) {
        Ambiq_Hardware_Timer_WaitNextTick();

        /*
         * En POWER_SAFE_SHUTDOWN los perifericos ya estan apagados; se omite el
         * orquestador BSP pero se conserva el mismo superloop determinista.
         */
        if (!power_manager_is_shutdown_latched()) {
            (void)bsp_sensors_orchestrate_tick(&nav_filter);
            Ambiq_BSP_GetPowerMetrics(&current_tick_power);
        }

        const uint8_t filter_quality_u8 = diagnostic_filter_quality_from_float(
            nav_filter.state.confidence.estimate_quality);

        BspSensorsBusStatus bus_status{};
        bsp_sensors_get_bus_status(&bus_status);

        diagnostic_update(
            &health_monitor,
            filter_quality_u8,
            ambiq_worst_bsp_bus_status(&bus_status));

        const float speed_mps = navstate_speed_mps(&nav_filter.state);
        const bool vehicle_stopped = speed_mps < kVehicleStoppedSpeedMps;

        power_manager_update((SystemHealthMode)health_monitor.mode, vehicle_stopped);

        Ambiq_System_AdvanceTick();
        Ambiq_MCU_Enter_DeepSleep();
    }

    return 0;
}
