/**
 * @file vehicle_bus_can_stub.cpp
 * @brief Stubs del bus CAN + integración On_Vehicle_Bus_Data_Received
 */
#include "vehicle_bus_adapter.hpp"

#include <cstdio>

namespace {

uint32_t g_vehicle_bus_tick_ms = 0U;

float Leer_Acelerometro_Bus_CAN(void)
{
    return 0.05f;
}

float Leer_Giroscopio_Bus_CAN(void)
{
    return 0.002f;
}

float Leer_Velocidad_Ruedas_Actual(void)
{
    return 15.0f;
}

bool Check_Marcha_Atras(void)
{
    return false;
}

void Actualizar_Pantalla_Navegador(float pos_x, float pos_y)
{
    std::printf("HMI nav: lat=%.6f lon=%.6f\n", pos_x, pos_y);
}

} /* namespace */

namespace NaviCore {
namespace VehicleBus {

void On_Vehicle_Bus_Data_Received(void)
{
    g_vehicle_bus_tick_ms += 100U;

    ImuCanFrame imu_frame{};
    imu_frame.acc_x = Leer_Acelerometro_Bus_CAN();
    imu_frame.acc_y = 0.0f;
    imu_frame.acc_z = 9.80665f;
    imu_frame.gyro_x = 0.0f;
    imu_frame.gyro_y = 0.0f;
    imu_frame.gyro_z = Leer_Giroscopio_Bus_CAN();
    imu_frame.timestamp_ms = g_vehicle_bus_tick_ms;
    Ingest_IMU(imu_from_can(imu_frame));

    OdoCanFrame odo_frame{};
    odo_frame.speed_mps = Leer_Velocidad_Ruedas_Actual();
    odo_frame.reverse = Check_Marcha_Atras();
    odo_frame.timestamp_ms = g_vehicle_bus_tick_ms;
    Ingest_WheelOdometry(odo_from_can(odo_frame));

    VehicleNavOutput hmi{};
    Get_VehicleNavOutput(&hmi);
    Actualizar_Pantalla_Navegador(hmi.pos_x, hmi.pos_y);
}

} /* namespace VehicleBus */
} /* namespace NaviCore */

#ifdef NAVICORE_VEHICLE_BUS_DEMO
int main(void)
{
    NaviCore::Initialize(NAVICORE_DOMAIN_AIR, vector3d_make(41.3874f, 2.1686f, 12.0f));

    for (int tick = 0; tick < 5; ++tick) {
        NaviCore::VehicleBus::On_Vehicle_Bus_Data_Received();
    }

    return 0;
}
#endif
