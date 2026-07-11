/**
 * @file vehicle_bus_adapter.hpp
 * @brief Adaptador CAN/vehículo → API de ingestión NaviCore
 */
#pragma once

#include <stdint.h>

#include "api_ingest.hpp"

namespace NaviCore {
namespace VehicleBus {

/**
 * @brief Trama IMU tal como llega del bus ABS/ESP del coche
 */
struct ImuCanFrame {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    uint32_t timestamp_ms;
};

/**
 * @brief Trama de odometría de ruedas del bus del vehículo
 */
struct OdoCanFrame {
    float speed_mps;
    bool reverse;
    uint32_t timestamp_ms;
};

ImuSample imu_from_can(const ImuCanFrame &frame);
WheelOdometry odo_from_can(const OdoCanFrame &frame);

/**
 * @brief Callback principal: lee el bus físico e inyecta al Core
 * @note Implementación en vehicle_bus_can_stub.cpp (stubs) o HAL real en target
 */
void On_Vehicle_Bus_Data_Received(void);

} /* namespace VehicleBus */
} /* namespace NaviCore */
