#include "vehicle_bus_adapter.hpp"

namespace NaviCore {
namespace VehicleBus {

ImuSample imu_from_can(const ImuCanFrame &frame)
{
    ImuSample sample{};

    sample.accel_mps2[0] = frame.acc_x;
    sample.accel_mps2[1] = frame.acc_y;
    sample.accel_mps2[2] = frame.acc_z;
    sample.gyro_radps[0] = frame.gyro_x;
    sample.gyro_radps[1] = frame.gyro_y;
    sample.gyro_radps[2] = frame.gyro_z;
    sample.mag_ut[0] = 0.0f;
    sample.mag_ut[1] = 0.0f;
    sample.mag_ut[2] = 0.0f;
    sample.timestamp_ms = frame.timestamp_ms;
    sample.valid = true;

    return sample;
}

WheelOdometry odo_from_can(const OdoCanFrame &frame)
{
    WheelOdometry odo{};

    odo.speed_mps = frame.speed_mps;
    odo.reverse = frame.reverse;
    odo.timestamp_ms = frame.timestamp_ms;

    return odo;
}

} /* namespace VehicleBus */
} /* namespace NaviCore */
