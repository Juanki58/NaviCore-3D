#include "api_ingest.hpp"

#include "fusion.hpp"

namespace NaviCore {
namespace {

DeadReckoningFilter g_filter{};
bool g_initialized = false;

void ensure_initialized(void)
{
    if (!g_initialized) {
        dead_reckoning_init(&g_filter, vector3d_zero(), NAVICORE_DOMAIN_AIR);
        g_initialized = true;
    }
}

} /* namespace */

void Initialize(NavDomain domain, Vector3D initial_position)
{
    dead_reckoning_init(&g_filter, initial_position, domain);
    g_initialized = true;
}

void Ingest_IMU(const ImuSample &imu_data)
{
    ensure_initialized();
    dead_reckoning_update_imu(&g_filter, &imu_data, NULL);
}

void Ingest_GNSS(const GpsSample &gnss_data)
{
    ensure_initialized();
    dead_reckoning_update_gps(&g_filter, &gnss_data, NULL);
}

void Ingest_WheelOdometry(const WheelOdometry &odo_data)
{
    ensure_initialized();
    dead_reckoning_update_wheel_odometry(
        &g_filter,
        odo_data.speed_mps,
        odo_data.reverse,
        odo_data.timestamp_ms);
}

void Get_CurrentState(::NavState *state_out)
{
    if (state_out == NULL) {
        return;
    }

    ensure_initialized();
    *state_out = g_filter.state;
}

void Get_VehicleNavOutput(VehicleNavOutput *output)
{
    if (output == NULL) {
        return;
    }

    NavState state{};
    Get_CurrentState(&state);

    output->pos_x = state.position.x;
    output->pos_y = state.position.y;
    output->pos_z = state.position.z;
    output->heading_deg = state.heading_deg;
    output->quality = state.confidence.estimate_quality;
}

} /* namespace NaviCore */
