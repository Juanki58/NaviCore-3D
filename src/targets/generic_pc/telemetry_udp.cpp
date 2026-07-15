#include "telemetry_udp.hpp"

#include "ins_ekf.hpp"
#include "NavState.h"
#include "sensor_types.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif

#if !defined(_WIN32)
#include <unistd.h>
#endif

namespace {

constexpr double kDegToRad = M_PI / 180.0;

float covariance_std(float variance)
{
    if (variance <= 0.0f) {
        return 0.0f;
    }
    return std::sqrt(variance);
}

float combined_rms(float a, float b, float c)
{
    return std::sqrt((a * a) + (b * b) + (c * c));
}

} /* namespace */

bool navigation_state_pack_from_ekf(
    const InsEkfFilter *ekf,
    uint32_t timestamp_ms,
    uint32_t health_flags,
    NavigationState *out_state)
{
    if (ekf == NULL || out_state == NULL || !ekf->initialized) {
        return false;
    }

    NavState nav_snapshot = navstate_zero(ekf->domain);
    ins_ekf_export_nav_state(ekf, &nav_snapshot, timestamp_ms, NULL);

    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    ins_ekf_get_attitude_rad(ekf, &roll_rad, &pitch_rad, &yaw_rad);

    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    ins_ekf_get_velocity_ned(ekf, vel_ned);

    const float pos_std_n = covariance_std(ins_ekf_get_covariance_flat(ekf, 0U));
    const float pos_std_e = covariance_std(ins_ekf_get_covariance_flat(ekf, 16U));
    const float pos_std_d = covariance_std(ins_ekf_get_covariance_flat(ekf, 32U));
    const float att_std_roll = covariance_std(ins_ekf_get_covariance_flat(ekf, 96U));
    const float att_std_pitch = covariance_std(ins_ekf_get_covariance_flat(ekf, 112U));
    const float att_std_yaw = covariance_std(ins_ekf_get_covariance_flat(ekf, 128U));

    NavigationState packed{};
    packed.timestamp_us = static_cast<uint64_t>(timestamp_ms) * 1000ULL;
    packed.lat_rad = static_cast<double>(nav_snapshot.position.x) * kDegToRad;
    packed.lon_rad = static_cast<double>(nav_snapshot.position.y) * kDegToRad;
    packed.alt_m = nav_snapshot.position.z;
    packed.vn_mps = vel_ned[0];
    packed.ve_mps = vel_ned[1];
    packed.vd_mps = vel_ned[2];
    packed.roll_rad = roll_rad;
    packed.pitch_rad = pitch_rad;
    packed.yaw_rad = yaw_rad;
    packed.health_flags = health_flags;
    packed.pos_uncertainty_m = combined_rms(pos_std_n, pos_std_e, pos_std_d);
    packed.att_uncertainty_rad = combined_rms(att_std_roll, att_std_pitch, att_std_yaw);

    *out_state = packed;
    return true;
}

TelemetryUdpSender::TelemetryUdpSender(const char *host, uint16_t port)
    : socket_(INVALID_SOCKET),
      dest_addr_{},
      ready_(false),
      send_failures_(0U)
{
#ifdef _WIN32
    WSADATA wsa_data{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
        std::cerr << "[-] TelemetryUdpSender: error inicializando Winsock" << std::endl;
        return;
    }
#endif

    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (socket_ == INVALID_SOCKET) {
        std::cerr << "[-] TelemetryUdpSender: no se pudo crear el socket UDP" << std::endl;
#ifdef _WIN32
        WSACleanup();
#endif
        return;
    }

    dest_addr_.sin_family = AF_INET;
    dest_addr_.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &dest_addr_.sin_addr) != 1) {
        std::cerr << "[-] TelemetryUdpSender: direccion invalida: " << host << std::endl;
#ifdef _WIN32
        closesocket(socket_);
        WSACleanup();
#else
        close(socket_);
#endif
        socket_ = INVALID_SOCKET;
        return;
    }

    ready_ = true;
    std::cout << "[*] NavigationState UDP -> " << host << ":" << port << std::endl;
}

TelemetryUdpSender::~TelemetryUdpSender()
{
    if (socket_ != INVALID_SOCKET) {
#ifdef _WIN32
        closesocket(socket_);
        WSACleanup();
#else
        close(socket_);
#endif
        socket_ = INVALID_SOCKET;
    }
    ready_ = false;
}

bool TelemetryUdpSender::is_ready() const
{
    return ready_;
}

bool TelemetryUdpSender::send(const NavigationState &state)
{
    if (!ready_) {
        return false;
    }

    const int sent = sendto(
        socket_,
        reinterpret_cast<const char *>(&state),
        sizeof(state),
        0,
        reinterpret_cast<const sockaddr *>(&dest_addr_),
        sizeof(dest_addr_));

    if (sent != static_cast<int>(sizeof(state))) {
        ++send_failures_;
        return false;
    }

    return true;
}

uint32_t TelemetryUdpSender::send_failures() const
{
    return send_failures_;
}
