#include "telemetry_udp.hpp"

#include "ins_ekf.hpp"

#include <iostream>

#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif

#if !defined(_WIN32)
#include <unistd.h>
#endif

bool navigation_state_pack_from_ekf(
    const InsEkfFilter *ekf,
    uint32_t timestamp_ms,
    uint32_t health_flags,
    NavigationState *out_state)
{
    return ins_ekf_pack_navigation_state(ekf, timestamp_ms, health_flags, out_state);
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
