#pragma once

#include "navigation_state.hpp"

#include <cstdint>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
typedef int SOCKET;
#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif
#endif

struct InsEkfFilter;

constexpr uint16_t TELEMETRY_UDP_NAV_STATE_PORT = 9090U;
constexpr const char *TELEMETRY_UDP_NAV_STATE_HOST = "127.0.0.1";

bool navigation_state_pack_from_ekf(
    const InsEkfFilter *ekf,
    uint32_t timestamp_ms,
    uint32_t health_flags,
    NavigationState *out_state);

class TelemetryUdpSender {
public:
    TelemetryUdpSender(
        const char *host = TELEMETRY_UDP_NAV_STATE_HOST,
        uint16_t port = TELEMETRY_UDP_NAV_STATE_PORT);
    ~TelemetryUdpSender();

    TelemetryUdpSender(const TelemetryUdpSender &) = delete;
    TelemetryUdpSender &operator=(const TelemetryUdpSender &) = delete;

    bool is_ready() const;
    bool send(const NavigationState &state);
    uint32_t send_failures() const;

private:
    SOCKET socket_;
    sockaddr_in dest_addr_;
    bool ready_;
    uint32_t send_failures_;
};
