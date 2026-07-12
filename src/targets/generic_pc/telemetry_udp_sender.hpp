#ifndef NAVICORE_TELEMETRY_UDP_SENDER_HPP
#define NAVICORE_TELEMETRY_UDP_SENDER_HPP

#include <cstdint>

void telemetry_udp_init(const char *ip, int port);
void telemetry_udp_send(
    float x,
    float y,
    float z,
    uint16_t score,
    uint8_t health_mode,
    uint16_t dropped);

#endif
