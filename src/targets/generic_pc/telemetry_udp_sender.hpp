#ifndef NAVICORE_TELEMETRY_UDP_SENDER_HPP
#define NAVICORE_TELEMETRY_UDP_SENDER_HPP

#include <cstdint>

void telemetry_udp_init(const char *ip, int port);
void telemetry_udp_send(
    uint32_t timestamp_ms,
    float x,
    float y,
    float z,
    float cross_track_m,
    float along_track_m,
    uint16_t score,
    uint8_t health_mode,
    uint16_t dropped,
    uint8_t scenario_id,
    uint8_t nav_mode,
    float temperature_c);
uint32_t telemetry_udp_send_failures();
void telemetry_udp_log_stats();

#endif
