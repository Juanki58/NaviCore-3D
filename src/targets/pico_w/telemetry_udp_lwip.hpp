#ifndef NAVICORE_TELEMETRY_UDP_LWIP_HPP
#define NAVICORE_TELEMETRY_UDP_LWIP_HPP

#include <cstdint>

bool telemetry_udp_lwip_init(const char *host_ip, uint16_t port);
void telemetry_udp_lwip_send(
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
void telemetry_udp_lwip_send_event(uint32_t timestamp_ms, uint8_t event_id, uint8_t param);

#endif
