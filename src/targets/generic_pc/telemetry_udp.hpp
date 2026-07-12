#ifndef NAVICORE_TELEMETRY_UDP_HPP
#define NAVICORE_TELEMETRY_UDP_HPP

#include <cstdint>

/** Magic "NC" (NaviCore) en little-endian → 0x4E43 en wire. */
constexpr uint16_t TELEMETRY_UDP_MAGIC = 0x4E43U;
constexpr uint16_t TELEMETRY_UDP_DEFAULT_PORT = 5005U;
constexpr const char *TELEMETRY_UDP_DEFAULT_HOST = "127.0.0.1";

#pragma pack(push, 1)
struct RemoteTelemetryPacket {
    uint16_t magic;
    uint16_t seq;
    uint32_t timestamp_ms;
    float pos_x;
    float pos_y;
    float pos_z;
    uint16_t health_score;
    uint16_t status_flags;
};
#pragma pack(pop)

static_assert(sizeof(RemoteTelemetryPacket) == 24U, "RemoteTelemetryPacket debe ocupar 24 bytes");

#endif
