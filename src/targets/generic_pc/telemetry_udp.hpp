#ifndef NAVICORE_TELEMETRY_UDP_HPP
#define NAVICORE_TELEMETRY_UDP_HPP

#include <cstdint>

/** Magic "NC" (NaviCore) en little-endian → 0x4E43 en wire. */
constexpr uint16_t TELEMETRY_UDP_MAGIC = 0x4E43U;
constexpr uint16_t TELEMETRY_UDP_DEFAULT_PORT = 5005U;
constexpr const char *TELEMETRY_UDP_DEFAULT_HOST = "127.0.0.1";

enum TelemetryScenarioId : uint8_t {
    TELEM_SCENARIO_HIGH_DEMAND = 0U,
    TELEM_SCENARIO_FAULT_INJECTION = 1U,
    TELEM_SCENARIO_CLEAN = 2U,
    TELEM_SCENARIO_GPS_LOSS = 3U,
    TELEM_SCENARIO_IMU_DRIFT = 4U,
    TELEM_SCENARIO_ODOM_LOSS = 5U,
    TELEM_SCENARIO_SUBMARINE = 6U,
    TELEM_SCENARIO_REPLAY = 7U,
    TELEM_SCENARIO_UNKNOWN = 255U,
};

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
    uint8_t scenario_id;
    uint8_t nav_mode;
    int16_t temperature_deci_c;
    int16_t cross_track_deci_m;
    uint16_t along_track_deci_m;
};
#pragma pack(pop)

static_assert(sizeof(RemoteTelemetryPacket) == 32U, "RemoteTelemetryPacket debe ocupar 32 bytes");

/** Magic "NE" (NavEvent) en little-endian → 0x4E45 en wire. */
constexpr uint16_t TELEMETRY_UDP_EVENT_MAGIC = 0x4E45U;

#pragma pack(push, 1)
struct RemoteTelemetryEvent {
    uint16_t magic;
    uint16_t packed; /**< (event_id << 8) | param */
    uint32_t timestamp_ms;
};
#pragma pack(pop)

static_assert(sizeof(RemoteTelemetryEvent) == 8U, "RemoteTelemetryEvent debe ocupar 8 bytes");

enum TelemetryEventId : uint8_t {
    TELEM_EVENT_SAFE_STOP = 1U,
    TELEM_EVENT_HOT_RESTART = 2U,
    TELEM_EVENT_HEALTH_DEGRADED = 3U,
    TELEM_EVENT_HEALTH_CRITICAL = 4U,
    TELEM_EVENT_HEALTH_NOMINAL = 5U,
    TELEM_EVENT_GPS_LOST = 6U,
    TELEM_EVENT_GPS_RESTORED = 7U,
    TELEM_EVENT_WCET_VIOLATION = 8U,
    TELEM_EVENT_POWER_CONSERVATION = 9U,
    TELEM_EVENT_PREDICTIVE_DEGRADE = 10U,
};

#endif
