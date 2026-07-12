/**
 * @file sil_protocol.hpp
 * @brief Protocolo UDP binario SIL — JSBSim ↔ NaviCore ↔ motor gráfico
 *
 * Magics independientes de la telemetría NaviCore (0x4E43 / 0x4E45).
 * Espejo Python: tools/sil_protocol.py
 */
#ifndef NAVICORE_SIL_PROTOCOL_HPP
#define NAVICORE_SIL_PROTOCOL_HPP

#include <cstdint>

/** Magic "NT" (NavTruth) → 0x4E54 en wire little-endian. */
constexpr uint16_t SIL_TRUTH_MAGIC = 0x4E54U;
/** Magic "NS" (NavSensor) → 0x4E53. */
constexpr uint16_t SIL_SENSOR_MAGIC = 0x4E53U;
/** Magic "NA" (NavActuator) → 0x4E41. */
constexpr uint16_t SIL_ACTUATOR_MAGIC = 0x4E41U;

constexpr uint8_t SIL_MAX_UAV_ID = 7U;

constexpr uint16_t SIL_TRUTH_BASE_PORT = 5301U;
constexpr uint16_t SIL_SENSOR_BASE_PORT = 5401U;
constexpr uint16_t SIL_ACTUATOR_BASE_PORT = 5501U;
constexpr uint16_t SIL_NAVICORE_TELEM_BASE_PORT = 5201U;

constexpr uint8_t SIL_FLAG_POS_VALID = 0x01U;
constexpr uint8_t SIL_FLAG_ATT_VALID = 0x02U;
constexpr uint8_t SIL_FLAG_VEL_VALID = 0x04U;
constexpr uint8_t SIL_FLAG_IMU_VALID = 0x01U;
constexpr uint8_t SIL_FLAG_GPS_VALID = 0x02U;
constexpr uint8_t SIL_FLAG_MAG_VALID = 0x04U;

enum SilActuatorSurface : uint8_t {
    SIL_SURFACE_THROTTLE = 0U,
    SIL_SURFACE_AILERON = 1U,
    SIL_SURFACE_ELEVATOR = 2U,
    SIL_SURFACE_RUDDER = 3U,
};

#pragma pack(push, 1)
/** Pose 6DOF NED local — JSBSim → motor gráfico (48 B). */
struct SilTruthPacket {
    uint16_t magic;
    uint8_t uav_id;
    uint8_t flags;
    uint16_t seq;
    uint32_t timestamp_ms;
    float pos_n_m;
    float pos_e_m;
    float pos_d_m;
    float vel_n_mps;
    float vel_e_mps;
    float vel_d_mps;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    uint16_t status_flags;
};
#pragma pack(pop)

#pragma pack(push, 1)
/** Sensores sintéticos — JSBSim → NaviCore api_ingest (70 B). */
struct SilSensorPacket {
    uint16_t magic;
    uint8_t uav_id;
    uint8_t flags;
    uint16_t seq;
    uint32_t timestamp_ms;
    float accel_mps2[3];
    float gyro_radps[3];
    float mag_ut[3];
    float lat_deg;
    float lon_deg;
    float alt_m;
    float speed_mps;
    float course_deg;
    uint8_t satellites;
    uint8_t fix_valid;
    uint16_t reserved;
};
#pragma pack(pop)

#pragma pack(push, 1)
/** Mandos de superficie — NaviCore → JSBSim (16 B). */
struct SilActuatorPacket {
    uint16_t magic;
    uint8_t uav_id;
    uint8_t surface_id;
    uint16_t seq;
    uint32_t timestamp_ms;
    float command_norm;
    uint16_t reserved;
};
#pragma pack(pop)

static_assert(sizeof(SilTruthPacket) == 48U, "SilTruthPacket debe ocupar 48 bytes");
static_assert(sizeof(SilSensorPacket) == 70U, "SilSensorPacket debe ocupar 70 bytes");
static_assert(sizeof(SilActuatorPacket) == 16U, "SilActuatorPacket debe ocupar 16 bytes");

#endif /* NAVICORE_SIL_PROTOCOL_HPP */
