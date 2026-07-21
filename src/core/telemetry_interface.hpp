#ifndef NAVICORE_TELEMETRY_INTERFACE_HPP
#define NAVICORE_TELEMETRY_INTERFACE_HPP

#include "NavState.h"
#include "diagnostic.hpp"
#include "guidance.hpp"
#include "mission.hpp"
#include "navigation_cortex.hpp"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

/*
 * TelemetryInterface — distribuidor central NaviCore Runtime → Sim / Unity / Logger.
 * Zero-heap en broadcast @ 100 Hz.
 */

struct InsEkfFilter;

/** Magic "NU" (NavUnity) en little-endian → 0x4E55 en wire. */
#define UNITY_TELEMETRY_MAGIC         0x4E55U
#define UNITY_TELEMETRY_DEFAULT_PORT  5556U
#define UNITY_TELEMETRY_DEFAULT_HOST  "127.0.0.1"

#define TELEMETRY_LOGGER_DEFAULT_PATH "docs/telemetria_navicore.csv"
#define TELEMETRY_SIM_CONSOLE_INTERVAL_MS 2000U

enum TelemetryScenarioId : uint8_t {
    TELEM_SCENARIO_HIGH_DEMAND = 0U,
    TELEM_SCENARIO_FAULT_INJECTION = 1U,
    TELEM_SCENARIO_CLEAN = 2U,
    TELEM_SCENARIO_GPS_LOSS = 3U,
    TELEM_SCENARIO_IMU_DRIFT = 4U,
    TELEM_SCENARIO_ODOM_LOSS = 5U,
    TELEM_SCENARIO_SUBMARINE = 6U,
    TELEM_SCENARIO_REPLAY = 7U,
    TELEM_SCENARIO_SUPER_TUNNEL = 8U,
    TELEM_SCENARIO_TUNNEL_STRESS = 9U,
    TELEM_SCENARIO_SLALOM = 10U,
    TELEM_SCENARIO_UNKNOWN = 255U,
};

/** Magic "NE" (NavEvent) — alertas en el mismo socket Unity. */
#define TELEMETRY_EVENT_MAGIC 0x4E45U

#pragma pack(push, 1)
struct UnityTelemetryPacket {
    uint16_t magic;
    uint16_t seq;
    uint32_t timestamp_ms;
    float pos_n_m;
    float pos_e_m;
    float pos_d_m;
    float vel_n_mps;
    float vel_e_mps;
    float vel_d_mps;
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    uint8_t nav_mode;
    uint8_t mission_state;
    uint8_t health_mode;
    uint8_t flags;
    uint16_t health_score;
};

struct TelemetryEventPacket {
    uint16_t magic;
    uint16_t packed;
    uint32_t timestamp_ms;
};
#pragma pack(pop)

NAVICORE_STATIC_ASSERT(sizeof(UnityTelemetryPacket) == 54U, "UnityTelemetryPacket debe ocupar 54 bytes");
NAVICORE_STATIC_ASSERT(sizeof(TelemetryEventPacket) == 8U, "TelemetryEventPacket debe ocupar 8 bytes");

#define UNITY_TELEM_FLAG_EKF_VALID 0x01U
#define UNITY_TELEM_FLAG_POS_VALID 0x02U
#define UNITY_TELEM_FLAG_VEL_VALID 0x04U
#define UNITY_TELEM_FLAG_ATT_VALID 0x08U

typedef struct {
    float des_speed_mps;
    float des_heading_rad;
    float des_climb_mps;
    float pid_speed_out;
    float pid_yaw_out;
    float pid_alt_out;
    float speed_meas_mps;
    float yaw_meas_rad;
    float climb_meas_mps;
    float forward_accel_mps2;
    float yaw_accel_radps2;
    float vertical_accel_mps2;
    float yaw_rate_cmd_radps;
    float yaw_rate_radps;
    bool active;
} TelemetryPidSnapshot;

typedef struct {
    bool gnss_update_this_cycle;
    float nis;
    float innov_ned[3];
} TelemetryEkfTick;

typedef struct {
    const InsEkfFilter *ekf;
    const SystemHealthMonitor *health;
    const GuidanceErrors *guidance;
    const TelemetryPidSnapshot *pid;
    TelemetryEkfTick *ekf_tick;
    uint8_t scenario_id;
    float temperature_c;
    /** Deriva horizontal/lateral vs verdad de escenario (m); NaN si no aplica. */
    float drift_m;
    bool drift_valid;
    /** ω_z medido (rad/s) que alimenta predict/NHC; p. ej. imu.gyro_z en SLALOM. */
    float measured_yaw_rate_radps;
    bool measured_yaw_rate_valid;
} TelemetryBindings;

typedef struct {
    bool enable_simulator_channel;
    bool enable_unity_channel;
    bool enable_logger_channel;

    const char *logger_csv_path;
    const char *unity_host;
    uint16_t unity_port;
    uint32_t sim_console_interval_ms;
} TelemetryConfig;

class TelemetryInterface {
public:
    TelemetryInterface();

    TelemetryInterface(const TelemetryInterface &) = delete;
    TelemetryInterface &operator=(const TelemetryInterface &) = delete;

    bool initialize(const TelemetryConfig &config);
    void bind_sources(const TelemetryBindings *bindings);

    void broadcast(const NavState &state, MissionState mission);

    void emit_event(uint32_t timestamp_ms, uint8_t event_id, uint8_t param);
    void emit_events(uint32_t timestamp_ms, const NavigationDecision *decision);

    void flush();
    void shutdown();
    void log_stats() const;

    static void set_active(TelemetryInterface *instance);
    static TelemetryInterface *active();

private:
    void broadcast_simulator(const NavState &state, MissionState mission);
    void broadcast_unity(const NavState &state, MissionState mission);
    void broadcast_logger(const NavState &state, MissionState mission);

    bool open_logger();
    bool open_unity_socket();
    void close_unity_socket();

    TelemetryConfig config_;
    const TelemetryBindings *bindings_;
    FILE *logger_file_;
    bool initialized_;
    uint32_t last_sim_console_ms_;
    uint16_t unity_seq_;
    uint32_t unity_send_failures_;
    bool unity_ready_;

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    intptr_t unity_socket_;
    uint8_t unity_dest_[16];
    uint16_t unity_dest_len_;
#endif
};

#endif /* NAVICORE_TELEMETRY_INTERFACE_HPP */
