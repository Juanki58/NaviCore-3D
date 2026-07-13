#ifndef NAVICORE_FLIGHT_RECORDER_HPP
#define NAVICORE_FLIGHT_RECORDER_HPP

#include "NavState.h"
#include "diagnostic.hpp"
#include "guidance.hpp"
#include "ins_ekf.hpp"
#include "mission.hpp"
#include "runtime_health.hpp"
#include "sensor_types.hpp"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*
 * FlightRecorder - registro append-only zero-heap del estado interno de navegacion.
 * Mismo esquema de columnas para sim (CSV) y futuro replay embebido (UART/flash).
 */

#define FLIGHT_RECORDER_SCHEMA_VERSION 1U

typedef struct {
    uint32_t timestamp_ms;
    const char *scenario_name;
    uint8_t scenario_id;

    const ImuSample *imu;
    const GpsSample *gps;
    const NavState *nav_state;
    const InsEkfFilter *ekf;
    bool gnss_update_accepted;

    const GuidanceErrors *guidance_errors;
    const GuidanceCommands *guidance_commands;
    bool guidance_commands_valid;

    MissionState mission_state;
    bool return_home_active;
    size_t active_waypoint_index;

    uint8_t health_score;
    NavHealthMode health_mode;
    uint8_t power_state;
    bool shutdown_latched;

    uint8_t odom_fault;
    size_t waypoint_count;
    uint8_t bsp_bus_status;
    uint32_t radio_dropped_packets;

    const RuntimeHealth *runtime_health;
    uint32_t loop_us;
    float temperature_c;
} FlightRecorderTickInput;

typedef struct {
    uint32_t timestamp_ms;
    char scenario_name[16];
    uint8_t scenario_id;

    float imu_accel_mps2[3];
    float imu_gyro_radps[3];
    bool imu_valid;

    bool gnss_fix_valid;
    uint8_t gnss_satellites;
    float gnss_lat_deg;
    float gnss_lon_deg;
    float gnss_alt_m;
    float gnss_speed_mps;
    float gnss_course_deg;

    float pos_x;
    float pos_y;
    float pos_z;
    float vel_x;
    float vel_y;
    float vel_z;
    float heading_deg;
    float estimate_quality;
    NavMode nav_mode;

    float cross_track_m;
    float along_track_m;
    float cross_track_signed_m;

    bool ekf_active;
    bool ekf_outlier;
    float gnss_nis;
    float gnss_innovation_ned[3];
    bool gnss_update_accepted;
    uint32_t gnss_accept_count;
    uint32_t gnss_reject_count;

    float ekf_bias_accel[3];
    float ekf_bias_gyro[3];
    float ekf_attitude_rad[3];
    float ekf_p_diag_sqrt[INS_EKF_STATE_DIM];

    float desired_heading_rad;
    float desired_speed_mps;
    float desired_climb_mps;
    bool guidance_commands_valid;

    uint8_t mission_state;
    bool return_home_active;
    size_t active_waypoint_index;

    uint8_t health_score;
    uint8_t health_mode;
    uint8_t power_state;
    bool shutdown_latched;

    uint8_t odom_fault;
    size_t waypoint_count;
    uint8_t bsp_bus_status;
    uint32_t radio_dropped_packets;

    uint32_t loop_us;
    uint32_t missed_ticks;
    uint32_t max_loop_us;
    float temperature_c;
} FlightRecorderSample;

void flight_recorder_sample_zero(FlightRecorderSample *sample);

bool flight_recorder_capture(
    FlightRecorderSample *sample,
    const FlightRecorderTickInput *input);

void flight_recorder_write_csv_header(FILE *file);
bool flight_recorder_write_csv_row(FILE *file, const FlightRecorderSample *sample);

#endif /* NAVICORE_FLIGHT_RECORDER_HPP */
