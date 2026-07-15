#include "telemetry_interface.hpp"

#include "guidance.hpp"
#include "ins_ekf.hpp"
#include "navigation_cortex.hpp"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
#include <errno.h>

#ifdef _WIN32
#include <direct.h>
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#ifndef INVALID_SOCKET
#define INVALID_SOCKET (-1)
#endif
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <unistd.h>
#define INVALID_SOCKET (-1)
#define closesocket close
#endif
#endif

namespace {

TelemetryInterface *g_active_telemetry = NULL;

#if defined(_WIN32)
bool g_winsock_ready = false;
#endif

void euler_ned_to_quaternion(
    float roll_rad,
    float pitch_rad,
    float yaw_rad,
    float out_wxyz[4])
{
    const float half_roll = roll_rad * 0.5f;
    const float half_pitch = pitch_rad * 0.5f;
    const float half_yaw = yaw_rad * 0.5f;

    const float cr = cosf(half_roll);
    const float sr = sinf(half_roll);
    const float cp = cosf(half_pitch);
    const float sp = sinf(half_pitch);
    const float cy = cosf(half_yaw);
    const float sy = sinf(half_yaw);

    out_wxyz[0] = (cr * cp * cy) + (sr * sp * sy);
    out_wxyz[1] = (sr * cp * cy) - (cr * sp * sy);
    out_wxyz[2] = (cr * sp * cy) + (sr * cp * sy);
    out_wxyz[3] = (cr * cp * sy) - (sr * sp * cy);
}

float heading_deg_to_yaw_rad(float heading_deg)
{
    const float normalized = navstate_normalize_heading(heading_deg);
    return normalized * (static_cast<float>(M_PI) / 180.0f);
}

bool ensure_docs_dir(void)
{
#if defined(_WIN32)
    return _mkdir("docs") == 0 || errno == EEXIST;
#elif defined(__linux__) || defined(__APPLE__)
    return mkdir("docs", 0755) == 0 || errno == EEXIST;
#else
    return true;
#endif
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

} /* namespace */

#define TELEMETRY_SOCKET_INVALID (-1)

TelemetryInterface::TelemetryInterface()
    : config_{},
      bindings_(NULL),
      logger_file_(NULL),
      initialized_(false),
      last_sim_console_ms_(0U),
      unity_seq_(0U),
      unity_send_failures_(0U),
      unity_ready_(false)
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
      ,
      unity_socket_(TELEMETRY_SOCKET_INVALID),
      unity_dest_len_(0U)
#endif
{
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    memset(unity_dest_, 0, sizeof(unity_dest_));
#endif
}

void TelemetryInterface::set_active(TelemetryInterface *instance)
{
    g_active_telemetry = instance;
}

TelemetryInterface *TelemetryInterface::active()
{
    return g_active_telemetry;
}

bool TelemetryInterface::open_logger()
{
    if (!config_.enable_logger_channel || config_.logger_csv_path == NULL) {
        return true;
    }

    ensure_docs_dir();
    logger_file_ = fopen(config_.logger_csv_path, "w");
    if (logger_file_ == NULL) {
        return false;
    }

    fprintf(
        logger_file_,
        "time_us,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,roll,pitch,yaw,"
        "bias_ax,bias_ay,bias_az,bias_gx,bias_gy,bias_gz,"
        "nis,innov_x,innov_y,innov_z,cov_pos_x,cov_pos_y,cov_pos_z,cov_yaw,"
        "des_speed,des_heading,des_climb,pid_speed,pid_yaw,pid_alt,"
        "speed_meas,yaw_meas,climb_meas,fwd_accel,yaw_rate,vert_accel,mission_state\n");
    return true;
}

bool TelemetryInterface::open_unity_socket()
{
#if !defined(_WIN32) && !defined(__linux__) && !defined(__APPLE__)
    (void)config_;
    return true;
#else
    if (!config_.enable_unity_channel) {
        return true;
    }

    const char *host = (config_.unity_host != NULL) ? config_.unity_host : UNITY_TELEMETRY_DEFAULT_HOST;
    const uint16_t port = (config_.unity_port != 0U) ? config_.unity_port : UNITY_TELEMETRY_DEFAULT_PORT;

#ifdef _WIN32
    if (!g_winsock_ready) {
        WSADATA wsa_data;
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            return false;
        }
        g_winsock_ready = true;
    }
#endif

    unity_socket_ = static_cast<intptr_t>(socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
    if (unity_socket_ == TELEMETRY_SOCKET_INVALID) {
        return false;
    }

    sockaddr_in dest{};
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(port);
    if (inet_pton(AF_INET, host, &dest.sin_addr) != 1) {
        closesocket(static_cast<SOCKET>(unity_socket_));
        unity_socket_ = TELEMETRY_SOCKET_INVALID;
        return false;
    }

    memcpy(unity_dest_, &dest, sizeof(dest));
    unity_dest_len_ = static_cast<uint16_t>(sizeof(dest));
    unity_ready_ = true;
    return true;
#endif
}

void TelemetryInterface::close_unity_socket()
{
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    if (unity_socket_ != TELEMETRY_SOCKET_INVALID) {
        closesocket(static_cast<SOCKET>(unity_socket_));
        unity_socket_ = TELEMETRY_SOCKET_INVALID;
    }
#endif
    unity_ready_ = false;
    unity_dest_len_ = 0U;
}

bool TelemetryInterface::initialize(const TelemetryConfig &config)
{
    shutdown();

    config_ = config;
    if (config_.sim_console_interval_ms == 0U) {
        config_.sim_console_interval_ms = TELEMETRY_SIM_CONSOLE_INTERVAL_MS;
    }
    if (config_.unity_port == 0U) {
        config_.unity_port = UNITY_TELEMETRY_DEFAULT_PORT;
    }
    if (config_.logger_csv_path == NULL) {
        config_.logger_csv_path = TELEMETRY_LOGGER_DEFAULT_PATH;
    }

    if (!open_logger()) {
        return false;
    }

    if (config_.enable_unity_channel && !open_unity_socket()) {
        if (logger_file_ != NULL) {
            fclose(logger_file_);
            logger_file_ = NULL;
        }
        return false;
    }

    initialized_ = true;
    return true;
}

void TelemetryInterface::bind_sources(const TelemetryBindings *bindings)
{
    bindings_ = bindings;
}

void TelemetryInterface::broadcast_simulator(const NavState &state, MissionState mission)
{
    if (!config_.enable_simulator_channel) {
        return;
    }

    const uint32_t interval_ms = config_.sim_console_interval_ms;
    if ((state.timestamp_ms - last_sim_console_ms_) < interval_ms) {
        return;
    }

    last_sim_console_ms_ = state.timestamp_ms;

    const uint8_t health_score =
        (bindings_ != NULL && bindings_->health != NULL) ? bindings_->health->health_score : 0U;

    printf(
        "[SIM t=%5.1fs] mission=%-12s nav=%u health=%u pos=(%.6f,%.6f,%.1f) speed=%.2f m/s\n",
        static_cast<float>(state.timestamp_ms) * 0.001f,
        mission_state_name(mission),
        static_cast<unsigned>(state.mode),
        static_cast<unsigned>(health_score),
        state.position.x,
        state.position.y,
        state.position.z,
        navstate_speed_mps(&state));
}

void TelemetryInterface::broadcast_unity(const NavState &state, MissionState mission)
{
#if !defined(_WIN32) && !defined(__linux__) && !defined(__APPLE__)
    (void)state;
    (void)mission;
    return;
#else
    if (!config_.enable_unity_channel || !unity_ready_ || unity_socket_ == TELEMETRY_SOCKET_INVALID) {
        return;
    }

    float pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = heading_deg_to_yaw_rad(state.heading_deg);
    uint8_t flags = UNITY_TELEM_FLAG_POS_VALID | UNITY_TELEM_FLAG_VEL_VALID | UNITY_TELEM_FLAG_ATT_VALID;

    const InsEkfFilter *ekf = (bindings_ != NULL) ? bindings_->ekf : NULL;
    if (ekf != NULL && ekf->initialized) {
        flags |= UNITY_TELEM_FLAG_EKF_VALID;
        ins_ekf_get_position_ned(ekf, pos_ned);
        ins_ekf_get_velocity_ned(ekf, vel_ned);
        ins_ekf_get_attitude_rad(ekf, &roll_rad, &pitch_rad, &yaw_rad);
    } else {
        vel_ned[0] = state.velocity.x;
        vel_ned[1] = state.velocity.y;
        vel_ned[2] = state.velocity.z;
    }

    euler_ned_to_quaternion(roll_rad, pitch_rad, yaw_rad, quat);

    const uint8_t health_mode =
        (bindings_ != NULL && bindings_->health != NULL)
            ? static_cast<uint8_t>(bindings_->health->mode)
            : static_cast<uint8_t>(HEALTH_NOMINAL);
    const uint16_t health_score =
        (bindings_ != NULL && bindings_->health != NULL)
            ? static_cast<uint16_t>(bindings_->health->health_score)
            : 100U;

    UnityTelemetryPacket packet{};
    packet.magic = UNITY_TELEMETRY_MAGIC;
    packet.seq = unity_seq_++;
    packet.timestamp_ms = state.timestamp_ms;
    packet.pos_n_m = pos_ned[0];
    packet.pos_e_m = pos_ned[1];
    packet.pos_d_m = pos_ned[2];
    packet.vel_n_mps = vel_ned[0];
    packet.vel_e_mps = vel_ned[1];
    packet.vel_d_mps = vel_ned[2];
    packet.quat_w = quat[0];
    packet.quat_x = quat[1];
    packet.quat_y = quat[2];
    packet.quat_z = quat[3];
    packet.nav_mode = static_cast<uint8_t>(state.mode);
    packet.mission_state = static_cast<uint8_t>(mission);
    packet.health_mode = health_mode;
    packet.flags = flags;
    packet.health_score = health_score;

    const int sent = sendto(
        static_cast<SOCKET>(unity_socket_),
        reinterpret_cast<const char *>(&packet),
        static_cast<int>(sizeof(packet)),
        0,
        reinterpret_cast<const sockaddr *>(unity_dest_),
        static_cast<socklen_t>(unity_dest_len_));
    if (sent != static_cast<int>(sizeof(packet))) {
        ++unity_send_failures_;
    }
#endif
}

void TelemetryInterface::broadcast_logger(const NavState &state, MissionState mission)
{
    if (!config_.enable_logger_channel || logger_file_ == NULL) {
        return;
    }

    const uint64_t time_us = static_cast<uint64_t>(state.timestamp_ms) * 1000ULL;
    const InsEkfFilter *ekf = (bindings_ != NULL) ? bindings_->ekf : NULL;
    const TelemetryEkfTick *ekf_tick = (bindings_ != NULL) ? bindings_->ekf_tick : NULL;
    const TelemetryPidSnapshot *pid = (bindings_ != NULL) ? bindings_->pid : NULL;

    float pos_ned[3] = {0.0f, 0.0f, 0.0f};
    float vel_ned[3] = {0.0f, 0.0f, 0.0f};
    float roll_rad = 0.0f;
    float pitch_rad = 0.0f;
    float yaw_rad = 0.0f;
    float bias_a[3] = {0.0f, 0.0f, 0.0f};
    float bias_g[3] = {0.0f, 0.0f, 0.0f};
    float nis = 0.0f;
    float innov[3] = {0.0f, 0.0f, 0.0f};
    float cov_pos_x = 0.0f;
    float cov_pos_y = 0.0f;
    float cov_pos_z = 0.0f;
    float cov_yaw = 0.0f;
    float des_speed = 0.0f;
    float des_heading = 0.0f;
    float des_climb = 0.0f;
    float pid_speed = 0.0f;
    float pid_yaw = 0.0f;
    float pid_alt = 0.0f;
    float speed_meas = 0.0f;
    float yaw_meas = 0.0f;
    float climb_meas = 0.0f;
    float fwd_accel = 0.0f;
    float yaw_rate = 0.0f;
    float vert_accel = 0.0f;

    if (ekf != NULL && ekf->initialized) {
        ins_ekf_get_position_ned(ekf, pos_ned);
        ins_ekf_get_velocity_ned(ekf, vel_ned);
        ins_ekf_get_attitude_rad(ekf, &roll_rad, &pitch_rad, &yaw_rad);
        ins_ekf_get_bias(ekf, bias_a, bias_g);

        cov_pos_x = ins_ekf_get_covariance_flat(ekf, 0U);
        cov_pos_y = ins_ekf_get_covariance_flat(ekf, 16U);
        cov_pos_z = ins_ekf_get_covariance_flat(ekf, 32U);
        cov_yaw = ins_ekf_get_covariance_flat(ekf, 80U);

        if (ekf_tick != NULL && ekf_tick->gnss_update_this_cycle) {
            nis = ekf_tick->nis;
            innov[0] = ekf_tick->innov_ned[0];
            innov[1] = ekf_tick->innov_ned[1];
            innov[2] = ekf_tick->innov_ned[2];
        }
    }

    if (pid != NULL && pid->active) {
        des_speed = pid->des_speed_mps;
        des_heading = pid->des_heading_rad;
        des_climb = pid->des_climb_mps;
        pid_speed = pid->pid_speed_out;
        pid_yaw = pid->pid_yaw_out;
        pid_alt = pid->pid_alt_out;
        speed_meas = pid->speed_meas_mps;
        yaw_meas = pid->yaw_meas_rad;
        climb_meas = pid->climb_meas_mps;
        fwd_accel = pid->forward_accel_mps2;
        yaw_rate = pid->yaw_rate_radps;
        vert_accel = pid->vertical_accel_mps2;
    }

    fprintf(
        logger_file_,
        "%llu,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%u\n",
        static_cast<unsigned long long>(time_us),
        pos_ned[0],
        pos_ned[1],
        pos_ned[2],
        vel_ned[0],
        vel_ned[1],
        vel_ned[2],
        roll_rad,
        pitch_rad,
        yaw_rad,
        bias_a[0],
        bias_a[1],
        bias_a[2],
        bias_g[0],
        bias_g[1],
        bias_g[2],
        nis,
        innov[0],
        innov[1],
        innov[2],
        cov_pos_x,
        cov_pos_y,
        cov_pos_z,
        cov_yaw,
        des_speed,
        des_heading,
        des_climb,
        pid_speed,
        pid_yaw,
        pid_alt,
        speed_meas,
        yaw_meas,
        climb_meas,
        fwd_accel,
        yaw_rate,
        vert_accel,
        static_cast<unsigned>(mission));
}

void TelemetryInterface::broadcast(const NavState &state, MissionState mission)
{
    if (!initialized_) {
        return;
    }

    broadcast_simulator(state, mission);
    broadcast_unity(state, mission);
    broadcast_logger(state, mission);
}

void TelemetryInterface::emit_event(uint32_t timestamp_ms, uint8_t event_id, uint8_t param)
{
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    if (!config_.enable_unity_channel || !unity_ready_ || unity_socket_ == TELEMETRY_SOCKET_INVALID) {
        return;
    }

    TelemetryEventPacket event{};
    event.magic = TELEMETRY_EVENT_MAGIC;
    event.packed = static_cast<uint16_t>((static_cast<uint16_t>(event_id) << 8) | param);
    event.timestamp_ms = timestamp_ms;

    const int sent = sendto(
        static_cast<SOCKET>(unity_socket_),
        reinterpret_cast<const char *>(&event),
        static_cast<int>(sizeof(event)),
        0,
        reinterpret_cast<const sockaddr *>(unity_dest_),
        static_cast<socklen_t>(unity_dest_len_));
    if (sent != static_cast<int>(sizeof(event))) {
        ++unity_send_failures_;
    }
#else
    (void)timestamp_ms;
    (void)event_id;
    (void)param;
#endif
}

void TelemetryInterface::emit_events(uint32_t timestamp_ms, const NavigationDecision *decision)
{
    if (decision == NULL) {
        return;
    }

    for (uint8_t i = 0; i < decision->event_count; ++i) {
        emit_event(timestamp_ms, decision->events[i].id, decision->events[i].param);
    }
}

void TelemetryInterface::flush()
{
    if (logger_file_ != NULL) {
        fflush(logger_file_);
    }
}

void TelemetryInterface::shutdown()
{
    if (logger_file_ != NULL) {
        fflush(logger_file_);
        fclose(logger_file_);
        logger_file_ = NULL;
    }

    close_unity_socket();
    bindings_ = NULL;
    initialized_ = false;
    last_sim_console_ms_ = 0U;
}

void TelemetryInterface::log_stats() const
{
    if (unity_send_failures_ > 0U) {
        printf("Telemetria Unity UDP: %u envios fallidos\n", unity_send_failures_);
    }
}
