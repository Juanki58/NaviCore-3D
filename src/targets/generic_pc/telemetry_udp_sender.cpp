#include "telemetry_udp_sender.hpp"
#include "telemetry_udp.hpp"

#include <cmath>
#include <cstring>
#include <iostream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
typedef int SOCKET;
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#endif

namespace {

uint32_t g_send_failures = 0U;

int16_t encode_temperature_deci_c(float temperature_c)
{
    const float clamped = (temperature_c < -327.0f) ? -327.0f : (temperature_c > 327.0f ? 327.0f : temperature_c);
    return static_cast<int16_t>(std::lround(clamped * 10.0f));
}

int16_t encode_deci(float value_m)
{
    const float clamped = (value_m < -3276.7f) ? -3276.7f : (value_m > 3276.7f ? 3276.7f : value_m);
    return static_cast<int16_t>(std::lround(clamped * 10.0f));
}

uint16_t encode_along_deci(float value_m)
{
    const float clamped = (value_m < 0.0f) ? 0.0f : (value_m > 6553.5f ? 6553.5f : value_m);
    return static_cast<uint16_t>(std::lround(clamped * 10.0f));
}

class TelemetrySender {
public:
    TelemetrySender(const char *ip, int port) : m_socket(INVALID_SOCKET), m_initialized(false), m_seq(0U)
    {
#ifdef _WIN32
        WSADATA wsa_data{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != 0) {
            std::cerr << "[-] Error inicializando Winsock" << std::endl;
            return;
        }
#endif

        m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (m_socket == INVALID_SOCKET) {
            std::cerr << "[-] No se pudo crear el socket UDP" << std::endl;
#ifdef _WIN32
            WSACleanup();
#endif
            return;
        }

        std::memset(&m_dest_addr, 0, sizeof(m_dest_addr));
        m_dest_addr.sin_family = AF_INET;
        m_dest_addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, ip, &m_dest_addr.sin_addr) != 1) {
            std::cerr << "[-] Direccion UDP invalida: " << ip << std::endl;
#ifdef _WIN32
            closesocket(m_socket);
            WSACleanup();
#else
            close(m_socket);
#endif
            m_socket = INVALID_SOCKET;
            return;
        }

        m_initialized = true;
        std::cout << "[*] Emisor UDP v3 (32B) configurado hacia " << ip << ":" << port << std::endl;
    }

    ~TelemetrySender()
    {
        if (m_socket != INVALID_SOCKET) {
#ifdef _WIN32
            closesocket(m_socket);
            WSACleanup();
#else
            close(m_socket);
#endif
        }
    }

    bool is_ready() const { return m_initialized; }

    void send_packet(
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
        float temperature_c)
    {
        if (!m_initialized) {
            return;
        }

        RemoteTelemetryPacket packet{};
        packet.magic = TELEMETRY_UDP_MAGIC;
        packet.seq = m_seq++;
        packet.timestamp_ms = timestamp_ms;
        packet.pos_x = x;
        packet.pos_y = y;
        packet.pos_z = z;
        packet.health_score = score;
        packet.status_flags =
            static_cast<uint16_t>((health_mode & 0x03U) | (static_cast<uint16_t>(dropped) << 2));
        packet.scenario_id = scenario_id;
        packet.nav_mode = nav_mode;
        packet.temperature_deci_c = encode_temperature_deci_c(temperature_c);
        packet.cross_track_deci_m = encode_deci(cross_track_m);
        packet.along_track_deci_m = encode_along_deci(along_track_m);

        const int sent = sendto(
            m_socket,
            reinterpret_cast<const char *>(&packet),
            sizeof(packet),
            0,
            reinterpret_cast<sockaddr *>(&m_dest_addr),
            sizeof(m_dest_addr));
        if (sent != static_cast<int>(sizeof(packet))) {
            ++g_send_failures;
        }
    }

private:
    SOCKET m_socket;
    sockaddr_in m_dest_addr;
    bool m_initialized;
    uint16_t m_seq;
};

TelemetrySender *g_udp_sender = nullptr;

} // namespace

void telemetry_udp_init(const char *ip, int port)
{
    static TelemetrySender sender(ip, port);
    g_udp_sender = sender.is_ready() ? &sender : nullptr;
}

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
    float temperature_c)
{
    if (g_udp_sender != nullptr) {
        g_udp_sender->send_packet(
            timestamp_ms,
            x,
            y,
            z,
            cross_track_m,
            along_track_m,
            score,
            health_mode,
            dropped,
            scenario_id,
            nav_mode,
            temperature_c);
    }
}

uint32_t telemetry_udp_send_failures()
{
    return g_send_failures;
}

void telemetry_udp_log_stats()
{
    if (g_udp_sender == nullptr) {
        return;
    }

    const uint32_t failures = telemetry_udp_send_failures();
    if (failures > 0U) {
        std::printf("Telemetria UDP: %u envios fallidos\n", failures);
    }
}
