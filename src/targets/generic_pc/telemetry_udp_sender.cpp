#include "telemetry_udp_sender.hpp"
#include "telemetry_udp.hpp"

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
        std::cout << "[*] Emisor UDP configurado hacia " << ip << ":" << port << std::endl;
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
        uint16_t score,
        uint8_t health_mode,
        uint16_t dropped)
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
    uint16_t score,
    uint8_t health_mode,
    uint16_t dropped)
{
    if (g_udp_sender != nullptr) {
        g_udp_sender->send_packet(timestamp_ms, x, y, z, score, health_mode, dropped);
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
