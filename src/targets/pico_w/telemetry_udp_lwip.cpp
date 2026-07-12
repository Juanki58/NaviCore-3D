#include "telemetry_udp_lwip.hpp"

#include "../generic_pc/telemetry_udp.hpp"

#include "pico/cyw43_arch.h"

#include <cmath>
#include <cstdio>
#include <cstring>

extern "C" {
#include "lwip/ip_addr.h"
#include "lwip/udp.h"
}

namespace {

int16_t encode_temperature_deci_c(float temperature_c)
{
    const float clamped =
        (temperature_c < -327.0f) ? -327.0f : (temperature_c > 327.0f ? 327.0f : temperature_c);
    return static_cast<int16_t>(std::lround(clamped * 10.0f));
}

int16_t encode_deci(float value_m)
{
    const float clamped =
        (value_m < -3276.7f) ? -3276.7f : (value_m > 3276.7f ? 3276.7f : value_m);
    return static_cast<int16_t>(std::lround(clamped * 10.0f));
}

uint16_t encode_along_deci(float value_m)
{
    const float clamped = (value_m < 0.0f) ? 0.0f : (value_m > 6553.5f ? 6553.5f : value_m);
    return static_cast<uint16_t>(std::lround(clamped * 10.0f));
}

struct TelemetryUdpLwip {
    udp_pcb *pcb = nullptr;
    ip_addr_t dest_addr{};
    uint16_t dest_port = 0U;
    uint16_t seq = 0U;
    bool ready = false;
};

TelemetryUdpLwip g_sender{};

bool udp_send_bytes(const void *data, uint16_t len)
{
    if (g_sender.pcb == nullptr || data == nullptr || len == 0U) {
        return false;
    }

    pbuf *packet = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_RAM);
    if (packet == nullptr) {
        return false;
    }

    std::memcpy(packet->payload, data, len);

    cyw43_arch_lwip_begin();
    const err_t err = udp_sendto(g_sender.pcb, packet, &g_sender.dest_addr, g_sender.dest_port);
    cyw43_arch_lwip_end();

    pbuf_free(packet);
    return err == ERR_OK;
}

} /* namespace */

bool telemetry_udp_lwip_init(const char *host_ip, uint16_t port)
{
    if (host_ip == nullptr) {
        return false;
    }

    g_sender.pcb = udp_new();
    if (g_sender.pcb == nullptr) {
        return false;
    }

    if (!ipaddr_aton(host_ip, &g_sender.dest_addr)) {
        udp_remove(g_sender.pcb);
        g_sender.pcb = nullptr;
        return false;
    }

    g_sender.dest_port = port;
    g_sender.seq = 0U;
    g_sender.ready = true;

    std::printf("[*] Telemetria UDP (32 B) -> %s:%u\n", host_ip, static_cast<unsigned>(port));
    return true;
}

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
    float temperature_c)
{
    if (!g_sender.ready) {
        return;
    }

    RemoteTelemetryPacket packet{};
    packet.magic = TELEMETRY_UDP_MAGIC;
    packet.seq = g_sender.seq++;
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

    (void)udp_send_bytes(&packet, static_cast<uint16_t>(sizeof(packet)));
}

void telemetry_udp_lwip_send_event(uint32_t timestamp_ms, uint8_t event_id, uint8_t param)
{
    if (!g_sender.ready) {
        return;
    }

    RemoteTelemetryEvent event{};
    event.magic = TELEMETRY_UDP_EVENT_MAGIC;
    event.packed = static_cast<uint16_t>((static_cast<uint16_t>(event_id) << 8) | param);
    event.timestamp_ms = timestamp_ms;

    (void)udp_send_bytes(&event, static_cast<uint16_t>(sizeof(event)));
}
