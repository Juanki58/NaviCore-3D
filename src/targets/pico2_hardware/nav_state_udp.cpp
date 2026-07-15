#include "nav_state_udp.hpp"

#include "lwip/inet.h"
#include "lwip/sockets.h"

#include <string.h>

namespace {

int g_udp_sock = -1;
struct sockaddr_in g_dest{};
bool g_ready = false;

} /* namespace */

bool pico2_nav_state_udp_init(const char *host, uint16_t port)
{
    pico2_nav_state_udp_close();

    if (host == nullptr || port == 0U) {
        return false;
    }

    g_udp_sock = lwip_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_udp_sock < 0) {
        return false;
    }

    memset(&g_dest, 0, sizeof(g_dest));
    g_dest.sin_family = AF_INET;
    g_dest.sin_port = htons(port);
    if (inet_aton(host, &g_dest.sin_addr) == 0) {
        lwip_close(g_udp_sock);
        g_udp_sock = -1;
        return false;
    }

    g_ready = true;
    return true;
}

bool pico2_nav_state_udp_is_ready(void)
{
    return g_ready && g_udp_sock >= 0;
}

bool pico2_nav_state_udp_send(const NavigationState *state)
{
    if (!pico2_nav_state_udp_is_ready() || state == nullptr) {
        return false;
    }

    const int sent = lwip_sendto(
        g_udp_sock,
        state,
        sizeof(NavigationState),
        0,
        reinterpret_cast<const struct sockaddr *>(&g_dest),
        sizeof(g_dest));

    return sent == static_cast<int>(sizeof(NavigationState));
}

void pico2_nav_state_udp_close(void)
{
    if (g_udp_sock >= 0) {
        lwip_close(g_udp_sock);
        g_udp_sock = -1;
    }
    g_ready = false;
}
