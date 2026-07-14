#include "waypoint.hpp"

#include <string.h>

Waypoint waypoint_make(
    const char *name,
    Vector3D position,
    NavDomain domain,
    uint32_t arrival_radius_m,
    float desired_speed_mps)
{
    Waypoint wp{};

    if (name != NULL) {
        strncpy(wp.name, name, NAVICORE_WAYPOINT_NAME_MAX - 1U);
        wp.name[NAVICORE_WAYPOINT_NAME_MAX - 1U] = '\0';
    }
    wp.position = position;
    wp.desired_speed_mps = desired_speed_mps;
    wp.domain = domain;
    wp.arrival_radius_m = arrival_radius_m;
    return wp;
}

bool waypoint_matches_name(const Waypoint *wp, const char *name)
{
    if (wp == NULL || name == NULL) {
        return false;
    }
    return strncmp(wp->name, name, NAVICORE_WAYPOINT_NAME_MAX) == 0;
}

bool waypoint_buffer_init(StaticWaypointBuffer *buffer)
{
    if (buffer == NULL) {
        return false;
    }

    buffer->head = 0U;
    buffer->count = 0U;
    return true;
}

bool waypoint_buffer_push(StaticWaypointBuffer *buffer, Waypoint wp)
{
    if (buffer == NULL) {
        return false;
    }

    if (buffer->count < NAVICORE_MAX_WAYPOINTS) {
        const size_t tail = (buffer->head + buffer->count) % NAVICORE_MAX_WAYPOINTS;
        buffer->items[tail] = wp;
        buffer->count++;
        return true;
    }

    /* Buffer lleno: sobrescribe el mas antiguo y avanza head (O(1)). */
    buffer->items[buffer->head] = wp;
    buffer->head = (buffer->head + 1U) % NAVICORE_MAX_WAYPOINTS;
    return true;
}

bool waypoint_buffer_pop(StaticWaypointBuffer *buffer, Waypoint *out)
{
    if (buffer == NULL || buffer->count == 0U) {
        return false;
    }

    if (out != NULL) {
        *out = buffer->items[buffer->head];
    }

    buffer->head = (buffer->head + 1U) % NAVICORE_MAX_WAYPOINTS;
    buffer->count--;
    return true;
}

bool waypoint_buffer_is_empty(const StaticWaypointBuffer *buffer)
{
    return buffer == NULL || buffer->count == 0U;
}

bool waypoint_buffer_is_full(const StaticWaypointBuffer *buffer)
{
    return buffer != NULL && buffer->count >= NAVICORE_MAX_WAYPOINTS;
}
