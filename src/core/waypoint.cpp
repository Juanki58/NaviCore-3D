#include "waypoint.h"

#include <string.h>

Waypoint waypoint_make(const char *name, Vector3D position, NavDomain domain, uint32_t arrival_radius_m)
{
    Waypoint wp{};

    if (name != NULL) {
        strncpy(wp.name, name, NAVICORE_WAYPOINT_NAME_MAX - 1U);
        wp.name[NAVICORE_WAYPOINT_NAME_MAX - 1U] = '\0';
    }
    wp.position = position;
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

bool waypoint_route_init(WaypointRoute *route)
{
    if (route == NULL) {
        return false;
    }

    route->count = 0U;
    return true;
}

bool waypoint_route_push(WaypointRoute *route, Waypoint wp)
{
    if (route == NULL || route->count >= NAVICORE_WAYPOINT_ROUTE_MAX) {
        return false;
    }

    route->items[route->count++] = wp;
    return true;
}

const Waypoint *waypoint_route_get(const WaypointRoute *route, size_t index)
{
    if (route == NULL || index >= route->count) {
        return NULL;
    }
    return &route->items[index];
}

size_t waypoint_route_count(const WaypointRoute *route)
{
    if (route == NULL) {
        return 0U;
    }
    return route->count;
}
