#ifndef NAVICORE_WAYPOINT_H
#define NAVICORE_WAYPOINT_H

#include <stddef.h>
#include <stdint.h>

#include "vector3d.h"

#define NAVICORE_WAYPOINT_NAME_MAX 32
#define NAVICORE_WAYPOINT_ROUTE_MAX 8

typedef struct {
    char name[NAVICORE_WAYPOINT_NAME_MAX];
    Vector3D position;
    NavDomain domain;
    uint32_t arrival_radius_m;
} Waypoint;

/*
 * Ruta con buffer embebido fijo: sin heap, sin std::vector, sin punteros externos.
 */
typedef struct {
    Waypoint items[NAVICORE_WAYPOINT_ROUTE_MAX];
    size_t count;
} WaypointRoute;

Waypoint waypoint_make(const char *name, Vector3D position, NavDomain domain, uint32_t arrival_radius_m);
bool waypoint_matches_name(const Waypoint *wp, const char *name);

bool waypoint_route_init(WaypointRoute *route);
bool waypoint_route_push(WaypointRoute *route, Waypoint wp);
const Waypoint *waypoint_route_get(const WaypointRoute *route, size_t index);
size_t waypoint_route_count(const WaypointRoute *route);

#endif /* NAVICORE_WAYPOINT_H */
