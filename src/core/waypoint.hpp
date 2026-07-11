#ifndef NAVICORE_WAYPOINT_HPP
#define NAVICORE_WAYPOINT_HPP

#include <stddef.h>
#include <stdint.h>

#include "vector3d.h"

#define NAVICORE_WAYPOINT_NAME_MAX 32U
#define NAVICORE_MAX_WAYPOINTS     64U

typedef struct {
    char name[NAVICORE_WAYPOINT_NAME_MAX];
    Vector3D position;
    NavDomain domain;
    uint32_t arrival_radius_m;
} Waypoint;

/*
 * Buffer circular estatico: array embebido, sin heap ni contenedores dinamicos.
 * Cuando esta lleno, push sobrescribe el waypoint mas antiguo (FIFO circular).
 */
typedef struct {
    Waypoint items[NAVICORE_MAX_WAYPOINTS];
    size_t head;
    size_t count;
} StaticWaypointBuffer;

Waypoint waypoint_make(const char *name, Vector3D position, NavDomain domain, uint32_t arrival_radius_m);
bool waypoint_matches_name(const Waypoint *wp, const char *name);

bool waypoint_buffer_init(StaticWaypointBuffer *buffer);
bool waypoint_buffer_push(StaticWaypointBuffer *buffer, Waypoint wp);
bool waypoint_buffer_pop(StaticWaypointBuffer *buffer, Waypoint *out);
bool waypoint_buffer_is_empty(const StaticWaypointBuffer *buffer);
bool waypoint_buffer_is_full(const StaticWaypointBuffer *buffer);

#endif /* NAVICORE_WAYPOINT_HPP */
