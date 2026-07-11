#include "command_ingest.hpp"

#include "diagnostic.hpp"
#include "geometry_guard.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define COMMAND_INGEST_DEFAULT_ARRIVAL_RADIUS_M 25U
#define COMMAND_INGEST_MIN_CRUISE_SPEED_MPS     0.0f
#define COMMAND_INGEST_MAX_CRUISE_SPEED_MPS     80.0f

static bool command_ingest_is_finite_float(float value)
{
    return !isnan(value) && !isinf(value);
}

static bool command_ingest_validate_header(const RadioCommandPacket *packet)
{
    if (packet == NULL) {
        return false;
    }

    if (packet->magic != RADIO_CMD_MAGIC) {
        return false;
    }

    if (packet->command_type > (uint8_t)CMD_CLEAR_WAYPOINTS) {
        return false;
    }

    const uint8_t expected = command_ingest_compute_checksum(packet);
    return packet->checksum == expected;
}

static bool waypoint_buffer_push_strict(StaticWaypointBuffer *buffer, const Waypoint *wp)
{
    if (buffer == NULL || wp == NULL) {
        return false;
    }

    if (waypoint_buffer_is_full(buffer)) {
        return false;
    }

    const size_t tail = (buffer->head + buffer->count) % NAVICORE_MAX_WAYPOINTS;
    buffer->items[tail] = *wp;
    buffer->count++;
    return true;
}

static bool command_ingest_handle_add_waypoint(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    SystemHealthMonitor *monitor)
{
    if (wpm_buffer == NULL) {
        return false;
    }

    if (!command_ingest_is_finite_float(packet->pos_x) ||
        !command_ingest_is_finite_float(packet->pos_y) ||
        !command_ingest_is_finite_float(packet->param)) {
        return false;
    }

    if (waypoint_buffer_is_full(wpm_buffer)) {
        return false;
    }

    if (!geometry_guard_validate_next(
            wpm_buffer,
            packet->pos_x,
            packet->pos_y,
            monitor)) {
        return false;
    }

    char wp_name[NAVICORE_WAYPOINT_NAME_MAX];
    (void)snprintf(
        wp_name,
        sizeof(wp_name),
        "WP%03u",
        (unsigned)packet->sequence);

    const Vector3D position = vector3d_make(packet->pos_x, packet->pos_y, packet->param);
    const Waypoint wp = waypoint_make(
        wp_name,
        position,
        NAVICORE_DOMAIN_AIR,
        COMMAND_INGEST_DEFAULT_ARRIVAL_RADIUS_M);

    return waypoint_buffer_push_strict(wpm_buffer, &wp);
}

static bool command_ingest_handle_set_cruise_speed(
    const RadioCommandPacket *packet,
    float *cruise_speed)
{
    if (cruise_speed == NULL) {
        return false;
    }

    if (!command_ingest_is_finite_float(packet->param)) {
        return false;
    }

    if (packet->param < COMMAND_INGEST_MIN_CRUISE_SPEED_MPS ||
        packet->param > COMMAND_INGEST_MAX_CRUISE_SPEED_MPS) {
        return false;
    }

    *cruise_speed = packet->param;
    return true;
}

static bool command_ingest_handle_clear_waypoints(StaticWaypointBuffer *wpm_buffer)
{
    if (wpm_buffer == NULL) {
        return false;
    }

    return waypoint_buffer_init(wpm_buffer);
}

uint8_t command_ingest_compute_checksum(const RadioCommandPacket *packet)
{
    if (packet == NULL) {
        return 0U;
    }

    const uint8_t *bytes = (const uint8_t *)packet;
    uint8_t sum = 0U;

    for (size_t i = 0U; i < sizeof(RadioCommandPacket); ++i) {
        if (i == 3U) {
            continue;
        }
        sum = (uint8_t)(sum + bytes[i]);
    }

    return sum;
}

bool command_ingest_parse(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    float *cruise_speed,
    SystemHealthMonitor *monitor)
{
    if (!command_ingest_validate_header(packet)) {
        return false;
    }

    switch ((CommandType)packet->command_type) {
    case CMD_NOP:
        return true;

    case CMD_ADD_WAYPOINT:
        return command_ingest_handle_add_waypoint(packet, wpm_buffer, monitor);

    case CMD_SET_CRUISE_SPEED:
        return command_ingest_handle_set_cruise_speed(packet, cruise_speed);

    case CMD_CLEAR_WAYPOINTS:
        return command_ingest_handle_clear_waypoints(wpm_buffer);

    default:
        return false;
    }
}
