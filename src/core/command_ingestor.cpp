#include "command_ingestor.hpp"

#include "diagnostic.hpp"
#include "geometry_guard.hpp"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define COMMAND_INGEST_DEFAULT_ARRIVAL_RADIUS_M 25U
#define COMMAND_INGEST_MIN_CRUISE_SPEED_MPS     0.0f
#define COMMAND_INGEST_MAX_CRUISE_SPEED_MPS     80.0f

/** Buffer circular RX estatico — emula la cola DMA/hardware (zero-heap). */
static struct {
    RadioCommandPacket items[COMMAND_INGESTOR_HW_RX_CAPACITY];
    uint32_t head;
    uint32_t count;
    uint32_t dropped_packets;
} g_radio_hw_rx{};

static bool command_ingestor_hw_try_dequeue(RadioCommandPacket *packet_out)
{
    if (packet_out == NULL || g_radio_hw_rx.count == 0U) {
        return false;
    }

    *packet_out = g_radio_hw_rx.items[g_radio_hw_rx.head];
    g_radio_hw_rx.head =
        (g_radio_hw_rx.head + 1U) % COMMAND_INGESTOR_HW_RX_CAPACITY;
    g_radio_hw_rx.count--;
    return true;
}

void command_ingestor_init(void)
{
    g_radio_hw_rx.head = 0U;
    g_radio_hw_rx.count = 0U;
    g_radio_hw_rx.dropped_packets = 0U;
}

bool command_ingestor_hw_has_data(void)
{
    return g_radio_hw_rx.count > 0U;
}

bool command_ingestor_hw_enqueue(const RadioCommandPacket *packet)
{
    if (packet == NULL) {
        return false;
    }

    if (g_radio_hw_rx.count >= COMMAND_INGESTOR_HW_RX_CAPACITY) {
        g_radio_hw_rx.head =
            (g_radio_hw_rx.head + 1U) % COMMAND_INGESTOR_HW_RX_CAPACITY;
        g_radio_hw_rx.count--;
        g_radio_hw_rx.dropped_packets++;
    }

    const uint32_t tail =
        (g_radio_hw_rx.head + g_radio_hw_rx.count) % COMMAND_INGESTOR_HW_RX_CAPACITY;
    g_radio_hw_rx.items[tail] = *packet;
    g_radio_hw_rx.count++;
    return true;
}

uint32_t command_ingestor_hw_pending_count(void)
{
    return g_radio_hw_rx.count;
}

uint32_t command_ingestor_hw_dropped_packets(void)
{
    return g_radio_hw_rx.dropped_packets;
}

static bool command_ingestor_is_finite_float(float value)
{
    return !isnan(value) && !isinf(value);
}

static bool command_ingestor_validate_header(const RadioCommandPacket *packet)
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

    const uint8_t expected = command_ingestor_compute_checksum(packet);
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

static bool command_ingestor_handle_add_waypoint(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    SystemHealthMonitor *monitor)
{
    if (wpm_buffer == NULL) {
        return false;
    }

    if (!command_ingestor_is_finite_float(packet->pos_x) ||
        !command_ingestor_is_finite_float(packet->pos_y) ||
        !command_ingestor_is_finite_float(packet->param)) {
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
        COMMAND_INGEST_DEFAULT_ARRIVAL_RADIUS_M,
        NAVICORE_WAYPOINT_DEFAULT_TRANSIT_SPEED_MPS);

    return waypoint_buffer_push_strict(wpm_buffer, &wp);
}

static bool command_ingestor_handle_set_cruise_speed(
    const RadioCommandPacket *packet,
    float *cruise_speed)
{
    if (cruise_speed == NULL) {
        return false;
    }

    if (!command_ingestor_is_finite_float(packet->param)) {
        return false;
    }

    if (packet->param < COMMAND_INGEST_MIN_CRUISE_SPEED_MPS ||
        packet->param > COMMAND_INGEST_MAX_CRUISE_SPEED_MPS) {
        return false;
    }

    *cruise_speed = packet->param;
    return true;
}

static bool command_ingestor_handle_clear_waypoints(StaticWaypointBuffer *wpm_buffer)
{
    if (wpm_buffer == NULL) {
        return false;
    }

    return waypoint_buffer_init(wpm_buffer);
}

uint8_t command_ingestor_compute_checksum(const RadioCommandPacket *packet)
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

bool command_ingestor_parse(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    float *cruise_speed,
    SystemHealthMonitor *monitor)
{
    if (!command_ingestor_validate_header(packet)) {
        return false;
    }

    switch ((CommandType)packet->command_type) {
    case CMD_NOP:
        return true;

    case CMD_ADD_WAYPOINT:
        return command_ingestor_handle_add_waypoint(packet, wpm_buffer, monitor);

    case CMD_SET_CRUISE_SPEED:
        return command_ingestor_handle_set_cruise_speed(packet, cruise_speed);

    case CMD_CLEAR_WAYPOINTS:
        return command_ingestor_handle_clear_waypoints(wpm_buffer);

    default:
        return false;
    }
}

static bool command_ingestor_should_apply_packet(
    const RadioCommandPacket *packet,
    CommandIngestContext *ctx)
{
    if (packet == NULL || ctx == NULL) {
        return false;
    }

    if ((CommandType)packet->command_type != CMD_ADD_WAYPOINT) {
        return true;
    }

    if (ctx->waypoint_buffer == NULL) {
        return false;
    }

    return geometry_guard_validate_next(
        ctx->waypoint_buffer,
        packet->pos_x,
        packet->pos_y,
        ctx->health_monitor);
}

uint32_t command_ingestor_process_queue(
    CommandIngestContext *ctx,
    CommandIngestTickStats *stats)
{
    if (ctx == NULL) {
        return 0U;
    }

    uint32_t packets_processed_this_tick = 0U;

    /*
     * Bucle acotado: exactamente NAVICORE_RADIO_MAX_PACKETS_PER_TICK iteraciones
     * como maximo — coste CPU O(1) independiente del tamano de la inundacion.
     * Los paquetes no consumidos permanecen en g_radio_hw_rx para el siguiente tick.
     */
    for (uint32_t slot = 0U; slot < NAVICORE_RADIO_MAX_PACKETS_PER_TICK; ++slot) {
        if (!command_ingestor_hw_has_data()) {
            break;
        }

        RadioCommandPacket packet{};

        if (!command_ingestor_hw_try_dequeue(&packet)) {
            break;
        }

        const uint8_t geometry_error_before =
            (ctx->health_monitor != NULL) ? ctx->health_monitor->last_geometry_error : 0U;
        bool ingest_ok = false;

        if (command_ingestor_should_apply_packet(&packet, ctx)) {
            ingest_ok = command_ingestor_parse(
                &packet,
                ctx->waypoint_buffer,
                ctx->cruise_speed_mps,
                ctx->health_monitor);
        }

        packets_processed_this_tick++;

        if (stats != NULL) {
            stats->packets_processed++;
            if (ingest_ok) {
                stats->ingest_ok++;
            } else {
                stats->ingest_fail++;
            }

            if (ctx->health_monitor != NULL &&
                ctx->health_monitor->last_geometry_error == GEOMETRY_ERROR_DISCONTINUITY &&
                geometry_error_before != GEOMETRY_ERROR_DISCONTINUITY) {
                stats->geometry_reject++;
            }
        }
    }

    return packets_processed_this_tick;
}
