/**
 * @file command_ingestor.hpp
 * @brief Ingesta determinista de comandos de radio (trama fija 16 B, zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "diagnostic.hpp"
#include "vector3d.h"
#include "waypoint.hpp"

#define RADIO_CMD_PACKET_SIZE_BYTES 16U
#define RADIO_CMD_MAGIC             0xA5U

/** Maximo estricto de comandos de radio a procesar en un unico tick de 100 ms. */
#define NAVICORE_RADIO_MAX_PACKETS_PER_TICK 5U

/** Capacidad del buffer circular RX de hardware (paquetes de 16 B). */
#define COMMAND_INGESTOR_HW_RX_CAPACITY 128U

/**
 * @brief Tipos de comando soportados por la trama de radio
 */
typedef enum {
    CMD_NOP = 0,
    CMD_ADD_WAYPOINT,
    CMD_SET_CRUISE_SPEED,
    CMD_CLEAR_WAYPOINTS
} CommandType;

/*
 * Trama fija de 16 bytes, alineada a 4 bytes para acceso float seguro en Cortex-M.
 * Layout en wire (no reordenar): [magic|type|seq|checksum][pos_x][pos_y][param] = 16 B
 */
typedef struct NAVICORE_ALIGNAS(4) {
    uint8_t magic;
    uint8_t command_type;
    uint8_t sequence;
    uint8_t checksum;
    float pos_x;
    float pos_y;
    float param;
} RadioCommandPacket;

NAVICORE_STATIC_ASSERT(sizeof(RadioCommandPacket) == RADIO_CMD_PACKET_SIZE_BYTES,
                       "RadioCommandPacket must be exactly 16 bytes");
NAVICORE_STATIC_ASSERT(sizeof(RadioCommandPacket) % 4U == 0U, "Error de alineación");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, magic) == 0U, "wire layout magic");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, command_type) == 1U, "wire layout command_type");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, sequence) == 2U, "wire layout sequence");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, checksum) == 3U, "wire layout checksum");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, pos_x) == 4U, "wire layout pos_x");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, pos_y) == 8U, "wire layout pos_y");
NAVICORE_STATIC_ASSERT(offsetof(RadioCommandPacket, param) == 12U, "wire layout param");

void command_ingestor_init(void);

bool command_ingestor_hw_has_data(void);

bool command_ingestor_hw_enqueue(const RadioCommandPacket *packet);

uint32_t command_ingestor_hw_pending_count(void);

uint32_t command_ingestor_hw_dropped_packets(void);

uint8_t command_ingestor_compute_checksum(const RadioCommandPacket *packet);

bool command_ingestor_parse(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    float *cruise_speed,
    SystemHealthMonitor *monitor);

typedef struct {
    StaticWaypointBuffer *waypoint_buffer;
    float *cruise_speed_mps;
    SystemHealthMonitor *health_monitor;
} CommandIngestContext;

typedef struct {
    uint32_t packets_processed;
    uint32_t ingest_ok;
    uint32_t ingest_fail;
    uint32_t geometry_reject;
} CommandIngestTickStats;

/**
 * @brief Procesa como maximo NAVICORE_RADIO_MAX_PACKETS_PER_TICK paquetes por tick.
 *
 * Coste CPU acotado O(1): como maximo 5 iteraciones fijas. Los paquetes sobrantes
 * permanecen en el buffer circular de hardware para el siguiente frame.
 */
uint32_t command_ingestor_process_queue(
    CommandIngestContext *ctx,
    CommandIngestTickStats *stats);
