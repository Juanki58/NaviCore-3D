/**
 * @file command_ingest.hpp
 * @brief Ingesta determinista de comandos de radio (trama fija 16 B, zero-heap)
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "diagnostic.hpp"
#include "waypoint.hpp"

#define RADIO_CMD_PACKET_SIZE_BYTES 16U
#define RADIO_CMD_MAGIC             0xA5U

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
 * Layout: [magic|type|seq|checksum][pos_x][pos_y][param] = 4 + 12 = 16 B
 */
typedef struct {
    uint8_t magic;
    uint8_t command_type;
    uint8_t sequence;
    uint8_t checksum;
    float pos_x;
    float pos_y;
    float param;
} RadioCommandPacket;

#if defined(__cplusplus) && (__cplusplus >= 201103L)
static_assert(sizeof(RadioCommandPacket) == RADIO_CMD_PACKET_SIZE_BYTES,
              "RadioCommandPacket must be exactly 16 bytes");
#endif

/**
 * @brief Calcula el checksum de la trama (suma uint8 de los 15 primeros bytes).
 */
uint8_t command_ingest_compute_checksum(const RadioCommandPacket *packet);

/**
 * @brief Valida checksum y ejecuta la accion asociada al CommandType.
 *
 * @param packet       Trama de radio recibida (16 B).
 * @param wpm_buffer   Buffer estatico de waypoints; puede ser NULL si no aplica.
 * @param cruise_speed Puntero a velocidad de crucero; actualizado en CMD_SET_CRUISE_SPEED.
 * @param monitor      Monitor de salud para geometry_guard; puede ser NULL.
 * @return true si la trama es valida y la accion se aplico; false en error, checksum
 *         invalido, buffer lleno o discontinuidad geometrica (sin modificar memoria).
 */
bool command_ingest_parse(
    const RadioCommandPacket *packet,
    StaticWaypointBuffer *wpm_buffer,
    float *cruise_speed,
    SystemHealthMonitor *monitor);
