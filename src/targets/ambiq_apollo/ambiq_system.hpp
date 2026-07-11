/**
 * @file ambiq_system.hpp
 * @brief Inicializacion de sistema, tick determinista y deep sleep
 */
#pragma once

#include <stdint.h>

extern "C" {
void Ambiq_LowPower_SystemInit(void);
void Ambiq_Hardware_Timer_Init(void);
void Ambiq_Hardware_Timer_WaitNextTick(void);
void Ambiq_MCU_Enter_DeepSleep(void);
}

/** Bandera de sincronizacion ISR STIMER -> superloop (bare-metal). */
extern volatile bool g_hardware_tick_ready;

uint32_t Ambiq_System_GetTickIndex(void);
void Ambiq_System_AdvanceTick(void);
