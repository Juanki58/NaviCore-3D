/**
 * @file ambiq_system.hpp
 * @brief Inicializacion de sistema, tick determinista y deep sleep
 */
#pragma once

#include <stdint.h>

extern "C" {
void Ambiq_LowPower_SystemInit(void);
void Ambiq_Hardware_Timer_WaitNextTick(void);
void Ambiq_MCU_Enter_DeepSleep(void);
}

uint32_t Ambiq_System_GetTickIndex(void);
void Ambiq_System_AdvanceTick(void);
