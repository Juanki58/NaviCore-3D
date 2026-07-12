/**
 * @file pico_system_stub.cpp
 * @brief Shim minimo para enlazar power_state_machine sin SDK Ambiq
 */
#include "../ambiq_apollo/ambiq_system.hpp"

#include "pico/stdlib.h"

extern "C" void Ambiq_MCU_Enter_DeepSleep(void)
{
    /* En Pico W no entramos en deep sleep real durante el banco de pruebas. */
    tight_loop_contents();
}
