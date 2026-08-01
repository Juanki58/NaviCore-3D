/**
 * @file bsp_ext_wdt.hpp
 * @brief External hardware watchdog supervisor (independent of RP2350 die).
 *
 * The Pico SDK `hardware/watchdog` is a real on-chip HW WDT — not a software
 * timer — but it lives on the same silicon as the firmware. A hung core that
 * still executes `watchdog_update` (or a silicon-wide lockup) defeats it.
 *
 * Wire a cheap supervisor (recommended: TI TPL5010, or MAX6822 / ADM811) to
 * `PICO2_EXT_WDT_GPIO`. Firmware pulses DONE/WDI only at the end of a healthy
 * main-loop iteration. If the MCU freezes, the external chip resets / cuts power.
 *
 * Enable with PICO2_EXT_WDT_ENABLE=1 once the chip is soldered; default 0 = no-op.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

bool pico2_bsp_ext_wdt_init(void);
/** Short pulse / toggle — call only from the healthy end-of-loop path. */
void pico2_bsp_ext_wdt_kick(void);
bool pico2_bsp_ext_wdt_enabled(void);
