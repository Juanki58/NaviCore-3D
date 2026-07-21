#include "bsp_ext_wdt.hpp"
#include "hw_config.hpp"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

bool pico2_bsp_ext_wdt_enabled(void)
{
#if PICO2_EXT_WDT_ENABLE
    return true;
#else
    return false;
#endif
}

bool pico2_bsp_ext_wdt_init(void)
{
#if PICO2_EXT_WDT_ENABLE
    gpio_init(PICO2_EXT_WDT_GPIO);
    gpio_set_dir(PICO2_EXT_WDT_GPIO, GPIO_OUT);
    gpio_put(PICO2_EXT_WDT_GPIO, 0);
    return true;
#else
    return false;
#endif
}

void pico2_bsp_ext_wdt_kick(void)
{
#if PICO2_EXT_WDT_ENABLE
    /* TPL5010 DONE: rising edge; MAX6822 WDI: edge either way. Pulse high. */
    gpio_put(PICO2_EXT_WDT_GPIO, 1);
    busy_wait_us(PICO2_EXT_WDT_PULSE_US);
    gpio_put(PICO2_EXT_WDT_GPIO, 0);
#else
    (void)0;
#endif
}
