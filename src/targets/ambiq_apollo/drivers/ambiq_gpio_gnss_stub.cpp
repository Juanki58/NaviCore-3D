/**
 * @file ambiq_gpio_gnss_stub.cpp
 * @brief Stub GPIO GNSS — sustituir por am_hal_gpio_*
 */
#include "ambiq_gpio_gnss.hpp"

#include "ambiq_driver_config.hpp"

#include "vector3d.h"

#include <string.h>

void ambiq_gpio_gnss_init(void)
{
    /* TODO(Ambiq): am_hal_gpio_pinconfig(AMBIQ_GPIO_GNSS_INT_PIN, ...); */
}

bool ambiq_gpio_gnss_data_ready(void)
{
    /*
     * TODO(Ambiq): return am_hal_gpio_input_read(AMBIQ_GPIO_GNSS_INT_PIN) != 0;
     * Stub: fix disponible cada AMBIQ_GNSS_FIX_INTERVAL_TICKS via tick externo.
     */
    return true;
}

bool ambiq_gpio_gnss_read_fix(GpsSample *gnss_out, uint32_t tick_index)
{
    if (gnss_out == NULL) {
        return false;
    }

    memset(gnss_out, 0, sizeof(*gnss_out));

    if (!ambiq_gpio_gnss_data_ready()) {
        gnss_out->fix_valid = false;
        return true;
    }

    if ((tick_index % AMBIQ_GNSS_FIX_INTERVAL_TICKS) != 0U) {
        gnss_out->fix_valid = false;
        return true;
    }

    gnss_out->fix_valid = true;
    gnss_out->position = vector3d_make(
        100.0f + ((float)tick_index * 0.1f),
        200.0f,
        50.0f);
    gnss_out->speed_mps = 1.0f;
    gnss_out->course_deg = 0.0f;
    gnss_out->satellites = 8U;
    return true;
}
