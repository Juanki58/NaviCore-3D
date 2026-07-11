/**
 * @file ambiq_iom_master.cpp
 * @brief Maestro IOM SPI — am_hal_iom_config_t @ 1 MHz, transferencias bloqueantes
 */
#include "ambiq_iom_master.hpp"

#include <stddef.h>
#include <string.h>

#ifdef NAVICORE_AMBIQ_SDK
#include "am_hal_iom.h"
#include "am_hal_pwrctrl.h"
#endif

#ifndef AMBIQ_IOM_HAL_STATUS_SUCCESS
#define AMBIQ_IOM_HAL_STATUS_SUCCESS 0U
#endif

typedef struct {
    bool initialized;
#ifdef NAVICORE_AMBIQ_SDK
    void *handle;
    am_hal_iom_config_t config;
#endif
} AmbiqIomModuleState;

static AmbiqIomModuleState g_iom_modules[AMBIQ_IOM_MODULE_MAX]{};

static bool ambiq_iom_module_index_valid(uint32_t module)
{
    return module < AMBIQ_IOM_MODULE_MAX;
}

#ifdef NAVICORE_AMBIQ_SDK

static bool ambiq_iom_power_enable(uint32_t module)
{
    uint32_t periph = 0U;

    switch (module) {
    case 0U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM0;
        break;
    case 1U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM1;
        break;
    case 2U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM2;
        break;
    case 3U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM3;
        break;
    case 4U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM4;
        break;
    case 5U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM5;
        break;
    case 6U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM6;
        break;
    case 7U:
        periph = AM_HAL_PWRCTRL_PERIPH_IOM7;
        break;
    default:
        return false;
    }

    return am_hal_pwrctrl_periph_enable(periph) == AM_HAL_STATUS_SUCCESS;
}

static bool ambiq_iom_configure_spi(AmbiqIomModuleState *module_state)
{
    module_state->config.eInterfaceMode = AM_HAL_IOM_SPI_MODE;
    module_state->config.ui32ClockFreq = AM_HAL_IOM_1MHZ;
    module_state->config.eSpiMode = AM_HAL_IOM_SPI_MODE_0;
    module_state->config.ui32NBTxnBufLength = 0U;
    module_state->config.pNBTxnBuf = NULL;

    if (am_hal_iom_configure(module_state->handle, &module_state->config) !=
        AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    return am_hal_iom_enable(module_state->handle) == AM_HAL_STATUS_SUCCESS;
}

static bool ambiq_iom_sdk_init(uint32_t module)
{
    AmbiqIomModuleState *module_state = &g_iom_modules[module];

    if (module_state->initialized) {
        return true;
    }

    if (!ambiq_iom_power_enable(module)) {
        return false;
    }

    if (am_hal_iom_initialize(module, &module_state->handle) != AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    if (am_hal_iom_power_ctrl(module_state->handle, AM_HAL_SYSCTRL_WAKE, false) !=
        AM_HAL_STATUS_SUCCESS) {
        return false;
    }

    if (!ambiq_iom_configure_spi(module_state)) {
        return false;
    }

    module_state->initialized = true;
    return true;
}

static bool ambiq_iom_sdk_read_trans(
    uint32_t module,
    uint32_t chip_select,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint32_t len)
{
    AmbiqIomModuleState *module_state = &g_iom_modules[module];

    if (!module_state->initialized || module_state->handle == NULL) {
        return false;
    }

    am_hal_iom_transfer_t transaction{};
    transaction.uPeerInfo.ui32SpiChipSelect = chip_select;
    transaction.ui32InstrLen = 0U;
    transaction.ui64Instr = 0U;
    transaction.eDirection = AM_HAL_IOM_FULLDUPLEX;
    transaction.ui32NumBytes = len;
    transaction.pui32TxBuffer = (uint32_t *)tx_buf;
    transaction.pui32RxBuffer = (uint32_t *)rx_buf;
    transaction.bContinue = false;
    transaction.ui8RepeatCount = 0U;
    transaction.ui32PauseCondition = 0U;
    transaction.ui32StatusSetClr = 0U;

    return am_hal_iom_blocking_transfer(module_state->handle, &transaction) ==
        AM_HAL_STATUS_SUCCESS;
}

#endif /* NAVICORE_AMBIQ_SDK */

static bool ambiq_iom_stub_read_trans(
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint32_t len)
{
    /*
     * Stub host: eco determinista TX->RX para validar el superloop sin silicio.
     * No reserva memoria; compatible con buffers estaticos del BSP.
     */
    for (uint32_t i = 0U; i < len; ++i) {
        rx_buf[i] = tx_buf[i];
    }

    return true;
}

bool ambiq_iom_spi_init(uint32_t module)
{
    if (!ambiq_iom_module_index_valid(module)) {
        return false;
    }

#ifdef NAVICORE_AMBIQ_SDK
    return ambiq_iom_sdk_init(module);
#else
    g_iom_modules[module].initialized = true;
    return true;
#endif
}

bool ambiq_iom_spi_read_trans(
    uint32_t module,
    uint32_t chip_select,
    uint8_t *tx_buf,
    uint8_t *rx_buf,
    uint32_t len)
{
    if (tx_buf == NULL || rx_buf == NULL || len == 0U) {
        return false;
    }

    if (!ambiq_iom_module_index_valid(module)) {
        return false;
    }

    if (!g_iom_modules[module].initialized) {
        return false;
    }

#ifdef NAVICORE_AMBIQ_SDK
    return ambiq_iom_sdk_read_trans(module, chip_select, tx_buf, rx_buf, len);
#else
    (void)chip_select;
    return ambiq_iom_stub_read_trans(tx_buf, rx_buf, len);
#endif
}
