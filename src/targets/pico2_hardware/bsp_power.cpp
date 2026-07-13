#include "bsp_power.hpp"
#include "hw_config.hpp"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/watchdog.h"
#include "pico/error.h"
#include "pico/stdlib.h"
#include "pico/time.h"

#include <stdio.h>

namespace {

enum class PowerState : uint8_t {
    Idle = 0U,
    I2cReadProbe,
    I2cReadBattery,
    RecoveryPrep,
    RecoveryPulse,
    RecoveryReinit,
    Offline,
};

PowerState g_state = PowerState::Idle;
bool g_power_ready = false;
bool g_power_permanently_offline = false;
bool g_battery_sample_ready = false;
uint16_t g_battery_mv_last = 0U;
uint8_t g_i2c_fail_streak = 0U;
uint8_t g_recover_sessions = 0U;
uint8_t g_recovery_pulse_count = 0U;
bool g_recovery_sda_released = false;

constexpr uint64_t kUartRxBudgetUs =
    (static_cast<uint64_t>(PICO2_UART_RX_BUDGET) * 10ULL * 1000000ULL)
    / static_cast<uint64_t>(PICO2_IMU_UART_BAUD);

static_assert(
    PICO2_I2C_STEP_TIMEOUT_US < kUartRxBudgetUs,
    "PICO2_I2C_STEP_TIMEOUT_US debe ser < tiempo equivalente de PICO2_UART_RX_BUDGET");

static_assert(
    PICO2_I2C_RECOVERY_PULSE_US < kUartRxBudgetUs,
    "PICO2_I2C_RECOVERY_PULSE_US debe ser < tiempo equivalente de PICO2_UART_RX_BUDGET");

uint16_t power_i2c_timeout_reg(void)
{
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    const uint64_t counts =
        (static_cast<uint64_t>(PICO2_I2C_STEP_TIMEOUT_US) * sys_hz) / 64000000ULL;
    if (counts == 0U) {
        return 1U;
    }
    if (counts > 0xFFFFU) {
        return 0xFFFFU;
    }
    return static_cast<uint16_t>(counts);
}

void power_i2c_configure_pins(void)
{
    i2c_init(PICO2_POWER_I2C, PICO2_POWER_I2C_HZ);
    gpio_set_function(PICO2_POWER_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO2_POWER_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO2_POWER_I2C_SDA_PIN);
    gpio_pull_up(PICO2_POWER_I2C_SCL_PIN);
    i2c_set_timeout(PICO2_POWER_I2C, power_i2c_timeout_reg());
}

bool power_i2c_sda_released(void)
{
    return gpio_get(PICO2_POWER_I2C_SDA_PIN) != 0U;
}

void power_enter_permanent_offline(void)
{
    g_state = PowerState::Offline;
    g_power_permanently_offline = true;
    g_power_ready = false;
    g_battery_sample_ready = false;
    g_i2c_fail_streak = 0U;
    g_recover_sessions = 0U;
    g_recovery_pulse_count = 0U;
    i2c_deinit(PICO2_POWER_I2C);
    printf(
        "Aviso: UPS I2C OFFLINE permanente (SDA bloqueada tras %u recoveries)\n",
        PICO2_I2C_RECOVER_MAX);
}

void power_begin_recovery(void)
{
    g_power_ready = false;
    g_i2c_fail_streak = 0U;
    g_recovery_pulse_count = 0U;
    g_recovery_sda_released = false;
    g_state = PowerState::RecoveryPrep;
}

void power_handle_recover_failure(void)
{
    if (g_recover_sessions < 255U) {
        ++g_recover_sessions;
    }

    if (g_recover_sessions >= PICO2_I2C_RECOVER_MAX) {
        power_enter_permanent_offline();
    } else {
        g_state = PowerState::Idle;
    }
}

bool power_i2c_read_step(uint8_t *buf, size_t len)
{
    const absolute_time_t deadline = make_timeout_time_us(PICO2_I2C_STEP_TIMEOUT_US);
    const int rc = i2c_read_blocking_until(
        PICO2_POWER_I2C,
        PICO2_POWER_I2C_ADDR,
        buf,
        len,
        false,
        deadline);

    return rc == static_cast<int>(len);
}

void power_handle_i2c_failure(void)
{
    if (g_i2c_fail_streak < 255U) {
        ++g_i2c_fail_streak;
    }

    if (g_i2c_fail_streak >= PICO2_I2C_RECOVER_AFTER) {
        watchdog_update();
        power_begin_recovery();
        return;
    }

    g_state = PowerState::Idle;
}

void power_state_recovery_prep(void)
{
    i2c_deinit(PICO2_POWER_I2C);

    gpio_set_function(PICO2_POWER_I2C_SDA_PIN, GPIO_FUNC_SIO);
    gpio_set_function(PICO2_POWER_I2C_SCL_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(PICO2_POWER_I2C_SDA_PIN, GPIO_IN);
    gpio_pull_up(PICO2_POWER_I2C_SDA_PIN);
    gpio_set_dir(PICO2_POWER_I2C_SCL_PIN, GPIO_OUT);
    gpio_put(PICO2_POWER_I2C_SCL_PIN, 1U);

    g_recovery_pulse_count = 0U;
    g_recovery_sda_released = power_i2c_sda_released();
    g_state = PowerState::RecoveryPulse;
}

void power_state_recovery_pulse(void)
{
    gpio_put(PICO2_POWER_I2C_SCL_PIN, 0U);
    busy_wait_us(PICO2_I2C_RECOVERY_SCL_LOW_US);
    gpio_put(PICO2_POWER_I2C_SCL_PIN, 1U);
    busy_wait_us(PICO2_I2C_RECOVERY_SCL_HIGH_US);
    watchdog_update();

    if (power_i2c_sda_released()) {
        g_recovery_sda_released = true;
    }

    ++g_recovery_pulse_count;
    if (g_recovery_sda_released) {
        g_state = PowerState::RecoveryReinit;
        return;
    }

    if (g_recovery_pulse_count >= 9U) {
        g_state = PowerState::RecoveryReinit;
        return;
    }

    g_state = PowerState::RecoveryPulse;
}

void power_state_recovery_reinit(void)
{
    if (g_recovery_sda_released) {
        power_i2c_configure_pins();
        g_recover_sessions = 0U;
        g_state = PowerState::Idle;
        return;
    }

    power_handle_recover_failure();
}

void power_state_i2c_read_probe(void)
{
    uint8_t probe = 0U;
    if (power_i2c_read_step(&probe, 1U)) {
        g_power_ready = true;
        g_i2c_fail_streak = 0U;
        g_recover_sessions = 0U;
        g_state = PowerState::I2cReadBattery;
        return;
    }

    power_handle_i2c_failure();
}

void power_state_i2c_read_battery(void)
{
    uint8_t raw[2] = {0U, 0U};
    if (power_i2c_read_step(raw, sizeof(raw))) {
        g_battery_mv_last = static_cast<uint16_t>(
            (static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
        g_battery_sample_ready = true;
        g_i2c_fail_streak = 0U;
        g_recover_sessions = 0U;
        g_state = PowerState::Idle;
        return;
    }

    power_handle_i2c_failure();
}

} /* namespace */

bool pico2_bsp_power_init(void)
{
    g_state = PowerState::Idle;
    g_power_ready = false;
    g_power_permanently_offline = false;
    g_battery_sample_ready = false;
    g_battery_mv_last = 0U;
    g_i2c_fail_streak = 0U;
    g_recover_sessions = 0U;
    g_recovery_pulse_count = 0U;
    g_recovery_sda_released = false;

    power_i2c_configure_pins();
    return true;
}

void pico2_bsp_power_poll(uint32_t nav_tick_count)
{
    if (g_power_permanently_offline || g_state == PowerState::Offline) {
        return;
    }

    if (g_state == PowerState::Idle) {
        if ((nav_tick_count % PICO2_BATTERY_POLL_TICKS) != 0U) {
            return;
        }

        g_state = g_power_ready ? PowerState::I2cReadBattery : PowerState::I2cReadProbe;
        return;
    }

    switch (g_state) {
    case PowerState::I2cReadProbe:
        power_state_i2c_read_probe();
        break;
    case PowerState::I2cReadBattery:
        power_state_i2c_read_battery();
        break;
    case PowerState::RecoveryPrep:
        power_state_recovery_prep();
        break;
    case PowerState::RecoveryPulse:
        power_state_recovery_pulse();
        break;
    case PowerState::RecoveryReinit:
        power_state_recovery_reinit();
        break;
    case PowerState::Idle:
    case PowerState::Offline:
    default:
        break;
    }
}

bool pico2_bsp_power_is_offline(void)
{
    return g_power_permanently_offline;
}

bool pico2_bsp_power_consume_battery(uint16_t *battery_mv_out)
{
    if (!g_battery_sample_ready) {
        return false;
    }

    g_battery_sample_ready = false;
    if (battery_mv_out != nullptr) {
        *battery_mv_out = g_battery_mv_last;
    }
    return true;
}
