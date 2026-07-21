#include "bsp_imu_secondary.hpp"
#include "hw_config.hpp"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include <stddef.h>
#include <string.h>

namespace {

#if PICO2_SECONDARY_IMU_ENABLE

constexpr uint8_t kMpuWhoAmIReg = 0x75U;
constexpr uint8_t kMpuPwrMgmt1 = 0x6BU;
constexpr uint8_t kMpuAccelXoutH = 0x3BU;
constexpr uint8_t kMpuExpectedWho = 0x68U;
constexpr float kMpuAccelScale = 9.80665f / 16384.0f; /* ±2g default */
constexpr float kMpuGyroScale = 0.0174532925f / 131.0f; /* ±250 dps → rad/s */

bool g_present = false;

bool sec_i2c_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    const int rc = i2c_write_blocking(
        PICO2_SEC_IMU_I2C,
        PICO2_SEC_IMU_ADDR,
        buf,
        2,
        false);
    return rc == 2;
}

bool sec_i2c_read_regs(uint8_t reg, uint8_t *out, size_t n)
{
    if (out == nullptr || n == 0U) {
        return false;
    }
    if (i2c_write_blocking(PICO2_SEC_IMU_I2C, PICO2_SEC_IMU_ADDR, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(PICO2_SEC_IMU_I2C, PICO2_SEC_IMU_ADDR, out, n, false)
        == static_cast<int>(n);
}

int16_t sec_be_i16(uint8_t hi, uint8_t lo)
{
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

#endif

} /* namespace */

bool pico2_bsp_imu_secondary_present(void)
{
#if PICO2_SECONDARY_IMU_ENABLE
    return g_present;
#else
    return false;
#endif
}

bool pico2_bsp_imu_secondary_init(void)
{
#if PICO2_SECONDARY_IMU_ENABLE
    g_present = false;
    i2c_init(PICO2_SEC_IMU_I2C, PICO2_SEC_IMU_I2C_HZ);
    gpio_set_function(PICO2_SEC_IMU_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO2_SEC_IMU_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO2_SEC_IMU_SDA_PIN);
    gpio_pull_up(PICO2_SEC_IMU_SCL_PIN);

    uint8_t who = 0U;
    if (!sec_i2c_read_regs(kMpuWhoAmIReg, &who, 1U)) {
        return false;
    }
    /* MPU6050=0x68, MPU6500/9250 often 0x70/0x71 — accept 0x68 family. */
    if (who != kMpuExpectedWho && who != 0x70U && who != 0x71U) {
        return false;
    }
    if (!sec_i2c_write_reg(kMpuPwrMgmt1, 0x00U)) {
        return false;
    }
    sleep_ms(10);
    g_present = true;
    return true;
#else
    return false;
#endif
}

bool pico2_bsp_imu_secondary_poll(ImuSample *imu_out)
{
#if PICO2_SECONDARY_IMU_ENABLE
    if (!g_present || imu_out == nullptr) {
        return false;
    }
    uint8_t raw[14];
    if (!sec_i2c_read_regs(kMpuAccelXoutH, raw, sizeof(raw))) {
        g_present = false;
        return false;
    }
    memset(imu_out, 0, sizeof(*imu_out));
    imu_out->accel_mps2[0] = static_cast<float>(sec_be_i16(raw[0], raw[1])) * kMpuAccelScale;
    imu_out->accel_mps2[1] = static_cast<float>(sec_be_i16(raw[2], raw[3])) * kMpuAccelScale;
    imu_out->accel_mps2[2] = static_cast<float>(sec_be_i16(raw[4], raw[5])) * kMpuAccelScale;
    imu_out->gyro_radps[0] = static_cast<float>(sec_be_i16(raw[8], raw[9])) * kMpuGyroScale;
    imu_out->gyro_radps[1] = static_cast<float>(sec_be_i16(raw[10], raw[11])) * kMpuGyroScale;
    imu_out->gyro_radps[2] = static_cast<float>(sec_be_i16(raw[12], raw[13])) * kMpuGyroScale;
    imu_out->valid = true;
    return true;
#else
    (void)imu_out;
    return false;
#endif
}
