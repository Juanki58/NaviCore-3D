# Target: RP2040 Adalogger (active DUT — scaffold)

**Status:** directory reserved for the active desk DUT. Firmware BSP is **not implemented yet**.

| Role | Part |
|------|------|
| MCU / SD | Adafruit Feather RP2040 Adalogger |
| IMU | BNO055 (AMG mode only) |
| GNSS | PA1010D (NMEA + PMTK) |

## Do not

- Flash `../archive/pico2_hardware/` as the product DUT
- Commit `wifi_config.h` or credentials (see root `.gitignore`)

## Next steps

1. Port plan: [`docs/TARGET_RP2040_ADALOGGER_PORT.md`](../../../docs/TARGET_RP2040_ADALOGGER_PORT.md)
2. BOM / wiring: [`docs/KIT_BOM_ADALOGGER_BNO055_PA1010D.md`](../../../docs/KIT_BOM_ADALOGGER_BNO055_PA1010D.md)
3. Copy patterns from `../archive/pico2_hardware/` (safe_log, rings, health) — replace WT61C/u-blox with BNO055 AMG + PA1010D

Intended files (when scaffolding): `CMakeLists.txt`, `hw_config.hpp`, `bsp_imu_bno055_amg.*`, `bsp_gnss_pa1010d.*`, `main.cpp`.
