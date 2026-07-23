# Target port plan — RP2040 Adalogger + PA1010D + BNO055 (AMG)

**Status:** planned · **starts when kit arrives** · **not** a substitute for Evidence until powered  
**BOM check:** [`KIT_BOM_ADALOGGER_BNO055_PA1010D.md`](KIT_BOM_ADALOGGER_BNO055_PA1010D.md)

## Principle

First work is **software**, not wiring: new BSP target + sensor drivers. Do **not** pretend WT61C-232 / NEO-M9N code will drop in unchanged.

When this platform is what actually runs, update README with the **same honesty** as the Comarruga correction (`33f4739`): say **Adalogger + PA1010D + BNO055 validated** only after powered evidence — never “Pico 2 W validated” if the DUT is Adalogger.

## Intended BOM (kit)

| Role | Part |
|------|------|
| MCU / SD / STEMMA | Adafruit Feather **RP2040 Adalogger** (5980) |
| IMU | Adafruit **BNO055** (ADA2472) — **AMG mode only** for ESKF |
| GNSS | Adafruit Mini GPS **PA1010D** — NMEA + **PMTK** (UART preferred) |
| RF (optional) | u.FL→SMA + magnetic antenna **if** breakout has u.FL |
| Power | 3.7 V Li-ion/LiPo **JST-PH** |

## Intended tree

```
src/targets/rp2040_adalogger/
  CMakeLists.txt                 # Pico SDK / RP2040 (patterns from pico2_hardware)
  hw_config.hpp                  # STEMMA I2C, UART GPS, SD SPI, battery N/A in FW
  bsp_gnss_pa1010d.*             # NMEA + PMTK (not u-blox UBX)
  bsp_imu_bno055_amg.*           # I2C BNO055 CONFIG→AMG → ImuSample (not WT61C)
  main.cpp                       # same core ESKF / NavState / safe_log pattern
```

Reuse from `pico2_hardware/` where possible: `safe_log`, health/WDT patterns, loop timing, `NavState` USB CSV. **Replace** `bsp_wt61c` + NEO-M9N-oriented `bsp_gnss`.

## Sensor gaps vs Comarruga design

| Need | Comarruga design | This kit |
|------|------------------|----------|
| GNSS | NEO-M9N · NMEA/UBX | **PA1010D** · NMEA + **PMTK** |
| IMU | WT61C-232 · UART 0x55 | **BNO055** · I2C **AMG** |
| MCU | Pico 2 W | **RP2040 Adalogger** + microSD |

Host `nmea_parser` helps; still need PMTK init + BNO055 AMG → `ImuSample` (FRD axes).

## Arrival checklist (software-first)

1. [ ] Scaffold `src/targets/rp2040_adalogger/` from `pico2_hardware` (strip Wi-Fi).  
2. [ ] `bsp_gnss_pa1010d`: UART RX ring → NMEA → `GpsSample`; PMTK rate/sentences documented.  
3. [ ] `bsp_imu_bno055_amg`: force AMG; fill `ImuSample`; **reject** accidental NDOF as nav input.  
4. [ ] Bring-up: CDC NavState @ ≥50–100 Hz, zero heap in core.  
5. [ ] README Evidence: DUT = Adalogger + PA1010D + BNO055 AMG.  
6. [ ] Allan / outage / PPK2 on this DUT if it is primary (or keep Pico separate and label both).

## Parallel (do not wait for the parcel)

| Item | Status |
|------|--------|
| GAP-3 videos ES/EN | **Done** |
| Nordic **PPK2** | **Obtain** — still the heaviest external-credibility datum |
| Pico2 power-on | Valid alternate path if available |

## Anti-patterns

- Wiring before UART/I2C drivers compile.  
- Using BNO055 fusion output as the product navigator.  
- Claiming “hardware validated” on unboxing.  
- README still saying Pico2 when the logged DUT is Adalogger.
