# Target port plan — RP2040 Adalogger + MTK3339 + I2C IMU (AMG)

**Status:** planned · **starts when kit arrives** · **not** a substitute for Pico2 Evidence (Allan / outage / PPK2)

## Principle

First work is **software**, not wiring: new BSP target + sensor drivers. Do **not** pretend WT61C-232 / NEO-M9N code will drop in unchanged.

When this platform is what actually runs, update README with the **same honesty** as the Comarruga correction (`33f4739`): say **Adalogger validated** only after powered evidence — never “Pico 2 W validated” if the DUT is Adalogger.

## Intended tree

```
src/targets/rp2040_adalogger/   # working name — rename if board SKU differs
  CMakeLists.txt                # Pico SDK / RP2040 (same family patterns as pico2_hardware)
  hw_config.hpp                 # pins: UART GPS, I2C IMU, SD if present
  bsp_gnss_mtk3339.*            # NMEA + PMTK (not u-blox UBX path)
  bsp_imu_i2c_amg.*             # I2C accel/gyro/mag “AMG” mode (not WT61C UART 0x55)
  main.cpp                      # same core ESKF / NavState / safe_log pattern
```

Reuse from `pico2_hardware/` where possible: `safe_log`, health/WDT patterns, loop timing, `NavState` USB CSV. **Replace** `bsp_wt61c` + `bsp_gnss` (NEO-M9N/UBX assumptions).

## Sensor gaps vs current repo

| Need | Current Comarruga design | New kit |
|------|--------------------------|---------|
| GNSS | NEO-M9N · NMEA/UBX-oriented BSP | **MTK3339** · NMEA + **PMTK** config |
| IMU | WT61C-232 · UART binary frames | **I2C IMU** · **AMG** mode (accel/gyro/mag) |
| MCU board | Pico 2 W | **RP2040 Adalogger** (Feather-class logger) |

Host parsers already have generic NMEA (`nmea_parser`) — still need **PMTK** setup (baud, update rate, sentences) and an I2C AMG driver that fills `ImuSample`.

## Arrival checklist (software-first)

1. [ ] Scaffold `src/targets/rp2040_adalogger/` from `pico2_hardware` (strip Wi-Fi if unused).  
2. [ ] `bsp_gnss_mtk3339`: UART RX ring → NMEA assembler → `GpsSample`; PMTK init sequence documented.  
3. [ ] `bsp_imu_i2c_amg`: init + read → `ImuSample` (units, axes, FRD contract).  
4. [ ] Bring-up: CDC NavState @ ≥50–100 Hz without heap in core.  
5. [ ] **README Evidence closeout:** state clearly DUT = Adalogger + MTK3339 + I2C IMU; Pico2 remains “implemented / pending” unless separately powered and measured.  
6. [ ] Only then: Allan / outage / PPK2 on **this** DUT if it becomes the primary bench (or keep Pico as primary — pick one and document).

## Parallel (do not wait for the parcel)

| Item | Status |
|------|--------|
| GAP-3 videos ES/EN | **Done** — GitHub `docs/video_gap3/` |
| Nordic **PPK2** instrument | **Buy / obtain** — still the heaviest external-credibility datum once *any* board is powered |
| Pico2 power-on (if available) | Still valid path for Allan/outage **without** Adalogger |

## Anti-patterns

- Wiring party before UART/I2C drivers compile on host stubs / target.  
- Claiming “hardware validated” because the kit arrived.  
- Mixing M8U closed DR into the product story without labeling it **reference only**.
