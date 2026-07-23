# Kit BOM verification — Adalogger + PA1010D + BNO055

**Date:** 2026-07-23  
**Verdict:** Coherent field/logger stack for NaviCore **if** drivers match these parts (not WT61C / NEO-M9N / generic “MTK3339”).  
**Port plan:** [`TARGET_RP2040_ADALOGGER_PORT.md`](TARGET_RP2040_ADALOGGER_PORT.md) (updated to this BOM).

## Line-by-line

| # | Item | Fit for NaviCore? | Notes |
|---|------|-------------------|--------|
| 1 | **Adafruit RP2040 Adalogger (5980)** | **Yes — MCU + logger** | RP2040 + STEMMA QT + microSD + USB-C + **JST-PH** LiPo jack. Same Pico-SDK family as `pico2_hardware`. Brain of the kit. |
| 2 | **Adafruit BNO055 (ADA2472)** | **Yes — with AMG only** | 9-DoF Bosch. For our ESKF use **AMG** (raw accel/gyro/mag), **not** NDOF/IMU fused quaternion as navigation truth (would double-fuse). STEMMA QT I2C. Mind I2C clock-stretch and axis → FRD mapping. |
| 3 | **Adafruit Mini GPS PA1010D** | **Yes — GNSS** | NMEA 0183 + **PMTK** (MediaTek-family commands). UART **and** I2C. **Not** u-blox NEO-M9N. Prefer **UART → Feather RX/TX** for 5–10 Hz NMEA; keep I2C STEMMA for BNO055. |
| 4 | **STEMMA QT / Qwiic ×2 (100 mm)** | **Yes** | Enough for `Adalogger → GPS → BNO055` **if** GPS is on I2C. If GPS on UART (recommended), one cable Adalogger→BNO055 is enough; second is spare / GPS-I2C experiments. |
| 5 | **u.FL → SMA female (25 cm)** | **Conditional** | Useful **only if** your PA1010D breakout exposes **u.FL**. Confirm on the PCB when it arrives. |
| 6 | **Magnetic GPS antenna 3 m SMA male** | **Conditional** | Pairs with (5) for roof/car. Internal patch alone is weak under metal/cabin. |
| 7 | **Li-ion 3.7 V 5200 mAh JST-PH 2.0** | **Yes — power** | Matches Feather JST-PH; USB charges on-board. “+60 h” depends on GPS+IMU rate — treat as order-of-magnitude, measure with PPK2 later. Polarity must match Adafruit LiPo convention. |

## Wiring sketch (recommended)

```
[Adalogger 5980]
    STEMMA QT ──► [BNO055]          I2C, mode AMG → ImuSample
    UART TX/RX ──► [PA1010D]        NMEA + PMTK init → GpsSample
    u.FL (GPS) ──► [u.FL–SMA] ──► [mag antenna]   if connector present
    JST-PH     ──► [5200 mAh]
    microSD    ──► (card — buy if not included)
    USB-C      ──► PC (CDC NavState / charge)
```

## Software impact (vs older draft)

| Old assumption | Actual kit |
|----------------|------------|
| “MTK3339” generic | **PA1010D** (NMEA + PMTK — same family of commands, different module) |
| “I2C IMU AMG” anonymous | **BNO055** in **CONFIG_MODE → AMG** |
| NEO-M9N / UBX | **Do not reuse** `bsp_gnss` u-blox paths as-is |
| WT61C UART 0x55 | **Do not reuse** `bsp_wt61c` |

Host `nmea_parser` still helps; add PMTK init + BNO055 AMG reader.

## Gaps / buy separately

| Item | Why |
|------|-----|
| **microSD** (if not in box) | Adalogger value is logging |
| **USB-C cable** | Flash / CDC |
| **Nordic PPK2** | Power claim — not in this kit |
| Optional CR1220 | PA1010D RTC backup on Adafruit holder |

## Risks (honest)

1. **BNO055 fused modes** used by mistake → conflicts with 15-state ESKF story. Lock AMG in bring-up checklist.  
2. **GPS on same I2C as BNO055** at high rate → bus contention; UART GPS is safer for 100 Hz IMU loop.  
3. **No u.FL on board** → outdoor antenna kit sits unused until a GPS with external antenna is added.  
4. **5200 mAh “60 h”** unverified — publish mA with PPK2, don’t print runtime claims early.

## When it arrives

1. Software-first: `rp2040_adalogger` + `bsp_gnss_pa1010d` + `bsp_imu_bno055_amg`.  
2. Power-on → CDC NavState.  
3. README DUT = **Adalogger + PA1010D + BNO055** (not Pico2) — same honesty as `33f4739`.  
4. Then Allan / outage / PPK2 on **this** DUT if it becomes primary.
