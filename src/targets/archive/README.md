# ARCHIVED — Pico 2 W / Comarruga target

**Status:** abandoned as the **active DUT** (2026-08-01).  
**Active hardware path:** Adafruit Feather RP2040 Adalogger + BNO055 + GPS (Ultimate FeatherWing / PA1010D) — see [`../TARGET_RP2040_ADALOGGER_PORT.md`](../TARGET_RP2040_ADALOGGER_PORT.md) and [`../KIT_BOM_ADALOGGER_BNO055_PA1010D.md`](../KIT_BOM_ADALOGGER_BNO055_PA1010D.md).

## Why not deleted

This tree is the best **reference implementation** (safe_log, UART rings, health/WDT, 100 Hz loop) for scaffolding `src/targets/rp2040_adalogger/`. Deleting it would slow the Adalogger port and erase history.

## Do not

- Flash this as the “current” product DUT  
- Write README Evidence as “Pico 2 W validated” when the desk kit is Adalogger  
- Mix Comarruga pin maps with Feather wiring  

## Build (reference only)

```powershell
cmake -S src/targets/archive/pico2_hardware -B build_pico2 -G Ninja
```

Lab notes moved to [`../archive_comarruga_lab_hardware.md`](../archive_comarruga_lab_hardware.md).
