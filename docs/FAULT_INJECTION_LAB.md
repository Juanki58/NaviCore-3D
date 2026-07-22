# Fault injection lab protocol (Pico 2 W — Comarruga)

Host CI proves **policy** (`health_policy_*`, `--safety-inject`, libFuzzer on wire
parsers). This document is the **on-target** checklist: physical faults must move
`SystemHealth` / confidence flags the same way — not merely “firmware keeps running”.

## Preconditions

- Firmware: `NaviCore3D_Pico2` with `safe_log` on USB CDC
- Scope on GP22 (sensors_tick) optional
- Serial capture: `python tools/serial_navstate_capture.py` **or** `minicom`/`picocom` watching `HM:` lines
- Do **not** use RF GNSS spoof/jam (illegal in ES/EU without CNMC). Use cable / power / UART only.

## Expected reactions (policy table)

| Physical action | Symptom on wire | Expected `health_monitor` / confidence |
|-----------------|-----------------|----------------------------------------|
| Unplug WT61C UART mid-run | No `0x55` frames; `pico2_bsp_wt61c_silence_ms` grows | ≥ `PICO2_IMU_SILENCE_DEGRADE_MS` (200 ms) → `imu_degraded`, `SystemHealth::DEGRADED`, quality ×0.5; EKF predict skips |
| Short UART0 RX to noise / flood | Ring overflows | `uart0_overflow_rate` → IMU degraded; stream contaminated until next `0x55` |
| Force mid-frame gap > 5 ms | Partial WT61C frame | Parser resync (`uart_frame_timeout`); no bogus IMU sample |
| Unplug NEO-M9N / silence UART1 | No GGA | Coast (no GNSS update); optional GNSS silence degrade after 5 s (host policy) |
| Flood UART1 | Ring overflows | `uart1_overflow_rate` → GNSS degraded / untrusted |
| Pull UPS I2C or hold bus | I2C recoveries climb | > `PICO2_FT_I2C_RECOVERY_OFFLINE_MAX` → force offline → **CRITICAL** |
| Hard cut 5 V to Pico | Brown-out / reboot | On restore: clean init; WDT must have fired if loop hung |
| Stall main loop (debug halt > WDT) | No `watchdog_update` / no ext kick | HW on-chip reset ≤ 50 ms; with `PICO2_EXT_WDT_ENABLE` external chip also trips |

## Procedure — IMU cable pull

1. Boot, wait for NOMINAL + valid NavState stream (~10 s).
2. Note timestamp; **disconnect IMU UART** (RX/TX or power to WT61C only).
3. Within ≤ 1 s: confirm `imu_degraded` / quality drop in CDC log or NavState CSV.
4. Reconnect: within a few hundred ms, new frames accepted; degraded may clear on next overflow window (1 s) if silence ends — document actual clear behaviour.
5. Pass criteria: **DEGRADED asserted**; no crash; no NaN in NavState; EKF still ticks (coast).

## Procedure — UART timeout / garbage

1. With IMU connected, briefly short RX to a noise source **or** run a host UART blaster at 115200 with random bytes into Pico RX (level-safe adapter).
2. Confirm ring overflow counters increase and confidence degrades.
3. Pass criteria: no accepted frame with bad checksum; after noise stops, resync on `0x55` / `\n`.

## Procedure — abrupt power

1. Capture CDC until NOMINAL.
2. Cut VSYS / USB power without shutdown command.
3. Restore power; confirm boot banner + `health_monitor_init` path; no hung WDT loop.
4. Pass criteria: boot completes; sensors re-init; no permanent CRITICAL latch without cause.

## Procedure — task starvation (debug)

1. In GDB, break inside a long critical section **or** busy-wait > `PICO2_RX_PUMP_MAX_IDLE_US` without feeding WDT (careful: may reset).
2. Prefer measuring **controlled** `health_monitor_check_task_deadline` by temporarily lowering thresholds in a lab build.
3. Pass criteria: CRITICAL + controlled restart (`watchdog_reboot`) or HW WDT — never silent hang.

## Host mirrors (run before lab)

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target navicore_unit_tests navicore_regression_test
.\build\navicore_unit_tests.exe "[fault],[nmea],[ubx],[wt61c]"
.\build\navicore_regression_test.exe --safety-inject
```

Fuzz (Linux/CI Clang):

```bash
cmake -S . -B build_fuzz -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DNAVICORE_BUILD_FUZZERS=ON
cmake --build build_fuzz --target navicore_sensor_wire_fuzz
./build_fuzz/navicore_sensor_wire_fuzz tests/fuzz/corpus -max_total_time=60 -jobs=2
```

## Record

Store pass/fail + CDC excerpts under `docs/benchmarks/fault_injection/<YYYYMMDD>/` when a campaign is run.

**Host smoke (policy mirrors, 2026-07-22):** [`docs/benchmarks/fault_injection/20260722_host/`](benchmarks/fault_injection/20260722_host/) — re-run with `python tools/run_fault_injection_host_smoke.py`.
