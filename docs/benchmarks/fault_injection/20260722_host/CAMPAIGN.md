# Fault injection — host smoke campaign

**Date:** 2026-07-22  
**Scope:** host mirrors only (policy + safety-inject). **Physical Pico bank still pending** per [`FAULT_INJECTION_LAB.md`](../../FAULT_INJECTION_LAB.md).

## Commands

```powershell
cmake --build build --target navicore_unit_tests navicore_regression_test
.\build\navicore_unit_tests.exe "[fault],[nmea],[ubx],[wt61c],[nhc_ops],[rapidcheck][integrity]" --reporter compact
.\build\navicore_regression_test.exe --safety-inject
```

## Expected

| Gate | Meaning |
|------|---------|
| Unit `[fault]` / wire parsers | Policy + fail-closed parse |
| `--safety-inject` | NaN IMU, spoof consistency, WCET, UART/power faults |
| `[nhc_ops]` | GAP-3 default OFF; ALWAYS not production-safe |
| `[rapidcheck][integrity]` | Teleport → INCONSISTENT; nudge ≠ spoof |

## Result (2026-07-22)

| Step | Exit |
|------|------|
| Unit `[fault],[nmea],[ubx],[wt61c],[nhc_ops],[rapidcheck][integrity]` | **0** (16 cases) |
| `navicore_regression_test --safety-inject` | **0** (9 tests) |

Log: [`host_smoke_log.txt`](host_smoke_log.txt)

Re-run:

```powershell
python tools/run_fault_injection_host_smoke.py
```
