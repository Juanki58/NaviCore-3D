# NaviCore safety coding standard (MISRA-inspired, not certified)

**Status:** reference for contributors and auditors — **not** a formal MISRA C++ certification.  
**Scope:** `src/core/` hot path first; host/sim code follows the spirit where practical.

## Intent

NaviCore-3D positions as **safety-oriented** (zero-heap tick, explicit guards, auditable ESKF).  
Avionics / automotive reviewers share a vocabulary with MISRA-style rules even when no certificate is purchased. This doc adopts that vocabulary without claiming compliance.

## Rules we adopt (core / embedded path)

| Rule (informal) | Practice in this repo |
|-----------------|------------------------|
| No recursion in hot path | Tick / predict / update must be iterative |
| No `goto` | Prefer early `return` / structured control |
| Explicit init | Aggregate/`{}` init; no uninit POD on stack in core |
| Check all returns | I/O, GNSS update, pack, UART — discard only with `(void)` + comment if intentional |
| No dynamic allocation in `core/` | No `new`/`malloc`/`std::vector`/`std::string` on tick |
| Bounded buffers | Fixed arrays + capacity macros; no unbounded growth |
| Explicit casts | Prefer `static_cast`; avoid C-style where it hides narrowing |
| Single exit preferred (soft) | Not mandatory; clarity > dogma |
| No exceptions across tick | Core APIs are `bool` / status codes |

## What we do *not* claim

- MISRA C++:2023 (or any revision) **certified** compliance  
- DO-178C / ISO 26262 process artefacts  
- Formal WCET proof (protocol exists; on-board campaign pending)

## Tooling that backs the discourse

| Tool | Role |
|------|------|
| `cppcheck --enable=all` | Cheap static sweep |
| `clang-tidy` (`cppcoreguidelines-*`, `bugprone-*`) | Guideline / bug-prone checks |
| ASan + UBSan (PC builds) | Runtime UB / buffer issues on host |
| gcov/lcov | Line coverage of `core/` (+ `fusion`) under regression |

See [`docs/benchmarks/static_analysis/`](benchmarks/static_analysis/) for published runs, [`tools/run_static_analysis.py`](../tools/run_static_analysis.py), and CI [`.github/workflows/code-audit.yml`](../.github/workflows/code-audit.yml).

## When adding code to `src/core/`

1. Keep zero-heap invariants (`static_assert` / fixed sizes).  
2. Prefer fail-closed: reject measurement / trip guard rather than silent NaN propagation.  
3. Add a regression or inject path for new error branches (NaN sensor, WCET trip, buffer full, sensor silence) — extend `run_regression_suite_safety_inject()`.  
4. Wire-format parsers (NMEA / UBX / WT61C) stay in host-linkable `src/core/*_parser.*` and must survive `navicore_sensor_wire_fuzz` / corpus smoke without overflow or UB.  
5. Re-run `--safety-inject`, unit `[fault]/[nmea]`, and static analysis before claiming the change is “safety-clean”.  
6. On-target reactions (IMU unplug, UART flood, power cut): follow [`FAULT_INJECTION_LAB.md`](FAULT_INJECTION_LAB.md) — host policy tests are necessary but not sufficient.
