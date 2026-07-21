# Static analysis report — baseline_20260721

Generated (UTC): 2026-07-21  
Scope: `src/core/` (ESKF / fusion / guards).  
Standard: [docs/SAFETY_CODING_STANDARD.md](../../SAFETY_CODING_STANDARD.md) (MISRA-inspired, **not** certified).

## Summary

| Tool | Status | Headline |
|------|--------|----------|
| **cppcheck** `--enable=all` | **Ran** | **32** findings on `src/core` (see top IDs) |
| **clang-tidy** `cppcoreguidelines-*` + `bugprone-*` | **Skipped** | LLVM/`clang-tidy` not on PATH (config ready: [`.clang-tidy`](../../../.clang-tidy)) |
| **ASan + UBSan** | **Blocked on this host** | MinGW 16 links fail (`-lasan` / `-lubsan` missing). Use Clang/Linux CI |
| **gcov coverage** (regression binary) | **Ran** | **`src/core` lines linked by regression ≈ 50%** (1705 / 3401) |

Artefacts: this folder (`cppcheck_*.xml`, `coverage_core.txt`, `coverage_html/`).

## cppcheck — top IDs

| ID | Count | Severity (typical) | Notes |
|----|------:|--------------------|-------|
| `normalCheckLevelMaxBranches` | 9 | information | Large functions (`ins_ekf.cpp`) — complexity / review cue |
| `memsetClassFloat` | 8 | portability | `memset` on structs with floats (IEEE zero OK; prefer `{}` init) |
| `compareValueOutOfTypeRangeError` | 3 | style | Always-true compares |
| `passedByValue` | 3 | performance | Large args by value |
| `unsignedPositive` | 3 | style | Redundant `>= 0` on unsigned |
| `wrongPrintfScanfArgNum` | 1 | **warning** | `fprintf` arg/format mismatch @ `ins_ekf.cpp:3412` |
| `dangerousTypeCast` | 1 | warning | C-style cast in `command_ingestor.cpp` |

Full XML: [`cppcheck_baseline_20260721.xml`](cppcheck_baseline_20260721.xml).

## Coverage — `navicore_regression_test` + gcov

Only translation units **linked into the regression binary** are measured (not the whole tree):

| File | Cover |
|------|------:|
| `geodesy.cpp` | 92% |
| `ins_ekf.cpp` | 50% |
| `ins_ekf_15_state.cpp` | 45% |
| `NavState.cpp` | 0% |
| `vector3d.cpp` | 8% |
| **TOTAL (linked core)** | **50%** |

**Gap (expected):** `fusion.cpp`, `guidance.cpp`, `mission.cpp`, guards, waypoint overflow paths are **not** in `navicore_regression_test` today — so error paths (NaN sensor, WCET trip, waypoint buffer full) need dedicated inject tests or a Sim-linked coverage build.

HTML: [`coverage_html/index.html`](coverage_html/index.html) · TXT: [`coverage_core.txt`](coverage_core.txt).

## Reproduce

```powershell
python tools\run_static_analysis.py --cppcheck
python tools\run_static_analysis.py --coverage-build
# after: pip install gcovr  (then re-run coverage or invoke gcovr on build_coverage)

# clang-tidy (when LLVM installed):
python tools\run_static_analysis.py --clang-tidy

# Sanitizers (Clang/Linux recommended):
cmake -S . -B build_asan -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug -DNAVICORE_ENABLE_SANITIZERS=ON
cmake --build build_asan --target navicore_regression_test
./build_asan/navicore_regression_test
```

## Next hardening (cheap → valuable)

1. Fix `wrongPrintfScanfArgNum` + replace float `memset` with value-init in hot structs.  
2. Install LLVM → publish clang-tidy baseline.  
3. CI job (Linux) with ASan/UBSan on regression + Sim smoke.  
4. Inject tests for NaN IMU, waypoint full, WCET/guard trips → raise **error-path** coverage.
