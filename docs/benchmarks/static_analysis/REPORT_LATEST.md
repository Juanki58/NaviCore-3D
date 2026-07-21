# Static analysis report — latest

Generated: 2026-07-21 (updated with safety-inject + CI)

Scope: `src/core/` · Standard: [docs/SAFETY_CODING_STANDARD.md](../../SAFETY_CODING_STANDARD.md)

## Summary

| Tool | Status | Headline |
|------|--------|----------|
| **cppcheck** `--enable=all` | Local + **CI** | Baseline ~32 findings on `src/core` |
| **clang-tidy** | Config + **CI** (Ubuntu) | `.clang-tidy` · cppcoreguidelines + bugprone |
| **ASan + UBSan** | **CI** (Clang) | `navicore_regression_test --safety-inject` |
| **gcov** | Local | ~50% on ESKF TUs pre-expansion; guards now linked |

## Safety-inject suite (CI gate)

```text
imu_nan_reject · waypoint_full_ingest_reject · time_guard_wcet ·
geometry_guard_discontinuity · gravity · spoof
→ RESULT: OK (6 tests)
```

Workflow: [`.github/workflows/code-audit.yml`](../../../.github/workflows/code-audit.yml)

## Reproduce

```powershell
python tools\run_static_analysis.py --cppcheck
cmake --build build --target navicore_regression_test
.\build\navicore_regression_test.exe --safety-inject
```

See also [`REPORT_baseline_20260721.md`](REPORT_baseline_20260721.md) for the first cppcheck/gcov dump.
