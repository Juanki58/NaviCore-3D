# Static analysis report — triage_a5

Generated (UTC): 20260721T215209Z

Scope: `src/core/` (safety-critical ESKF / fusion / guards).
Standard: [docs/SAFETY_CODING_STANDARD.md](../../SAFETY_CODING_STANDARD.md) (MISRA-inspired, not certified).

| Tool | Status | Notes |
|------|--------|-------|
| cppcheck | **ok** | issues≈38; top=[('normalCheckLevelMaxBranches', 12), ('memsetClassFloat', 9), ('compareValueOutOfTypeRangeError', 3), ('passedByValue', 3), ('unsignedPositive', 3)] |

## Reproduce

```powershell
python tools\run_static_analysis.py --all
```

Sanitizers (separate build tree):

```powershell
cmake -S . -B build_asan -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DNAVICORE_ENABLE_SANITIZERS=ON
cmake --build build_asan --target navicore_regression_test
.\build_asan\navicore_regression_test.exe
```

Coverage:

```powershell
python tools\run_static_analysis.py --coverage-build
```

