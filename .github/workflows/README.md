# Code audit & safety CI

Runs on push / PR to `main` or `master`.

## Jobs

| Job | What |
|-----|------|
| `cppcheck` | `cppcheck --enable=all` on `src/core` |
| `clang-tidy` | `.clang-tidy` (cppcoreguidelines + bugprone) on core `.cpp` |
| `asan-ubsan` | Clang Debug + ASan/UBSan → `--safety-inject` |
| `unit-tests` | Catch2 + **RapidCheck** properties (`navicore_unit_tests`) |

`--safety-inject` covers NaN IMU reject, waypoint-full ingest reject, WCET time_guard, geometry discontinuity, plus gravity + spoof gates — without legacy NHC/TC benchmarks that are still red locally.

## Local mirrors

```powershell
python tools\run_static_analysis.py --cppcheck
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build --target navicore_regression_test
.\build\navicore_regression_test.exe --safety-inject
```

Linux/Clang ASan:

```bash
cmake -S . -B build_asan -G Ninja \
  -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug \
  -DNAVICORE_ENABLE_SANITIZERS=ON
cmake --build build_asan --target navicore_regression_test
./build_asan/navicore_regression_test --safety-inject
```
