# SLALOM `--imu-mode` plumbing confirmation

**Verdict:** `--imu-mode` is **not wired** into the SLALOM sensor path. Within each Jacobian row, ideal vs dirty telemetries are identical.

## Code facts

| Item | Finding |
|------|---------|
| `run_slalom_scenario` signature | `(TelemetryInterface *telemetry, SlalomNavEmitFn emit_nav = nullptr)` -- **no** `imu_mode` (`src/scenarios/slalom_scenario.hpp`) |
| IMU generator (grep) | Only `make_ideal_slalom_imu` -- called from `slalom_scenario.cpp` loop; definition also exists in `slalom_benchmark.cpp` (same name, separate TU) |
| Dirty / tunnel IMU helpers in SLALOM path | **None** |
| `main.cpp` | `tunnel_imu_mode` passed only to `run_tunnel_stress_scenario(...)`. SLALOM branch: `run_slalom_scenario(&telemetry, emit_ekf_navigation_state)` |
| CLI `--imu-mode` under SLALOM | If set to dirty: prints `AVISO: SLALOM usa IMU ideal cinematica; --imu-mode dirty no aplica a este escenario` -- does not change generation |

## Telemetry identity (matrix cells)

Files compared on columns `drift_m`, `vel_x`, `vel_y`, `vel_z`, `yaw` (and full-file SHA-256).

| Pair | Byte-identical (SHA-256) | Max abs diffs | Identical within Jacobian row? |
|------|--------------------------|---------------|--------------------------------|
| A vs B (jcorrect, ideal vs dirty labels) | True | {"drift_m": 0.0, "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "n_rows_compared": 2501, "len_a": 2501, "len_b": 2501, "identical_rows": true} | **YES** |
| C vs D (jlegacy, ideal vs dirty labels) | True | {"drift_m": 0.0, "vel_x": 0.0, "vel_y": 0.0, "vel_z": 0.0, "yaw": 0.0, "n_rows_compared": 2501, "len_a": 2501, "len_b": 2501, "identical_rows": true} | **YES** |

SHA-256:

- A: `5c2046cab93a2514b750a4ee3bf79504c48df0ae9a187d5a3028dcdc013776c6` (885842 bytes)
- B: `5c2046cab93a2514b750a4ee3bf79504c48df0ae9a187d5a3028dcdc013776c6` (885842 bytes)
- C: `7f49da11c90e3e488369532168db54314d138a9f4d39d9c7b0860fc591d44d87` (891889 bytes)
- D: `7f49da11c90e3e488369532168db54314d138a9f4d39d9c7b0860fc591d44d87` (891889 bytes)

**Implication:** On SLALOM, the ideal/dirty half of the 2×2 matrix is plumbing noise. The informative contrast is **A vs C** (correct vs legacy NHC Jacobian under identical ideal IMU kinematics). TUNNEL_STRESS does use `--imu-mode`.

## Sources

- `src/scenarios/slalom_scenario.hpp` / `.cpp`
- `src/targets/generic_pc/main.cpp` (CLI parse + scenario dispatch)
- Telemetry: `docs/benchmarks/slalom_cell{A,B,C,D}_*_s71_telemetry.csv`
