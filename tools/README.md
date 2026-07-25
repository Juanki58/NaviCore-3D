# tools/

Python utilities organized by role:

| Folder | Contents |
|--------|----------|
| `lib/` | Shared modules (`geodesy`, `attitude_kinematics`, telemetry/SIL codecs) |
| `analysis/` | Allan, mobile-log parse, plots, listen |
| `benchmarks/` | `run_all_benchmarks`, Monte Carlo |
| `experiments/` | H-series / NHC / mount experiment runners (former repo-root scripts) |
| `audits/` | One-off GAP/autopsy audit scripts |
| `campaigns/` | Campaign runners (`run_gap*`, `run_ekf*`, `run_h_*`, slalom, …) |
| `sil/` | Visualizers, UDP/SIL protocol tests, JSBSim bridge |
| `field/` | Serial NavState capture |
| `ci/` | Static analysis + regression orchestrators (used by GitHub Actions) |
| `media/` | Video/render/EKF-explorer helpers |
| `reports/` | Small report generators |

Examples:

```bash
python tools/analysis/parse_mobile_log.py --input-dir data/real_run --output docs/benchmarks/real_run_replay.csv
python tools/benchmarks/run_all_benchmarks.py
python tools/ci/run_static_analysis.py --cppcheck
python tools/sil/visualizer.py
```
