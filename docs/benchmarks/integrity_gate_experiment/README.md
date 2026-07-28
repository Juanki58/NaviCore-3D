# Integrity gate experiment

SW-injection only (no RF spoof/jam).

| File | Role |
|------|------|
| `SUMMARY.json` | Exit codes + `claims_ok` |
| `sweep.json` | Per-trial jump×gap / velocity-lie outcomes |
| `host_run_log.txt` | Catch2 + `--safety-inject` log |

Narrative + use-case examples (ES):
[`../../diagnostics/24-integrity-gate-experiment.md`](../../diagnostics/24-integrity-gate-experiment.md)

```powershell
cmake --build build --target navicore_unit_tests
python tools/campaigns/run_integrity_gate_experiment.py
```
