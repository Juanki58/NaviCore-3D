# Benchmarks evidence packs

Artefacts under this tree are regenerable campaign outputs (CSV/logs) plus
committed summaries (`*.md` / `*.json`).

## Historical drift figures and the GAP-6 Bowring fix

Absolute drift numbers produced by `real_run_replay.cpp` **before** the
Bowring correction in `src/core/geodesy.cpp` (`ecef_to_lla`) can include a
systematic adapter offset of up to **~30 m** on the PC
`InsEkf15State` NED→LLA→NED path. Prefer post-fix
[`ekf_v2_ab_3routes/SUMMARY.md`](ekf_v2_ab_3routes/SUMMARY.md) for current
headlines. Investigation:
[`ekf_v2_ab_3routes/gap6_origin_mismatch_investigation.md`](ekf_v2_ab_3routes/gap6_origin_mismatch_investigation.md).

Pico 2 firmware GNSS updates are **not** affected (LLA in → `lla_to_ned` only).

## Integrity gate (SW injection)

Detection-boundary sweep + plain-language use cases:
[`integrity_gate_experiment/`](integrity_gate_experiment/) ·
[`../diagnostics/24-integrity-gate-experiment.md`](../diagnostics/24-integrity-gate-experiment.md).
Regenerate: `python tools/campaigns/run_integrity_gate_experiment.py`.
