# EKF v1 vs v2 — A/B NHC-off (3 routes)

Re-run after **Bowring `ecef_to_lla` fix** (GAP-6). Pre-fix snapshot: `SUMMARY_prev.json`.

| Route | Core | Accept rate | Final drift H [m] | Residual @60s [m] |
|-------|------|-------------|-------------------|-------------------|
| REF_19082026 | v1 | 0.3333 | None | None |
| REF_19082026 | v2 | 0.8781 | 13.852 | None |
| REF_19082026 | verdict | accept_up=True drift_down=False | pass=False | |
| ALT_16072026 | v1 | 0.0967 | 352260.031 | None |
| ALT_16072026 | v2 | 1.0000 | 6.233 | None |
| ALT_16072026 | verdict | accept_up=True drift_down=True | pass=True | |
| JUL17_20260717 | v1 | 0.1063 | 354011.25 | None |
| JUL17_20260717 | v2 | 0.9790 | 87.551 | None |
| JUL17_20260717 | verdict | accept_up=True drift_down=True | pass=True | |

**Overall:** REVIEW — see `SUMMARY.json` (REF v1 drift parse/`None` keeps `drift_down=False`).

## v2 drift before → after Bowring fix

| Route | Accept | H [m] | V_log [m] | 3D_log [m] |
|-------|--------|------:|----------:|-----------:|
| REF | 100%→87.8% | 35.3→13.9 | 41.6→123.3 | 54.5→124.1 |
| ALT | 100%→100% | 38.0→6.2 | 24.6→4.0 | 45.3→7.4 |
| JUL17 | 100%→97.9% | 110.3→87.6 | 58.0→23.0 | 124.6→90.5 |

Detail: `gap6_bowring_fix_impact.json`, `gap6_d_axis_autopsy.md`, `gap6_origin_mismatch_investigation.md`.

## Historical note (pre-fix reports)

Absolute drift figures produced by `real_run_replay.cpp` **before** the Bowring `ecef_to_lla` fix can carry a systematic origin/adapter offset of up to **~30 m** (constant bias on the NED↔LLA roundtrip used by the PC `InsEkf15State` adapter). Horizontal headlines (~35 / 38 / 110 m) remain useful as order-of-magnitude; vertical / 3D numbers that mixed `Ultimo GPS` with filter state were the most distorted. Pico firmware GNSS path was not affected (see investigation doc).
