# Three-route NHC-off common failure

Arms: **REF** (`h_nhc_policy_ab/B_nhc_disabled`), **ALT** (`ALT_16072026`), **JUL17** (`ROUTE_20260717`).

## What is common across all three?

**Shared failure property:** EKF abandons GPS exclusively through the **5-DoF GNSS NIS gate** (`reject_reason=1`: `gnss_nis_gate > nis_threshold`). Every reject on every arm is this path (663/663, 323/323, 639/639). No other reject reason appears.

Also true on all three:

- First accept->sustained-reject regime change uses `n_meas=5`, `nis_threshold=15.086`.
- At that regime-change epoch, the **largest abs NIS contribution is a horizontal velocity term** (`ve` on REF; `vn` on ALT and JUL17); combined `|vn|+|ve|` abs share is **0.946684 / 0.623322 / 0.985306**.
- Each arm starts with a consecutive accept streak, then the gate closes; each ends in a permanent reject streak (GPS abandoned until end of audit).

## Per-arm summary

| arm | n_accept | n_reject | accept_rate | t_perm_reject | n_rej (t<=60s) | mean/median nis rejects (t<=60s) | after last early accept |
|-----|----------|----------|-------------|---------------|----------------|----------------------------------|-------------------------|
| REF | 18 | 663 | 0.026432 | 509.301513672 | 40 | 3898 / 3671.58 | 488 rejects until next_accept |
| ALT | 8 | 323 | 0.024169 | 66.268447876 | 50 | 976.388 / 71.1841 | 55 rejects until next_accept |
| JUL17 | 76 | 639 | 0.106294 | 265.140136719 | 0 | n/a / n/a | 34 rejects until next_accept |

## First regime-change reject (onset)

| arm | t | gps_index | gnss_nis_gate | thr | dominant (abs share) | innov_h_m | innov_vn | innov_ve | vel_pred_h | gps_speed | n_meas |
|-----|---|-----------|---------------|-----|----------------------|-----------|----------|----------|------------|-----------|--------|
| REF | 20.301353455 | 18 | 36.459000 | 15.086 | ve (0.921357) | 9.830275 | -1.182476 | 10.242094 | 10.675867 | 15.695425 | 5 |
| ALT | 11.268433571 | 8 | 26.883972 | 15.086 | vn (0.525474) | 27.864634 | -8.207237 | 3.813111 | 3.920029 | 6.051460 | 5 |
| JUL17 | 113.139518738 | 75 | 23.264248 | 15.086 | vn (0.550475) | 6.604749 | -5.759985 | -5.304136 | 12.224888 | 7.806792 | 5 |

## What differs? (onset signatures)

- **Onset time:** REF 20.301 s, ALT 11.268 s, JUL17 113.140 s (JUL17 has **0** rejects in the first 60 s).
- **Dominant channel:** REF `ve` (0.921); ALT `vn` (0.525) with large position `n` share (0.346); JUL17 `vn` (0.550) with large `ve` co-share (0.435).
- **NIS magnitude at onset:** 36.459 / 26.884 / 23.264 (all > 15.086).
- **Early reject NIS (t<=60 s):** REF mean 3897.995 median 3671.578 (n=40); ALT mean 976.388 median 71.184 (n=50); JUL17 n/a (n=0).
- **Post-early streak:** 488 / 55 / 34 consecutive rejects until the next accept (none of these three initial closed-gate streaks is permanent).
- **Permanent abandonment:** t=509.302 / 66.268 / 265.140 s; accept_rate 0.0264 / 0.0242 / 0.1063.

## Not claimed

This note states only what the three `gnss_nis_audit.csv` files show. It does not identify a root cause beyond the observed gate path and velocity-dominated onset contributions.
