# Diagnosis: why h_nhc_off_predict_terms showed Delta-v_E ~ +0.42 vs B -8.3



**Verdict:** Not a predict-term mystery and not an `a_lin_y` column-mapping bug.

The original predict-terms replay ran **without** `--gap3-gnss-nis-audit-csv`, which (due to a control-flow bug) **disables GNSS velocity observations** even when `--gnss-obs-mode pos_vel` is set. That yields a different filter trajectory; the +0.42 is sum(a_lin_E*dt) on that wrong trajectory. B's -8.3 is sum(dv_pred_e) / sum(a_lin_E*dt) on the real pos+vel trajectory.



---



## 1. B_nhc_disabled -- constraint_pipeline_audit (19.301, 20.301]



| | timestamp_s | vel_before_e | dv_pred_e | vel_after_zupt_e |

|--|-------------|--------------|-----------|------------------|

| FIRST | 19.307970047 | 10.771195 | -0.057722 | 10.713473 |

| LAST | 20.296186447 | 2.575526 | -0.103605 | 2.471921 |



- **n ticks:** 98

- **sum dv_pred_e:** **-8.299271**

- NHC/ZUPT off -> `vel_after_zupt_e == vel_after_pred_e`; net Delta matches sum dv_pred.



---



## 2. Original h_nhc_off_predict_terms -- h8_propagation (same window)



| | timestamp_s | vel_pre_e | a_lin_y | dt_s | vel_post_e | roll_deg | pitch_deg | yaw_deg |

|--|-------------|-----------|---------|------|------------|----------|-----------|---------|

| FIRST | 19.307970047 | **26.427713** | 3.591253 | 0.010000 | 26.427713 | -19.86 | 12.00 | -65.60 |

| LAST | 20.296186447 | **26.811462** | -1.074366 | 0.010000 | 26.811462 | -31.53 | 24.80 | -92.20 |



- **n ticks:** 98

- **sum a_lin_y*dt:** **+0.419667** (= TABLE.md / verdict "integrator East")

- **Delta vel_post-vel_pre (endpoints):** +0.384



State at window entry is already unrelated to B (v_E ~ 26.4 vs 10.8; yaw ~ -66 deg vs ~ -124 deg).



Column check: H8 maps `a_nav_x/y/z` and `a_lin_x/y/z` -> N/E/D. On the **corrected** trajectory (below), sum a_lin_y*dt = -8.299, so **y = East is correct**.



---



## 3. GNSS accepts -- gps_index 1..20



### B (`gnss_nis_audit.csv`)



- **Accept count:** **17** / 20

- Accepts: indices **1..17** at t = 2.672, 4.301, ..., **19.301**

- Rejects: **18, 19, 20** at 20.301, 21.301, 22.301 (`reject_reason=1`)

- Log: **`n_meas=5`** from GPS #2 onward (pos+vel)



### Original predict_terms



- No `gnss_nis_audit.csv`

- `replay.log`: **`n_meas=3` always** (pos-only), **18 accepts / 0 rejects** through t~20.3 (including accept at 20.301)

- Same CLI label `gnss_obs_mode=pos_vel`, but velocity meas never armed



### Diag re-run (B audits + h8 + `--replay-end-s 21`)



- gps_index **1..18** in file (run ends at 21 s)

- **Accept 17 / 18**; #18 rejected -- matches B through the interval of interest

- **`n_meas=5`**



---



## 4. Re-run: B exactly + h8 + `--replay-end-s 21`



Command (artifacts under `docs/benchmarks/h_nhc_off_predict_terms/diag_*`):



```text

build\NaviCore3D_Replay.exe

  --input docs\benchmarks\real_run_19082026_baseline\real_run_replay.csv

  --mount-mode calibration --mount-calibration calibration\imu_mount.json

  --yaw-init zero --h9a-gravity-tilt-init

  --constraint-policy disabled --nhc-policy disabled

  --gnss-obs-mode pos_vel --p-pv-policy none

  --replay-end-s 21

  --gap3-gnss-nis-audit-csv ...\diag_gnss_nis_audit.csv

  --gap3-nhc-block-audit-csv ...\diag_nhc_block_audit.csv

  --gap3-cov-step-audit-csv ...\diag_cov_step_audit.csv

  --gap3-constraint-pipeline-audit-csv ...\diag_constraint_pipeline_audit.csv

  --h8-propagation-audit-csv ...\diag_h8_propagation.csv

  --output ...\diag_B_plus_h8_replay_output.csv

```



| Metric (window) | Diag | B original |

|-----------------|------|------------|

| sum dv_pred_e | -8.299271 | -8.299271 |

| sum a_lin_y*dt | **-8.299274** | (same physics) |

| vel_before_e @ first | 10.771195 | 10.771195 |

| vel_after_zupt_e @ last | 2.471921 | 2.471921 |



**`--replay-end-s 21` is innocent** for this discrepancy: with B's audits present, end=21 reproduces -8.3.



### B `replay_output` near 19.3 / 20.3



NED velocity is **not** in `replay_output` (only `vel_body_*`). Nearest GPS rows:



| t | row_type | vel_body_x/y/z | yaw_deg |

|---|----------|----------------|---------|

| 19.301353455 | GPS | -14.384 / 2.662 / -6.008 | -124.32 |

| 20.301353455 | GPS | -6.075 / 3.335 / -8.210 | -157.08 |



NED from audits: after Accept #17 `vel_after_e=10.771`; at Reject #18 `vel_pred_e=2.472` -> Delta-v_E ~ -8.30.



---



## Root cause (code)



In `real_run_replay.cpp`, GNSS course (Delta-NED between consecutive GPS) and thus `has_vel_obs` depend on `gap3_has_prev_gps_pos`, but that flag is updated **only** inside:



```cpp

if (gap3_gnss_nis_audit_fp != nullptr) {

    ...

    gap3_has_prev_gps_pos = true;

}

```



Without `--gap3-gnss-nis-audit-csv`:



1. `has_gps_course` stays false

2. `has_vel_obs = speed>0 && has_gps_course` stays false

3. `update_gnss_with_velocity(..., has_vel_obs=false)` -> EKF **`n_meas=3`**

4. No GNSS velocity corrections -> divergent attitude/velocity by t=19.3

5. Predict-term budget on that trajectory is sum(a_lin_E*dt) ~ **+0.42**, not B's coasting drain of **-8.3**



`tools/run_h_nhc_off_predict_terms.py` never passed any `--gap3-*-audit-csv`, while `tools/run_h_nhc_policy_ab.py` always did -- so A/B and predict-terms were **not** the same filter experiment.



---



## What this is / is not



| Hypothesis | Result |

|------------|--------|

| Different state at t17 / window entry | **Yes** -- consequence of missing vel obs |

| Different a_lin on same state | **No** -- once audits match B, a_lin_E integrates to -8.3 |

| Bug in H8 column mapping (y!=East) | **No** -- confirmed by matched re-run |

| `--replay-end-s 21` caused +0.42 | **No** -- end=21 + B audits -> -8.3 |

| Predict missing Coriolis/Earth rate explains -8.3 | **N/A on wrong traj**; on B traj the -8.3 **is** sum(a_lin_E*dt) (integrator), not an unlogged term |



---



## Fix for a valid predict-term budget



Re-run predict-terms with at least `--gap3-gnss-nis-audit-csv` (or move prev-GPS / course tracking out of the audit-only block), then recompute the term table on the B-equivalent trajectory (expect sum(a_lin_E*dt) ~ -8.3).