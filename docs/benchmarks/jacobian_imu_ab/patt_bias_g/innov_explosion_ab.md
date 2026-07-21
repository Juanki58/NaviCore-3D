# Innov explosion A vs B — truth vs filter v_body [1.69, 1.79]s

**Verdict (latch):** `A_CASCADE`
**Verdict (ctrl):** `A_CASCADE`

Truth v_body lat/vert stays ~0 (NHC assumption holds for scenario kinematics); filter v_body diverges — innov explosion is filter/state cascade, not NHC-too-rigid.

## Framing

- Truth: `slalom_kinematics_at_time` — yaw-only, velocity along heading → **v_lat ≡ 0, v_vert ≡ 0 by construction**.
- Filter: `v_body_y/z_before` from NHC audit (= −innov).
- Window: [1.69, 1.79] s; sub-tramos S1/S2/S3 to avoid homogeneous average.

## Summary latch

| qty | max | mean |
|-----|-----|------|
| |truth v_lat| | 8.882e-16 | 4.441e-16 |
| |truth v_vert| | 0.000e+00 | 0.000e+00 |
| |filter v_lat| | 1.8047 | 1.1009 |
| |filter v_vert| | 2.1816 | 1.4383 |
| innov_norm | 2.3384 | 1.9873 |

## Sub-tramos latch

| Sub | t | max|truth_lat| | max|filt_lat| | max|filt_vert| | mean‖y‖ | tag |
|-----|---|-----------------|----------------|------------------|---------|-----|
| S1 | [1.690,1.720] | 8.88e-16 | 1.805 | 1.672 | 2.026 | S1:A |
| S2 | [1.730,1.750] | 8.88e-16 | 1.298 | 1.977 | 1.826 | S2:A |
| S3 | [1.760,1.780] | 8.88e-16 | 0.863 | 2.182 | 2.096 | S3:A |

## Tick table — latch (primary)

| t | truth_v_lat | truth_v_vert | filter_v_lat | filter_v_vert | resid_lat | resid_vert | ‖y‖ |
|---|-------------|--------------|--------------|--------------|-----------|------------|-----|
| 1.690 | +0.000e+00 | +0.000e+00 | -1.6350 | +1.6718 | -1.6350 | +1.6718 | 2.3384 |
| 1.700 | -8.882e-16 | +0.000e+00 | -1.7965 | +1.3040 | -1.7965 | +1.3040 | 2.2198 |
| 1.710 | +0.000e+00 | +0.000e+00 | -1.8047 | +0.6222 | -1.8047 | +0.6222 | 1.9089 |
| 1.720 | +0.000e+00 | +0.000e+00 | -1.6228 | -0.2235 | -1.6228 | -0.2235 | 1.6381 |
| 1.730 | +0.000e+00 | +0.000e+00 | -1.2977 | -0.9981 | -1.2977 | -0.9981 | 1.6371 |
| 1.740 | +0.000e+00 | +0.000e+00 | -0.8969 | -1.5809 | -0.8969 | -1.5809 | 1.8175 |
| 1.750 | +8.882e-16 | +0.000e+00 | -0.4357 | -1.9772 | -0.4357 | -1.9772 | 2.0246 |
| 1.760 | -8.882e-16 | +0.000e+00 | +0.0827 | -2.1816 | +0.0827 | -2.1816 | 2.1831 |
| 1.770 | -8.882e-16 | +0.000e+00 | +0.5744 | -2.1168 | +0.5744 | -2.1168 | 2.1934 |
| 1.780 | +8.882e-16 | +0.000e+00 | +0.8627 | -1.7069 | +0.8627 | -1.7069 | 1.9125 |

## Tick table — ctrl (comparison)

| t | truth_v_lat | filter_v_lat | filter_v_vert | resid_lat | ‖y‖ |
|---|-------------|--------------|--------------|-----------|-----|
| 1.690 | +0.000e+00 | +0.2451 | -0.2459 | +0.2451 | 0.3472 |
| 1.700 | -8.882e-16 | +0.1658 | -0.1920 | +0.1658 | 0.2536 |
| 1.710 | +0.000e+00 | +0.1218 | -0.1543 | +0.1218 | 0.1966 |
| 1.720 | +0.000e+00 | +0.0965 | -0.1271 | +0.0965 | 0.1596 |
| 1.730 | +0.000e+00 | +0.0807 | -0.1067 | +0.0807 | 0.1338 |
| 1.740 | +0.000e+00 | +0.0701 | -0.0912 | +0.0701 | 0.1150 |
| 1.750 | +8.882e-16 | +0.0626 | -0.0791 | +0.0626 | 0.1009 |
| 1.760 | -8.882e-16 | +0.0572 | -0.0699 | +0.0572 | 0.0903 |
| 1.770 | -8.882e-16 | +0.0532 | -0.0627 | +0.0532 | 0.0823 |
| 1.780 | +8.882e-16 | +0.0504 | -0.0574 | +0.0504 | 0.0764 |

## Implication

A_CASCADE → resume attitude-Z cascade thread (onset→break→innov lag); do not open NHC-too-rigid design conversation from this ideal-slalom evidence. B would require a scenario with real sideslip/bank in truth.

Figure: `fig_innov_explosion_ab.png`
JSON: `innov_explosion_ab.json`
