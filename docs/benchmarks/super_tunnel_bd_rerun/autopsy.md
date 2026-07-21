# Autopsia tick-a-tick — super_tunnel_bd_rerun

Protocolo: `docs/diagnostics/16-super-tunnel-ieee952-rerun-protocol.md` §3–§4.
Salto a t≈55010 ms = reaparición GNSS (esperado); excluido del conteo de jumps en outage.

## A

- rows=6001 nhc_ticks=0 drift_exit@55s=299.07 m final=2.09 m
- P_vv_frob 0s→10s: 1.7322e+00 → 1.0954e-01
- P_pv_frob 0s→10s: 1.7321e-02 → 1.0747e-01
- drift_h 0s→10s: 0.230 → 2.145 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**

## A_dirty

- rows=6001 nhc_ticks=0 drift_exit@55s=303.64 m final=2.09 m
- P_vv_frob 0s→10s: 1.7322e+00 → 1.0943e-01
- P_pv_frob 0s→10s: 1.7321e-02 → 1.0734e-01
- drift_h 0s→10s: 0.230 → 2.145 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**

## B

- rows=6001 nhc_ticks=4500 drift_exit@55s=995.25 m final=1083.32 m
- P_vv_frob 0s→10s: 1.7322e+00 → 1.0611e-01
- P_pv_frob 0s→10s: 1.7321e-02 → 1.0451e-01
- drift_h 0s→10s: 0.230 → 2.154 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**
- max dx_pos_norm in outage NHC: 0.3029 m @t=38600 ms k_max=1.552 innov=0.268 drift=629.50
- top |Δdrift_h| among outage NHC ticks:
```
 t_ms  d_drift  drift_h_m  dx_pos_norm    k_max  innov_norm  P_vv_frob
37180 0.323974 595.781616     0.081103 1.771591    0.368861   3.280338
37150 0.323974 594.809875     0.081385 1.777839    0.373358   3.317817
37130 0.323974 594.161987     0.081687 1.781263    0.376338   3.342814
37140 0.323914 594.485901     0.081525 1.779624    0.374850   3.330316
37120 0.323914 593.838013     0.081869 1.782761    0.377823   3.355309
```
- first 2 s outage: drift 2.15→38.80 m; max|d_drift|=0.252; max k_max=0.202; max innov=6.019

## B_dirty

- rows=6001 nhc_ticks=4500 drift_exit@55s=1220.81 m final=1307.01 m
- P_vv_frob 0s→10s: 1.7322e+00 → 1.0610e-01
- P_pv_frob 0s→10s: 1.7321e-02 → 1.0448e-01
- drift_h 0s→10s: 0.230 → 2.156 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**
- max dx_pos_norm in outage NHC: 3.5240 m @t=26630 ms k_max=0.639 innov=4.204 drift=529.46
- top |Δdrift_h| among outage NHC ticks:
```
 t_ms  d_drift  drift_h_m  dx_pos_norm    k_max  innov_norm  P_vv_frob
26620 2.527466 526.955322     3.131545 0.582306    4.485946   0.572694
26630 2.507141 529.462463     3.523995 0.639122    4.204432   0.568936
26640 2.131714 531.594177     3.303260 0.615425    3.777213   0.565168
26610 2.120666 524.427856     2.229626 0.448945    4.534708   0.575568
26330 1.700013 516.841370     2.246707 0.463452    3.734177   0.612578
```
- first 2 s outage: drift 2.16→49.84 m; max|d_drift|=0.404; max k_max=0.202; max innov=7.190

## N_always

- rows=6001 nhc_ticks=6001 drift_exit@55s=1421.56 m final=1984.71 m
- P_vv_frob 0s→10s: 1.6218e+00 → 3.9030e+00
- P_pv_frob 0s→10s: 1.6217e-02 → 1.8022e+01
- drift_h 0s→10s: 0.230 → 255.270 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**
- max dx_pos_norm in outage NHC: 3.3283 m @t=10120 ms k_max=0.748 innov=4.420 drift=258.42
- top |Δdrift_h| among outage NHC ticks:
```
 t_ms  d_drift   drift_h_m  dx_pos_norm     k_max  innov_norm  P_vv_frob
54990 1.054444 1420.501465     0.843122 13.723899    0.058101 321.378052
54980 1.054199 1419.447021     0.842994 13.722520    0.058093 321.387817
54950 1.053345 1416.286133     0.842279 13.721889    0.058097 321.417023
54960 1.053345 1417.339478     0.842317 13.722205    0.058086 321.407318
54970 1.053344 1418.392822     0.842450 13.720871    0.058077 321.397583
```
- first 2 s outage: drift 255.27→314.80 m; max|d_drift|=0.487; max k_max=0.782; max innov=4.720
- pre-outage NHC ticks=1000 drift@10s=255.270 m max innov_pre=6.920 max|d_drift|_pre=1.504

## N_always_dirty

- rows=6001 nhc_ticks=6001 drift_exit@55s=1327.38 m final=1452.20 m
- P_vv_frob 0s→10s: 1.6218e+00 → 3.5380e-02
- P_pv_frob 0s→10s: 1.6217e-02 → 7.9602e-02
- drift_h 0s→10s: 0.230 → 212.259 m
- outage single-tick jumps (|Δdrift|>5 or |ΔP_vv_rel|>0.5): **0**
- max dx_pos_norm in outage NHC: 1.0562 m @t=28150 ms k_max=0.078 innov=14.527 drift=774.10
- top |Δdrift_h| among outage NHC ticks:
```
 t_ms  d_drift  drift_h_m  dx_pos_norm    k_max  innov_norm  P_vv_frob
12010 0.779511 305.323761     0.464835 0.023919   18.857454   0.027029
12020 0.779480 306.103241     0.461763 0.023913   18.907274   0.026976
12000 0.779388 304.544250     0.467639 0.023927   18.806042   0.027087
12030 0.779053 306.882294     0.458409 0.023908   18.955324   0.026928
11990 0.778931 303.764862     0.470156 0.023935   18.753349   0.027147
```
- first 2 s outage: drift 212.26→304.54 m; max|d_drift|=0.779; max k_max=0.065; max innov=18.806
- pre-outage NHC ticks=1000 drift@10s=212.259 m max innov_pre=25.247 max|d_drift|_pre=0.682

## Lectura causal (post-autopsia)

- Overall preregistrado: **IEEE952_BIAS_REJECTED** (panel_B=IEEE952_BIAS_REJECTED, panel_N=IEEE952_BIAS_REJECTED).
- NHC empeora con IMU **ideal** (D1): no atribuir a sesgo IEEE-952; el daño limpio es del propio update NHC / acoplamiento.
- Panel N_always: Δ_dirty < Δ_clean (dirty no empeora más que ideal) — opuesto a la hipótesis IEEE-952.
- Baseline A actual ~299 m ≠ histórico ~481 m: binario distinto (protocolo §0); comparar deltas, no anclar al 481 absoluto.
