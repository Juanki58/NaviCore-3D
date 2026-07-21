# f_va vs ΔP[ATT_Y,VEL_N] [1.10→1.54]

**Verdict:** `FVA_NOT_PATTVEL_DRIVER`

dP[ATT_Y,VN] does not track f_va materially — look elsewhere for P_av growth (other Φ terms / H / multi-path FPFᵀ).

**CORR_ABS_SCALE:** toda correlación abajo lleva mean|·| y max|·|.

Index: `f_va_vn_atty` = F discrete [VEL_N, ATT_Y] (= `f_va[0][1]`, incl. −dt).
`dP_predict` = P_pre[t] − P_post[t−1] (rebuild entre NHC).

## Latch corr packs

| pair | pearson | mean\|x\| | mean\|y\| | max\|x\| | max\|y\| |
|------|---------|---------|---------|--------|--------|
| dP_predict vs f_va_vn_atty | +0.444 | 1.747e-04 | 1.167e-02 | 1.813e-04 | 1.336e-02 |
| dP_predict vs |f_va| | -0.444 | 1.747e-04 | 1.167e-02 | 1.813e-04 | 1.336e-02 |
| dP_net_pre vs f_va_vn_atty | -0.939 | 2.666e-05 | 1.167e-02 | 7.168e-05 | 1.336e-02 |
| dP_joseph vs f_va_vn_atty | -0.976 | 1.655e-04 | 1.167e-02 | 2.516e-04 | 1.336e-02 |
| |dP_predict| vs |f_va| | -0.444 | 1.747e-04 | 1.167e-02 | 1.813e-04 | 1.336e-02 |
| |dP_predict| vs |a|_h | +0.615 | 1.747e-04 | 1.396e+00 | 1.813e-04 | 2.224e+00 |
| dP_joseph vs dx_z_applied | +nan | 1.655e-04 | 0.000e+00 | 2.516e-04 | 0.000e+00 |
| dP_joseph vs dx_z_raw | -0.949 | 1.655e-04 | 2.240e-04 | 2.516e-04 | 1.293e-03 |

## Ctrl corr packs (primary)

| pair | pearson | mean\|x\| | mean\|y\| | max\|x\| | max\|y\| |
|------|---------|---------|---------|--------|--------|
| dP_predict vs f_va_vn_atty | -0.551 | 1.614e-04 | 1.192e-02 | 1.748e-04 | 1.337e-02 |
| |dP_predict| vs |f_va| | +0.551 | 1.614e-04 | 1.192e-02 | 1.748e-04 | 1.337e-02 |

## Arm budgets

| Arm | Σ dP_predict | Σ dP_joseph | mean\|dP_pred\| | mean\|dP_jos\| | ΔP_pre |
|-----|--------------|-------------|-----------------|----------------|--------|
| ctrl | +7.103e-03 | -9.301e-03 | 1.614e-04 | 2.114e-04 | -1.858e-03 |
| latch | +7.687e-03 | -7.284e-03 | 1.747e-04 | 1.655e-04 | +4.952e-04 |

- f_va latch vs ctrl pearson = +0.982 (mean|L|=1.167e-02, mean|C|=1.192e-02)
- mean\|dP_joseph\| latch/ctrl = 0.783
- excess dP_predict vs f_va_L: pearson=+0.891, mean|excess|=1.326e-05

Figure: `fig_fva_pattvel_110_154.png`
