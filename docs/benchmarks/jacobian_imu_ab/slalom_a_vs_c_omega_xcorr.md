# SLALOM A vs C — cross-correlation ‖ω‖ × |d(Δdrift)/dt|

**ω source:** `csv_measured_A`  
**τ sweep:** 0 … 3.0 s  
**Figure:** `fig_slalom_omega_xcorr.png`  

## Measured ω (CSV yaw_rate)

- A all-zero: **False** (max|yaw_rate|=0.216)
- C all-zero: **False** (max|yaw_rate|=0.216)

### Measured vs truth kinematics

- pearson r: **1.000000**
- max|err|: 1.434e-06
- rms err: 3.987e-07
- ideal SLALOM: imu.gyro_z := truth.yaw_rate (make_ideal_slalom_imu); CSV now logs that measured ω

## Cross-correlation peaks

| Window | τ_peak (s) | r_peak | r(0) | Δr | clear_peak |
|--------|------------|--------|------|-----|------------|
| analysis_0_20s | 1.960 | 0.3274 | 0.2997 | 0.0277 | **False** |
| focus_0_8s | 1.930 | 0.3826 | 0.3117 | 0.0708 | **True** |

### Local peaks (focus 0–8 s)

- τ=1.930 s, r=0.3826

## Prior argmax lag (continuity)

- t_‖ω‖=2.000 s, t_|dΔ/dt|=3.380 s, lag=**+1.380 s** (not the primary test)

## Verdict

**DELAYED_COUPLING_SUPPORTED — clear xcorr peak at τ>0; attitude→vel→pos chain remains viable; anchor K/P to reported lag(s)**

Next: anchor K/P autopsy to τ_peak / local peaks above (and/or ~3.4 s if still a rate max), not to instantaneous argmax.
