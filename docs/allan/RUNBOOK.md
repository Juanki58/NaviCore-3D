# Allan variance — capture & publish runbook

**Tool (shipped):** [`analyze_allan.py`](../analyze_allan.py) — IEEE Std 952 overlapping σ_A(τ)  
**Published ARW/BI table in README:** still **pending** a multi-hour static DUT log.

## Goal

Replace engineering Q (σ_a / σ_g compile defaults) with **measured** ARW/VRW, bias instability, RRW from a quiet IMU on the same mechanical path as the product.

## Capture (hardware)

1. Mount IMU (WT61C / flight IMU) rigid, no fans/vibration, constant temperature if possible.
2. Warm-up ≥ 10 min powered; then record **≥ 2 h** (prefer 4–8 h) at fixed rate (100 Hz OK).
3. CSV columns (any time name from the tool’s list):

```text
time_ms,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z
```

- accel: m/s² (or G — tool autodetects)  
- gyro: rad/s (or deg/s — tool autodetects)

4. Save as `docs/imu_static_log.csv` (git-lfs if huge) **or** `docs/allan/<DUT>_<YYYYMMDD>.csv`.

Host capture ideas:
- Pico CDC / `tools/serial_navstate_capture.py` if you log raw IMU rates  
- USB IMU vendor logger → convert column names to the schema above

## Fit

```powershell
python tools/generate_imu_static_smoke.py   # optional: prove tool wiring (NOT publish)
python analyze_allan.py --csv docs\imu_static_log.csv --sensor both --axis auto
```

Paste IEEE units into README § Allan **only** from the real multi-hour file. Mark DUT, rate, duration, temperature notes.

## Smoke artefact (CI / desktop)

| Path | Meaning |
|------|---------|
| `docs/allan/smoke/imu_static_smoke_60s.csv` | Synthetic 60 s — **not** a datasheet |
| `docs/allan/smoke/README.md` | Disclaimer |

```powershell
python tools/generate_imu_static_smoke.py
python analyze_allan.py --csv docs\allan\smoke\imu_static_smoke_60s.csv --sensor gyro --axis gyro_z --output docs\allan\smoke\allan_gyro_z_smoke.png
```


## Done when

- [ ] `docs/imu_static_log.csv` (hours) committed or linked  
- [ ] PNG + ARW/BI numbers in README Evidence scorecard  
- [ ] One-line note: DUT model, fs, duration, date
