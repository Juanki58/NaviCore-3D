# Technique arbiter (N-way viability + hysteresis)

**Status:** building block — **not** wired into the Pico/EKF `nav_mode_select` path yet.

## What this is

`technique_arbiter` scores *modalities* (GNSS, IMU dead reckoning, camera, …) each cycle
and selects a primary with **enter/exit hysteresis** to avoid flicker.

## What this is not

- Not a replacement for `NavMode` (`GPS` / `HYBRID` / `DEAD_RECKONING`).
- Not a claim that camera / LiDAR / acoustic are fused in the ESKF (stubs return 0).
- Not TinyML — `ArbiterEnvContext` may later be filled by a classifier; the arbiter stays rule-based.

## Why technique ≠ NavMode

| Concept | Meaning in NaviCore |
|---------|---------------------|
| `NavMode::HYBRID` | Fresh GNSS **accepted** into the ESKF (INS+GNSS) |
| `NavTechnique::GNSS` | GNSS modality looks viable |
| `NavTechnique::IMU_DR` | Coast / backup modality looks viable |

Picking IMU_DR as “primary technique” under GNSS denial is related to entering
`DEAD_RECKONING`, but HYBRID is not “GNSS won a tournament against IMU” — both run.

## Defaults

- `NAV_ARB_ENTER_THRESHOLD` = 0.55
- `NAV_ARB_EXIT_THRESHOLD` = 0.40 (must be **&lt;** enter)

## Priority domains (product)

1. Field / air — real sales path  
2. Maritime submerged — demo / portfolio (acoustic still stub)  
3. Road — only on concrete demand  

## Next hooks (after Aug 3 field hardware)

1. Fit priors from DUT outage / Allan evidence  
2. Optionally map arbiter output → logging only, then maybe aids enable mask  
3. STEMMA env channels when optical techniques leave stub status  
