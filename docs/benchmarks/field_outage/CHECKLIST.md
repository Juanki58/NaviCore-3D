# Field outage capture checklist (Pico 2 W)

**Status:** template ready · **artefacts pending** a real DUT run  
**Goal:** one honest coast curve vs ground truth (phone GPX OK) — beats another synthetic MC for external credibility.

## Scenario

1. Open-sky warm-up ≥ 60 s (HYBRID / good fix).  
2. Enter GNSS-denied segment: tunnel, underground parking, or **SW fix hold** (silence UART1 / cover antenna — no RF jam).  
3. Durations to log: **30 s / 60 s / 120 s** (mark each).  
4. Exit to open sky; wait for reacquire.  
5. Truth: phone GPX or survey path; same start/stop marks.

## DUT

- Firmware: `NaviCore3D_Pico2` + `safe_log` CDC  
- Capture: `python tools/serial_navstate_capture.py` → CSV under  
  `docs/benchmarks/field_outage/<YYYYMMDD>/`

## Metrics to publish

| Metric | Where |
|--------|--------|
| Horizontal drift @ 30/60/120 s | vs GPX |
| Mode timeline | GPS / HYBRID / DR |
| `estimate_quality` / reject reasons | CSV columns |
| Reacquire time | after exit |

## Pass criteria (honest)

- No crash / no NaN NavState  
- DR engaged when fix lost  
- Numbers reported **as measured** (tens–hundreds of m OK — do not cherry-pick)

## After the run

1. Commit CSV + short `NOTES.md` (route, weather, IMU, GNSS module).  
2. **Mandatory:** paste drift @ 30/60/120 s (+ reacquire) into **README Evidence** — see [`EVIDENCE_CLOSEOUT.md`](../../EVIDENCE_CLOSEOUT.md).  
3. Flip roadmap **B1** to Hecho with links to folder **and** README anchor.  
4. Optional: still plot for LinkedIn alongside GAP-3 video.

## Done when

- [ ] CSV + NOTES under `docs/benchmarks/field_outage/<date>/`  
- [ ] README Evidence table updated (not only the folder)  
- [ ] Honest numbers — no cherry-pick  

**CSV alone does not close B1.**

