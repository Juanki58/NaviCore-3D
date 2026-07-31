# Field campaign checklist — Pico DUT (from 2026-08-03)

**Goal:** turn architectural claims into **measured** Evidence. Do not expand multi-technique fusion this week.

**After smoke is green:** run the adversarial plan — [FIELD_ADVERSARIAL_CAMPAIGN.md](FIELD_ADVERSARIAL_CAMPAIGN.md) (hard / cheap / legal stress).

**Out of scope this campaign:** STEMMA optical env as product claim, TinyML, acoustic/DVL product path, Show HN.

---

## 0. Before power-on

- [ ] Confirm last piece arrived and BOM matches locked blueprint / Pico 2 W bank
- [ ] Laptop + serial capture: `tools/field/serial_navstate_capture.py` (or current field logger)
- [ ] Note firmware SHA / tag on the device (`git rev-parse --short HEAD`)
- [ ] Wall clock / NTP sync on host (timestamps must be comparable)
- [ ] Spare cables, SD/logs folder empty and named `campaign_YYYYMMDD/`

## 1. Health smoke (15–30 min)

- [ ] Boot → `INITIALIZING` → `HYBRID` with outdoor sky view
- [ ] IMU stream continuous (no silence ≥ 200 ms in health)
- [ ] GNSS accept path live; `estimate_quality` in expected HYBRID band
- [ ] Save 2–5 min nominal log as `01_smoke_hybrid.csv`

## 2. GNSS outage / coast (core claim)

- [ ] Start HYBRID outdoor; mark `t0` in log notes
- [ ] Deny GNSS (antenna shield / connector pull / receiver disable — pick **one** method and stick to it)
- [ ] Confirm mode → `DEAD_RECKONING`; quality falls with fix age
- [ ] Hold **30 s**, **60 s**, **120 s** coasts (repeat ≥ 3 runs each if time)
- [ ] Reacquire → `HYBRID`; note residual horizontal error vs phone/RTK ref if available
- [ ] Save logs as `02_coast_*.csv` + short handwritten table (t, residual_m, method)

## 3. Integrity gate (software “GPS lies”)

- [ ] Replay or inject path that triggers consistency reject (`reject_reason=3`) if available on this build
- [ ] Confirm update drop / DR behaviour matches `docs/NAV_MODE_DEGRADATION.md`
- [ ] Log as `03_integrity_*.csv` — **do not** market as RF anti-spoof

## 4. Optional same day (only if 1–3 clean)

- [ ] Static Allan / bias collect (IMU at rest, duration per calibration doc)
- [ ] Power note: if PPK2 present, one profile table row; else mark **TBD** (no ULP claim)

## 5. After campaign (same evening)

- [ ] Copy raw logs + notes into repo `data/campaign_YYYYMMDD/` (or agreed path)
- [ ] Update README Evidence **only** with measured numbers (label TARGET vs MEASURED)
- [ ] Open issues for failures; do **not** tune arbiter priors from a single run
- [ ] Arbiter (`docs/TECHNIQUE_ARBITER.md`) stays **unwired** until coast Evidence is published

## Pass / fail (honest)

| Gate | Pass |
|------|------|
| Smoke HYBRID | Stable outdoor HYBRID + continuous IMU |
| Coast | Mode DR on deny; residual table for ≥30 s |
| Integrity | Behaviour matches docs (if exercised) |
| Publish | README numbers match logs; no new modality claims |

Related: [TECHNIQUE_ARBITER.md](TECHNIQUE_ARBITER.md) · [NAV_MODE_DEGRADATION.md](NAV_MODE_DEGRADATION.md) · [ROADMAP_PNT_RESILIENCE.md](ROADMAP_PNT_RESILIENCE.md) (if present)
