# IP inventory — public git vs local-only vs documented

**Date:** 2026-08-01  
**Purpose:** clarify what is already disclosed for patent / trade-secret thinking.  
**Not legal advice.** Have counsel review before filing or relying on secrecy.

## Legend

| Bucket | Meaning |
|--------|---------|
| **A — Public code** | In `main` on GitHub → treat as disclosed |
| **B — Public docs** | Methods / results described in tracked markdown/JSON/PNG |
| **C — Local-only (gitignored)** | On disk / ignored → may still be secret **if not shared elsewhere** |
| **D — Outside this repo** | Folders you keep elsewhere — list manually |

---

## A — Public code (core math / nav) — DISCLOSED

These are tracked and fetchable from the public repo (examples verified live):

| Area | Paths |
|------|--------|
| ESKF / math | `src/core/ins_ekf.*`, `ins_ekf_math.*`, `ins_ekf_15_state.*`, `ins_ekf_v2.hpp` |
| Geodesy | `src/core/geodesy.*` |
| Fusion / modes | `src/core/fusion.*`, `nav_mode_policy.*`, `NavState.*` |
| Integrity / reject | `src/core/meas_reject.hpp`, guards, `imu_cross_check.*` |
| NHC policy | `src/core/nhc_ops_policy.hpp` |
| Arbiter (new) | `src/core/technique_arbiter.*` |
| Rest of `src/core/` | parsers, health, guidance, etc. |

**Implication:** treating “the mathematical module” as secret **is not accurate** for this tree — it is public source.

---

## B — Public documentation — DISCLOSED (ideas / results)

Even without every CSV, the **methods and conclusions** are largely public, including:

- Integrity gate experiment docs + summaries  
- NHC / GAP-3 mechanism and policy docs  
- EKF v2 A/B summaries and diagnostics  
- `NAV_MODE_DEGRADATION.md`, Allan runbook, field / adversarial checklists  
- `TECHNIQUE_ARBITER.md`  

**Implication:** for patents, examiners look at **what was taught**, not only `.cpp` line counts. Detailed diagnostic write-ups count as disclosure of technical teaching.

---

## C — Local-only / gitignored — POSSIBLE SECRECY (if unshared)

| Item | Status in this workspace | Notes |
|------|--------------------------|--------|
| Bulk `*.csv` under benchmarks / monte_carlo / nhc / real_run | Ignored by `.gitignore` | Raw data often secret; **summaries in git may already teach the result** |
| `ekf_explorer/` | Ignored; **folder missing** on this machine clone | If you have it on another PC, keep it offline |
| `wifi_config.h` | Ignored | Credentials — not invention |
| Build trees | Ignored | Not IP |
| Audit dumps (`*audit*.csv`, etc.) | Ignored | May contain sensitive run detail |

On **this** clone: `ekf_explorer/` and `docs/monte_carlo/` were **not present**; `data/` and `calibration/` are almost empty / sparsely tracked.

---

## D — Outside the repo (fill in)

No second “private math” tree was found under obvious Desktop/Documents names on this PC.  
**You should list** any of:

- [ ] Notebooks / Mathematica / hand derivations  
- [ ] Another folder of filter code not in git  
- [ ] Private Google Drive / USB copies  
- [ ] Messages / email where you pasted equations  

If it only lives there and never went to GitHub → **candidate trade secret / patent intake**.

---

## Practical split for August field work

| Do | Don't |
|----|--------|
| Publish **MEASURED tables** (coast m vs s) when ready to sell trust | Paste **novel tuning laws / new gate equations** before IP screen |
| Keep raw DUT logs local until you decide | Assume “few GitHub visitors” = not disclosed |
| Invention disclosure note for anything **new vs sections A+B** | Re-patent ESKF / generic DR / generic hysteresis |

---

## One-line verdict

**Public:** almost all navigation **source** + a large body of **method docs**.  
**Still protectable in principle:** unpublished raw campaigns, unpublished new algorithms, and anything only in your private folders (bucket D) — screen those before `git push`.
