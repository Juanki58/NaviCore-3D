# NaviCore-3D: Edge INS for GPS-degraded / GPS-denied resilience

> **Repo boundary:** this tree is **INS/ESKF navigation only** — not Victron/JK BMS (`solar-telemetry`) and not `automation-scripts`. See [`docs/REPO_BOUNDARY.md`](docs/REPO_BOUNDARY.md).


> **Auditors / cached fetches:** do not trust a stale HTML view of `/`. Pin a commit.  
> Current `main` HEAD at push of this note: see latest on GitHub. Correction of overclaimed Pico bench: **`33f4739`**. Latest plan docs: **`ea07d37`**.  
> Raw README (no HTML cache): https://raw.githubusercontent.com/Juanki58/NaviCore-3D/ea07d37/README.md  
> Mission row must say *hardware bench validation pending* + *SensorLogger* â€” **not** â€œPico 2 W validatedâ€. **Active DUT path:** Adafruit Adalogger kit (see Roadmap).

```cpp
// Haiku del Programador Defensivo en C++
//
//   Sin heap en el tick â€”
//   static_assert al amanecer:
//   parada segura.
```

**ES** Â· NÃºcleo **INS/ESKF 15 estados** para MCU edge: **resiliencia de navegaciÃ³n cuando el GNSS se degrada o deniega** (dead reckoning + integridad), no un â€œautopiloto multidominioâ€ completo. **DUT activo (pedido):** Feather RP2040 Adalogger + BNO055 + GPS Adafruit Â· port en curso Â· replay SensorLogger. Consumo: **medir con PPK2**.  
**EN** Â· **15-state INS/ESKF** edge core aimed at **navigation resilience under degraded/denied GNSS** (dead reckoning + integrity) â€” not a full multi-domain autopilot. **Active DUT (ordered):** Feather RP2040 Adalogger + BNO055 + Adafruit GPS Â· port in progress Â· SensorLogger replay. Power: **measure with PPK2**.

---

## Proven vs Planned

| Bucket | Status (honest) |
|--------|-----------------|
| **Proven (repo / host)** | Zero-heap `src/core/` ESKF · PC Sim + Replay · Catch2/RapidCheck + CI audits · NHC experiment summaries in git · EKF v2 A/B on **SensorLogger mobile** traces · GAP-3 videos |
| **Reproducible, not shipped as artefacts** | Monte Carlo `TUNNEL_STRESS` runner (`tools/benchmarks/run_monte_carlo.py`) — regenerate locally; bulk CSVs under `docs/monte_carlo/` are **gitignored** / may be absent in a fresh clone |
| **Planned DUT** | **Adafruit Feather RP2040 Adalogger** + BNO055 AMG + Adafruit GPS · BSP `src/targets/rp2040_adalogger/` **not scaffolded yet** ([port plan](docs/TARGET_RP2040_ADALOGGER_PORT.md)) · powered Allan / field outage / PPK2 **pending** |
| **Archived (not active Evidence)** | Pico 2 W Comarruga tree → `src/targets/archive/pico2_hardware/` — **do not** read as “Pico validated” |

**Maturity:** method/lab strong; **hardware Evidence not closed**. Snapshot: [`docs/STATUS_ASSESSMENT.md`](docs/STATUS_ASSESSMENT.md).

---

## Contents / Contenido

1. [Proven vs Planned](#proven-vs-planned)
2. [Quick start (local)](#quick-start-local)
3. [Positioning â€” GPS-denied resilience](#positioning--gps-denied--pnt-resilience)
4. [Executive summary](#executive-summary--resumen-ejecutivo)
5. [Evidence â€” published results](#evidence--published-results)
6. [NavMode degradation matrix](#navmode-degradation-matrix)
7. [Fusion algorithm (audit)](#fusion-algorithm--what-it-is--what-it-is-not)
8. [What validates the real firmware](#what-validates-the-real-firmware--adalogger-dut)
9. [Power â€” measure before more hardware](#power--measure-before-more-hardware-ppk2)
10. [Repository layout](#repository-layout--estructura)
11. [Architecture](#architecture--arquitectura)
12. [Build](#build--compilar)
13. [Run simulator](#run-simulator--ejecutar-simulador)
14. [Real-run replay pipeline](#real-run-replay-pipeline)
15. [EKF diagnostics (H0â€“H9d, GAP-1â€¦5)](#ekf-diagnostics-real-run)
16. [Calibration](#calibration--calibraciÃ³n)
17. [Python tooling](#python-tooling)
18. [Validated stress scenarios](#validated-stress-scenarios)
19. [Digital Twin / telemetry](#digital-twin--telemetry)
20. [Roadmap](#roadmap)
21. [License](#license--author)

---

## Quick start (local)

Requisitos mÃ­nimos en PC:

| Tool | Version |
|------|---------|
| CMake | â‰¥ 3.15 |
| C++ compiler | C++17 (MinGW, MSVC, or Clang) |
| Python | â‰¥ 3.10 (replay / audits / visualizers) |
| pip packages | `numpy`, `matplotlib`, `pandas` |

```powershell
# 1) Clone / open repo
cd C:\NaviCore-3D

# 2) Python deps (visualizers + GAP audits)
pip install numpy matplotlib pandas

# 3) Configure + build all PC targets
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build

# 4) Smoke: stress simulator (writes docs\telemetria_navicore.csv)
.\build\NaviCore3D_Sim.exe --no-udp

# 5) Smoke: Catch2 units + safety-inject
.\build\navicore_unit_tests.exe
.\build\navicore_regression_test.exe --safety-inject
# or orchestrated:
python tools\run_regression_suite.py
```

Real-run EKF replay (datos en `data/real_run/`):

```powershell
python tools/analysis/parse_mobile_log.py --input-dir data\real_run --output docs\benchmarks\real_run_replay.csv
cmake --build build --target NaviCore3D_Replay
.\build\NaviCore3D_Replay.exe --help
```

DocumentaciÃ³n de reproducciÃ³n completa: [`docs/diagnostics/06-reproduction.md`](docs/diagnostics/06-reproduction.md).

---

## Positioning â€” GPS-denied / PNT resilience

### The turn

| Old framing (weak as sole pitch) | Better framing |
|----------------------------------|----------------|
| â€œUnified land / air / sea navigation coreâ€ | **Navigation resilience when GNSS is degraded, denied, or lying** (civil *assured PNT*-style coast + integrity) |
| Multi-domain as the product | Shared ESKF + `NavState` **machinery**; domain aiding still per vertical |
| Compete with Honeywell / BAE / Thales / Collins | **Do not.** That tier is certified, export-controlled, defence/avionics-priced |
| Compete with ArduPilot / sealed u-blox modules | Fill the gap below mil-grade: **lightweight, auditable, open-source (GPL or commercial), zero-heap** resilience for cost-sensitive platforms |

### Who this is (and is not) for

| Segment | Fit |
|---------|-----|
| Defence / certified avionics PNT (Honeywell, BAE, Northrop, Thales, Collins, â€¦) | **Out of scope** â€” price, ITAR/export, certification |
| **Civil / commercial underserved** â€” ag drones, low-cost AUVs, logistics robots, asset trackers, ocean buoys | **Target niche**: vulnerable to jamming/spoofing (accidental or not), no budget for mil PNT |
| Simulation / AV digital-twin as sole story | Secondary; less urgency and more crowded than GNSS resilience |

Volume and urgency sit in that middle band: enough GNSS dependency to hurt when the sky lies, not enough margin for a Collins box.

### What you already have (resilience-relevant)

| Mechanism | Where | Role |
|-----------|--------|------|
| Mode `GPS` â†’ `HYBRID` â†’ `DEAD_RECKONING` | `nav_mode_select` / EKF export | Coast when fix is weak or rejected â€” **full matrix:** [NAV_MODE_DEGRADATION.md](docs/NAV_MODE_DEGRADATION.md) |
| `estimate_quality` + fix age | `NavConfidence` | Degraded trust for mission / guards |
| GNSS NIS reject / accept | ESKF update | Integrity vs inconsistent innovation |
| INS predict without GNSS | ESKF @ ~100 Hz | Dead reckoning backbone |
| v2 fusion policy | `--ekf-core v2` | Keep position when velocity NIS fails (lab, 3 drives) |
| Adalogger + `safe_log` (port) | Embedded **DUT path** | Adafruit kit on desk â€” Evidence only after powered bring-up ([port](docs/TARGET_RP2040_ADALOGGER_PORT.md)) |
| Consistency gate (`reject_reason=3`) | ESKF GNSS update | Fix present but IMU-incompatible â†’ reject (SW spoof) |

Today you mostly react to **loss of fix** (age, reject, sats). That is necessary but not what â€œPNT resilienceâ€ sells hardest.

### The one technical wedge (not a redesign)

**Spoof / inconsistency detection (v1 shipped):** a fix that is still *present* but **physically incompatible with the IMU / INS** â€” gated on short GNSS gaps (`reject_reason=3`). Validated by **software injection** only (no RF spoof/jam).

| Today | Done |
|-------|------|
| Detect **GPS lost** (no fix / aged fix / NIS reject) | Detect **GPS lying** while `fix_valid` stays true (continuous track) |
| Dead reckoning after dropout | Flag spoof-suspect â†’ refuse update; reacquire after long outage still via NIS |

Do **not** claim anti-jam RF, CRPA, or mil anti-spoof. Claim: **IMU-consistent integrity** on an auditable edge core.

### What you still must measure

| Gap | Why |
|-----|-----|
| Forced **outage** coast curve on **Adalogger** DUT (+ optional truth logger) | Residual vs time under deny |
| Forced **spoof-like** injection in replay (teleport / velocity lie) | Shows the new detector before field RF |
| PPK2 current | Edge/low-power claim |

**Bottom line:** same architecture; civil GPS-denied resilience; consistency gate **shipped**; **MC / NHC / Allan tooling / EKF v2 A/B already banked in Evidence**. Next external credibility: **measured power (PPK2) + field outage**, not another slogan.

---

## Executive Summary / Resumen ejecutivo

| | **English** | **EspaÃ±ol** |
|---|---|---|
| **Mission** | Provide a single navigation state model across domains, with dead reckoning when GNSS fails, on bare-metal MCUs. Fusion validated with real-world IMU+GPS traces (SensorLogger); **active DUT:** Adalogger kit (port pending). | Ofrecer un modelo Ãºnico de estado de navegaciÃ³n; fusiÃ³n validada con SensorLogger; **DUT activo:** kit Adalogger (port pendiente). |
| **Estimator** | Explicit ESKF: position, velocity, attitude error, accel/gyro biases @ ~100 Hz. See [Fusion algorithm](#fusion-algorithm--what-it-is--what-it-is-not). | ESKF explÃ­cito: posiciÃ³n, velocidad, error de actitud, sesgos @ ~100 Hz. Ver [Fusion algorithm](#fusion-algorithm--what-it-is--what-it-is-not). |
| **Language** | C++17, embedded-oriented: fixed structs, no heap in `core/`. | C++17, estilo embebido: estructuras fijas, sin heap en `core/`. |
| **Memory** | **Zero dynamic allocation** in `core/`: no `std::vector`, no `std::string`, fixed buffers, stack-only hot paths. | **Cero asignaciÃ³n dinÃ¡mica** en `core/`: sin `std::vector`/`std::string`, buffers fijos, hot path en stack. |
| **Frames** | Nav: **NED**. Body: **FRD** (+X forward, +Y right, +Z down). Quaternions: Hamilton. | Nav: **NED**. Cuerpo: **FRD**. Cuaterniones: Hamilton. |
| **Coordinates (NavState API)** | Permanent 3D axes: **X = latitude**, **Y = longitude**, **Z = altitude (air) / hydrostatic pressure (sea)**. | Ejes 3D permanentes: **X = latitud**, **Y = longitud**, **Z = altitud / presiÃ³n hidrostÃ¡tica**. |
| **Host modes** | (1) Synthetic stress sim `NaviCore3D_Sim`. (2) Real vehicle replay `NaviCore3D_Replay`. | (1) Simulador de estrÃ©s. (2) Replay de vehÃ­culo real. |
| **Scientific rigor (banked)** | **Monte Carlo N=100**, **NHC experiment matrix** (GAP-3), **Allan IEEE 952 tooling**, EKF v2 A/B on real drives â€” see [Evidence](#evidence--published-results). | **MC N=100**, **matriz NHC**, **Allan IEEE 952 (herramienta)**, EKF v2 A/B en trazas reales â€” ver [Evidence](#evidence--published-results). |

---

## Evidence â€” published results

Numbers below are from **artefacts already in the repo** (not aspirational). Reproduce from the linked paths. Gaps called out honestly.

### Scientific rigor scorecard (what is already done)

This is the leap in method â€” not just â€œscenarios that look good.â€ Artefacts live under `docs/monte_carlo/`, `docs/nhc_experiments/`, and `tools/analysis/analyze_allan.py`.

| Campaign | Status | Headline result | Artefacts / how to reproduce |
|----------|--------|-----------------|------------------------------|
| **Monte Carlo** `TUNNEL_STRESS` | **Done** (re-run locally; CSVs gitignored) | N=100 Â· mean exit drift **13.0 m** Â· p95 **16.1 m** Â· **0%** diverge (>30 m) | `docs/monte_carlo/run_0000â€¦0099` Â· `python tools/benchmarks/run_monte_carlo.py --runs 100` |
| **NHC matrix** (super-tunnel + R/G arms) | **Done** (GAP-3 closed) | NHC-off baseline **493 m** exit; `B_always` **1408 m** â€” naive high-rate NHC can **hurt** | [`docs/nhc_experiments/manifest.json`](docs/nhc_experiments/manifest.json) Â· `NaviCore3D_Sim.exe --nhc-experiments` |
| **Allan variance** (IEEE Std 952) | **Tooling done** Â· fit publish pending | Overlapping Ïƒ_A(Ï„) â†’ ARW/VRW, BI, RRW; Q today = engineering Ïƒ_a/Ïƒ_g | [`tools/analysis/analyze_allan.py`](tools/analysis/analyze_allan.py) Â· needs multi-hour `docs/imu_static_log.csv` |
| **EKF v2 vs v1** (3 phone drives) | **Done** (post Bowring fix) | Accept â†’ **~88 / 100 / 98%**; drift H â†’ **~14 / 6 / 88 m** (was ~35 / 38 / 110 m pre-fix) | [`docs/benchmarks/ekf_v2_ab_3routes/`](docs/benchmarks/ekf_v2_ab_3routes/) Â· **SensorLogger mobile**, not Pico bench |

**Integrator takeaway:** coasting and aiding policy are **measured and falsifiable**, not folklore. Remaining for field credibility: **powered Adalogger** Allan fit, Adalogger outage curve, PPK2 mA (tooling/checklists ship; DUT campaigns pending).

### EKF v2 vs v1 â€” real phone drives (NHC-off shell)

Same logs, same harness; only `--ekf-core`. Source: [`docs/benchmarks/ekf_v2_ab_3routes/`](docs/benchmarks/ekf_v2_ab_3routes/) â€” **SensorLogger mobile IMU+GNSS**, not a powered Pico 2 W bench campaign.

| Route | Core | GNSS accept rate | Final horizontal drift | V_log / 3D_log (post-fix) |
|-------|------|------------------|------------------------|----------------------------|
| REF_19082026 | v1 | ~33% | (see SUMMARY) | â€” |
| REF_19082026 | **v2** | **~88%** | **~14 m** | ~123 m / ~124 m |
| ALT_16072026 | v1 | ~10% | hundreds of km | â€” |
| ALT_16072026 | **v2** | **100%** | **~6 m** | ~4 m / **~7 m** |
| JUL17_20260717 | v1 | ~11% | hundreds of km | â€” |
| JUL17_20260717 | **v2** | **~98%** | **~88 m** | ~23 m / ~91 m |

**Verdict:** v2 still anchors far better than v1. After the GAP-6 Bowring `ecef_to_lla` fix, reported H improved; ALT 3D is now **~7 m**. Pre-fix headlines (~35 / 38 / 110 m H) can include up to **~30 m** systematic adapter offset â€” see [`gap6_origin_mismatch_investigation.md`](docs/benchmarks/ekf_v2_ab_3routes/gap6_origin_mismatch_investigation.md). Pico GNSS path unaffected.

### Monte Carlo â€” `TUNNEL_STRESS` (synthetic)

`tools/benchmarks/run_monte_carlo.py` â†’ `docs/monte_carlo/run_0000â€¦run_0099` (**N = 100** seeds). Metric: horizontal drift at tunnel exit **t = 30 s**.

| Stat | Value |
|------|------:|
| Runs | **100** |
| Mean drift | **13.0 m** |
| Std | **1.26 m** |
| Min / max | 11.8 / 18.6 m |
| p95 / p99 | 16.1 / 18.5 m |
| Divergence rate (drift > 30 m) | **0%** |

**Why it matters:** distributional claim (mean/std/p95/diverge), not a single lucky seed. Re-run anytime; CSVs are in-repo.

### NHC â€” what the matrix actually showed

Not a marketing â€œNHC always helpsâ€ claim. Super-tunnel + R-sweep artefacts in [`docs/nhc_experiments/`](docs/nhc_experiments/) (`manifest.json`):

| Condition | Drift @ tunnel exit | Drift final (post-GNSS) |
|-----------|--------------------:|------------------------:|
| **A â€” NHC off** (baseline) | **493 m** | **~2 m** (reacquire) |
| Best G-arm (`G_l10_v10`) | **758 m** | **887 m** |
| `B_always` (NHC every tick) | **1408 m** | **1554 m** |

G-arm dose-response (lateral/vertical Ïƒ variants) is fully tabulated in the manifest â€” every published arm **worse** than NHC-off at exit.

**Finding (GAP-3 closed):** naive high-rate NHC over-observes body velocity, compresses `P_vv`, and can **worsen** coasting vs NHC-off; dose-response N=1â€¦20 documented. Operational policy: **NHC off** or **gap-triggered (v2)** â€” not â€œalways onâ€. Frozen in code: [`nhc_ops_policy.hpp`](src/core/nhc_ops_policy.hpp) Â· [`docs/NHC_OPS_POLICY.md`](docs/NHC_OPS_POLICY.md) Â· CI `[nhc_ops]`.

Reproduce: `NaviCore3D_Sim.exe --nhc-experiments` Â· diagnostics [`docs/diagnostics/10-gap3-ins-model-audit.md`](docs/diagnostics/10-gap3-ins-model-audit.md) Â· **video pack** [`docs/VIDEO_GAP3_PRODUCTION.md`](docs/VIDEO_GAP3_PRODUCTION.md).

**Explainer videos (published on GitHub):**

| Lang | Watch / download |
|------|------------------|
| ES | [blob](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC.mp4) Â· [raw](https://raw.githubusercontent.com/Juanki58/NaviCore-3D/main/docs/video_gap3/NaviCore_GAP3_NHC.mp4) |
| EN | [blob](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC_en.mp4) Â· [raw](https://raw.githubusercontent.com/Juanki58/NaviCore-3D/main/docs/video_gap3/NaviCore_GAP3_NHC_en.mp4) |

Pack / regenerate: [`docs/VIDEO_GAP3_PRODUCTION.md`](docs/VIDEO_GAP3_PRODUCTION.md) Â· `python tools/media/render_gap3_video.py`. Optional YouTube mirrors: paste URLs here when uploaded.

### Allan variance (IEEE 952) â€” methodology shipped

| Item | Status |
|------|--------|
| Tool | [`tools/analysis/analyze_allan.py`](tools/analysis/analyze_allan.py) â€” overlapping Allan Ïƒ_A(Ï„); ARW/VRW, bias instability, RRW (IEEE Std 952-1997) |
| Shipped Q scalars | Ïƒ_a = **0.05 m/sÂ²**, Ïƒ_g = **0.002 rad/s** (car-log / mount order of magnitude in `ins_ekf.hpp`) |
| **Published ARW/BI table from multi-hour static IMU** | **Pending data** â€” commit `docs/imu_static_log.csv` (hours), run Allan, paste IEEE units into this section |
| Capture / publish steps | [`docs/allan/RUNBOOK.md`](docs/allan/RUNBOOK.md) |
| Tool smoke (synthetic 60 s â€” **not** for Evidence) | `docs/allan/smoke/` Â· `tools/media/generate_imu_static_smoke.py` |

```powershell
python tools/analysis/analyze_allan.py --csv docs\imu_static_log.csv --axis gyro_z
# smoke only (do not publish numbers):
python tools\generate_imu_static_smoke.py
python tools/analysis/analyze_allan.py docs\allan\smoke\imu_static_smoke_60s.csv --sensor gyro --axis gyro_z -o docs\allan\smoke\allan_gyro_z_smoke.png
```

**Honest framing:** the **pipeline** for replacing Q folklore with IEEE numbers is done and documented; the **published fit table** waits on a multi-hour static log. Until then, treat Q as engineering defaults â€” not a certified IMU datasheet.

### Integrity / spoof (software only)

| Check | Result |
|-------|--------|
| Teleport +500 m with `fix_valid=true` (short gap) | Rejected Â· `reject_reason=3` Â· test `gnss_physical_inconsistency_spoof` |
| RapidCheck: random teleport 200â€“800 m (short gap) | Reject Â· `MEAS_REJECT_INCONSISTENT` Â· state/P hold Â· `[rapidcheck][integrity]` |
| RapidCheck: sub-metre GNSS nudge | Never classified as physical spoof |
| **Gate sweep** (jumps 1â€“500 m Ã— gaps 0.2/1/3 s + velocity lie) | `claims_ok` Â· [`integrity_gate_experiment/`](docs/benchmarks/integrity_gate_experiment/) Â· [`24-integrityâ€¦`](docs/diagnostics/24-integrity-gate-experiment.md) |
| RF GPS spoof / jam | **Out of scope** â€” illegal in ES/EU without CNMC authorisation |

### Catch2 unit tests (isolated â€” not the Sim pipeline)

Formal C++ unit tests (**Catch2 v3**, FetchContent) over kernels only â€” no Monte Carlo orchestration, no tunnel scenario:

| File | Covers |
|------|--------|
| `tests/unit/test_navstate_math.cpp` | `NavState` heading wrap, quality clamp, speed EPS (`math_utils.hpp`) |
| `tests/unit/test_ins_ekf_math.cpp` | `navicore_quat_normalize` (zeroâ†’identity), `mat_invert2x2/3x3` singular â†’ false |
| `tests/unit/test_fusion_isolated.cpp` | `dead_reckoning_*` init + NaN/Inf reject |
| `tests/unit/test_properties_rapidcheck.cpp` | **RapidCheck** properties (see below) |
| `tests/unit/test_ins_ekf_edge.cpp` | EKF init/NaN/invalid GNSS/dt edge cases |
| `tests/unit/test_nhc_ops_policy.cpp` | GAP-3 NHC ops freeze (`OFF` default; `ALWAYS` not production-safe) |

Math kernels live in [`src/core/ins_ekf_math.hpp`](src/core/ins_ekf_math.hpp) so the ESKF can be tested **without** linking Sim/Replay.

### RapidCheck property tests (above unit cases)

Properties that must hold for *all* generated inputs (default **300** trials via `RC_PARAMS=max_success=300`; shrinks counterexamples):

| Property | Statement |
|----------|-----------|
| Fix-age quality | `age_a â‰¤ age_b` â‡’ `quality(age_a) â‰¥ quality(age_b)` (`nav_confidence_quality_from_fix_age_ms`) |
| Heading | `normalize(h) âˆˆ [0, 360)` for bounded `h` |
| Quaternion | after normalize, â€–qâ€– â‰ˆ 1 (zero-norm â†’ identity, no `/0`) |
| Sâ»Â¹ | if `invert2x2` succeeds â‡’ `SÂ·Sâ»Â¹ â‰ˆ I` |
| EKF @ 100 Hz | one 10 ms predict: horizontal â€–Î”pâ€– â‰¤ civil envelope (~1.3 m for \|v\|â‰¤80, \|a\|â‰¤40) |
| Integrity teleport | short-gap jump 200â€“800 m â‡’ reject `INCONSISTENT`; position & `P_nn`/`P_ee` hold |
| Integrity nudge | \|Î”p\| â‰¤ 2 m â‡’ never `INCONSISTENT` / not suspect |

This is stricter than Monte Carlo over fixed scenarios: the fuzzer searches for breaks, not just average drift.

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release -DNAVICORE_BUILD_UNIT_TESTS=ON
cmake --build build --target navicore_unit_tests
$env:RC_PARAMS="max_success=500"
.\build\navicore_unit_tests.exe
# filter properties only:
.\build\navicore_unit_tests.exe "[rapidcheck]"
```

Disable all unit/property builds with `-DNAVICORE_BUILD_UNIT_TESTS=OFF`.

### Safety inject tests (error paths)

Fail-closed paths exercised by `navicore_regression_test --safety-inject` (CI + ASan):

| Test | What it proves |
|------|----------------|
| `imu_nan_reject` | `ins_ekf_predict` / DR reject NaN/Inf IMU; state unchanged |
| `waypoint_full_ingest_reject` | Radio `ADD_WAYPOINT` rejected at 64/64 (no silent overwrite) |
| `time_guard_wcet` | Over-budget ticks â†’ `TIME_GUARD_ERROR_WCET` + health penalty |
| `geometry_guard_discontinuity` | Step â‰« 150 m â†’ `GEOMETRY_ERROR_DISCONTINUITY` |
| `nmea_ubx_corrupt_wire` | NMEA assembler / UBX oversize / WT61C noise â†’ fail-closed (no bogus sample) |
| `fault_imu_silence_policy` | IMU silence â‰¥ 200 ms â†’ `DEGRADED` + `imu_should_degrade` (host mirror of Pico) |
| `fault_uart_overflow_and_power` | UART overflow rate â†’ degrade; power offline / task starvation â†’ **CRITICAL** |
| (+ gravity, spoof) | Smoke of core ESKF + consistency gate |

```powershell
.\build\navicore_regression_test.exe --safety-inject
```

Full suite (includes legacy NHC/TC benchmarks that may still FAIL): omit the flag.

### Sensor wire fuzzing (NMEA / UBX / WT61C)

Parsers live in host-buildable core (`nmea_parser`, `ubx_parser`, `wt61c_parser`) â€” same code Pico BSP calls. libFuzzer (Clang) + ASan/UBSan hunt overflows / UB on corrupt frames; seed corpus in `tests/fuzz/corpus/`.

```bash
# Linux / CI (Clang)
cmake -S . -B build_fuzz -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DNAVICORE_BUILD_FUZZERS=ON
cmake --build build_fuzz --target navicore_sensor_wire_fuzz
./build_fuzz/navicore_sensor_wire_fuzz tests/fuzz/corpus -max_total_time=60

# Windows / AFL-style: corpus or stdin driver (no libFuzzer runtime)
cmake -S . -B build -DNAVICORE_BUILD_FUZZERS=ON
cmake --build build --target navicore_sensor_wire_fuzz_standalone
.\build\navicore_sensor_wire_fuzz_standalone.exe tests\fuzz\corpus\nmea_valid_gga
```

CI job: `sensor-wire-fuzz` in [code-audit.yml](.github/workflows/code-audit.yml).

## Hardware fault injection (lab)

Host tests assert **policy**. On-target checklist (unplug IMU, UART flood, UPS/I2C, hard power cut, WDT):  
[`docs/FAULT_INJECTION_LAB.md`](docs/FAULT_INJECTION_LAB.md). Pico now treats IMU silence â‰¥ `PICO2_IMU_SILENCE_DEGRADE_MS` as `imu_degraded`.

Before multi-cycle brownout/WDT: confirm the lab build is **not** writing flash/config on every boot (flash endurance). Physical campaign closeout = pass/fail table in Evidence ([`EVIDENCE_CLOSEOUT.md`](docs/EVIDENCE_CLOSEOUT.md)).

### Code audit (A5) â€” baseline + CI

| Item | Result |
|------|--------|
| Standard | [docs/SAFETY_CODING_STANDARD.md](docs/SAFETY_CODING_STANDARD.md) (MISRA-inspired, not certified) |
| cppcheck `--enable=all` on `src/core` | Local baseline + **CI job** â€” [REPORT_LATEST](docs/benchmarks/static_analysis/REPORT_LATEST.md) |
| gcov (`navicore_regression_test`) | **~50%** line cover on linked ESKF TUs (preâ€“inject expansion); re-run after `--safety-inject` links guards |
| clang-tidy | `.clang-tidy` + **CI job** (Ubuntu/Clang) |
| ASan + UBSan | **CI job** builds with Clang `-fsanitize=address,undefined` and runs `--safety-inject` |
| Catch2 + RapidCheck | **CI job** `navicore_unit_tests` â€” units + properties + wire/health |
| libFuzzer (NMEA/UBX/WT61C) | **CI job** `sensor-wire-fuzz` â€” 60 s smoke on corpus |
| Workflow | [`.github/workflows/code-audit.yml`](.github/workflows/code-audit.yml) |
| Runner (local) | `python tools/ci/run_static_analysis.py --cppcheck` Â· `--clang-tidy` Â· `--coverage-build` |

### Still missing for â€œdemo-readyâ€ credibility

**Closeout rule:** Allan fit, Adalogger field outage, and physical fault-injection each end with a **README Evidence** table â€” not only a CSV folder. See [`docs/EVIDENCE_CLOSEOUT.md`](docs/EVIDENCE_CLOSEOUT.md).

| Gap | Why it matters | Done when |
|-----|----------------|-----------|
| PPK2 current on **Adalogger** DUT | â€œUltra-low powerâ€ stays architectural until measured | mA/mW table in README Power |
| Forced field outage (**Adalogger** + truth GPX) | Coast curve vs time on hardware | CSV **+** Evidence drift table ([checklist](docs/benchmarks/field_outage/CHECKLIST.md)) |
| Allan **fit** table from hours of static IMU | Tool ships; paste IEEE ARW/BI after `imu_static_log.csv` | PNG **+** Evidence Allan row ([runbook](docs/allan/RUNBOOK.md)) |
| Logged lab fault-injection campaign | Host smoke done; physical bank pending | Pass/fail **+** Evidence table ([lab](docs/FAULT_INJECTION_LAB.md); care: flash on brownout cycles) |
| Field + PPK2 artefacts published | Coast curve + mA/mW still empty â€” needed before â€œgoing viralâ€ | Both visible in README |

**Do not break the sequence:** GAP-3 video (done) → Allan on **Adalogger** → outage Adalogger → PPK2 → then external talk / Artemis–Ambiq silicon.
**Do not wait for Artemis to start Allan or outage** — both target the **active Adalogger desk DUT** (BSP port still planned; archived Pico2 is reference only). They need the **physical bench powered**, not the Artemis kit. PPK2 needs the Nordic instrument (independent buy), measured on **Adalogger** first ([roadmap](docs/ROADMAP_PNT_RESILIENCE.md#orden-operativo-recomendado)).


### Diagnostic campaign (real-run)

| Phase | Status |
|-------|--------|
| GAP-1 â€¦ GAP-4, G-ext | **CLOSED** |
| GAP-5 v1 (adaptive NHC) | **CLOSED** â€” passive path inactive under preregistered ops |
| GAP-5 v2 | Preregistered / paused |
| Attitude snapshot (static 0â€“2 s) | EKFâ†”Orientation **0.05Â°**, gravity **0.09Â°** |

Full map: [EKF diagnostics](#ekf-diagnostics-real-run).

---

## NavMode degradation matrix

Integrator-facing contract (transitions + honest precision envelopes):  
**[`docs/NAV_MODE_DEGRADATION.md`](docs/NAV_MODE_DEGRADATION.md)**

| Mode | When (EKF / DUT) | Trust sketch |
|------|-------------------|--------------|
| `HYBRID` | Fix valid **and** GNSS accept â‰¤ 2 s | Highest â€” INS+GNSS; quality `0.55+0.03Ã—sats` âˆˆ [0.55, 0.95] |
| `GPS` | Fix valid, accept **stale** (> 2 s) | Weak GNSS â€” quality 0.65; do not treat as fresh hybrid |
| `DEAD_RECKONING` | No fix **or** GNSS outlier | Coast; quality from fix age (or 0.25 on outlier). Lab: ~13 m @ 30 s synthetic; phone v2 coast tensâ€“~110 m |
| `INITIALIZING` | Not seeded | Do not steer |

Overlays (halve quality, may not change mode): IMU silence, UART overflow, **IMU vigilante cross-check**, GNSS degrade.

Tests: `navicore_unit_tests "[navmode],[imu_cross]"`.

### Watchdogs (on-chip + external)

| Layer | What | Independence |
|-------|------|----------------|
| RP2350 `hardware/watchdog` | On-chip HW WDT, 50 ms, kick **end-of-loop only** | Same silicon as firmware â€” not a software timer, but **not** die-independent |
| External supervisor (TPL5010 / MAX6822) | Optional pulse on **GP15** (`PICO2_EXT_WDT_ENABLE`) | **Independent** of the MCU die â€” use this for true hang survival |

I2C UPS recovery **no longer** pets the WDT (a stuck recovery must trip).

### Dual-IMU vigilante

Optional MPU-6050 on I2C0 (GP8/9): compares with WT61C; on disagreement sets `imu_cross_fail` and halves quality â€” **does not** replace the primary in the EKF. Enable `PICO2_SECONDARY_IMU_ENABLE`.

---

## Fusion algorithm â€” what it is / what it is not

This is the section a serious reviewer should read first.  
**Not** a vague â€œdead reckoning + sensor fusionâ€ blend: a **15-state error-state Kalman filter (ESKF)** with an explicit state, explicit innovations, and **documented Q / R / Pâ‚€** (source of truth: [`src/core/ins_ekf.hpp`](src/core/ins_ekf.hpp) + `ins_ekf_init` / `ins_ekf_build_process_noise`).

**Sensor fusion status:** IMU + GNSS actively fused in the 15-state EKF.
Magnetometer and barometer are scaffolded ([`sensor_types.hpp`](src/core/sensor_types.hpp)) but not yet
consumed as measurement updates. NHC (non-holonomic constraints) exists as
an optional correction, gated OFF by default in production per the GAP-3
finding â€” see [`docs/NHC_OPS_POLICY.md`](docs/NHC_OPS_POLICY.md).

### What it is

| Item | Fact |
|------|------|
| **Filter class** | **Error-state Kalman filter (ESKF / MEKF-style)** â€” Joseph covariance update. Not complementary filter, not Î±-Î², not â€œGPS glueâ€. |
| **Nominal** | `p_NED`, `v_NED`, `q` (bodyâ†’NED), `b_a`, `b_g` |
| **Error state** | 15Ã—1 Î´x (table below); mean reset after each inject |
| **Predict @ ~100 Hz** | `Ï‰_corr = Ï‰ âˆ’ b_g` â†’ quat; `a_lin = RÂ·(f âˆ’ b_a) âˆ’ g`; integrate `v`,`p`; sparse Î¦; add Q |
| **Updates** | GNSS (pos / pos+vel / vel); optional **NHC**, **ZUPT** â€” policy-armed |
| **Platform** | Float-only, **zero heap** in `src/core/` |
| **Cores** | v1: `ins_ekf.cpp`. v2: `--ekf-core v2` (`ins_ekf_v2.hpp`) â€” same 15-state container, different predict/GNSS/NHC policy |

Body frame: [`docs/diagnostics/08-body-frame-contract.md`](docs/diagnostics/08-body-frame-contract.md).

### Error-state vector (15)

| Index | Symbol | Meaning | Unit |
|------:|--------|---------|------|
| 0â€“2 | Î´p_N, Î´p_E, Î´p_D | Position error | m |
| 3â€“5 | Î´v_N, Î´v_E, Î´v_D | Velocity error | m/s |
| 6â€“8 | Î´Î¸_x, Î´Î¸_y, Î´Î¸_z | Attitude error (small-angle) | rad |
| 9â€“11 | Î´b_ax, Î´b_ay, Î´b_az | Accel bias error | m/sÂ² |
| 12â€“14 | Î´b_gx, Î´b_gy, Î´b_gz | Gyro bias error | rad/s |

Enum: `InsEkfErrorIdx` in [`ins_ekf.hpp`](src/core/ins_ekf.hpp).

### Innovation sources (what feeds the filter)

| Source | In EKF today? | Observation / notes |
|--------|---------------|---------------------|
| **IMU accel + gyro** | Yes (predict) | Specific force & angular rate @ tick; mount matrix in replay |
| **GNSS position** LLAâ†’NED | Yes | `y = z_NED âˆ’ pÌ‚`; `R_pos` diagonal |
| **GNSS horizontal velocity** | Yes (optional) | From speed Ã— course; `pos_vel` / `vel_only` |
| **NHC** | Optional | Pseudo-meas: body lateral/vertical velocity â‰ˆ 0 |
| **ZUPT** | Optional | Pseudo-meas: velocity â‰ˆ 0 when stationary policy fires |
| **Barometer / depth** | **No** (not in ESKF update) | Domain fields exist on NavState API; not an EKF innovation yet |
| **Magnetometer** | **No** | Not used as attitude update in this core |
| **Vision / wheel odometry** | **No** | Out of scope of current core |

Honesty for auditors: multi-domain *NavState* is the shared API; **domain-specific aiding** (mag, baro, DVL, airspeed) is **not** claimed as already fused.

### Tuning â€” process noise Q, measurement R, initial Pâ‚€

An EKF lives or dies on Q and R. Defaults below are the **compile-time / init values** shipped in the core (overrideable at runtime for GNSS vel std, NHC Ïƒ, scales in replay). Field campaigns should **re-estimate** these from logged residuals â€” that dataset is more convincing than any synthetic scenario.

#### Process noise Q (diagonal blocks per `ins_ekf_build_process_noise`)

Built each predict from PSD-style scalars Ã— `dt`:

| Block | Construction | Default scalar |
|-------|--------------|----------------|
| Q_pos | `Ïƒ_aÂ² Â· dtÂ²` | `Ïƒ_a = 0.05 m/sÂ²` (`NAVICORE_INS_EKF_ACCEL_NOISE_STD_MPS2`) |
| Q_vel | `Ïƒ_aÂ² Â· dt` | same Ïƒ_a |
| Q_att | `Ïƒ_gÂ² Â· dt` | `Ïƒ_g = 0.002 rad/s` (`NAVICORE_INS_EKF_GYRO_NOISE_STD_RADPS`) |
| Q_ba | `PSD_ba Â· dt` | `1e-6` (`NAVICORE_INS_EKF_BIAS_ACCEL_RW_VAR`) |
| Q_bg | `PSD_bg Â· dt` | `1e-10` (`NAVICORE_INS_EKF_BIAS_GYRO_RW_VAR`) |

Comments in header: Ïƒ_a / Ïƒ_g aligned with **real car-log / mount vibration** order of magnitude, not optimistic lab IMU.

#### Measurement noise R

| Measurement | Default | Macro / field |
|-------------|---------|---------------|
| GNSS position (per axis) | **6.0 mÂ²** (Ïƒ â‰ˆ 2.45 m) | `NAVICORE_INS_EKF_GNSS_POS_VAR_M2` â†’ `gnss_pos_var_m2` |
| GNSS horizontal velocity | **Ïƒ = 1.5 m/s** â†’ 2.25 mÂ²/sÂ² | `NAVICORE_INS_EKF_GNSS_VEL_STD_MPS` |
| NHC lateral / vertical | Ïƒ = 0.5 / 1.0 m/s | `NAVICORE_INS_EKF_NHC_*_STD_MPS` |
| ZUPT velocity | Ïƒ = 0.05 m/s | `NAVICORE_INS_EKF_ZUPT_VEL_STD_MPS` |

Replay can raise GNSS Ïƒ from phone accuracy columns (floored) and override `--gnss-vel-std-mps`, `--nhc-sigma`.

#### NIS gates (Ï‡Â² @ 99%)

| Mode | DoF | Threshold |
|------|-----|-----------|
| Position only | 3 | 11.345 |
| Velocity only | 2 | 9.210 |
| Pos+vel (v1 joint) | 5 | 15.086 |

#### Initial covariance Pâ‚€ (`ins_ekf_init` diagonals)

| State | Pâ‚€ diag |
|-------|---------|
| pos N,E / D | 4 / 4 / 9 mÂ² |
| vel N,E,D | 1 mÂ²/sÂ² |
| att roll,pitch / yaw | (5Â°)Â² / (10Â°)Â² |
| accel bias | 0.25 (m/sÂ²)Â² |
| gyro bias | 0.01 (rad/s)Â² |

### What it is not (do not over-claim)

| Claim to avoid | Reality |
|----------------|---------|
| â€œCompetes with u-blox / full ArduPilot stackâ€ | Different product class. Niche: **lightweight, auditable, zero-heap ESKF core** (GPL-3.0-or-later or commercial) between heavy autopilot stacks and sealed commercial modules â€” **potential**, not traction yet. |
| â€œOne untuned filter owns land/air/sea opsâ€ | Shared **state + predict**; each domain still needs its own aiding/R. Multidomain is **architecture**, not the headline â€” see [Positioning](#positioning--gps-denied--pnt-resilience). |
| â€œAssured PNT / anti-jam productâ€ | Civil **integrity + DR** story only. No RF anti-jam; spoof gate = **EKF consistency check** (`reject_reason=3`, SW injection only), not mil stack. No Honeywell comparison. |
| â€œProven automotive / maritime DRâ€ | **Not yet.** Phone-log replay â‰  certified outage campaign on the Adalogger DUT. |
| â€œBaro/mag already in the EKFâ€ | **Not in the update.** See innovation table. |
| â€œv2 is a smaller toy filterâ€ | Same 15-state ESKF; different fusion **policy**. |

### v1 vs v2 fusion policy

| | **v1** (`--ekf-core v1`) | **v2** (`--ekf-core v2`) |
|--|--------------------------|--------------------------|
| GNSS `pos_vel` | Joint **5-DoF** NIS; bad velocity can reject **whole** fix | Pos / vel **separate**; position keeps anchoring |
| Predict | Semi-implicit `p`; NHC may run inside predict | Trapezoidal `p`; term budget; no NHC in predict |
| NHC | Stride when enabled | GNSS-gap triggered (doc 17) |
| Status | Historical control | Candidate â€” A/B 3 routes [`ekf_v2_ab_3routes`](docs/benchmarks/ekf_v2_ab_3routes/) |

### Where vibration and multipath still hurt

Phone IMU + phone GNSS stay in the loop on the published city drives. Between fixes, attitude / `RÂ·f` errors still write Î”v. Expect **tensâ€“hundreds of metres** residual with current Q/R â€” not rail-grade â€” until **field outage residuals** are used to retune Q/R and (later) domain aiding.

**Next evidence that raises confidence more than any feature:** forced GNSS outage on the **Adalogger** DUT (or phone replay), with residual growth vs time and the Q/R used frozen in the report.

---

## What validates the real firmware â€” Adalogger DUT

The **active desk DUT** is the **Adafruit Feather RP2040 Adalogger** stack (BNO055 AMG + Adafruit GPS + PPK2), not the archived Pico 2 W Comarruga bank. Same C++ ESKF in `src/core/`; new BSP under `src/targets/rp2040_adalogger/` (planned). **Do not** label Evidence as Pico-validated if the hardware is Adalogger.

Reference-only former target: `src/targets/archive/pico2_hardware/` Â· [`docs/archive_comarruga_lab_hardware.md`](docs/archive_comarruga_lab_hardware.md).

Published fusion Evidence to date is still largely from **mobile SensorLogger** drives (EKF v2 A/B), not from a powered Adalogger campaign.

| Role | Platform | Validates? |
|------|----------|------------|
| **DUT / instrument (goal)** | **Adalogger** + USB CDC / microSD | **When ported + powered** â€” real ESKF binary on the desk kit |
| Host replay | `NaviCore3D_Replay` on PC | Deterministic regression on logged CSVs â€” same algorithms, **not** a substitute for embedded timing/I/O |
| Mobile SensorLogger traces | Phone IMU+GNSS CSVs | **Current** published fusion A/B evidence (see Evidence scorecard) |
| Optional external truth | e.g. phone / survey GNSS logging in parallel | Independent **reference track** â€” **not** a second NaviCore |
| Archived Pico 2 W tree | `src/targets/archive/pico2_hardware/` | Reference BSP patterns only â€” **not** the active Evidence DUT |

### Why not â€œrun NaviCore on a Pi Zero insteadâ€

A Python (or other) EKF on a Pi Zero measures **that** stack. It does **not** measure the zero-heap C++ core on the Adalogger RP2040, the real IMU/GNSS path, or on-device logging under the nav loop.

Use a second device only if you need a **ground-truth logger** riding along â€” then compare truth vs Adalogger stream.

### Correct next measurement (dead reckoning)

1. Bring up Adalogger firmware (`rp2040_adalogger` port) with BNO055 **AMG** + Adafruit GPS.
2. PC / microSD captures NavState while GNSS is **denied or blocked** for a known interval.
3. Score residual growth vs time â€” optionally vs a parallel truth logger.
4. Freeze Q/R used in the report.

Replay of phone CSVs remains valuable for algorithm A/B (v1/v2); **embedded DR proof** requires the Adalogger path above.

---

## Power â€” measure before more hardware (PPK2)

The project tagline includes **ultra-low power / edge MCU**. That claim is currently **architectural** (zero-heap hot path, RP2040 target, non-blocking USB) â€” **not** yet backed by a published current draw from a Nordic **Power Profiler Kit II (PPK2)** or equivalent.

**Priority:** power the **Adalogger kit** you already ordered and measure it with **PPK2**. Allan fit and field outage use **that same DUT**.

| Do first (Adalogger DUT) | Do later |
|--------------------------|----------|
| Power Adalogger + BNO055 AMG + GPS; Allan static â†’ README | Artemis / Ambiq ladder |
| Field outage + truth GPX â†’ README | Claim ULP without mA |
| PPK2 on Adalogger + document mA / mW here | Revive archived Pico2 as product story |

### Measurement protocol (lab)

1. **Instrument:** Nordic PPK2 (source or ampere meter mode as appropriate for the board supply).
2. **DUT:** Adalogger with firmware under test (record git commit / build id).
3. **Supply:** Measure **board + MCU path**; note USB CDC vs battery, GPS+IMU on.
4. **Profiles** (run â‰¥30â€“60 s steady each; report mean and peak):

| ID | Profile | Expected intent |
|----|---------|-----------------|
| P0 | MCU idle / clocks only (if available) | Floor |
| P1 | Nav loop, USB CDC on, sensors as wired | Typical lab estimate |
| P2 | P1 + microSD logging | Upper bound with logger |
| P3 | Deep sleep / safe shutdown path (if exercised) | Conservation claim |

5. **Publish:** fill the table below â€” do **not** invent placeholders as facts. Until filled, treat â€œultra-low powerâ€ as **design goal**, not measured evidence.

### Measured results (fill after PPK2)

| Profile | V_supply [V] | I_mean [mA] | I_peak [mA] | P_mean [mW] | Build / commit | Date | Notes |
|---------|-------------:|------------:|------------:|------------:|----------------|------|-------|
| P0 | â€” | **TBD** | TBD | TBD | | | |
| P1 | â€” | **TBD** | TBD | TBD | | | Adalogger + sensors |
| P2 | â€” | **TBD** | TBD | TBD | | | |
| P3 | â€” | **TBD** | TBD | TBD | | | |

Artefacts (CSV/PNG from nRF Connect / PPK2 export) belong under `docs/benchmarks/power_ppk2/` when available.

---

## Repository layout / Estructura

```
NaviCore-3D/
â”œâ”€â”€ src/
â”‚   â”œâ”€â”€ core/                      # Motor universal (EKF, geodesia, fusion, guards)
â”‚   â”œâ”€â”€ scenarios/                 # TUNNEL_STRESS, SLALOM
â”‚   â””â”€â”€ targets/
â”‚       â”œâ”€â”€ generic_pc/            # Sim, VehicleDemo, Replay, benchmarks
â”‚       â””â”€â”€ archive/pico2_hardware/ # ARCHIVED Comarruga Pico 2 W (reference only)
â”‚       â””â”€â”€ rp2040_adalogger/      # PLANNED active DUT (dir not in tree yet - see port doc)
â”œâ”€â”€ data/
â”‚   â””â”€â”€ real_run/                  # CSVs Android Sensor Logger (~332 s)
â”œâ”€â”€ calibration/
â”‚   â””â”€â”€ imu_mount.json             # Matriz sensorâ†’body FRD (Rodrigues)
â”œâ”€â”€ tests/unit/                    # Catch2 + RapidCheck (kernels aislados)
â”œâ”€â”€ .github/workflows/             # CI code-audit (cppcheck Â· tidy Â· ASan Â· units)
â”œâ”€â”€ docs/
â”‚   â”œâ”€â”€ diagnostics/               # MetodologÃ­a H0â€“H9d + GAP-1â€¦5
â”‚   â”œâ”€â”€ benchmarks/                # Evidence packs (EKF v2 A/B, static analysis, â€¦)
â”‚   â”œâ”€â”€ ROADMAP_PNT_RESILIENCE.md  # Tracks cÃ³digo / hardware / visibilidad
â”‚   â”œâ”€â”€ SAFETY_CODING_STANDARD.md  # GuÃ­a MISRA-inspired (no certificaciÃ³n)
â”‚   â”œâ”€â”€ monte_carlo/               # Trazas Monte Carlo
â”‚   â”œâ”€â”€ nhc_experiments/           # Experimentos NHC del sim
â”‚   â””â”€â”€ telemetria_navicore.csv    # Black-box del simulador
â”œâ”€â”€ tools/                         # Python tooling (categorized â€” see tools/README.md)
â”‚   â”œâ”€â”€ lib/                       # Shared modules (geodesy, codecs, path bootstrap)
â”‚   â”œâ”€â”€ analysis/                  # Allan, parse_mobile_log, plots
â”‚   â”œâ”€â”€ benchmarks/                # run_all_benchmarks, Monte Carlo
â”‚   â”œâ”€â”€ experiments/               # H-series / NHC experiment runners
â”‚   â”œâ”€â”€ audits/                    # GAP / autopsy one-offs
â”‚   â”œâ”€â”€ campaigns/                 # run_gap*, run_ekf*, slalom campaigns
â”‚   â”œâ”€â”€ sil/                       # Visualizers + UDP/SIL tests
â”‚   â”œâ”€â”€ field/                     # Serial NavState capture
â”‚   â”œâ”€â”€ ci/                        # Static analysis + regression (GitHub Actions)
â”‚   â”œâ”€â”€ media/                     # Video / EKF Explorer helpers
â”‚   â””â”€â”€ reports/
â”œâ”€â”€ CMakeLists.txt                 # Build PC (+ optional unit tests)
â”œâ”€â”€ DEVELOPMENT.md
â”œâ”€â”€ README.md
â””â”€â”€ build/                         # Salida CMake local (no versionar)
```

| Path | Role |
|------|------|
| `src/core/` | INS/EKF 15-state (ESKF), NavState, geodesy WGS84, guards â€” see Fusion algorithm |
| `src/scenarios/` | Escenarios cuantitativos (`tunnel_stress`, `slalom_scenario`) |
| `src/targets/generic_pc/` | Host: sim, replay, UDP telemetry, adaptive NHC controller |
| `src/targets/archive/pico2_hardware/` | **Archived** Comarruga Pico 2 W BSP (reference for Adalogger port) |
| `src/targets/rp2040_adalogger/` | **Planned** active DUT (directory not scaffolded yet) â€” see port doc |
| `tests/unit/` | Catch2 + RapidCheck formal units/properties |
| `data/real_run/` | Phone SensorLogger logs â€” **local only** (not in git; see `data/real_run/README.md`) |
| `docs/benchmarks/` | Evidence packs: keep `*.md` / `*.json` / `*.png` in git; **bulk CSV is gitignored** (regenerable locally â€” see `.gitignore`) |
| `docs/diagnostics/` | DocumentaciÃ³n cientÃ­fica del pipeline EKF |
| `tools/` | Categorized Python tooling â€” map in [`tools/README.md`](tools/README.md) |
| `calibration/` | CalibraciÃ³n de montaje IMU |

Unity EKF Explorer (`ekf_explorer/`) is **local-only** (Cesium/Unity binaries â‰« GitHub limits). Protocol: [`docs/diagnostics/19-ekf-explorer-protocol.md`](docs/diagnostics/19-ekf-explorer-protocol.md).

Contexto de desarrollo y prioridades RT: [`DEVELOPMENT.md`](DEVELOPMENT.md).

---

## Architecture / Arquitectura

```mermaid
flowchart LR
    subgraph Host["PC host"]
        SIM[NaviCore3D_Sim]
        REPLAY[NaviCore3D_Replay]
        PY[Python audits / visualizers]
    end
    subgraph Core["src/core â€” zero heap"]
        EKF[INS/EKF 15-state]
        GEO[Geodesy WGS84]
        NS[NavState]
        GUARDS[Safety guards]
    end
    subgraph Data["Data"]
        RR[data/real_run]
        CAL[calibration/imu_mount.json]
        CSV[docs/benchmarks]
    end
    subgraph Embedded["Adalogger DUT (active)"]
        PICO[rp2040_adalogger]
        IMU[BNO055 AMG]
        GNSS[Adafruit GPS]
    end

    RR --> PY
    PY -->|real_run_replay.csv| REPLAY
    CAL --> REPLAY
    REPLAY --> EKF
    SIM --> EKF
    EKF --> NS
    GEO --> EKF
    GUARDS --> NS
    REPLAY --> CSV
    SIM --> CSV
    IMU --> PICO
    GNSS --> PICO
    PICO --> EKF
```

**NavState** is the navigation **product faÃ§ade** (LLA, heading, GPS/HYBRID/DR). Underneath, the reusable engine speaks estimate quality, reject codes, and AIDED/COAST â€” see [`docs/ESTIMATE_ENGINE_VS_NAV_VOCAB.md`](docs/ESTIMATE_ENGINE_VS_NAV_VOCAB.md).

**INS/EKF** â€” see [Fusion algorithm](#fusion-algorithm--what-it-is--what-it-is-not): real **15-state ESKF** (predict + Joseph GNSS/NHC/ZUPT), not a simplified complementary blend. Replay selects `--ekf-core v1|v2`.

Body-frame contract (normative): [`docs/diagnostics/08-body-frame-contract.md`](docs/diagnostics/08-body-frame-contract.md).

---

## Build / Compilar

### PC targets (root `CMakeLists.txt`)

```powershell
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

| Target | Binary | Description |
|--------|--------|-------------|
| `NaviCore3D_Sim` | `build\NaviCore3D_Sim.exe` | Stress simulator + CSV/UDP telemetry |
| `NaviCore3D_VehicleDemo` | `build\NaviCore3D_VehicleDemo.exe` | CAN vehicle-bus demo + HMI |
| `NaviCore3D_Replay` | `build\NaviCore3D_Replay.exe` | Real-run EKF replay + audit hooks |
| `navicore_regression_test` | `build\navicore_regression_test.exe` | C++ regression / `--safety-inject` |
| `navicore_unit_tests` | `build\navicore_unit_tests.exe` | Catch2 + RapidCheck (NavState / math / fusion / wire / health) |
| `navicore_sensor_wire_fuzz(_standalone)` | optional `-DNAVICORE_BUILD_FUZZERS=ON` | libFuzzer or corpus/stdin driver |
| `ring_stress_test` | `build\ring_stress_test.exe` | Host SPSC UART stress (S7 campaign) |

Build a single target:

```powershell
cmake --build build --target NaviCore3D_Replay
cmake --build build --target NaviCore3D_Sim
```

On Windows, the sim links `ws2_32` for UDP telemetry.

### Audit builds (sanitizers / coverage)

```powershell
# Coverage (gcov)
cmake -S . -B build_coverage -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Debug -DNAVICORE_ENABLE_COVERAGE=ON
cmake --build build_coverage --target navicore_regression_test
.\build_coverage\navicore_regression_test.exe --safety-inject

# ASan+UBSan â€” prefer Clang/Linux (MinGW often lacks libasan)
cmake -S . -B build_asan -G Ninja -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug -DNAVICORE_ENABLE_SANITIZERS=ON
cmake --build build_asan --target navicore_regression_test
./build_asan/navicore_regression_test --safety-inject
```

CI: [`.github/workflows/code-audit.yml`](.github/workflows/code-audit.yml) (cppcheck Â· clang-tidy Â· ASan Â· Catch2+RapidCheck).  
Report: [`docs/benchmarks/static_analysis/REPORT_LATEST.md`](docs/benchmarks/static_analysis/REPORT_LATEST.md) Â· standard: [`docs/SAFETY_CODING_STANDARD.md`](docs/SAFETY_CODING_STANDARD.md).

### Pico 2 W â€” ARCHIVED (reference only)

**Active DUT:** Adalogger kit ([port plan](docs/TARGET_RP2040_ADALOGGER_PORT.md)).

Former Comarruga firmware lives under `src/targets/archive/pico2_hardware/` Â· notes: [`docs/archive_comarruga_lab_hardware.md`](docs/archive_comarruga_lab_hardware.md).

```powershell
# Reference build only â€” not the product Evidence path
$env:PICO_SDK_PATH = 'C:\path\to\pico-sdk'
cmake -S src\targets\archive\pico2_hardware -B build_pico2 -G Ninja
cmake --build build_pico2
```

---

## Run simulator / Ejecutar simulador

```powershell
.\build\NaviCore3D_Sim.exe
```

| Flag | Effect |
|------|--------|
| `--no-udp` | Disable UDP telemetry (CI / headless) |
| `--run-tests` | Run embedded regression suite |
| `--scenario SLALOM` | Slalom tracking scenario |
| `--scenario TUNNEL_STRESS` | Multi-phase tunnel profile |
| `--super-tunnel` | NHC vs no-NHC comparison |
| `--nhc-experiments` | Super-tunnel NHC experiment matrix |
| `--stress` / `--high-stress` | WCET / radio-burst stress |
| `--clean` | Clean mission (no fault injection) |
| `--seed N` | RNG seed (default: system clock) |
| `--csv-out <path>` | Override telemetry CSV path |
| `--replay <csv>` | SiL replay from CSV |

Default black-box export: `docs/telemetria_navicore.csv`.

Vehicle bus demo:

```powershell
.\build\NaviCore3D_VehicleDemo.exe
```

Quantitative sim benchmarks:

```powershell
python tools/benchmarks/run_all_benchmarks.py
```

Live / offline visualization:

```powershell
# Offline 3D CSV replay
python tools\visualizer.py

# Live UDP (run Sim in another terminal without --no-udp)
python tools\remote_visualizer.py
```

---

## Real-run replay pipeline

End-to-end flow for Android Sensor Logger captures:

```
data/real_run/*.csv
    â†’ parse_mobile_log.py
    â†’ docs/benchmarks/real_run_replay.csv
    â†’ NaviCore3D_Replay.exe (+ calibration/imu_mount.json)
    â†’ docs/benchmarks/*_output.csv / *_report.json / *.png
    â†’ tools/audit_gap*.py  (scientific auditors)
```

### Input data (`data/real_run/`)

| File | Use |
|------|-----|
| `AccelerometerUncalibrated.csv` | **Primary IMU accel input** for replay |
| `Gyroscope.csv` | Gyro |
| `Location.csv` | GNSS lat/lon/alt, speed, bearing |
| `Orientation.csv` | Android attitude (external reference, not GT) |
| `Gravity.csv` | Android gravity estimate |
| `TotalAcceleration.csv` | Total acceleration |
| `Metadata.csv` | Recording metadata |
| `Accelerometer.csv` | Identity / audit only â€” **do not** feed as replay IMU |
| `Annotation.csv` | Optional annotations |

Short legacy capture archived under `docs/_archive_short_run_jul15/`.

### Prepare replay CSV

```powershell
python tools/analysis/parse_mobile_log.py `
  --input-dir data\real_run `
  --output docs\benchmarks\real_run_replay.csv
```

### Example: H9 predict-only (60 s)

```powershell
.\build\NaviCore3D_Replay.exe `
  --input docs\benchmarks\real_run_replay.csv `
  --output docs\benchmarks\h9_predict_only_output.csv `
  --constraint-policy disabled `
  --predict-only --predict-only-end-s 60 `
  --h9a-gravity-tilt-init `
  --mount-calibration calibration\imu_mount.json `
  --predict-audit-csv docs\benchmarks\h9_predict_only_audit.csv
```

### Example: scientific baseline (constraints explicit)

```powershell
.\build\NaviCore3D_Replay.exe `
  --input docs\benchmarks\real_run_replay.csv `
  --output docs\benchmarks\gap3_baseline_output.csv `
  --constraint-policy imu_stationary `
  --nhc-policy enabled `
  --nhc-every-n-ticks 1 `
  --mount-calibration calibration\imu_mount.json
```

### Important replay flags

| Flag | Values / purpose |
|------|------------------|
| `--input`, `--output` | Replay CSV paths |
| `--mount-mode` | `none`, `legacy`, `calibration` |
| `--mount-calibration` | Path to `imu_mount.json` |
| `--yaw-init` | `zero`, `gnss_stable`, `h3` |
| `--predict-only` | Disable GNSS/NHC/ZUPT updates |
| `--predict-only-end-s`, `--replay-end-s` | Time window |
| `--h9a-gravity-tilt-init` | Init roll/pitch from gravity |
| `--constraint-policy` | **Required.** `forced_time`, `gps_stop`, `imu_stationary`, `disabled` (`auto` rejected) |
| `--nhc-policy` | `enabled`, `disabled` |
| `--nhc-every-n-ticks N` | NHC decimation |
| `--gnss-obs-mode` | `pos`, `pos_vel`, `vel_only` |
| `--p-pv-policy` | `none`, `gap_le_1s`, `zero`, `cos_pos`, `cos_tot` |
| `--adaptive-nhc` | `off`, `passive`, `active` |
| `--p0-scale`, `--q-scale`, `--nhc-sigma` | Covariance / NHC tuning |
| `--gap3-*-audit-csv` | GAP-3 audit exports |
| `--help` | Full CLI |

> **Validity note (Jul 2026):** full-filter runs between H9 and GAP-3.7 used legacy ZUPT (`forced_time`: `tâ‰¤30s OR gps_speedâ‰¤0.1`). Re-run with `--constraint-policy imu_stationary` before citing those results. Predict-only (H9) is unaffected by ZUPT math, but the CLI still requires an explicit `--constraint-policy` (use `disabled`). See [`docs/diagnostics/11-replay-zupt-provenance.md`](docs/diagnostics/11-replay-zupt-provenance.md).

---

## EKF diagnostics (real-run)

Pipeline experimental sobre grabaciones reales: consistencia NEES/NIS, geodesia WGS84, sincronizaciÃ³n, propagaciÃ³n inercial, conformidad body-frame y auditorÃ­as GAP.

**Index:** [`docs/diagnostics/README.md`](docs/diagnostics/README.md)

### Aiding design (future)

Ãndice: [`docs/design/README.md`](docs/design/README.md) â€” orden **ZUPT â†’ mag/baro â†’ ultrasonido (reserva)**. ZUPT: diseÃ±o **abierto** ([ZUPT_DESIGN.md](docs/design/ZUPT_DESIGN.md)); no implementar hasta cerrar checklist de detector por dominio.

### Document map

| Doc | Content |
|-----|---------|
| [01-overview](docs/diagnostics/01-overview.md) | Methodology, H0â†’H9d chain |
| [02-data-and-frames](docs/diagnostics/02-data-and-frames.md) | Data sources, frame chain |
| [03-experiments](docs/diagnostics/03-experiments.md) | Experiment catalog + scripts |
| [04-findings](docs/diagnostics/04-findings.md) | Consolidated results |
| [05-attitude-investigation](docs/diagnostics/05-attitude-investigation.md) | H9 attitude block |
| [06-reproduction](docs/diagnostics/06-reproduction.md) | **Build + reproduce all audits** |
| [07-signal-traceability](docs/diagnostics/07-signal-traceability.md) | Android â†’ mount â†’ EKF |
| [08-body-frame-contract](docs/diagnostics/08-body-frame-contract.md) | Formal body FRD contract |
| [09-predict-conformance-audit](docs/diagnostics/09-predict-conformance-audit.md) | predict() conformance + GAP-1/2 |
| [10-gap3-ins-model-audit](docs/diagnostics/10-gap3-ins-model-audit.md) | GAP-3 detailed audit |
| [11-replay-zupt-provenance](docs/diagnostics/11-replay-zupt-provenance.md) | Legacy ZUPT validity warning |
| [12-gap3-synthesis](docs/diagnostics/12-gap3-synthesis.md) | **GAP-3 closed synthesis** |
| [13-gap4-gnss-velocity-protocol](docs/diagnostics/13-gap4-gnss-velocity-protocol.md) | **GAP-4 diagnostic closed** |
| [14-adaptive-nhc-protocol](docs/diagnostics/14-adaptive-nhc-protocol.md) | GAP-5 v1 preregistration |
| [15-gap5-passive-outcome](docs/diagnostics/15-gap5-passive-outcome.md) | GAP-5 v1 passive outcome |
| [16-gap5-v2-observable-selection](docs/diagnostics/16-gap5-v2-observable-selection.md) | **GAP-5 v2** (preregistrada; pausa post G-ext) |
| [reference/CURRENT_STATE_OF_THE_RESEARCH](docs/diagnostics/reference/CURRENT_STATE_OF_THE_RESEARCH.md) | **Estado de la investigaciÃ³n** (artÃ­culo interno) |
| [reference/RESEARCH_STATUS](docs/diagnostics/reference/RESEARCH_STATUS.md) | Pausa / cadena consolidado vs abierto |

Frozen reference set: [`docs/diagnostics/reference/`](docs/diagnostics/reference/)  
(`STATE_OF_KNOWLEDGE.md`, `OPEN_QUESTIONS.md`, `DECISION_LOG.md`, `RESEARCH_MAP.md`).

### Research status (Jul 2026)

| Phase | Question | Status |
|-------|----------|--------|
| **GAP-1** | Body FRD mount vs vehicle | **CLOSED** |
| **GAP-2** | Measured â†’ NED dynamics | **CLOSED** |
| **GAP-3** | INS model autopsy (NHC / P / K / gate) | **CLOSED** |
| **GAP-4** | GNSS velocity / P_pv diagnostic | **CLOSED** (`gap4-diagnostic-complete`) |
| **GAP-5 v1** | Adaptive NHC via Î“Ì„ | **CLOSED** â€” passive controller inactive under preregistered operationalization |
| **G-ext** | External lockout validation (19082026) | **CLOSED** â€” K14/K15; see `INTERPRETATION.md` |
| **GAP-5 v2** | Observable / regime selection | **PREREGISTERED** â€” pause before benchmark ([RESEARCH_STATUS](docs/diagnostics/reference/RESEARCH_STATUS.md)) |

### Snapshot findings (attitude)

| Regime | EKF â†” Orientation | EKF â†” gravity | `a_lin,h` |
|--------|-------------------|---------------|-----------|
| Static 0â€“2 s | **0.05Â°** | **0.09Â°** | **0.016 m/sÂ²** |
| Dynamic 2â€“10 s | **~4.1Â°** | **~4.3Â°** | **0.74 m/sÂ²** |

### GAP tooling (`tools/`)

| Area | Scripts (examples) |
|------|--------------------|
| GAP-1 | `tools/audits/audit_gap1_delta_psi_constancy.py`, `tools/audits/audit_gap1_body_forward_axis.py` |
| GAP-2 | `tools/audits/audit_gap2_gravity_identity_tick.py`, `tools/audits/audit_gap2_specific_force_decomposition.py`, `tools/audits/audit_gap2_predict_chain_break.py` |
| GAP-3 | `tools/campaigns/run_gap3_constraint_matrix.py`, `tools/campaigns/run_gap3_f1_nhc_dose_response.py`, `audit_gap3_*.py` |
| GAP-4 | `run_gap4_*.py`, `audit_gap4_*.py`, `tools/media/render_gap4_diagnostic_synthesis.py` |
| GAP-5 | `tools/campaigns/run_gap5_p0_passive_validation.py`, `tools/audits/audit_gap5_passive_controller_validation.py` |
| Support | `tools/audits/audit_body_frame_conformance.py`, `tools/audits/audit_android_signal_identity.py`, `tools/experiments/audit_imu_chain.py` |

Artifacts land in `docs/benchmarks/` (and subfolders `gap3_*`, `gap4_gnss_velocity/`, `gap5_adaptive_nhc/`, `constraint_matrix/`, â€¦).

---

## Calibration / CalibraciÃ³n

### Mount (vehicle FRD)

Primary mount calibration: [`calibration/imu_mount.json`](calibration/imu_mount.json)

- Method: gravity-alignment Rodrigues (`tools/experiments/audit_imu_chain.py`)
- Maps sensor frame â†’ vehicle body **FRD**
- Applied in replay via `--mount-calibration` / `--mount-mode calibration`

```powershell
python tools/experiments/audit_imu_chain.py --export-calibration calibration\imu_mount.json
```

### Allan variance (IMU noise â†’ Q)

Methodology and CLI are **shipped** â€” see [Evidence scorecard](#scientific-rigor-scorecard-what-is-already-done). Remaining work is a multi-hour static CSV, not the algorithm.

```powershell
# Needs multi-hour static IMU CSV (see analyze_allan.py header for columns)
python tools/analysis/analyze_allan.py --csv docs\imu_static_log.csv --axis gyro_z
```

Until the fit table is pasted into Evidence, process-noise defaults remain the Ïƒ_a / Ïƒ_g macros in `ins_ekf.hpp`.
### Monte Carlo

```powershell
python tools/benchmarks/run_monte_carlo.py --runs 100
# Artefacts: docs\monte_carlo\run_*.csv
```

---

## Python tooling

There is no `requirements.txt` yet; install the documented minimum:

```powershell
pip install numpy matplotlib pandas
```

| Script | Role |
|--------|------|
| `tools/analysis/analyze_allan.py` | Allan variance (IEEE 952) â†’ ARW/VRW / BI / RRW |
| `tools/benchmarks/run_monte_carlo.py` | TUNNEL_STRESS Monte Carlo â†’ `docs/monte_carlo/` |
| `tools/ci/run_static_analysis.py` | cppcheck / clang-tidy / gcov coverage runner |
| `tools/ci/run_regression_suite.py` | Orchestrates Catch2 units + `--safety-inject` |
| `.github/workflows/code-audit.yml` | CI: cppcheck Â· tidy Â· ASan Â· Catch2+RapidCheck |
| `tools/analysis/parse_mobile_log.py` | Sensor Logger folder â†’ `real_run_replay.csv` |
| `tools/experiments/audit_imu_chain.py` | IMU chain audit + mount export |
| `tools/benchmarks/run_all_benchmarks.py` | Quantitative sim benchmarks |
| `tools/sil/visualizer.py` | Offline 3D CSV replay |
| `tools/field/serial_navstate_capture.py` | USB CDC â†’ NavState CSV (Pico/Artemis; EKF on-device) |
| `tools/sil/remote_visualizer.py` | Live UDP telemetry |
| `tools/ci/run_regression_suite.py` | Regression orchestrator (`--full` includes ring stress) |
| `tools/audit_gap*.py` / `tools/run_gap*.py` | Scientific GAP campaign |
| Root `run_h*.py` | H-series experiment runners |

UDP protocol unit tests:

```powershell
python tools\test_udp_telemetry.py
```

---

## Validated stress scenarios

Both scenarios run in `NaviCore3D_Sim` at **100 ms** ticks and export every sample to the black-box CSV.

### 1 Â· GPS Loss (Air / Land)

| | |
|---|---|
| **Setup** | Cruise at **15 m/s**, heading **90Â°**, **8 satellites** with valid fix. |
| **Event** | At **t = 5 s**, satellites drop to **0** for **10 s**; GNSS updates stop. |
| **Expected** | Mode â†’ **`DEAD_RECKONING`**; `estimate_quality` degrades with `fix_age_ms`; recovery at **t = 15 s**. |
| **Result** | Quality drops **0.790 â†’ 0.295** during outage; full GNSS recovery after restore. |

### 2 Â· Submarine Immersion

| | |
|---|---|
| **Setup** | Domain **SEA**, no GNSS; hydrostatic pressure rises at **+10 000 Pa/s**. |
| **Expected** | `Pos_Z` tracks pressure in Pa; `Vel_Z` â‰ˆ **10 000 Pa/s** after first sample. |
| **Result** | `pos.z` reaches **201 325 Pa** at 10 s; `vel.z` stable at **10 000 Pa/s**. |

Additional quantitative scenarios: `SLALOM`, `TUNNEL_STRESS` (see [Monte Carlo](#monte-carlo--tunnel_stress-synthetic)), `--super-tunnel`, `--nhc-experiments` (see [NHC results](#nhc--what-the-matrix-actually-showed)).

---

## Digital Twin / Telemetry

### Live System Telemetry Mockup

RepresentaciÃ³n estÃ¡tica del frame UART / consola del simulador (`NaviCore3D_Sim`) a **100 ms** â€” valores ilustrativos del escenario de crucero nominal.

```
â•”â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•—
â•‘  NAVICORE-3D Â· LIVE INERTIAL DASHBOARD          tick=050  t=5.00s  Î”t=100ms â•‘
â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£
â•‘  MODE â–ˆâ–ˆâ–ˆâ–ˆâ–‘â–‘â–‘â–‘  HYBRID          HEALTH â–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–ˆâ–‘â–‘  NOMINAL (85)               â•‘
â•‘  POWER PERFORMANCE              SHUTDOWN â–‘ latched=0                         â•‘
â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•¦â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£
â•‘  ATTITUDE (deg)              â•‘  VELOCITY (m/s)                                â•‘
â•‘  â”Œâ”€ heading â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”   â•‘  N (Vel_X)  +15.000    E (Vel_Y)   +0.000     â•‘
â•‘  â”‚         N             â”‚   â•‘  Z (Vel_Z)   +0.000    |V|         15.00      â•‘
â•‘  â”‚    W â”€â”€â”€â—â”€â”€â”€ E  90.0Â° â”‚   â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£
â•‘  â”‚         S             â”‚   â•‘  POSITION                                     â•‘
â•‘  â””â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”˜   â•‘  Lat (X)  41.387402Â°   Lon (Y)    2.168611Â°   â•‘
â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£  Alt (Z)  12.00 m      Quality     0.850      â•‘
â•‘  IMU (body frame)            â•‘  GNSS sats  17         fix_age     120 ms     â•‘
â•‘  ax  +0.02   ay  +0.00       â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£
â•‘  az  +9.81   gx  +0.00       â•‘  GUARDS (last tick)                           â•‘
â•‘  gy  +0.00   gz  +0.00       â•‘  WCET ok   GEOM ok   DIV ok   SLIP ok         â•‘
â• â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•©â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•£
â•‘  CROSS_TRACK   +0.4 m    ALONG_TRACK  14.1 m    WP queue  3/64    BSP IDLE    â•‘
â•šâ•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•â•
```

### Black-box CSV

**File:** `docs/telemetria_navicore.csv` (rewritten each sim run)

| Column | Description |
|--------|-------------|
| `Timestamp_ms` | Simulation time [ms] |
| `Escenario` | `GPS_LOSS` or `SUBMARINE` |
| `Modo` | `GPS` Â· `DEAD_RECKONING` Â· `HYBRID` Â· `INITIALIZING` |
| `Calidad` | Confidence score 0.0 â€“ 1.0 |
| `Satelites` | Scenario satellite count |
| `Pos_X` Â· `Pos_Y` Â· `Pos_Z` | Unified 3D position (lat Â°, lon Â°, alt m or Pa) |
| `Vel_X` Â· `Vel_Y` Â· `Vel_Z` | Velocity (m/s or Pa/s) |
| `Rumbo` | Heading [Â°] |

Export uses **`fprintf`** â€” no dynamic allocations inside the simulation loop. Suitable as a reference pattern for SD-card logging on target hardware.

UDP telemetry v3 (32-byte frames) feeds `tools/sil/remote_visualizer.py` for live HIL-style visualization.

**Next step for the twin:** ingest CSV â†’ time-series store â†’ 3D scene (Cesium / Unity / Unreal) with mode/confidence colour coding.

SIL architecture notes: [`docs/sil_architecture.md`](docs/sil_architecture.md).

---

## Roadmap

Prioridades vigentes (cÃ³digo / hardware / visibilidad â€” **no** solo WCET):  
[`docs/ROADMAP_PNT_RESILIENCE.md`](docs/ROADMAP_PNT_RESILIENCE.md) Â· honest snapshot: [`docs/STATUS_ASSESSMENT.md`](docs/STATUS_ASSESSMENT.md)

| Phase | Target |
|-------|--------|
| **Done** | MC Â· NHC Â· Allan tooling Â· EKF v2 Â· estimate vocab Â· A5 Â· edge Â· NHC ops + integrity RC Â· **GAP-3 MP4 ES+EN** Â· host fault smoke Â· Allan runbook/smoke Â· field-outage checklist |
| **Now (you)** | Power **Adalogger kit** + **PPK2** â†’ Allan/outage â†’ README Â· port BSP ([plan](docs/TARGET_RP2040_ADALOGGER_PORT.md)) |
| **Hardware** | DUT = Adalogger + BNO055 AMG + Adafruit GPS Â· Pico2 **archived** (reference only) |
| **Also pending** | `rp2040_adalogger` BSP (MTK3339 PMTK + I2C AMG) Â· physical fault bank Â· WCET on-board Â· A3 domain Q/R |
| **Visibility** | **GAP-3 published** â€” [ES](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC.mp4) Â· [EN](https://github.com/Juanki58/NaviCore-3D/blob/main/docs/video_gap3/NaviCore_GAP3_NHC_en.mp4) |
| **Closeout** | [`EVIDENCE_CLOSEOUT.md`](docs/EVIDENCE_CLOSEOUT.md) â€” CSV without README does not count |

**Spoofing:** validate only via **software NMEA / trajectory injection**. Do **not** RF-spoof or jam GNSS without spectrum authorisation (illegal in ES/EU).

Pico RT detail (health monitor / WCET protocol): [`DEVELOPMENT.md`](DEVELOPMENT.md) Â§ Prioridades RT â€” subordinate to the PNT roadmap above.

---

## License & Author

**Author:** Juan Carlos Pulido Mellado  
**Copyright:** Â© 2026 Juan Carlos Pulido Mellado

**License (dual):** **GPL-3.0-or-later** *or* a **commercial** license from the copyright holder. See also [NOTICE](NOTICE).

| Path | Where | Use when |
|------|--------|----------|
| Open / copyleft | [LICENSE](LICENSE) (GPL-3.0 / GPL-3.0-or-later) | Community, research, products that can meet GPL obligations |
| Commercial | [COMMERCIAL.md](COMMERCIAL.md) Â· [LICENSE-COMMERCIAL.md](LICENSE-COMMERCIAL.md) | Proprietary embed / OEM / redistribution without GPL obligations |

Contributions require accepting the Individual CLA â€” see [CONTRIBUTING.md](CONTRIBUTING.md) and [CLA.md](CLA.md). **No CLA â†’ no merge.**

**Historical MIT:** tag `v0.1.0-mit-last` marks the last commit published under MIT. Those snapshots remain MIT as tagged; they are not revoked. Going forward, do **not** treat this repo as MIT-only.

**Contact (commercial):** `navicore.licensing@yahoo.com`

*Draft licensing docs pending counsel review â€” not legal advice.*

**NaviCore-3D** â€” *Resilience when the sky lies. Zero heap on the edge.*
