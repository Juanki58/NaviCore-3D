# Status assessment — NaviCore-3D

**Date:** 2026-07-23  
**Repo HEAD at write:** `a057e67` (update this line when revising the assessment).  
**Nature:** honest internal snapshot — not marketing.

---

## One-line verdict

Excellent **auditable lab**; product maturity **~40–50%** if 100% means “measured DUT + demonstrated power claim + an integrator can copy the bench.”

Engineering core above average for MIT inertial-fusion repos; **not** OEM-ready.

---

## What is strong (method)

- Monte Carlo with a real distribution (not a single lucky seed)
- NHC experiment matrix that **falsifies** a myth (always-on can hurt)
- Inconsistency gate with tests + RapidCheck integrity properties
- Frozen NHC ops policy (`OFF` / gap-triggered; `ALWAYS` not production-safe)
- GAP-3 explainers (ES/EN) with banked numbers
- Public correction of overclaim: “Comarruga bench validated” → fusion Evidence is **SensorLogger** mobile; Pico2 = implemented/building, powered-bench pending (`33f4739`)

That discipline is rarer — and worth more — than “yet another 15-state ESKF.”

---

## What is weak (hardware story)

Still largely a **promise**:

- Pico2 **builds**; no published powered campaign
- No Allan **fit** from multi-hour static IMU on a DUT
- No Pico field-outage curve in Evidence
- No PPK2 mA/mW table

Until those land in the README Evidence scorecard, “edge / low-power / Comarruga” means **design + PC/mobile lab**, not a characterized DUT. Coast at hundreds of metres is civil and honest — it does not compete with tactical INS or sealed u-blox modules.

---

## Development potential

**High in the right niche, low if you scatter.**

Niche: civil GNSS-degraded/denied resilience on zero-heap MCUs, MIT, falsifiable evidence — below mil assured-PNT, beside (not above) PX4/ArduPilot. Room for 2–3 years without inventing a new slogan.

### Levers that raise value (order)

1. Power a DUT (Pico2 and/or Adalogger) → Allan + outage → README  
2. Nordic **PPK2** → “ultra-low power” stops being architecture-only  
3. Clean Adalogger port (MTK3339 NMEA/PMTK + I2C AMG) without lying about the DUT — [`TARGET_RP2040_ADALOGGER_PORT.md`](TARGET_RP2040_ADALOGGER_PORT.md)  
4. Domain Q/R profiles + richer aiding policy (**without** reopening NHC always-on)  
5. Only then Artemis/Ambiq as *same core, fewer mA*

### What kills the project fastest

- New kit before numbers  
- Building the product on closed M8U/DR  
- Ambiq / ZUPT / Unity video as a substitute for Evidence  
- Overclaiming again in the README  

Closeout rule: [`EVIDENCE_CLOSEOUT.md`](EVIDENCE_CLOSEOUT.md).

---

## Realistic ceiling

| Dimension | Ceiling |
|-----------|---------|
| **Technical** | Solid civil edge filtering + SW integrity — not “assured PNT”, not RF anti-jam |
| **Business** | Library/module for ag drones, trackers, cheap AUVs, warehouse robots — MIT or dual licence, reproducible bench |
| **Not** | Drop-in u-blox replacement or full PX4 stack |

---

## Closing

Well aimed and credible on **software/method**. Most upside is closing the **physical loop with the same honesty**. Do that → something you can show a customer with data. Don’t → a very good README.
