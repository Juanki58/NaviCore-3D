# GAP-7 — Divergencia vertical / picos de deriva en REF_19082026 v2

**Alcance:** solo diagnóstico. Sin cambios en `ins_ekf.cpp`.  
**Datos:** post Bowring fix (`ekf_v2_ab_3routes/REF_19082026/v2`).  
**Artefacto numérico:** `docs/benchmarks/ekf_v2_ab_3routes/gap7_ref_divergence_diag.json`.

## Pregunta

¿El mal comportamiento de REF (V_log final 123 m; picos H hasta 524 m; accept 87.8%) es:

- **(a)** mala recepción GNSS episódica (multipath / cañón / huecos) que estresa la costa IMU, o  
- **(b)** sesgo sistemático del canal vertical del EKF, o  
- **(c)** perfil de ruta (altitud real / duración)?

---

## PASO 1 — Rechazos vs picos de deriva H

### Hechos base

| Ítem | Valor |
|------|------:|
| GPS medidas | 681 |
| Aceptadas | 598 (87.8%) |
| Rechazadas | 83 |
| `reject_reason` | **todas = 3** (`INS_EKF_GNSS_REJECT_INCONSISTENT`) |

Reason=3 = gate de **consistencia física** INS↔GNSS (innovación horizontal / salto de velocidad en track continuo), **no** NIS. El comentario del código indica que tras outage largo (&gt;~2 s) un |innov| grande debería ir a NIS, no a reason=3.

### Picos H en `replay.log` (drift &gt; 100 m)

| t [s] | drift H [m] |
|------:|------------:|
| 40–50 | 102 → 170 |
| 220–230 | **232** → 172 |
| 390–410 | 81 → **158** → 135 |
| 580–610 | 147 → 408 → **524** → 230 |

Luego recupera (p.ej. t=670 → 9.9 m; final H 13.9 m).

### ¿Los rechazos se concentran en esos picos?

Sí, de forma fuerte.

Ventanas de pico (usuario) + 15 s de pre-roll:

| Ventana | max drift H | rejects en ventana | rejects +15 s pre | accept rate en ventana | \|innov_d\| max |
|---------|------------:|-------------------:|------------------:|-----------------------:|----------------:|
| 220–240 | 232 m | 11 | 14 | 45% | 75 m |
| 390–420 | 158 m | 16 | 16 | 47% | 124 m |
| 580–640 | **524 m** | 36 | 37 | 41% | **232 m** |

- **80.7%** de los 83 rejects caen en esas ventanas (+15 s pre).  
- Tasa de rechazo: **0.43 / s** en picos vs **0.031 / s** fuera → **~14×**.  
- Bins de 30 s con más rejects: 600–630 (19), 390–420 (16), 30–60 (14), 210–240 (14), 570–600 (14). Fuera de clusters, muchos bins con **0**.

Cluster temprano **t≈25–60** (16 rejects) alinea con el pico H de t=40–50 (no listado por el usuario pero presente en el log).

### ¿Huecos GNSS largos?

**No.** Los mayores `dt_since_prev_accept_s` en el audit son ~**3.0 s**. No hay outages de decenas de segundos. El patrón es: track continuo → |innov_h| enorme → reason=3 en ráfaga → deriva H sube → reaceptación → recuperación.

Eso encaja más con **multipath / fixes espurios (o filtro ya divergido) en track continuo** que con túnel/outage clásico.

### Innovación vertical `innov_d` (= z_d − pred_d)

| Segmento | mean innov_d | std | frac signo + | accept |
|----------|-------------:|----:|-------------:|-------:|
| Tercios aceptados (1→2→3) | 8.3 → 11.8 → 25.7 | 27 / 26 / 52 | — | solo accepted |
| Quiet t=300–350 | **+19.5** | 13.3 | **92%** | **100%** |
| Pico 220–240 | +13.5 | 33.7 | 50% | 45% |
| Pico 390–420 | +23.8 | 49.4 | 57% | 47% |
| Pico 580–640 | +21.5 | **98.9** | **38%** | 41% |
| Toda la ruta | +16.5 | 42.9 | — | — |

- Correlación `innov_d` vs tiempo (solo accepted): **0.20** → **no** es un bias monótono limpio.  
- En quiet hay un **sesgo positivo leve/moderado** (~+20 m, casi siempre +).  
- En picos, `innov_d` **explota y cambia de signo** (mismo timing que picos H), no un ramp lento independiente.

---

## PASO 2 — Perfil REF vs ALT / JUL17

### Altitud real (`Location.csv`)

| Ruta | alt min | alt max | **Δalt** | duración | n GPS |
|------|--------:|--------:|---------:|---------:|------:|
| REF_19082026 | 51.2 | 84.4 | **33.2 m** | **677 s** | 681 |
| ALT_16072026 | 65.7 | 84.4 | **18.7 m** | 333 s | 331 |
| JUL17_20260717 | 54.0 | 90.6 | **36.6 m** | **800 s** | 715 |

- REF **no** es un perfil vertical extremo frente a JUL17 (Δalt similar o menor).  
- JUL17 **dura más** que REF (800 s &gt; 677 s) y no muestra el mismo patrón de picos/accept-rate.  
→ Se descarta la explicación banal “ruta más larga ⇒ más deriva” y “REF es mucho más montañosa”.

---

## Conclusión (explícita, multi-causa)

1. **Causa dominante de los picos H y del drop de accept-rate: (a) episodios de GNSS/consistencia**, no un outage largo.  
   Los 83 rejects son reason=3, concentrados (~14×) justo en las ventanas de drift H extremo; gaps de aceptación ≤~3 s; tras el episodio el filtro se recupera.

2. **Sesgo vertical sistemático suave (b parcial):** en tramos quietos `innov_d` media ~+20 m y signo casi siempre +. Eso puede contribuir al V_log final (123 m), pero **no** explica por sí solo los picos H de cientos de metros ni el patrón de rejects.

3. **Perfil de ruta (c) descartado como explicación principal:** Δalt REF ≈ JUL17; JUL17 es más larga y no replica el síndrome.

4. **Implicación para el desacoplo H/V:** no forzar ese fix todavía. El next-step más alineado con (1) sería entender por qué reason=3 dispara en ráfagas con |innov_h|≫120 m en track continuo (calidad GNSS vs filtro ya divergido), y aparte medir si el sesgo D quieto merece R_v distinto. Eso se decide en una fase posterior; este doc no propone implementación.

---

## Referencias

| Pieza | Ruta |
|-------|------|
| Audit GNSS | `docs/benchmarks/ekf_v2_ab_3routes/REF_19082026/v2/gnss_nis_audit.csv` |
| Replay log | `…/REF_19082026/v2/replay.log` |
| JSON diagnóstico | `docs/benchmarks/ekf_v2_ab_3routes/gap7_ref_divergence_diag.json` |
| Reason=3 | `INS_EKF_GNSS_REJECT_INCONSISTENT` en `ins_ekf.hpp` / `meas_reject.hpp` |
| Location REF | `data/real_run/19082026/Location.csv` |
