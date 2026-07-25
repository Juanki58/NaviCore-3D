# 20 — H-seed-v: inicialización de velocidad a cero

**Estado:** preregistro congelado · **implementación en curso** — 2026-07-20  
**Tipo:** experimento causal localizado (una intervención).  
**CLI:** `--seed-velocity zero|gnss` (default `zero` = ctrl).

No exige que desaparezcan ~1700 m de residual.  
Exige cambiar el **régimen de los primeros segundos**.

---

## 1. Hipótesis (H-seed-v)

> La inicialización de la velocidad a **cero** (`seed_from_ned_pos`), combinada con la **ausencia de corrección de velocidad** en el primer GNSS aceptado (`corr_vel≈0`), es **suficiente** para explicar la **incoherencia cinemática inicial** (`speed_vs_gps` siempre FAIL; A nace con `|v| ≪` GNSS).

**No afirma** (aún): que basten para explicar toda la deriva kilométrica.  
Puede ser **necesaria** y no **suficiente**.

Cadena causal bajo prueba:

```
seed v=0
  → predict correcto (v≈ε)
  → GNSS accept pos, corr_vel=0
  → NHC sobre “casi parado”
  → rechazos GNSS / v no converge al régimen real
  → A incoherente, course−yaw / speed_vs_gps rotos en 0–6 s
```

---

## 2. Evidencia que motiva (no es el experimento)

| Hecho | Fuente |
|-------|--------|
| `seed_from_ned_pos` fuerza `vel=0` | `ins_ekf_15_state.cpp` |
| Primer accept: `corr_vel_h=0` | `gnss_nis_audit` @ t≈2.67 |
| `|v|_h` media ~0.15 m/s en t≤6; GNSS ~8 m/s | `constraint_pipeline` / tick_stage |
| `speed_vs_gps` FAIL 338/338 (t≤6) | `audit_stage_invariants` |
| Predict I1/I2/I3 PASS | kinematic identity audit |
| Primer FAIL course−yaw en A; A✔→B✘=0 | invariantes |

---

## 3. Intervención única (una sola; dos brazos opcionales)

**Prohibido** en este experimento: tocar NHC, predict, Q/R, gates NIS, covarianzas, mount, yaw-init, pos_vel policy salvo lo listado.

| Brazo | Cambio |
|-------|--------|
| **ctrl** | Baseline actual: `seed_from_ned_pos` → `v=0` (sin cambios) |
| **H1** | Tras el mismo seed de posición, asignar  
| | `v_N = speed·cos(course)`, `v_E = speed·sin(course)`, `v_D=0`  
| | desde el **mismo** primer fix GNSS usado para seed (speed/course del CSV; si speed=0 o sin course, **no** inventar — marcar brazo inválido o usar segundo GPS con speed>umbral) |
| **H2** *(opcional, solo si H1 no aplica por speed=0 en seed)* | Mantener seed `v=0`; en el **primer** `update_gnss` aceptado con `speed≥3` m/s, forzar inyección de velocidad GNSS (una vez). Sin cambiar gates de aceptación posteriores. |

Ejecutar **ctrl vs H1** (o ctrl vs H2). No mezclar H1+H2 en el mismo run.

Implementación prevista (cuando “adelante”): flag CLI único, p.ej.  
`--seed-velocity zero|gnss` (default `zero` = ctrl).

---

## 4. Pack / corrida

| Campo | Valor |
|-------|--------|
| Input | `docs/benchmarks/real_run_19082026_baseline/real_run_replay.csv` |
| Mount / yaw / constraints | **Idénticos** al baseline G-ext usado en tick_stage (calibration, yaw-init zero, h9a gravity tilt, constraint disabled, nhc enabled, pos_vel, p_pv none) |
| Ventana primaria | **0–10 s** (métricas en **0–6 s**) |
| Artefactos | `tick_stage_audit.csv` + `constraint_pipeline` (o `audit_velocity_provenance`) por brazo |
| Salida | `docs/benchmarks/h_seed_v/{ctrl,H1}/` |

---

## 5. Criterios (congelados **antes** de ver H1)

No-gate (no PASS/FAIL del residual final): residual @ fin de recorrido.

### Gates (PASS del experimento = hipótesis **reforzada**)

Evaluar en **t ∈ [0, 6] s** salvo t_sep.

| ID | Métrica | Ctrl (referencia) | PASS H1/H2 |
|----|---------|-------------------|------------|
| **P1** | `speed_vs_gps` fail_frac (valid samples, stage B) | ~1.0 | **≤ 0.30** |
| **P2** | Primer t con `vel_h_after_nhc ≥ 0.25 · gps_speed` (gps_speed≥3) | ≥6 s o nunca en ventana | **≤ 4.0 s** |
| **P3** | Primer FAIL OK→FAIL de `course_yaw` (stage A) | t≈5.52 s | **ausente en 0–6 s** **o** retrasado **≥ +2.0 s** vs ctrl del mismo binario |

### Secundarios (informativos; no gates)

| ID | Métrica | Lectura |
|----|---------|---------|
| S1 | `t_sep` residual>30 m sostenido | ¿se mueve? |
| S2 | residual_h @ t=6 s | orden de magnitud |
| S3 | A✔→B✘ course_yaw count | debe seguir ~0 si NHC inocente |

### Veredicto

| Resultado | Conclusión |
|-----------|------------|
| **P1 ∧ P2 ∧ P3** | H-seed-v **reforzada** — v-init es mecanismo causal del régimen inicial; seguir cadena (¿suficiente para km?) |
| **P1∨P2 mejoran, P3 no** | Régimen de velocidad sí; course−yaw tiene otra causa → documentar |
| **Ningún P** | H-seed-v **debilitada** — v=0 es síntoma o insuficiencia; no reescribir más seed sin nueva hipótesis |
| Residual km desaparece | Bonus; **no** requerido |

---

## 6. Qué no hacer

- No retocar NHC “de paso”.
- No declarar victoria por residual final.
- No mezclar H1 y H2.
- No cambiar umbrales P* después de ver H1.

---

## 7. Resultado ctrl vs H1 (2026-07-20) — preregistro intacto

Corrida: `tools/campaigns/run_h_seed_v.py` · artefactos `docs/benchmarks/h_seed_v/`  
H1 applied @ **t=4.301 s** · speed=8.38 · course=93.6° (primer GPS con speed≥3).

| Gate | Ctrl | H1 | ¿PASS? |
|------|------|----|--------|
| **P1** fail_frac speed_vs_gps | 1.00 | **0.00** | **Sí** |
| **P2** t(v≥0.25·gps) | null | **4.303 s** | No (≤4.0; primer GPS elegible ~4.30) |
| **P3** first course−yaw FAIL | 5.52 s | **4.30 s** (antes) | **No** — empeora |
| **Conjunto** | | | **FAIL** (hipótesis solo parcialmente reforzada) |

`speed_max_B`: 0.30 → **11.0** m/s.

**Frase discriminante (congelada):**

> La inyección de velocidad GNSS elimina la incoherencia de **magnitud** de v, pero no restaura la coherencia cinemática: la **actitud permanece incompatible** con la dirección del movimiento.

H1 demuestra que `v=0` **era** un problema real (P1). No era el mecanismo completo (P3 empeora).  
P2 falla el umbral por ~0.3 s (timing del primer GPS elegible).

**Siguiente:** auditoría yaw (§21) + preregistro H2 — no retocar gates P* de este doc.

---

## 8. Enlace

Programa: [`docs/ekf_explorer/RESEARCH_PROGRAM.md`](../ekf_explorer/RESEARCH_PROGRAM.md)  
Auditorías previas: `tools/audits/audit_velocity_provenance.py`, `tools/audits/audit_stage_invariants.py`  
Runner: `tools/campaigns/run_h_seed_v.py`
