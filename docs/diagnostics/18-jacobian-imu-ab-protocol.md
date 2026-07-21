# A/B 2×2 — estado tras fontanería + autopsia SLALOM A×C

**Estado:** sesión **no cerrada**; **bisect aplazado**  
**Seed:** 71  

## 1. Fontanería (confirmada)

`--imu-mode` **no llega** a SLALOM (`make_ideal_slalom_imu` fijo). A≡B y C≡D en slalom son identidad forzada.  
Detalle: `docs/benchmarks/jacobian_imu_ab/slalom_imu_mode_plumbing.md`

→ En SLALOM, la matriz válida colapsa a **A vs C** (Jacobiano). Ideal vs dirty en esa fila es ruido.

## 2. Matriz TUNNEL (sí usa imu-mode) + SLALOM (solo J)

| | IMU ideal (TUNNEL) | IMU dirty (TUNNEL) | SLALOM (imu ignorado) |
|--|--------------------|--------------------|------------------------|
| **J corregido** | A: **152** m | B: **487** m | **54.0** m |
| **J legado** | C: **17.4** m | D: **15.9** m | **0.377** m |

Comparaciones aisladas (una variable):

| Par | Variable | Efecto |
|-----|----------|--------|
| **A vs C** | solo J (IMU ideal) | slalom 54→0.38 (~143×); tunnel 152→17 (~9×) |
| **B vs D** | solo J (IMU dirty) | tunnel 487→16 (~30×) |
| A vs B | solo dirty (J correct) | tunnel 152→487 (~3×); slalom idéntico (fontanería) |
| C vs D | solo dirty (J legacy) | tunnel 17→16 (~1×) |

El efecto **Jacobiano** domina al efecto **dirty** en las celdas válidas.

## 3. Autopsia SLALOM A vs C — patrón revisado (burstiness + ω)

Artefactos: `slalom_a_vs_c_autopsy.md`, `slalom_a_vs_c_omega_burstiness.md`, `slalom_a_vs_c_omega_xcorr.md`, `fig_slalom_omega_vs_ddrift.png`, `fig_slalom_omega_xcorr.png`

| Check | Resultado |
|-------|-----------|
| Primera \|Δdrift\| > 0.01 m | t ≈ **1.35 s** |
| Burstiness 1.3–2.0 s (`B=max\|Δ\|/Σ\|Δ\|`, top3) | **NOT bursty** — B=0.086, top3_share=0.226 (umbrales 0.25 / 0.5) |
| Whole-run B | 0.0036 (ventana corta no sobreconcentra) |
| CSV `yaw_rate` (antes) | muerto (PID path); fontanería corregida: `TelemetryBindings.measured_yaw_rate_*` ← `imu.gyro_z` |
| CSV `yaw_rate` (tras fix, re-run A/C) | **vivo**; vs truth: r=1.000, max\|err\|≈1.4e-6 (ideal SLALOM: medido ≡ cinemática) |
| Argmax ‖ω‖ vs \|d(Δdrift)/dt\| (1–5 s) | lag **+1.38 s** — descarta acoplamiento **instantáneo**, no la familia actitud |
| Xcorr ‖ω‖(t) × \|dΔ/dt\|(t+τ), τ∈[0,3] s | pico claro @ **τ≈1.93 s**, r≈0.38 (ventana 0–8 s); 0–20 s: τ≈1.96 s, r≈0.33 (Δr vs r(0) más débil) |
| Xcorr **por viraje** (10 picos \|ω\| @ 2…20 s) | **(a) + alias de periodo** — ver §5b |

Argmax era el test equivocado para cadena actitud→velocidad→posición. La xcorr agregada **soporta acoplamiento retardado** (~1.9–2.0 s). El Δr débil en 0–20 s **no** es (b): el lag característico ≈ espaciado de picos (2 s) produce alias τ≈0 en virajes seguidores.

## 4. Estado al parar esta sesión (2026-07-19 noche — cierre final)

| Pieza | Estado |
|-------|--------|
| Mecanismo **temprano** A×C | **Cerrado** — cadena 11 eslabones (§13.15) |
| Familia **“atacar Z vía δx”** | **Cerrada** — b1/c/λ=1 FAIL; déficit Joseph → P_av (§13.20) + fuga `bias_gz` |
| Detector cand1 | **Válido en slalom** (P3-C); **no generaliza** a túnel — `CAND1_GENERALIZATION_REVIEW.md` |
| Fusión early ↔ OQ9 vía bias | **Descartada** (§12.9) |
| Umbrales E2E A/B/C/D | **Siguen FAIL** |
| OQ8 / H-ATT-d | Open — hipótesis intacta; **instrumento de gate** bloquea P2-tunnel/P4 limpios |
| OQ9 | Open, separado |

| Proyecto | Estado | Siguiente sesión |
|----------|--------|------------------|
| **(1) Early-loop** | Mapa cerrado; H-ATT-d implementada+barrida | **No** P2-tunnel/P4 con cand1 actual. Cerrar generalización cand1 (hecho). Luego: alcance slalom-only **o** observable invariante (preregistro) |
| **(2) OQ9 late** | Open, sin identidad con (1) | dP_pp/dt vs bias **sin** latch |

**Prohibido al retomar:** zero-out/λ en δx; tocar f_va/zero-P a ciegas; reabrir familia Z-δx; fusionar OQ9; **buscar T₂ / cand2 por inercia**; tratar P2-tunnel H-ATT-d FAIL como refutación de unobs.

**Lección de proceso (esta noche):** varios “cierres” intermedios (P_yy, S, Joseph in-tick, f_va unificador) eran trampas; magnitud absoluta / ctrl los desmontó. Parar ahí habría mal-diseñado H-ATT-d. Entendido ≠ resuelto: mecanismo cerrado; E2E y H-ATT-d aún abiertos. Detalle: `SESSION_CLOSE_2026-07-19.md`.

### 4b. Cierre consolidado 2026-07-19

1. Jacobiano NHC → bucle `dx_att_z` → onset → pitch/`K_y0`/`P[ATT_Y,VN]` por déficit Joseph → surge vel → innov → (Z cerrado) `bias_gz`.
2. Familia Z-δx descartada; principio: Ky+λ post-hoc ≠ “no observar”.
3. OQ9 desvinculado.
4. `CORR_ABS_SCALE` congelada (§13.18).
5. Candidato H-ATT-d borrador §13.21 — **sin código hoy**.

**Disciplina:** cero intervenciones sobre causa no confirmada; cada hipótesis fácil exigió una comprobación más.


## 5. ω / burstiness + xcorr (SLALOM A vs C)

| Artifact | Path |
|----------|------|
| Burstiness report | `docs/benchmarks/jacobian_imu_ab/slalom_a_vs_c_omega_burstiness.md` |
| Xcorr report | `docs/benchmarks/jacobian_imu_ab/slalom_a_vs_c_omega_xcorr.md` |
| Per-turn xcorr | `docs/benchmarks/jacobian_imu_ab/slalom_a_vs_c_omega_xcorr_per_turn.md` |
| Per-turn figure | `docs/benchmarks/jacobian_imu_ab/fig_slalom_omega_xcorr_per_turn.png` |
| Scripts | `tools/slalom_a_vs_c_omega_*.py` |

### 5b. Per-turn (a vs b) — con prueba de alias

Artefacto: `slalom_a_vs_c_omega_xcorr_per_turn.md`  
Prueba: para cada `alias0`, xcorr con stim en giro anterior `[(t_c−2)±1]`.

| Check | Resultado |
|-------|-----------|
| Alias-shift | **3/4 confirmados** (T3, T5, T8 → τ≈1.74–2.08); **T6 unconfirmed** (abierto) |
| Clean-regime explainable(a) | **0.86** (n=7; no incluye degradados) |
| Turns 9–10 | `other/degraded` (drift_A 20–32 m) — **aparte**; no en el % |
| r_peak vs std\|dΔ/dt\| | corr **−0.61** (subida de r no es artefacto de amplitud; Pearson ya normaliza) |

Verdict: **A_CONFIRMED_WITH_PERIOD_ALIAS**  
K/P: turn 1 (efecto ≈4.0 s); control turns 2 y 4 (`delayed`). Bisect deferred.

### 5c. OQ9 — tramo tardío (problema nuevo; sesión parada)

**No es cola del mecanismo temprano.** T6 ≠ OQ9.

| Check | Resultado |
|-------|-----------|
| Burstiness 22–25 s | NOT bursty — descarta cliff puntual |
| dx_att_z sign late | **`SIGN_MIXED`** — descarta lockstep de signo temprano como explicación del late |
| Label `FEEDBACK_CONTINUES_OMEGA_DECOUPLED` | **Descartado** (no confirmado; no rebautizar aún) |

**Next session (barato):** dP_pp/dt (14–25 s) vs `bias_gz` acumulado vs actitud/`dx_att_z`.  
Artefactos: `slalom_oq9_late_dxattz_sign.md`, `slalom_oq9_late_p_and_burstiness.md`.

---

## 11. Preregistro — Intervención H-ATT (mecanismo temprano)

**Estado:** preregistrado, **no ejecutado**  
**Fecha:** 2026-07-19  
**Analogía de proceso:** GAP-4 §11 (`13-gap4-gnss-velocity-protocol.md`)  
**Prohibido:** mirar outcomes de intervención mientras se eligen hiperparámetros; rediseñar tras ver drift.

### 11.0 Pregunta e hipótesis

> ¿Romper el bucle de refuerzo temprano actitud↔NHC (signo `H_att` → `dx_att_z` sesgado → `v_body` contaminada → innov NHC del mismo signo) mejora SLALOM celda A sin degradar C/D, sin pretender resolver OQ9?

| ID | Intervención | Mecanismo que ataca |
|----|--------------|---------------------|
| **H-ATT-b1** (primaria) | Atenuación **fija** del `dx_att_z` NHC en el punto de aplicación al estado | Entrada del bucle: limita cuánto puede mover la actitud cada update |
| **H-ATT-b2** (variante separada; no mezclar con b1) | Misma variable (`dx_att_z` aplicado), factor **adaptativo** `α(\|s_z\|)` | Misma entrada del bucle, más fuerte cuando el sesgo acumulado ya es grande |
| **H-ATT-a** (secundaria / opcional) | Clamp de ganancia estilo `ZUPT_MAX_GAIN` sobre `k_att_max` | Hipótesis burda de *loop-gain*. **No** es “A tiene más K que C” |

**Fuera de alcance (explícito):** OQ9 / régimen tardío (t≳14 s). Si el drift E2E sigue FAIL por el tramo tardío tras PASS de P1–P3 tempranos, eso **no** es FAIL de H-ATT — es evidencia de mecanismos separados.

**Descartado como justificación de diseño:** elegir umbral de `k_att_max` “porque tick 0 = 0,059” sin mirar el rango y sin decidir enfoque.

**Descartado como objetivo de regularización:** actuar sobre **`P_att`** (techo / olvido de covarianza de actitud). Motivación: `k_att` A≈C ya muestra que la ganancia no es lo que diverge; regularizar `P_att` actúa sobre una consecuencia aguas abajo, no sobre la causa directa (`dx_att_z` de signo sesgado que contamina `v_body`). Queda fuera de H-ATT-b*.

### 11.0b Evidencia pre-intervención que fija el orden de decisión

Audits post-fix (seed 71), ventana turn-1 temprana, **sin código nuevo**:

| Ventana | `k_att` A vs C | `dx_att_z` rms A vs C |
|---------|----------------|------------------------|
| 0–2 s | mean 0,01512 / 0,01518 (ratio **0,996**, corr **0,939**); max ambos **0,05937** | **1,50e-3** / **9,5e-8** (~1,6×10⁴×) |
| 0–0,75 s | ratio **0,983**, corr **1,000**; max ambos 0,05937 | 3,4e-7 / 4,0e-8 |

**Consecuencia de diseño (congelada aquí):**

1. **Primero** se elige familia (b vs a), **después** se fijan números.
2. Un clamp solo sobre magnitud de `k_att` **no** puede justificarse como “selectivo porque A tiene más K” — A≈C en `k_att`.
3. Si se corre H-ATT-a, es como control de *loop-gain* (análogo a H1b en espíritu: intervención burda), no como hipótesis causal principal.
4. H-ATT-b ataca el canal donde A y C **sí** divergen (`dx_att_z` / acumulación de corrección de actitud).

**Decisiones de diseño (congeladas 2026-07-19):**

| # | Decisión | Valor |
|---|----------|--------|
| D1 | Variable de actuación | **`dx_att_z` solo** en el punto de aplicación, tras `δx = K y`, antes de `x ← x + δx`. **No** `P_att`. **No** ATT_X/Y. |
| D2 | Familia esta tanda | **H-ATT-b1** únicamente |
| D3 | H-ATT-b2 / H-ATT-a | **Fuera de esta tanda** |
| D4 | λ | Barrido **`λ ∈ {0,3, 0,5, 0,7}`** (no un único valor a ciegas). Preferencia de diseño: 0,5 como ancla; extremos = sensibilidad. `λ = 0` = control (mismo binario). |
| D5 | P2 | **`ρ = 1,20` por escenario** (slalom y túnel scoreados **por separado**, no agregados) |
| D6 | P3-check | Consistencia **`k_att` A≈C** post-intervención (0–2 s): ratio mean ∈ [0,9, 1,1] y corr ≥ 0,9 — chequeo de pipeline, **no** gate de éxito |

### 11.1 Diseño experimental

| Parámetro | Valor fijado |
|-----------|--------------|
| Escenario primario | SLALOM seed **71** (misma cohorte A×C del diagnóstico) |
| Matriz | A/B/C/D = J correct/legacy × IMU ideal/dirty (TUNNEL usa imu-mode; SLALOM imu-mode sigue sin llegar — A≡B, C≡D en slalom) |
| Control | Sin intervención — artefactos ya existentes (`slalom_cell{A,C}_*_nhc_block_audit.csv`, telemetría baseline) |
| Intervención | Misma matriz con flag/env de H-ATT activo (un brazo = una política) |
| Constraint / NHC | Política NHC del benchmark actual (sin cambiar otras perillas entre control e intervención) |
| Orden | Completar todos los brazos preregistrados **antes** de mirar PASS/FAIL agregados |

**Brazos esta tanda:**

| Brazo | Política |
|-------|----------|
| Control | `λ = 0` (mismo binario) |
| **b1-0.3** | `δx[ATT_Z] ← (1−0,3)·dx_att_z_raw` |
| **b1-0.5** | idem con λ=0,5 |
| **b1-0.7** | idem con λ=0,7 |

Por cada λ: matriz A/B/C/D (SLALOM + TUNNEL). Audits NHC A×C en SLALOM para P3 + D6.

### 11.1b Definición operativa (pre-código) — dónde actúa λ y qué es `s_z`

**Punto de inserción (común a b1 y b2):**  
Tras calcular `δx = K y` en el update NHC, **modificar solo el bloque actitud de `δx`** (`INS_ERR_ATT_X/Y/Z`, como mínimo `ATT_Z`; alcance XYZ vs solo Z = freeze numérico).  
Luego aplicar el `δx` ya atenuado al estado.  
La covarianza sigue el update Joseph/estándar del filtro con el **K original** (no se retoca `P` como regularizador). Así λ mide “fracción del corrección de actitud NHC que se deja pasar”, no “olvido de P”.

**Explícitamente no es:**

| No | Por qué |
|----|---------|
| Techo / decay de `P_att` | `k_att` no diverge A vs C; sería intervenir la consecuencia, no la entrada del bucle |
| Clamp de `k_att_max` | Eso es H-ATT-a, brazo distinto |
| Factor sobre la innovación `y` | Cambiaría también el canal velocidad del NHC; fuera de alcance |

---

**H-ATT-b1 — λ fijo, solo ATT_Z (esta tanda — CONGELADO)**

Por cada update NHC aceptado:

```
dx_att_z_raw ← δx[ATT_Z]          # sale de K·y; k_att_max ya medido sobre K
δx[ATT_Z] ← (1 − λ) · dx_att_z_raw
# ATT_X/Y, vel, pos, bias intactos
# Joseph/cov con K original (sin retocar P como regularizador)
aplicar δx al estado
```

- CLI: `--nhc-att-z-forget <λ>` (`λ=0` default = off).
- Audit `dx_att_z` = valor **aplicado** (post-λ). `k_att_max` = de K **pre-λ** (para D6).

**H-ATT-b2 / H-ATT-a:** diferidos; definiciones previas siguen como borrador, no se implementan ahora.

### 11.2 Endpoints

**Primarios:**

1. **P1 — Outcome SLALOM A:** error lateral (métrica E2E ya usada en la matriz §2) con intervención.
2. **P2 — No daño legado:** SLALOM C (y D si se corre) no empeoran vs control más allá de ε.
3. **P3 — Mecanismo temprano:** en ventana **0–2 s / turn 1**, deja de haber `dx_att_z` con signo opuesto sostenido A_interv vs C_control (o vs patrón control A), medido como en la autopsia K/P — no solo el número final de drift.

**Secundarios (informativos, no gate):**

4. **S1 — TUNNEL A/B:** drift tunnel con misma política (regresión cruzada).
5. **S2 — OQ9 late:** fracción de `|drift|` en t≥14 s; **no** entra en PASS/FAIL de H-ATT.

### 11.3 Criterios PASS / FAIL (congelar números antes del run)

Constantes propuestas para freeze (editables solo **antes** del primer binario con H-ATT):

| ID | Criterio | PASS (congelado) |
|----|----------|------------------|
| **P1** | SLALOM A lateral | **≤ 2,0 m** **∧** ≥ **10×** vs control A del mismo binario (`λ=0`). Gana el más estricto. *Aspiracional no-gate: ≤ 0,15 m.* |
| **P2-slalom** | SLALOM C y D | `drift_interv ≤ 1,20 × drift_control(λ=0)` **por celda** |
| **P2-tunnel** | TUNNEL A/B/C/D exit | idem **`ρ=1,20` por celda**, scoreado **aparte** de slalom (no agregar) |
| **P3** | Mecanismo 0–2 s | `opp_sign_frac` ≤ 0,60 **o** `rms_A/rms_C ≤ 100`; **y** no `FEEDBACK_GROWTH` 0–0,75 s |
| **D6** | Consistencia `k_att` 0–2 s | mean(A)/mean(C) ∈ [0,9, 1,1] **y** corr(A,C) ≥ 0,9 — **reporte obligatorio**; fallo D6 = alerta de fontanería, no redefine P1 |

**PASS H-ATT-b1 (por λ):** P1 ∧ P2-slalom ∧ P2-tunnel ∧ P3. D6 en el reporte.

**No-FAIL explícito:** E2E histórico FAIL **solo** por t≥14 s tras P1–P3 → **H-ATT PASS + OQ9 residual**.

### 11.4 Checklist

- [x] Diseño D1–D6 + números congelados
- [x] Alcance solo Z; barrido λ; ρ=1,20 por escenario; solo b1
- [x] Implementar `--nhc-att-z-forget` + runner `tools/run_hatt_b1.py`
- [x] Ejecutar barrido; scorear sin retocar umbrales

### 11.5 Resultado H-ATT-b1 (2026-07-19) — sin reinterpretar umbrales

Artefacto: `docs/benchmarks/jacobian_imu_ab/hatt_b1/hatt_b1_report.json`

| λ | SLALOM A (m) | P1 | P2-slalom | P2-tunnel | P3 | D6 | HATT |
|---|--------------|----|-----------|-----------|----|----|------|
| 0 (ctrl) | 54,0 | FAIL | — | — | FAIL (esperado) | PASS | FAIL |
| 0,3 | **141,6** | FAIL | PASS | PASS | FAIL | PASS | FAIL |
| 0,5 | **140,3** | FAIL | FAIL | PASS | FAIL | PASS | FAIL |
| 0,7 | **137,0** | FAIL | FAIL | PASS | FAIL | FAIL | FAIL |

**Veredicto preregistrado:** H-ATT-b1 **FAIL** en todo el barrido. La atenuación de `dx_att_z` **empeora** SLALOM A (~2,6× vs control), no cierra el bucle. **Lección causal:** atenuar Z siempre rompe C/D (λ≥0,5) → familia “suprimir canal Z” descartada; falta guarda condicional. Siguiente: **§12 H-ATT-c**.

---

## 12. Preregistro — H-ATT-c (detector cand1 + b1 guardado)

**Estado:** congelado, listo para implementar  
**Fecha:** 2026-07-19  
**Prerequisitos:** §11.5 FAIL; `hatt_cand12_A_vs_C_discrimination.*` (cand1 OK, cand2 descartado)

### 12.0 Hipótesis

> ¿Aplicar la atenuación λ de b1 **solo tras** detectar crecimiento superlineal de Σ|dx_att_z| (cand1) rompe el bucle en A **sin** disparar en C (donde b1 ciego destruía el canal Z útil)?

| Pieza | Definición |
|-------|------------|
| **Detector** | Online: `S ← S + \|dx_att_z_raw\|` (pre-λ). Dispara (latch) si `S ≥ T` **y** `t ≤ t_max`. |
| **Acción post-disparo** | Reutilización **guardada** de H-ATT-b1: `δx[ATT_Z] ← (1−λ)·dx_raw` en todo NHC **posterior** al latch. Joseph/K intactos. |
| **Cand2** | Fuera (no discrimina; consecuencia trivial de k_att A≈C). |

**Por qué T absolutos + t_max (no |ΣA|/|ΣC| online):** el filtro no ve C. Calibración desde control A en los instantes donde |ΣA|/|ΣC| cruzó ×2 / ×5. Sin `t_max`, C cruzaría los mismos T a 1,21 s / 1,86 s → falso disparo (P3-C).

### 12.1 Constantes congeladas

| Símbolo | Valor | Origen |
|---------|-------|--------|
| **T₂** (brazo early) | `3,736646e-6` rad | Σ\|dx\|_A @ t≈0,39 s (cruz ×2) |
| **T₅** (brazo late) | `1,224574e-5` rad | Σ\|dx\|_A @ t≈0,58 s (cruz ×5) |
| **t_max** | **0,65 s** | Bound racha opuesta A×C; C no alcanza T₂/T₅ antes |
| **λ** | `{0,3, 0,5, 0,7}` | Mismo barrido b1; condicional → no ampliar aún |
| **ρ** | 1,20 por escenario | Igual §11 |
| Epoch `t` | tiempo desde **primer update NHC** | Alinea audits |

### 12.2 Brazos

| Brazo | T | λ | Nota |
|-------|---|---|------|
| Control | — | 0 (gate off) | mismo binario |
| **c-E-λ** | T₂ | 0,3 / 0,5 / 0,7 | early fire (~0,39 s en A) |
| **c-L-λ** | T₅ | 0,3 / 0,5 / 0,7 | late fire (~0,58 s en A) |

Matriz por brazo: A/B/C/D × SLALOM+TUNNEL. Audits A×C en SLALOM para P3.

CLI (propuesto): `--nhc-att-z-forget <λ>` + `--nhc-att-z-forget-gate <T>` + `--nhc-att-z-forget-tmax 0.65`  
(`T≤0` ⇒ gate off = b1 ciego / off).

### 12.3 Criterios PASS / FAIL (por brazo T×λ)

| ID | Criterio | PASS |
|----|----------|------|
| **P1** | SLALOM A | ≤ 2,0 m **∧** ≥10× vs control A |
| **P2-slalom** | C, D | ≤ 1,20 × control |
| **P2-tunnel** | A–D exit | ≤ 1,20 × control (**aparte**) |
| **P3-A** | Mecanismo en A | latch **sí** en t∈(0, t_max]; post-intervención: no FEEDBACK_GROWTH en 0–0,75 s **o** rms_A/rms_C ≤ 100 |
| **P3-C** | No-regresión detector | en C: **no latch** en toda la corrida (esp. durante racha larga 0–2 s) |
| **D6** | k_att A≈C 0–2 s | ratio∈[0,9,1,1] ∧ corr≥0,9 (reporte; no gate) |

**PASS H-ATT-c:** P1 ∧ P2-slalom ∧ P2-tunnel ∧ P3-A ∧ **P3-C**.

**No-FAIL:** OQ9 late residual tras P1–P3.

### 12.4 Prohibido

- Retocar T₂/T₅/t_max/λ tras ver drift.
- Aplicar λ antes del latch.
- Ampliar a ATT_XYZ o a cand2 en esta tanda.

### 12.5 Resultado H-ATT-c (2026-07-19) — sin retocar umbrales

Artefacto: `docs/benchmarks/jacobian_imu_ab/hatt_c/hatt_c_report.json`

| Brazo | Fire A | Fire C | SLALOM A (m) | P1 | P2-s | P2-t | P3-A | P3-C | HATT |
|-------|--------|--------|--------------|----|------|------|------|------|------|
| c-E-0,3 | 0,39 s | no | 151,7 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |
| c-E-0,5 | 0,39 s | no | 110,3 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |
| c-E-0,7 | 0,39 s | no | 143,0 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |
| c-L-0,3 | 0,58 s | no | 143,5 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |
| c-L-0,5 | 0,58 s | no | 128,1 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |
| c-L-0,7 | 0,58 s | no | 144,3 | FAIL | PASS | PASS | FAIL | **PASS** | FAIL |

Control A = 54,0 m. Timing early vs late: early λ=0,5 es el menos malo (110 m) pero sigue **peor** que control.

**Veredicto:** H-ATT-c **FAIL** en outcome (P1/P3-A).  
**Éxito parcial del detector:** **P3-C PASS** en todos los brazos (C no dispara; C/D slalom intactos — a diferencia de b1 ciego).  
**Lección:** la guarda condicional resuelve el daño a legado; atenuación parcial (λ≤0,7) no rompe el bucle en A.

### 12.6 Extensión preregistrada — λ=1 (congelar Z post-latch)

**Motivación (pre-run):** λ parcial solo ralentiza; el bucle vive del **signo** sostenido. λ=1 = no dejar pasar nada de `dx_att_z` tras disparo (no probado en §12.2). Misma guarda T₂/T₅, t_max=0,65; mismos P1–P3-C.

| Brazo | T | λ |
|-------|---|---|
| **c-E-1** | T₂ | **1,0** |
| **c-L-1** | T₅ | **1,0** |

Prohibido: retocar T/t_max; pasar a reset de Σdx sin scorear estos dos brazos.

**Resultado §12.6** (`hatt_c_lambda1_report.json`):

| Brazo | Fire A | Fire C | SLALOM A | P1 | P2-s | P2-t | P3-A | P3-C | HATT |
|-------|--------|--------|----------|----|------|------|------|------|------|
| c-E-1 | 0,39 s | no | **95,8** | FAIL | PASS | FAIL | FAIL | PASS | FAIL |
| c-L-1 | 0,58 s | no | **66,2** | FAIL | PASS | FAIL | FAIL | PASS | FAIL |

Control A = 54,0 m. λ=1 es el mejor atenuador visto (66 m late) pero **sigue peor que control**; P3-C intacto. P2-tunnel FAIL (detalle en JSON).  
**Veredicto:** congelar flujo futuro de `dx_att_z` (λ=1) **no basta**. Familia magnitud/flujo **cerrada**.

### 12.7 Option A vs B (post λ=1) — antes de preregistrar reset

Artefactos: `hatt_c/hatt_c_lambda1_option_A_vs_B.{json,md}`, figuras `fig_hatt_c_lambda1_option_A_vs_B*.png`.

| Check | c-E-l1 (latch 0,39) | c-L-l1 (latch 0,58) |
|-------|---------------------|---------------------|
| Σdx applied post-latch | **plano** (Δ=0) | **plano** (Δ=0) |
| \|drift\| @ latch | 0,005 m | 0,007 m |
| \|drift\| final | 95,8 m | 65,4 m |
| Share del drift **post**-latch | **≈100 %** | **≈100 %** |

**Opción A ingenua rechazada:** el daño E2E **no** está ya materializado en metros a 0,39/0,58 s.  
**Opción B a nivel de flujo NHC-Z confirmada:** con `dx_att_z` clavado a 0, el drift sigue creciendo con pendiente fuerte (latch→2 s ≈ 0,6–0,7 m/s).

Antes de preregistrar reset de Σdx: separar **B1** (actitud ya torcida en el estado; reset de actitud/undo Σ puede ayudar) vs **B2** (vía independiente, p.ej. bias). No asumir que reset de Σdx_NHC basta.

### 12.8 B1 vs B2 — veredicto (A λ=1 vs C, post-latch)

Artefactos: `hatt_c/hatt_c_B1_vs_B2.{md,json}`, `fig_hatt_c_B1_vs_B2_*.png`.

Comparación **A−C** en [latch, 2 s] (no absolutos):

| | Δyaw A−C | Δbias_gz A−C |
|--|----------|--------------|
| Esperado B1 | offset ~constante (cicatriz) | estable vs C |
| Esperado B2 | — | **diverge** vs C |
| **Observado (E y L)** | ~0→0,012 rad (parecido a ctrl) | **0→−0,027…−0,029**, slope **−0,02…−0,028 /s** (ctrl solo +0,002 /s) |

**Veredicto provisional §12.8: B2** (bias activo vs C post-latch).  
**Corregido en §12.9:** esa divergencia es en gran parte **fuga inducida por el latch**, no motor preexistente ni identidad con OQ9.

### 12.9 Coherencia bias — latch artifact vs OQ9 (2026-07-19)

Artefactos: `hatt_c/hatt_c_bias_coherence_early_oq9_latch.{md,json}`, `fig_hatt_c_bias_latch_vs_oq9.png`.

| Check | Resultado |
|-------|-----------|
| **¿Pendiente bias depende del latch?** | **Sí.** ctrl A [0,39→2]: **+0,0023 /s**; E_l1: **−0,0208 /s** (~−9×, signo opuesto). Motor fuerte **no** preexistente. |
| **¿Signo early↔OQ9 continuo en ctrl?** | **No como una sola deriva.** ctrl early/late ambos leve **+**, pero mid [2→14] **flip** (−). Sin línea causal ininterrumpida. |

**Veredicto:** no fusionar early-slalom + OQ9 vía bias. El “B2” de §12.8 es artefacto de cerrar Z (error migra a `bias_gz`).  
**Diseño siguiente:** no preregistrar ataque a bias como causa raíz del early loop; considerar P_att–bias / no cerrar la única vía de escape. OQ9 sigue aparte, en telemetría **sin** latch.

---

## 13. Caracterización P_att–bias_g (pre-intervención)

**Estado:** en curso (2026-07-19 retoma)  
**No es intervención** — solo instrumentación + lectura. Prohibido diseñar clamp/zero de P hasta ver el veredicto de §13.2.

### 13.0 Pregunta

> ¿El bloque cruzado `P_att,bias_g` (esp. `P[ATT_Z,BIAS_GZ]`) predice la dirección/magnitud de la fuga a `bias_gz` cuando se cierra el flujo `dx_att_z` (latch λ=1), análogo a cómo `P_pv` predijo el arrastre pos→vel en GAP-4?

### 13.1 Instrumentación

NHC block audit añade (tras `gps_speed_mps`):

- `P_pre/post_att_bias_g_frob`, `P_*_att_bias_g_max_abs`, `P_*_att_z_bias_gz`, `P_*_bias_g_frob`
- `k_bias_gz`, `dx_bias_gx/gy/gz`

### 13.2 Brazos de caracterización (sin retocar T/λ)

| Brazo | Config |
|-------|--------|
| **ctrl** | A correct, λ=0, gate off |
| **latch** | A correct, λ=1, gate=T₂, tmax=0,65 |

Métricas a fijar **antes** de mirar: en [0,39→2] s, correlación/signo de `P_pre_att_z_bias_gz` vs `dx_bias_gz` y vs Δ`bias_gz` telemetría; comparación ctrl vs latch.

**PASS caracterización (para desbloquear preregistro de intervención):** corr(|P|, dx_bias) ≥ 0,3 **y** el bloque se reorganiza vs ctrl bajo latch **y** escape amplificado.  
**FAIL / WEAK:** P~igual ctrl vs latch, o corr débil → fuga puede ser innov amplificada por `K_bias` sin migración vía P; **no** zero-P a ciegas.

### 13.3 Resultado caracterización (2026-07-19)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/patt_bias_g_char_report.json`

| Métrica [0,39→2] s | ctrl | latch λ=1 |
|--------------------|------|-----------|
| mean `P[ATT_Z,BIAS_GZ]` | −0,00320 | −0,00321 (**≈igual**) |
| mean ‖P_att,bias_g‖_F | 0,00666 | 0,00662 (**≈igual**) |
| Σ `dx_bias_gz` | +0,0084 | **−0,0268** (~3×, signo flip) |
| mean innov | 0,113 | **0,258** (~2×) |
| mean `k_bias_gz` | 0,00448 | 0,00459 (~igual) |
| corr(P, dx_bias) | 0,06 | **0,13** (débil) |
| sign-agree P↔dx_bias | 0,60 | 0,71 |
| slope telem bias_gz | +0,0023 | **−0,021** |

**Veredicto: CHAR_WEAK — reading `innov_amplified_K_bias`.**  
El bloque P_att–bias_g **existe** (no es ~0) pero **no se reorganiza** bajo latch; la fuga es innov ~2× con K_bias casi igual → más `dx_bias_gz` sin que P sea el predictor.  
**No desbloquear** intervención “P_att_bias←0”.  
**No forzar PASS** de “reorganización de covarianza”: corr(P, dx_bias)=0,13 no lo sostiene.

### 13.4 Composición de innovación (ctrl vs latch) — antes de descomponer K_bias

**Pregunta discriminante:** ¿el ×2 de ‖y‖ es el mismo residuo más grande (tautología: cerrar Z → yaw sin explicar → bias absorbe), o cambia la composición del vector?

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/innov_composition_ctrl_vs_latch.json`

| Fase | Dirección (cos) | ‖y‖ latch vs ctrl | Σ `dx_bias_gz` latch |
|------|-----------------|-------------------|----------------------|
| **[0,39→1,5] s** | SAME (median cos≈0,99; 100% cos>0,9) | Latch **menor** (mean 0,013 vs 0,022; ratio≈0,74) | −0,0043 ≈ ctrl (−0,0060) — **sin fuga** |
| **[1,5→2,0] s** | **COMPOSITION_BREAK** (median cos≈−0,65; ~59% cos<0) | Latch ≫ (mean 0,79 vs 0,31; mean ratio≈5) | **−0,0225** vs ctrl **+0,0144** — fuga + flip |
| Aggregate [0,39→2] | mediana cos alta (dominada por early) | mean ×2 | media engañosa |

**Veredicto: `COMPOSITION_BREAK_DRIVES_BIAS_ESCAPE` — no es tautología uniforme.**

- El ×2 agregado **no** es “mismo residuo, escala 2×” en todo el post-latch.
- Early: misma dirección pero innov **más pequeña**; la fuga a bias **no** ocurre ahí.
- Late: cambia composición (`iz` flip de signo; fracción Y/Z se reordena) y **ahí** vive casi toda la fuga `bias_gz` y ~97% de Σ‖y‖_latch.
- Por tanto: **segunda lectura** para la ventana interesante — hay algo estructuralmente distinto en la innov, no solo residuo de actitud sin explicar a mayor escala.

### 13.5 Break fino vs ‖ω‖ (antes de descomponer K_bias)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/innov_break_vs_omega.{json,md,png}`

**Pregunta:** ¿la ruptura @~1,5 s es el bin arbitrario, o coincide con el estímulo angular del primer giro interior (pico ‖ω‖@2,0 / xcorr τ≈1,93)?

| Ancla | t (s) |
|-------|-------|
| Latch fire | ~0,39 |
| ω zero-crossing (fin brazo desc. pico t=0) | **1,00** |
| Onset ‖ω‖≥0,5·peak → pico@2 | **1,34** |
| **Break primario** (roll-med-5 cos<0,5 ×3) | **1,590** |
| Prior max \|dΔdrift/dt\| A×C (burstiness) | **1,610** (Δ vs break **−20 ms**) |
| Pico ‖ω‖ interior | **2,000** (break **0,41 s antes**; ‖ω‖/peak@break≈**0,80**) |
| Max drop puntual de cos | 1,820 |

**Fases cinemáticas (no gate cruido `|ω|≥0,5·peak` — está polucionado por el brazo descendente del pico t=0):**

| Fase | median cos | ‖y‖_L/‖y‖_C | Σdx_bias_gz latch |
|------|------------|-------------|-------------------|
| Desc. [0,39→1,0) | 0,996 | 0,77 | ≈0 |
| Valley [1,0→1,34) | 0,992 | 0,73 | ≈0 |
| Rise pre-break [1,34→1,59) | 0,972 | **0,55** (latch menor) | −0,012 |
| Rise post-break [1,59→2,0] | **−0,815** | **5,98** | −0,014 |

**Veredicto: `BREAK_ON_RISING_LIMB_COLOCATED_WITH_PRIOR_DDRIFT_PEAK`.**

- El bin 1,5 era empírico; el break fino es **1,59 s**, en el **brazo ascendente** del primer giro interior — no en el pico@2,0 ni en el latch.
- Coincidencia fuerte con el pico de tasa de divergencia A×C @1,61 s de la sesión ω/burstiness.
- Early latch **menor** innov: coherente con “cortar el bucle de refuerzo ayuda mientras no hay estímulo angular nuevo”.
- El coste aparece cuando el giro nuevo ya lleva ‖ω‖~80% del pico y el canal Z sigue congelado.
- Gate `|ω|` indiferenciado **no** sirve (mezcla dos brazos). Condicionar por **brazo ascendente del giro interior** (post zero-x @1 s / post-break).
- xcorr τ≈1,93 es un **lag** drift←ω, no el instante del break; compatible con onset→break@1,59→acoplamiento drift más tarde.

### 13.6 Continuidad del ratio ‖y‖ en el rise (onset→break vs break→pico)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/innov_rise_continuity.{json,md,png}`

**Pregunta:** ¿el ratio≈6 post-break es tendencia continua desde onset (1,34) que solo “dispara” la mediana del cos en 1,59, o hay un cambio de régimen en el break?

| Ventana | median ratio | mean ratio | Σ‖y‖_L/Σ‖y‖_C | median cos | Σdx_bias_gz latch | share Σ‖y‖_L |
|---------|--------------|------------|---------------|------------|-------------------|--------------|
| onset→break [1,34→1,59) | **0,55** | 0,55 | 0,51 | +0,97 | −0,0124 (~48% signed) | **8%** |
| break→pico [1,59→2,0] | 2,44 | **5,98** | 3,38 | **−0,82** | −0,0136 | **92%** |

- Pre-break: ratio **plano ~0,55** (latch **menor** innov) — no hay subida continua hacia 6.
- En el borde ±0,10 s: cos **+0,91→−0,93** (evento discreto de composición); ratio solo **0,48→1,11** (cruza 1, no salta a 6).
- Explosión a mean≈6 / bins ~24: **después**, pico ~**1,69–1,79** — no en 1,59.

**Veredicto: `REGIME_CHANGE_AT_BREAK_NOT_CONTINUOUS_FROM_ONSET`.**  
Sospecha “tendencia continua cruzando umbral estadístico” — **rechazada para el ratio de innov**. El flip de cos en 1,59 es cambio de régimen real (“latch ayuda”→“latch empeora innov”); el ≈6 es post-break puro (y mean-inflado). Aun así 1,59 **no** es el único instante causal: bias silencioso desde onset; daño de innov máximo ~1,7–1,8.

### 13.7 Cierre caracterización — cronología + brazos K_bias (congelado)

**Estado:** listo para arrancar próxima sesión. **No** es preregistro de intervención; solo ventana(s) de descomposición.

#### Cronología del mecanismo (seed 71, ctrl vs latch λ=1 @ T₂)

| t (s) | Evento |
|------:|--------|
| ~1,00 | Fin brazo descendente del pico ‖ω‖ anterior (zero-crossing) |
| ~1,34 | **Onset** estímulo angular del giro interior; `bias_gz` signed ya acumula; innov aún pequeña |
| ~1,59 | **Cambio de régimen** composición innov (cos +0,91→−0,93) — salto real, no umbral estadístico de ratio |
| ~1,61 | Pico \|dΔdrift/dt\| A×C (ancla sesión ω/burstiness; Δ vs break ≈ −20 ms) |
| ~1,69–1,79 | **Explosión** magnitud innov (median ratio bins → ~24; ~92% Σ‖y‖_latch post-1,59) |
| 2,00 | Pico ‖ω‖ interior |

**Dos relojes:** (A) signed Σ`dx_bias_gz` — ~48% ya en onset→break; (B) magnitud innov — explota ~0,2 s después del flip de cos.

#### Brazos de descomposición K_bias (att vs vel) — CONGELADOS

Tratar el rise como **tres tramos**, no una ventana homogénea [1,34→2,0] ni solo post-break:

| Brazo | Ventana | Qué aísla |
|-------|---------|-----------|
| **R1** | **[1,34 → 1,59)** | Onset silencioso: bias acumula, innov ratio ~0,55, cos≈1 |
| **R2** | **[1,59 → 1,74)** | Post-flip de composición hasta antes del pico de explosión |
| **R3** | **[1,74 → 2,00]** | Explosión de magnitud → pico ‖ω‖ |

Comparar cada brazo **ctrl vs latch**. Prohibido promediar R1∪R2∪R3 como un solo estadístico primario (mismo error de resolución que gate \|ω\| y bin 1,5).

**Fuera de alcance hasta preregistro aparte:** cualquier clamp/zero de P o K; reabrir familia Z; fusionar con OQ9.

### 13.8 Resultado descomposición K_bias att/vel — R1/R2/R3 (2026-07-19)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/k_bias_r123_decompose.{json,md,png}`  
**Definición:** `K_via_X = P H_X^T S^{-1}` (S congelada del update completo); `dx = K_via_X·y`; X∈{vel,att}. Identidad `via_vel+via_att=dx` OK (rms~1e-9).

| Arm | Tramo | Σ total | Σ via_vel | Σ via_att | path_scale | cancel_ratio |
|-----|-------|---------|-----------|-----------|------------|--------------|
| ctrl | R1 | −0,014 | +1,15 | −1,16 | 2,31 | 0,006 |
| ctrl | R2 | +0,021 | −0,09 | +0,11 | 0,20 | 0,105 |
| ctrl | R3 | +0,002 | −0,26 | +0,26 | 0,52 | 0,005 |
| latch | R1 | −0,012 | +0,35 | −0,36 | 0,71 | 0,018 |
| latch | R2 | **−0,039** | **+3,13** | **−3,17** | **6,31** | 0,006 |
| latch | R3 | **+0,026** | −0,25 | +0,28 | 0,53 | 0,049 |

**Veredicto: `NEAR_CANCEL_PATHS_LATCH_R2_SCALE_EXPLOSION`.**

- No hay cambio de motor att→vel entre tramos: ambas vías ~iguales y **opuestas** (cancel_ratio≪1; frac≈0,50 no interpretable como dominante).
- Efecto latch accionable: **explosión de path_scale en R2** (~6,3 vs ~0,2 ctrl) con neto aún pequeño.
- Net latch cambia de signo R1/R2 (−) → R3 (+). Sub-tramos: R3a/R3b flip de signo de vías bajo latch; R2 coherente.
- **No** diseñar “cortar solo via_att” o “solo via_vel” — se desenmascararían mutuamente.

**Siguiente (diseño, no aún código):** anclar intervención al reloj de neto/onset (R1 silencioso → R2 coste), no a un “path switch” inexistente.

### 13.9 path_scale vs ‖y‖ — ¿mecanismo nuevo o la misma explosión?

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/path_scale_vs_innov.{json,md,png}`  
**Pregunta:** ¿el salto ×6 de path_scale en R2 es acoplamiento de ganancia aparte, o ‖y‖ vista a través de K?

| Métrica (R2, latch) | Valor |
|---------------------|------:|
| pearson(path_scale, ‖y‖) | **0,951** |
| frac var(path) explicada por ‖y‖ | **0,89** |
| mean‖y‖ L/C | 2,78× |
| mean path_scale L/C | 3,24× |
| mean (path_scale/‖y‖) L/C | **1,16×** (~estable) |

**Veredicto: `PATH_SCALE_TRACKS_INNOV`.**  
path_scale sigue a ‖y‖; el ppi no se dispara bajo latch. La explosión de path_scale en R2 **es** la explosión de innov propagándose por ambas vías H — no un mecanismo de K a atacar por separado.  
**No** preregistrar intervención sobre path_scale / cortar vías K_bias.  
**Sí** volver a la pregunta de fondo: por qué ‖y‖ explota ~1,69–1,79 s.

**R3 segundo split (no homogéneo — no cerrar R3 como un bloque):**

| Sub | Σ total latch | Σ via_att | mean‖y‖ |
|-----|---------------|-----------|---------|
| R3a1 [1,74→1,805) | **+0,043** | +0,202 | **1,75** (cola explosión) |
| R3a2 [1,805→1,87) | −0,014 | +0,106 | 0,36 |
| R3b1 [1,87→1,935) | −0,002 | **−0,029** (flip) | 0,15 |
| R3b2 [1,935→2,0] | −0,001 | −0,003 | 0,07 |

R3a1 concentra el neto positivo y la innov alta; R3b* flip de signo de vías con scale colapsada. Agregar R3 entero sigue mintiendo.

**Estado investigación:** familia Z cerrada; P_att–bias CHAR_WEAK; paths near-cancel; path_scale≠nuevo motor. Siguiente hilo causal: **origen de la explosión de innov en el rising limb** (no K_bias).

### 13.10 Cierre hilo covarianza/ganancia — arranque próxima sesión (congelado)

**Cerrado hoy:** P_att–bias, descomposición K_bias R1/R2/R3, path_scale-vs-innov.  
**Abierto:** ¿por qué ‖y‖_NHC explota en ~1,69–1,79 s?

**Nota de framing:** la innov del audit es **NHC** (`z≡0` sobre v_lat/v_vert cuerpo), no un residuo GNSS. Con `z` sintético, “medida anómala” **no** aplica como en GNSS. El discriminante es:

| Candidato | Predicción | Cómo falsar | Si se confirma, qué significa |
|-----------|------------|-------------|------------------------------|
| **A — cascada del bucle** (priora) | Truth `v_body` lat/vert ~sano (~0); filtro `v_body` (=−innov) se despega | En [1,69→1,79]: \|v_body\|_truth ≪ \|innov\| | Una cascada con retardos (onset@1,34 → break@1,59 → explosión innov) — sigue siendo problema de filtro/estado |
| **B — NHC demasiado rígido para la cinemática del giro** | Truth **también** tiene \|v_lat\|/\|v_vert\| cuerpo no despreciable en el giro cerrado | \|v_body\|_truth grande en la misma ventana (derrape / inclinación / no-holonomía imperfecta del slalom) | **Conclusión de diseño, no de bug:** el modelo NHC penaliza física real del escenario; la intervención razonable sería relajar/condicionar NHC en ese régimen — **no** cazar otro fallo interno del EKF |

Ctrl vs latch. Mirar primero el par de columnas truth `v_body` vs filtro `v_body` en [1,69→1,79].

Artefacto de cierre: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/SESSION_CLOSE_2026-07-19.md`  
**Prohibido:** intervención sobre K/P/vías antes de resolver A vs B; si B, no reinterpretar como “bug más”.

### 13.11 Resultado A vs B — truth vs filtro `v_body` [1,69→1,79] (2026-07-19)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/innov_explosion_ab.{json,md,png}` + `*_ticks.csv`

**Truth:** `slalom_kinematics_at_time` — yaw-only, vel a lo largo del heading → **v_lat≡0, v_vert≡0 por construcción**.  
**Filtro:** `v_body_y/z_before` del audit NHC (= −innov).

| Arm | max\|truth_lat\| | max\|filt_lat\| | max\|filt_vert\| | max‖y‖ | Sub-tramos |
|-----|------------------|-----------------|------------------|--------|------------|
| latch | ~0 (1e-15) | **1,80** | **1,44** | **2,31** | S1/S2/S3 todos **A** |
| ctrl | ~0 | 0,70 | 0,58 | 0,86 | S1/S2/S3 todos **A** |

**Veredicto: `A_CASCADE`** (latch y ctrl).  
Truth cumple NHC; el filtro se despega — la explosión de innov es cascada del estado/bucle, **no** `B_nhc_too_rigid`.  
B no es falsable de forma interesante en SLALOM ideal (sideslip truth≡0); haría falta escenario con derrape/banco real.

**Siguiente:** retomar hilo cascada actitud-Z (onset@1,34 → break@1,59 → explosión@1,69–1,79) — no abrir conversación de diseño NHC-rígido desde esta evidencia.

### 13.12 Δyaw vs v_body — cierre cuantitativo (no forzar la ecuación simple)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/yaw_error_vs_vbody.{json,md,png}`

**Hipótesis a probar:** `filter_v_lat ≈ -V·sin(Δyaw)` en [1,69→1,79] latch.

| Check | Resultado |
|-------|-----------|
| Δyaw | solo **0,28→1,20°** |
| max\|-V sinΔψ\| | **0,29** m/s |
| max\|filter_v_lat\| | **1,80** m/s (~6×) |
| pearson(v_lat, −V sinΔψ) | **−0,71** (falla) |
| pearson(v_lat, filter cross-track NED) | **0,98** |
| frac var v_lat ~ cross-track | **0,96** |
| v_vert vs att-only (truth vel @ filter RPY) | pearson **0,985** (forma OK; amplitud parcial) |
| roll / pitch | roll **−3→−10°**; pitch **+3,8→−3,9°** |

**Veredicto: `CASCADE_VIA_VEL_STATE_NOT_INSTANT_YAW_PROJECTION`.**

- La ecuación simple **no cierra** — Δyaw es demasiado pequeño para la amplitud de v_lat.
- Eslabón próximo en la explosión: **vel_NED del filtro ya contaminada** (componente cross-track ≈ v_lat cuerpo); v_vert sigue la proyección por roll/pitch.
- La cascada cualitativa **sigue en pie** (A_CASCADE + bucle actitud aguas arriba); el cierre cuantitativo en t≈1,7 es *actitud→vel_NED sucia→innov*, no *Δψ instantáneo→v_lat*.
- Diseño: intervenir **antes** de que la velocidad ya esté mal (onset/early), no asumir que anular Δyaw @1,7 basta.

| t | Δyaw° | roll° | v_cross | v_lat | −VsinΔψ | v_vert |
|---|------:|------:|--------:|------:|--------:|-------:|
| 1,690 | +0,28 | −3,5 | −1,64 | −1,64 | −0,07 | +1,67 |
| 1,730 | +0,92 | −5,6 | −0,83 | −1,30 | −0,22 | −1,00 |
| 1,760 | +1,20 | −8,0 | +0,53 | +0,08 | −0,29 | −2,18 |
| 1,780 | +0,90 | −10,0 | +0,84 | +0,86 | −0,22 | −1,71 |

### 13.13 vel_NED antes de la explosión NHC [0,4→1,69]

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/vel_ned_pre_explosion.{json,md,png}`

**Pregunta:** ¿el error de `vel_NED` (esp. cross-track vs heading truth) ya crece antes de [1,69→1,79], de modo que el giro solo lo revela?

| Arm | cross 0,41→1,69 | max\|cross\| | \|e_horiz\| 0,41→1,69 |
|-----|-----------------|--------------|----------------------|
| ctrl | +0,027→+0,25 | 0,47 | 0,027→0,90 |
| latch | +0,027→**−1,64** | **1,64** | 0,027→**1,67** |

**Latch por fase (no promediar [0,4→1,69] a ciegas):**

| Fase | cross start→end | slope \|cross\| | Lectura |
|------|-----------------|----------------|---------|
| Early [0,40→1,34) | +0,027→−0,003 | ~0 | **Casi limpio** — bucle actitud temprano aún no ensucia cross-track |
| Rise [1,34→1,59) | −0,004→−0,26 | +0,86 | Empieza a ensuciarse con el estímulo angular |
| Break→explode [1,59→1,69] | −0,31→**−1,64** | **+15,5** | **Surge** en ~100 ms — ya material al abrir la ventana NHC |

**Veredicto: `VEL_NED_DIRTY_BEFORE_NHC_EXPLOSION`** — con matiz de fases.

- Sí: a t=1,69 el cross-track latch ya es **−1,64 m/s** (= el v_lat que NHC ve al entrar en la explosión). El giro **revela** contaminación preexistente.
- No: no es un goteo silencioso uniforme desde t=0,4. Early loop está casi limpio; el daño a vel_NED se concentra en **rise + sobre todo [1,59→1,69]**.
- Cadena corregida: Jacobiano NHC → bucle actitud → (tras onset) `f_va`/dinámica ensucia vel_NED → post-break **surge** de cross-track → heading/giro lo proyecta a v_body → innov NHC explota.
- Intervención: **antes o en el rise / acoplamiento actitud→vel**, no en el instante NHC @1,7. El sub-tramo [1,59→1,69] es el último eslabón silencioso (aún sin explosión de ‖y‖ NHC agregada).

### 13.14 ¿Surge multiplicativo |a| × ‖δθ‖? (f_va)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/fva_multiplicative_surge.{json,md,png}`

**Pregunta:** ¿el salto de cross-track en [1,59→1,69] es |a_corr| alta × actitud sesgada (cliff de aceleración en el giro), o un salto de error de actitud?

| Fase latch | mean \|a\|_horiz | mean ‖δθ‖ | mean \|a\|·‖δθ‖ | Δ\|cross\| |
|------------|-----------------|-----------|-----------------|-----------|
| rise [1,34→1,59] | 2,01 | **0,45°** | 0,017 | +0,26 |
| surge [1,59→1,69] | 2,54 (**1,27×**) | **3,37° (7,47×)** | 0,151 (**8,8×**) | +1,33 (**5,2×**) |

pearson(d\|cross\|/dt, \|a\|·‖δθ\|) full [1,34→1,69]: **0,97**; en surge: **0,84**.

**Veredicto: `ATTITUDE_JUMP_DRIVEN_SURGE`.**

- \|a\|_horiz truth = 3\|cos(ωt)\| **sube suave** hacia el pico@2,0 — **no** hay cliff de aceleración en 1,59.
- El factor que salta es **‖δθ‖ (~7,5×)** en la misma ventana del break de composición — coherente con dinámica de actitud/latch en el break, no con un pico nuevo de `a_corr`.
- El producto \|a\|·‖δθ\| correlaciona con d\|cross\|/dt porque **δθ domina** el producto; f_va sigue siendo la vía, pero el disparo abrupto es por el lado actitud.
- **Diseño:** no anclar la próxima intervención solo a “amortiguar f_va en \|a\| alta”. Prioridad = **error de actitud en/antes del break** (early loop / rise). Un gate condicional f_va (\|a\| alta **y** ‖δθ\| ya elevado) queda como candidato secundario a preregistrar — no hoy.

### 13.15 Mapa causal completo — early-loop (congelado cierre 2026-07-19)

Cadena de **11 eslabones**, todos con datos. **Sin intervención implementada.** Cierre: `patt_bias_g/SESSION_CLOSE_2026-07-19.md`.

1. Signo Jacobiano NHC corregido (math + FD) — noche 1.
2. Bucle refuerzo `dx_att_z` desde tick 0 (`FEEDBACK_GROWTH`, R²≈0,996) — noche 1.
3. Early [0,40→1,34): vel_NED casi limpia (§13.13).
4. Onset [1,34→1,59): actitud acumula; cross-track modesto.
5. Break: ‖δθ‖ salta (~7,5×), **pitch-dominado**; no cliff \|a\| (§13.14, §13.17).
6. Latch: `dx_att_y` pierde la inversión de signo que ctrl consigue (|dx_y|≈igual).
7. Esa pérdida rastrea `K_y0` divergiendo desde ~1,10 s — trayectoria, no Joseph in-tick (§13.19).
8. `K_y0` ← `P[ATT_Y,VEL_N]` (no S, no P_yy, no P_att cruzado puro).
9. ΔP_av ← **déficit Joseph lineal** (Σ/growth≈1,10; 86% joseph) — §13.20.
10. Contaminación → surge vel_NED (cross≈v_lat); giro → v_body → innov NHC.
11. Z cerrado (brazos exp.) → fuga `bias_gz` vía K normal (§13.3–§13.9).

**Principio estructural:** atenuar/congelar una componente de δx **después** de Ky, sin reducir H/R y recalcular Joseph, deja déficit de recorte de P que contamina canales correlacionados. Por eso fallaron b1 y H-ATT-c.

**Descartado:** reorg P_att–bias; Joseph in-tick; P_yy/S como portadores (CORR_ABS_SCALE); f_va diferencial L↔C; cliff aceleración; ×260 como interés compuesto misterioso.

### 13.16 Arranque — eslabón ‖δθ‖ @1,59 (RESUELTO en la misma sesión)

La pregunta abierta de la mañana (§13.16 borrador) se cerró en §13.17–§13.20: no es K spike; es rampa innov + pitch vía K_y0/P_av/déficit Joseph. Ver mapa §13.15.

### 13.17 Autopsia salto ‖δθ‖ [1,54→1,64] — K vs y (2026-07-19)

Artefacto: `docs/benchmarks/jacobian_imu_ab/patt_bias_g/dtheta_jump_154_164.{json,md,png}`

| Arm | ‖dx_att‖ post/pre | ‖y‖ | k_att_max | g_eff | ‖δθ‖_state |
|-----|-------------------|-----|-----------|-------|------------|
| latch | **2,41×** | **2,24×** | 1,12× | 1,08× | 2,10× |
| ctrl | 1,59× | 1,04× | **0,85×** | 1,55× | 1,66× |

Máx. salto tick a tick latch ‖dx_att‖: solo **1,20×** (no cliff).  
**Latch: `dx_att_z ≡ 0` en todos los ticks** (λ=1).

**Veredicto: `Y_JUMP_MULTI_TICK_NOT_K_SPIKE_DXZ_LATCHED_ZERO`.**

- **Falsificado:** ganancia puntual K_att (patrón ZUPT/GNSS) — k y g_eff planos.
- ‖dx_att‖ sigue a ‖y‖ en una **rampa multi-tick**, no un tick único.
- El crecimiento de ‖δθ‖_state **no** viene de updates NHC en Z (latched off); bajo latch domina **pitch** (0,69°→2,60° en la tabla; roll~0,1°). pearson(‖δθ‖_state, ‖y‖)≈**1,00**.
- Ctrl también rampa ‖δθ‖_state (roll crece ahí); latch amplifica la vía innov.

**Siguiente:** qué hace crecer **pitch**/‖δθ‖_state en [1,59→1,69] si no es `dx_att_z` NHC — predict (gyro+bias), `dx_att_x/y`, o colateral del latch.

### 13.18 Lección metodológica — `CORR_ABS_SCALE` (congelada 2026-07-19)

**Tres falsos positivos en la misma sub-investigación** (P_yy vs ΔK_y0; s_yz / S⁻¹ vs ΔK_y0; correlaciones “bonitas” con numerador ~0):

> pearson alto **≠** portador causal si la magnitud absoluta del numerador (o del denominador relevante) es despreciable frente a la escala del sistema.

**Regla operativa a partir de ahora (obligatoria en reportes de esta cadena):**

1. Toda correlación / pearson se reporta **junto con** `mean|x|`, `mean|y|`, `max|x|`, `max|y|` (y, si aplica, el umbral de materialidad usado).
2. Un pico de relΔ cerca de un cruce por cero **no** cuenta como divergencia sin `|Δ|` absoluto.
3. Si `mean|Δcandidato| ≪` escala del término hermano (p.ej. `|Δs_yz| ≪ mean|s_yy|`, `|ΔP_yy| ≪ P_yy`), el veredicto por defecto es **ruido / trampa de escala**, no “S diverge” / “P_yy porta”.

Instancias cazadas hoy: `P_yy` (relΔ&lt;10%, movimiento minúsculo); `s_yz` (relΔ~28% @ |Δ|~9e-6); correlaciones auxiliares con `|Δs_zz|` microscópico.

### 13.19 Cadena unificada candidata — f_va → P[ATT,VEL] → K_y0

Tras descartar Joseph in-tick, P_att cruzado puro, P_yy, y S (escala absoluta):

- `K_y0` diverge de forma gradual post-latch; aceleración **[1,10→1,54]** co-tiempo con `P[ATT_Y,VEL_N]` (relΔ→0,59).
- Hipótesis económica: el mismo `f_va` (predict) que ensucia vel_NED (§13.13–§14) **construye** el bloque cruzado P_att–vel; en ctrl Joseph+`dx_att_z` lo recorta; en latch no.

**Comprobación:** `tools/audit_fva_pattvel_110_154.py` — ΔP_predict vs `f_va[VN,ATT_Y]` en [1,10→1,54], con `CORR_ABS_SCALE`.

**Resultado: `FVA_NOT_PATTVEL_DRIVER` (con matiz Joseph).**

| Hecho | latch | ctrl | lectura |
|-------|-------|------|---------|
| pearson(dP_predict, f_va) | +0,44 | −0,55 | **no** porta tick-a-tick (umbral 0,70) |
| mean\|dP_predict\| / mean\|f_va\| | 1,7e-4 / 1,2e-2 | similar | escalas materiales; f_va **≈ igual** L↔C (r=0,98) |
| Σ dP_predict | +7,7e-3 | +7,1e-3 | rebuild predict **casi igual** |
| Σ dP_joseph | −7,3e-3 | −9,3e-3 | cut Joseph **más débil** en latch (ratio mean\|·\| = 0,78) |
| ΔP_pre net | **+5,0e-4** | **−1,9e-3** | ctrl recorta de más; latch acumula |

**Lectura corregida (no forzar la unificación):** `f_va` es el constructor común (mismo en ambos brazos) — no el discriminante latch↔ctrl. Lo que abre `P[ATT_Y,VN]` bajo latch es el **sub-recorte Joseph** a lo largo de la trayectoria (K/P ya divergidos; λ no edita Joseph in-tick), no un `f_va` diferencial. La vía estado (surge vel_NED vía f_va×δθ) y la vía cov (P_av → K_y0) **comparten Φ** pero el fallo del latch en cov es del lado **update/Joseph**, no del predict.

**Cerrado en §13.20:** el discriminante es el déficit Joseph (no hace falta descomponer H hoy). Intervención candidata: §13.21.

### 13.20 Presupuesto Joseph underclip — ¿explica ΔP? (2026-07-19)

Artefacto: `joseph_clip_deficit_110_154.{json,md,png}`

**Veredicto: `JOSEPH_DEFICIT_LINEAR_SUFFICES`.**

| qty | valor |
|-----|-------|
| ΔP growth (L−C) [1,10→1,54] | +2,35e-3 |
| Σ joseph deficit | +2,02e-3 (**86%**) |
| Σ predict excess | +0,58e-3 (14%) |
| Σ linear / growth | **1,10** (identidad contable de los déficits observados) |
| ×260 / ×291 | max\|ΔP\| late / onset — baseline onset ~8e-6, **no** factor compuesto misterioso |

- No hace falta motor nuevo: acumular el underclip observado **es** el ΔP.
- El déficit por tick **no** es plano (late/early ≈ **12,8×**); corr(\|def\|,\|P_l\|)=0,39 (débil) pero corr(\|def\|,\|ΔP\|)≈1 — la brecha de recorte crece con la brecha (familia FEEDBACK en la *diferencia* Joseph), sin incumplir el cierre lineal del presupuesto.
- CF: latch predict + ctrl joseph → P_end más cerca de ctrl (−1,8e-3 vs latch).

### 13.21 Preregistro H-ATT-d — Z no-observado en H (congelado 2026-07-19)

**Estado:** preregistro **CONGELADO**. Implementación solo tras confirmación explícita “adelante”.  
**Ajuste vs borrador oral:** P1 **no** usa c-L-l1=66,2 m como PASS (sigue peor que ctrl≈54 m). P1 = espíritu §11/§12.

#### Hipótesis

Excluir el acoplamiento a `ATT_Z` del modelo de observación **antes** de resolver Kalman (H reducido → S/K/δx/Joseph coherentes), en vez de Ky completo + `δx[ATT_Z]←0` post-hoc, evita el déficit de recorte Joseph que abrió `P[ATT_Y,VEL_N]` bajo H-ATT-c.

#### Contraste explícito con H-ATT-c

| | H-ATT-c (FAIL) | H-ATT-d (esta) |
|--|----------------|----------------|
| K | Sobre H **completo** (incluye columnas ATT_Z) | Sobre H **reducido** (`H[*][ATT_Z]=0`) |
| δx | Truncar `δx_z` después | `δx = K_reduced · y` sin truncar |
| Joseph | Usa K completo (incoherente con δx aplicado) | Usa el **mismo** K que produce δx |

#### Diseño de implementación (normativo)

1. **Gate:** reutilizar detector **cand1** de H-ATT-c (Σ\|dx_att_z\|, T₂, t_max=0,65). No reinventar. Control negativo: no fire en C (ya validado; reusar).
2. **Pre-gate / no fire:** H/R/S/K/Joseph **idénticos** al baseline actual (full H).
3. **Post-fire:** construir `H_reduced` = H NHC con **columnas** `ATT_Z` puestas a 0 en ambas filas de medida (lat/vert). NHC sigue siendo 2×15 — no se elimina una “fila de medida Z” (no existe); se deja de modelar sensibilidad innov←att_z.
4. Con `H_reduced` y R sin cambio: S, K, δx, Joseph — todo sobre ese H. **Prohibido** `δx_z←0` post-hoc.
5. Nota: `K[ATT_Z,:]` puede seguir ≠0 vía P cruzada; eso es coherente. No reintroducir truncado.

CLI tentativo: `--nhc-att-z-unobs` (o flag equivalente) + mismos gate args que H-ATT-c.

#### Brazos

| Brazo | Política |
|-------|----------|
| **ctrl** | gate off / unobs off — baseline jcorrect seed 71 |
| **c-L-l1** (negativo) | H-ATT-c λ=1 post-hoc (regresión conocida; referencia déficit Joseph) |
| **d-L** | H-ATT-d post cand1 (T₂ o T₅ — **fijar T₂** como brazo primario early, alineado al mapa de hoy) |

Matriz A/B/C/D completa solo si P1∧P3∧P4 pasan en SLALOM A smoke.

#### Criterios PASS/FAIL (antes de código)

| ID | Qué | Umbral |
|----|-----|--------|
| **P1** | SLALOM A lateral E2E | **≤ 2,0 m** **∧** ≥ **10×** mejor que control A del **mismo binario**. *Aspiracional no-gate: ≤ 0,15 m.* **No-PASS:** “≤ 66,2 m” (c-L-l1) — insuficiente. |
| **P2-slalom** | C y D slalom | `drift_interv ≤ 1,20 × drift_control` por celda (§11) |
| **P2-tunnel** | C y D tunnel | idem ρ=1,20 por escenario |
| **P3** | Mecanismo cov | En [1,10→1,54] s: `P[ATT_Y,VEL_N]` latch-d vs ctrl — max\|Δ\| y/o relΔ **no** muestran la divergencia ×~100 de H-ATT-c; concreto: max\|ΔP\| ≤ **5×** max\|ΔP\| onset [0,40→1,10] del mismo brazo d (o ≤ 5× max\|ΔP\| ctrl en late — el más estricto). Reportar CORR_ABS_SCALE. |
| **P4** | Déficit Joseph | Misma métrica §13.20: Σ joseph deficit (L−C) en [1,10→1,54]. **PASS:** Σ\|deficit\|_d ≤ **0,25 ×** Σ\|deficit\|_c-L-l1 (reducción sustancial). Si déficit ≈ c-L-l1 → hipótesis estructural FAIL, no solo bug de código. |
| **P3-C** | Control negativo | cand1 **no** dispara en C (racha sana) — reusar §12 |

**PASS H-ATT-d:** P1 ∧ P2-slalom ∧ P2-tunnel ∧ P3 ∧ P4 ∧ P3-C.

**No-FAIL:** E2E FAIL solo por OQ9 (t≳14 s) tras P1–P4 tempranos → H-ATT-d PASS + OQ9 residual.

#### Fuera de alcance

OQ9; atenuar f_va; zero-P; reabrir λ/δx post-hoc; cambiar cand1 thresholds sin preregistro aparte.

#### Artefacto cierre mapa

`docs/benchmarks/jacobian_imu_ab/patt_bias_g/SESSION_CLOSE_2026-07-19.md`

### 13.22 Preregistro — cand1 túnel: gracia de arranque vs escala (congelado 2026-07-19)

**Estado:** preregistro **CONGELADO**. No retocar hipótesis H-ATT-d (§13.21) ni umbrales P1–P4 hasta sanear el gate.

**Motivación (dato §13.21 barrido):** bajo H-ATT-d, cand1 en `TUNNEL_STRESS` dispara a `t_s=0.0` en A/B/C/D (sumabs/T₂ hasta ~2870× en dirty). En slalom: fuego A@0,39 s, no-fuego C. El fallo P2-tunnel no certifica fuga estructural de unobs — certifica **falso positivo de arranque en frío** del detector calibrado solo en slalom. Latch único + unobs permanente ≠ ruido a lo largo del run.

**Nota de reloj:** `t_s` del fire es tiempo desde el **primer NHC con gate activo**, no tiempo de escenario. En túnel NHC solo arma con GPS outage (~10 s escenario) → `t_s=0` = onset NHC post-outage, no seed del filtro.

**Principio:** mismo que ZUPT/NHC condicional ([17](17-conditional-constraints-architecture.md)): no evaluar el gate en el transitorio de cold-start; condicionar a estado del sistema, no umbral absoluto ciego entre escenarios.

#### Hipótesis de saneamiento (dos brazos, E1 prioritario)

| Brazo | Intervención en el **detector** (acción H-ATT-d sin cambio) |
|-------|--------------------------------------------------------------|
| **E1** | Periodo de gracia: durante los primeros **N** ticks NHC de *cualquier* escenario, **no acumular** `sumabs` y **no evaluar** latch. Tras gracia, cand1 idéntico (T₂, t_max). |
| **E2** | Umbral normalizado: latch si `sumabs / scale ≥ κ`, con `scale` = proxy local de arranque (p.ej. `P[ATT_Z,ATT_Z]` al primer NHC post-gracia o media móvil corta). κ fijado para reproducir fuego slalom A@~0,39 s en ctrl-scale. |

**Fijación de N / κ (antes de scorecard):** con auditorías **ctrl** (gate off) slalom+túnel A/B/C/D — reconstruir Σ\|dx_att_z\| y comparar `sumabs`, `sumabs/P_zz`, y cuántos ticks dura el régimen anómalo de túnel. E1 prioriza el N mínimo que excluye ese régimen; E2 solo si la normalización hace escalas comparables.

#### Criterios PASS/FAIL (solo gate; sin reabrir P1/P4)

| ID | Qué | Umbral |
|----|-----|--------|
| **G1** | Túnel A/B/C/D | **ningún** fuego en arranque frío: `fire` ausente **o** `t_fire > t_grace` con `t_fire` no en el primer tick post-gracia si ese tick sigue siendo el spike (práctico: **no fire en t≤0,05 s** en ninguna celda túnel bajo d-unobs+gate saneado) |
| **G2** | Slalom A | fuego @ **0,39±0,02 s** (no-regresión vs §13.21) |
| **G3** | Slalom C | **no fire** (P3-C) |

**PASS brazo:** G1 ∧ G2 ∧ G3.  
**Siguiente solo si ≥1 brazo PASS:** re-correr P2-tunnel (+ scorecard H-ATT-d) con ese gate; entonces autopsia P4 sobre datos limpios.  
**Si ambos FAIL:** no subir T₂ a ciegas; documentar y proponer exclusión explícita de evaluación en túnel hasta nuevo diseño.

#### Fuera de alcance

Cambiar acción unobs; retocar P1; autopsia Joseph/P4; OQ9; recalibrar T₂ sin mirar escala.

#### Resultado barrido (2026-07-19) — ambos brazos FAIL (gate no saneado)

Artefactos: `docs/benchmarks/jacobian_imu_ab/cand1_gate_e12/`.

| Hallazgo escala (ctrl) | |
|--|--|
| Reloj fire | `t_s` = epoch NHC; túnel onset @ escenario ~10 s (GPS outage), no seed |
| Pzz onset túnel | ~0,88 vs slalom t0 ~0,030 (~29×) |
| sumabs/Pzz t0 túnel A | ~1,7e-5 ≪ ratio slalom@fuego (~8e-4) — escalas **no** O(1) |
| Dirty B/D t0 | sumabs/T₂ ~2870×; sumabs/Pzz ~0,012 ≫ κ — normalizar no basta |
| Pzz tick2 | colapsa 0,88→~5e-4; norm con Pzz corriente es tóxica → E2 congela Pzz post-gracia |

| Brazo | G1 early | G1b (nofire≤tmax) | G2 A@0,39 | G3 C | PASS |
|-------|----------|-------------------|-----------|------|------|
| E0 | FAIL | FAIL | PASS | PASS | FAIL |
| E1 N=1 | FAIL | FAIL (A/B/D); **C nofire** | PASS | PASS | FAIL |
| E1 N=32 | PASS* | FAIL @0,32 s | FAIL @0,44 | PASS | FAIL |
| E2 freeze-Pzz | FAIL | FAIL; **C nofire** | PASS | PASS | FAIL |
| E1n1+E2 | FAIL | FAIL | FAIL | FAIL | FAIL |

\*G1 early con N=32 es loophole: solo retrasa el falso positivo; A/B siguen con \|dx_z\|≳T₂ cada tick post-gracia.

**Conclusión:** ni gracia ni umbral normalizado cierran G1∧G2∧G3. El spike de 1 tick explica C legacy; A/B jcorrect en túnel tienen régimen sostenido distinto de slalom. **No** re-correr P2-tunnel / P4 hasta decisión de diseño (p.ej. no evaluar cand1 en túnel, o detector por escenario). H-ATT-d intacta.

**Cierre de instrumento (no de hipótesis):** `docs/benchmarks/jacobian_imu_ab/CAND1_GENERALIZATION_REVIEW.md` — cand1 no es invariante entre dominios; pérdida de generalidad en operacionalización/gate, no (aún) en H-ATT-d.

**Patrón de programa + pausa:** [OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md](reference/OPERATIONALIZATION_FAILURES_DESIGN_PATTERN.md) (Caso B) · **D22** — no abrir OQ8 experimental / cand2 de inmediato.
