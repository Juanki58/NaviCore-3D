# 21 — H-seed-yaw (H2): velocidad + yaw alineado al course

**Estado:** preregistro **CONGELADO** — 2026-07-20  
**Precede:** H-seed-v H1 ([20-h-seed-v-protocol.md](20-h-seed-v-protocol.md))  
**No implementar hasta “adelante”.**

H2 **no** se vende como “la solución”. Es el experimento natural tras H1.  
Cautela: forzar `yaw:=course` puede **borrar el síntoma** y ocultar falta de observabilidad / marco / init de actitud.

---

## 1. Lectura discriminante de H1 (congelada)

> La inyección de velocidad GNSS elimina completamente la incoherencia de **magnitud** de la velocidad del estado, pero **no** restaura la coherencia cinemática, ya que la **actitud permanece incompatible** con la dirección del movimiento.

| Componente | H1 |
|------------|-----|
| Velocidad (magnitud vs GNSS) | ✔ (P1: fail_frac 1→0) |
| Actitud (course−yaw) | ✘ (P3 empeora) |

---

## 2. Auditoría previa (obligatoria — hecha)

Pregunta: ¿de dónde sale yaw en el instante de la inyección H1 (t≈4.301 s)?

| Pregunta | Respuesta (evidencia) |
|----------|------------------------|
| ¿Solo IMU? | Tras init: **sí** — integración `predict` (gyro) |
| ¿Estado inicial? | `seed_from_ned_pos` → `ins_ekf_init(..., yaw=0)` |
| ¿Congelado en 0? | **No** — @ inyección yaw ≈ **−2.06°** (ctrl=H1 idénticos pre-inyección) |
| ¿Espera observabilidad? | `yaw-init=zero` en baseline: **no** hay H2/H3 heading seed |
| ¿Otro marco? | Sin indicios en este instante; discrepancia es **init 0 + drift IMU** vs course GNSS ≈94° |

Trayectoria yaw (H1=ctrl hasta inyección): 0° @2.67 → ~−2° @4.30.  
Post-inyección H1: yaw se mueve con fuerza (p.ej. ~26° @4.303) — probable acoplamiento NHC/actitud con `|v|` ya grande; **fuera del alcance de H2** salvo métricas secundarias.

---

## 3. Hipótesis H-seed-yaw (H2)

> La incoherencia cinemática **inmediata** tras H1 proviene de sembrar **solo** la velocidad, dejando una actitud (yaw) incompatible con el rumbo del movimiento.

**No afirma:** que yaw forzado cure la deriva kilométrica ni sustituya un init de actitud observable.

---

## 4. Intervención única

Misma base que H1 (`--seed-velocity gnss`): mismo instante, misma posición, misma `v←speed·(cos,sin)course`.

**Único añadido:** en ese mismo shot, `yaw := course` preservando roll/pitch  
(reutilizar `set_ekf_yaw_preserve_roll_pitch` ya existente en replay).

| Brazo | Cambio |
|-------|--------|
| **ctrl** | Baseline (`seed-velocity zero`) |
| **H1** | Solo v←GNSS (ya corrido) |
| **H2** | v←GNSS **y** yaw:=course en el mismo shot |

Prohibido: NHC, predict, Q/R, gates, cov, cambiar mount.

CLI previsto: `--seed-velocity gnss --seed-yaw-from-course` (o `--seed-attitude course`).

---

## 5. Pack / corrida

Idéntico a §20 (G-ext baseline, 0–10 s, métricas 0–6 s).  
Salida: `docs/benchmarks/h_seed_v/{ctrl,H1,H2}/`.

---

## 6. Criterios (congelados antes de H2)

### Gates

| ID | Métrica | PASS H2 |
|----|---------|---------|
| **P1** | `speed_vs_gps` fail_frac (B, 0–6 s) | ≤ 0.30 (como H1) |
| **P3a** | Primer OK→FAIL `course_yaw` en A tras el shot de seed | **ausente** en [t_seed, t_seed+2 s] |
| **P3b** | Primer FAIL `course_yaw` en 0–6 s | ausente **o** ≥ ctrl+2 s (como P3 de §20) |

### Secundarios (no gates)

| ID | Métrica |
|----|---------|
| S1 | `t_sep` residual>30 m |
| S2 | residual @ t=6 s |
| S3 | ¿yaw post-seed permanece cerca de course o NHC lo arrastra? (diagnóstico) |

### Veredicto

| Resultado | Lectura |
|-----------|---------|
| P1∧P3a∧P3b | H2 reforzada: faltaba alinear actitud en el seed |
| P1∧P3a, P3b falla luego | Coherencia inmediata OK; ruptura posterior = otro mecanismo |
| P3a falla | Forzar yaw no basta / se reescribe al instante (p.ej. NHC) |
| Residual km | No-gate |

---

## 7. Cautela metodológica (A ≠ B)

| | |
|--|--|
| **A** | H2 mejora el régimen inicial |
| **B** | “El algoritmo correcto debe inicializar yaw:=course” |

**No son equivalentes.** A demuestra que la **coherencia inicial** actitud–velocidad importa.  
B depende de sensores/hipótesis (GNSS course válido, vehículo en movimiento, sin mag, etc.).

Si H2 funciona espectacularmente → el régimen dependía críticamente de esa incoherencia.  
Si funciona solo parcialmente → v importaba, yaw importaba, **falta otro mecanismo**.  
Tras H2: comparar la tabla Control / H1 / H2 — **no** saltar a H3.

Si H2 “limpia” course−yaw solo porque **imponemos** la actitud, eso valida necesidad de coherencia al arranque; **no** observabilidad de yaw en vuelo.

---

## 8. Enlace

H1: [20-h-seed-v-protocol.md](20-h-seed-v-protocol.md) · Programa: [`docs/ekf_explorer/RESEARCH_PROGRAM.md`](../ekf_explorer/RESEARCH_PROGRAM.md)
