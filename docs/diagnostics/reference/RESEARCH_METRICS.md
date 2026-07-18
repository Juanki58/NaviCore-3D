# Research Metrics — Indicadores del programa

**Tipo:** documentación de referencia viva — **métricas del proyecto**, no del EKF.  
**Última actualización:** 2026-07-18 (post G-ext / D13)  
**Regla:** actualizar solo al cerrar una fase o congelar un K/OQ nuevo. Cada nueva fase debe mover al menos uno de estos números (o justificar por qué no).

---

## 1. Resumen (cantidad + calidad)

| Categoría | Indicador | Valor |
|-----------|-----------|------:|
| Conocimiento consolidado | `K*` en STATE_OF_KNOWLEDGE | **15** |
| Hipótesis refutadas | framing falsificado (no reintentar) | **11** |
| Preguntas abiertas | `OQ*` Open | **7** |
| Validaciones independientes | recorridos distintos, mismo shell mecanicista | **2** (G1, G-ext) |
| Prerregistros congelados | protocolos fijados *antes* de ejecutar | **3** |
| Experimentos reproducidos | reejecución bajo protocolo fijo / twin de control | **4** |

```
X = 15   consolidado
Y = 11   refutado
Z =  7   abierto
V =  2   validaciones independientes de trayectoria
P =  3   prerregistros congelados
R =  4   experimentos reproducidos / twins
```

Las tres primeras filas miden **stock de conocimiento**.  
Las tres últimas miden **calidad del proceso**: no solo acumuláis resultados — los congeláis y los reproducís.

---

## 2. Regla dura — todo experimento cita un OQ*

> **Ningún experimento nuevo debería existir sin responder explícitamente a una pregunta abierta.**

| Pregunta prohibida | Pregunta obligatoria |
|--------------------|----------------------|
| «¿Qué vamos a probar?» | **«¿Qué OQ responde?»** |

Si no responde a ningún `OQ*`, **no** abrir fase nueva (D11).  
Si propone reabrir un `K*`, requiere evidencia nueva + entrada en [DECISION_LOG.md](DECISION_LOG.md).

---

## 3. Detalle de indicadores de calidad

### V — Validaciones independientes (trayectoria)

| # | Dataset | Rol |
|---|---------|-----|
| 1 | G1 (`real_run` histórico / `gap4_gnss_velocity/G1`) | Recorrido de descubrimiento del mecanismo |
| 2 | G-ext (`19082026` / `real_run_19082026_baseline`) | Validación externa del **núcleo de bloqueo** (K14/K15) |

**Claim permitido:** el mecanismo reaparece en un recorrido independiente; otros aspectos no.  
**Claim prohibido:** «G-ext confirma G1» de extremo a extremo.

### P — Prerregistros congelados

| # | Protocolo | Tag / ancla |
|---|-----------|-------------|
| 1 | GAP-5 v1 (Γ̄ / NHC adaptativo) | `gap5-preregistration-frozen` |
| 2 | GAP-5 v2 (observable / régimen) | `gap5-v2-observable-preregistration-v1.2` |
| 3 | GAP-4 §11 intervención `P_pv` | preregistrado en doc 13; **no ejecutado** (OQ7) |

Tags históricos de la misma línea v2 (`…-frozen`, `…-v1.1`) no suman campañas distintas.

### R — Experimentos reproducidos / twins

| # | Qué | Evidencia |
|---|-----|-----------|
| 1 | G0 baseline (reproducibilidad pos-only) | `tools/run_gap4_g0_baseline.py` / `G0/` |
| 2 | G1 control twin `ppv=none` | `G1_control_full_ppv_none/` |
| 3 | G-ext = mismo shell G1, otro CSV | `real_run_19082026_baseline/` + `run_gext_19082026_baseline.py` |
| 4 | GAP-5 v1 f1-bridge vs PoC config | `p0_passive_f1_bridge` / `p0_passive_validation` |

---

## 4. X / Y / Z (stock)

### X — Cerrados / confirmados (K1–K15)

| Bloque | IDs |
|--------|-----|
| Núcleo NHC / gate / Γ offline | K1–K7 |
| `P_pv` / bifurcación / logs | K8–K10 |
| Γ operacional / invarianza | K11–K13 |
| G-ext | K14–K15 |

### Y — Refutadas (11)

F1 dose→accepts · F1.1 restore K · F1.2 decimation · ZUPT-only · Joseph-only · cliff=frecuencia pura · `P_pv`=bug · retune solo R/Q/K · Γ̄ operacional · retune umbral Γ̄ · «G-ext confirma G1 completo».

### Z — Abiertas (OQ1–OQ7)

OQ1–OQ4 → GAP-5 v2 · OQ5–OQ6 → v3+ · OQ7 → GAP-4 §11 (separado).

Detalle completo: [STATE_OF_KNOWLEDGE.md](STATE_OF_KNOWLEDGE.md), [OPEN_QUESTIONS.md](OPEN_QUESTIONS.md), [RESEARCH_MAP.md](RESEARCH_MAP.md).

---

## 5. Serie temporal (reconstruida — convergencia)

**No es auditoría forense** de cada commit. Es una **reconstrucción aproximada** para vigilar si el programa madura como se espera:

| Hito | K (X) | Refutadas (Y) | OQ (Z) | Nota |
|------|------:|--------------:|-------:|------|
| Pre GAP-3 | ~0 | ~0 | alta | exploración |
| Cierre GAP-3 + F1 | ~7 | ~6 | ↓ | mecanismo NHC/gate |
| Cierre GAP-4 | ~10 | ~8 | ↓ | `P_pv` legítimo |
| Cierre GAP-5 v1 | ~13 | ~10 | ~8 | Γ̄ refutada operacionalmente |
| **Post G-ext (ahora)** | **15** | **11** | **7** | validación externa; pausa D13 |

Tendencia esperada en un programa maduro:

- **X crece** (conocimiento consolidado);
- **Y crece** (falsar es progreso, no fracaso);
- **Z baja o se refina** (preguntas más nítidas, no más ruido);
- **V y R crecen** cuando hay generalización real.

Si dentro de ~6 meses esa tendencia se mantiene, es evidencia de **convergencia** — no solo de más CSV.

*(Serie detallada fecha-a-fecha: opcional; no mezclar con el resumen §1 hasta tener snapshots fechados al cerrar cada fase.)*

---

## 5b. Diferido — cobertura del espacio de hipótesis (post GAP-5 v2)

**No añadir ahora.** Cuando H6 / GAP-5 v2 cierre, complementar K/OQ con una vista por *área del sistema* (distinta de la cronología GAP-n):

| Área | Estado (ejemplo de forma) |
|------|---------------------------|
| Dinámica de covarianza | consolidada |
| Gate GNSS | consolidada |
| Acoplamiento `P_pv` | consolidada |
| Identificación de régimen | en curso → actualizar al cierre v2 |
| Política de control | no iniciada (v3+) |

Pregunta que responde (≠ X/Y/Z): **¿qué partes del sistema entendemos y cuáles siguen siendo cajas negras?**  
Útil para un lector nuevo; no sustituye V/P/R ni la regla OQ*.

---

## 6. Ciclo del programa

```
Observación → Hipótesis → Prerregistro → Instrumentación
      → Falsación → Congelación → Nueva pregunta
```

El software es herramienta para este ciclo. Un experimento «que sale mal» y aumenta Y o aclara Z **aumenta** el conocimiento disponible.

---

## 7. Cómo actualizar

| Evento | Qué tocar |
|--------|-----------|
| Nuevo `K*` | X += 1; §1 + changelog |
| Hipótesis falsificada congelada | Y += 1 |
| Nueva `OQ*` justificada | Z += 1 solo si no cubierta |
| Nueva validación de trayectoria independiente | V += 1 |
| Nuevo prerregistro con tag/doc pre-ejecución | P += 1 |
| Twin / reejecución protocolizada | R += 1 |
| Cierre de fase | fila en serie §5 + changelog |

### Changelog

| Fecha | X | Y | Z | V | P | R | Nota |
|-------|---|---|---|---|---|---|------|
| 2026-07-18 | 15 | 11 | 7 | — | — | — | Primera página X/Y/Z |
| 2026-07-18 | 15 | 11 | 7 | 2 | 3 | 4 | + calidad V/P/R; serie temporal reconstruida |
| 2026-07-18 | 15 | 11 | 7 | 2 | 3 | 4 | Evidence Review: OQ1 sigue abierta (D14/D15); Z sin cambio |
| 2026-07-18 | 15 | 11 | 7 | 2 | 3 | 4 | Strength audit D17; OQ3/OQ5 acotadas; Z=7 (ninguna eliminada); pausa cerrada |
