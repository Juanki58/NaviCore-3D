# H6 Preflight — ¿Ejecutable literalmente sin decidir en vuelo?

**Fecha:** 2026-07-18  
**Protocolo:** [16-gap5-v2-observable-selection.md](../16-gap5-v2-observable-selection.md) v1.2  
**Paso 0:** [paso0_property_justification.md](../../benchmarks/gap5_v2_observable_selection/paso0_property_justification.md)  
**Modo:** solo búsqueda de decisiones no preregistradas — sin código de benchmark aún en esta hoja.

---

## Checklist (10 minutos)

| Elemento | ¿Fijado en v1.2 / Paso 0? | Evidencia | ¿Decisión en vuelo? |
|----------|---------------------------|-----------|---------------------|
| Candidatos O1–O5 | **Sí** | Paso 0 tabla (O6 prohibido sin v1.1) | No |
| O2 (Γ̄) aún en catálogo | **Sí** (con nota v1 parcial) | Paso 0 | Caracterizar; no reabrir umbrales v1 |
| Preguntas C1–C7 | **Sí** | §5.2 | No |
| Dimensiones I1–I3 / S* | **Sí** | §8 | No score durante análisis |
| Configs C-F1 / C-PoC | **Sí** | §7 pos-only vs pos_vel+p_pv none | No |
| Regímenes R0–R4 marcadores | **Sí** (cualitativos) | §6 | Ventanas temporales: ver § Gaps |
| Criterios falsación H6-OBS / H7-MIN | **Sí** | §2, §9 | No |
| Outcomes válidos (1 obs / H7-MIN / ninguno / colapso B) | **Sí** | §0, §5.1 A/B/C | No |
| Formatos salida | **Sí** | §10 JSON + regime_model.md + figures | No |
| Prohibiciones mid-run | **Sí** | §0.2, §8, §11 | No |
| Resultado inconcluso | **Sí** | Escenario B §5.1; «ninguno cumple I2» §9 | No |
| Script nombrado | **Previsto, no existe** | §10 `audit_gap5_v2_observable_selection.py` | Implementar = ingeniería del protocolo, no ciencia nueva |

---

## Gaps (honestidad)

### G1 — Etiquetas ordinales C7 `{bajo\|pico\|alto\|meseta\|N/A}`

**No hay umbrales numéricos** en v1.2. Riesgo clásico: «cuando veamos el gráfico, decidimos si es pico o alto».

**Binding de ejecución (no cambia O1–O5 ni C1–C7):**  
[`c7_labeling_binding.md`](../../benchmarks/gap5_v2_observable_selection/c7_labeling_binding.md) — congelado **antes** de mirar figuras H6.

Procedimiento obligatorio:

1. Escribir **solo** estadísticas numéricas por (Oi × config × R) en JSON.  
2. **Después**, aplicar el binding para rellenar ordinales.  
3. Prohibido asignar ordinales “a ojo” antes de tener el JSON numérico escrito.

### G2 — Rutas exactas de logs C-F1 / C-PoC

§7 define configs; §10 dice «logs existentes». No nombra paths.

**Binding de datos (implícito por artefactos ya congelados GAP-5 v1):**

| Config | Fuente canónica |
|--------|-----------------|
| **C-F1** | `docs/benchmarks/gap5_adaptive_nhc/p0_passive_f1_bridge/` (`cov_step_audit.csv`, `gnss_nis_audit.csv`, `controller_audit.csv` si aplica) |
| **C-PoC** | `docs/benchmarks/gap5_adaptive_nhc/p0_passive_validation/` (mismos audits) |

No usar G-ext como brazo de caracterización H6 (addendum §12b = contexto, no nuevo candidato/brazo).

### G3 — Ventanas R0–R4 en tiempo absoluto

Marcadores §6 vía fix#2/#3/#4. Timestamps salen de `gnss_nis_audit` accepts del log de cada config (misma regla F1). Si falta fix#4 en C-F1 → **R3 = N/A** (vocabulario ya permitido).

### G4 — Definiciones de Oi

| Oi | Ancla de fórmula (ya existente) |
|----|----------------------------------|
| O1 Γ_inst | Doc 14 §3.1 / F1: Σ\|ΔP_vv\|_NHC / Σ\|ΔP_vv\|_predict en ventana corta |
| O2 Γ̄ | EWMA/media τ=1 s de O1 (doc 14) — **solo caracterizar**, no retune umbrales |
| O3 ‖P_pv‖/P_vv | `P_pv_frob / P_vv_frob` en cov_step (pre GNSS o post NHC — **fijar en binding:** usar `gnss` phase `pre` y, entre fixes, último `nhc` post por tick) |
| O4 Λ_N | \|innov_n\|/√s_nn en `gnss_nis_audit` (F1.1) |
| O5 dΛ_N/dt | Diferencia finita de Λ_N entre updates GNSS sucesivos |

Detalle operativo de series → [`h6_series_binding.md`](../../benchmarks/gap5_v2_observable_selection/h6_series_binding.md).

---

## Veredicto preflight

| Campo | Valor |
|-------|-------|
| ¿Aparece «lo decidiremos al ver gráficos» en ciencia (candidatos/C/falsación)? | **No** |
| ¿Hay huecos de operacionalización? | **Sí** — C7 ordinales, paths, series (G1–G4) |
| ¿Bloquea H6? | **No**, si se congelan bindings **antes** de ejecutar |
| ¿Modificar protocolo v1.2? | **No** — bindings de ejecución aparte |
| Siguiente | ~~Congelar bindings (D18) → script → caracterización~~ **hecho** → **síntesis solo al final** (`regime_model.md`) |

**Ideas nuevas durante H6:** aparcar en [`IDEAS_DURING_H6.md`](../../benchmarks/gap5_v2_observable_selection/IDEAS_DURING_H6.md) — **nunca** al experimento en curso.

---

## Cuatro desenlaces esperados (recordatorio)

1. Un observable representa la propiedad.  
2. Dos (o más) son complementarios.  
3. Ninguno basta solo → H7-MIN.  
4. Todos equivalentes / propiedad mal formulada.  

Los cuatro son éxito científico. Solo (1) no cuenta como “éxito”.
