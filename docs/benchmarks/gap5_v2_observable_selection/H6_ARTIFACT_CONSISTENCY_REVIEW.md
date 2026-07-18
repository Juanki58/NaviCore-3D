# H6 — Revisión de consistencia de artefactos (D19)

**Fecha:** 2026-07-18  
**Estado:** congelada; alimenta [`regime_model.md`](regime_model.md)  
**Fuente:** [`observable_characterization.json`](observable_characterization.json) / [`.md`](observable_characterization.md)  
**Protocolo:** v1.2 · Paso 0 · bindings D18  

**Modo:** no ranking · no ganador · OQ1 no se decide aquí.

---

## Resultados del benchmark (sólidos — no son aún el modelo de régimen)

### B1 — El protocolo ha discriminado candidatos

Escenario **A** (§5.1: todos los Oi con la misma caracterización) queda **descartado**.

H6 aportó información: el diseño experimental distingue comportamientos distintos entre familias de propiedad (consumo Γ, estructura P_pv, consistencia Λ, dΛ/dt).

### B2 — Colinealidad parcial O1–O2

O1 y O2 producen el **mismo vector C7** en C-F1 y en C-PoC.

Esto **no** afirma que sean el mismo observable físico. Afirma que, con las dimensiones C1–C7 y las dos configuraciones estudiadas, **no pueden diferenciarse completamente**.

Es un resultado **metodológico** (poder discriminativo del benchmark), no necesariamente una conclusión sobre la física del filtro.

---

## Vocabulario Paso 0 (inequívoco)

| Estado | Significado |
|--------|-------------|
| **Evaluada** | La hipótesis pudo contrastarse con los datos del artefacto |
| **Refutada** | Los datos contradicen la hipótesis preregistrada |
| **No refutada** | Los datos son compatibles con la hipótesis |
| **Inconclusa** | La evidencia no permite decidir |

Una marca **no** significa «confirmada». No existe estado «confirmada» en esta hoja.

---

## Limitaciones (dos grupos)

### L-datos — Limitaciones de datos

- Baja densidad GNSS en O4/O5 en ventanas tempranas  
- Celdas con `n_samples ≈ 1` (R0–R2 para O4/O5)  
- O5·R0 = N/A (derivada requiere ≥2 samples)

### L-ops — Limitaciones operativas del benchmark

- Binding temporal C7: O1·R1 aparece como **alto** pese a elevación numérica clara (no como **pico**)  
- Discretización ordinal de C7 (`bajo|pico|alto|meseta|N/A`)

---

## 1. Perfiles C7 y vectores C

### 1.1 Vectores C7 ordinales (R0→R4)

| Oi | C-F1 | C-PoC |
|----|------|-------|
| **O1** | bajo, alto, alto, bajo, bajo | bajo, alto, alto, bajo, meseta |
| **O2** | bajo, alto, alto, bajo, bajo | bajo, alto, alto, bajo, meseta |
| **O3** | bajo, meseta, meseta, alto, pico | bajo, meseta, bajo, alto, pico |
| **O4** | meseta, meseta, meseta, bajo, pico | meseta, meseta, meseta, bajo, pico |
| **O5** | N/A, bajo, alto, pico, alto | N/A, bajo, alto, pico, alto |

### 1.2 Discriminación

| Par | ¿Mismo C7 en ambas configs? |
|-----|------------------------------|
| O1 vs O2 | **Sí** → B2 |
| O1/O2 vs O3, O4, O5 | No |
| O3 vs O4 vs O5 | No entre sí |

Escenario B **global** (todos colineales) descartado. Escenario B **local O1–O2** afirmado (= B2).

### 1.3 C1–C6 (resumen)

C1/C2/C4 discriminan entre Oi. C5/C6 son T por construcción causal — no usan como superioridad.  
C4(O2)=T solo en C-F1 (memoria vs O1); en C-PoC no se dispara.  
C2(O2)=T por predicción `unknown` en Paso 0 — no es evidencia positiva de coherencia R2.

---

## 2. Paso 0 — estado por observable

Criterio: columna de refutación del Paso 0 + JSON. El mapa R0–R4 orientativo (§5.2) es predicción de forma, no la hipótesis núcleo.

| Oi | ¿Evaluada? | Estado | Nota breve |
|----|:----------:|--------|------------|
| **O1** | Sí | **No refutada** | C1+C3 compatibles; elevación R1 numérica clara |
| **O2** | Sí | **Refutada** | Operacionalización/memoria (C4 C-F1; alineado v1); C7≡O1 |
| **O3** | Sí | **Inconclusa** | No colapsa (C3); mapa R3/R4 no coincide con predicción |
| **O4** | Sí | **Refutada** | Plano en R2/R3 vs «crece en R2»; fuerza **rebajada** por L-datos |
| **O5** | Sí | **Inconclusa** | Forma R2/R4 no decide; L-datos + R0 N/A |

### 2.1 Mapa R0–R4 (forma) — contestado, no confundir con §2

| Oi | Contrastes relevantes | Lectura |
|----|----------------------|---------|
| O1 | R1 `flat_vs_peak`; R2 `higher` | Forma ≠ tabla §5.2; núcleo no refutado |
| O3 | R3 `flat_vs_peak`; R4 `peak_vs_flat` | Mapa crucero anti-predicción |
| O4 | R2/R3 `lower`; R4 `peak_vs_flat` | Anti-mapa |
| O5 | R2 `flat_vs_peak`; R4 `higher` | Transición/meseta no como se predijo |

---

## 3. H7-MIN / mono-Oi (posición fija pre-síntesis)

**Frase congelada (D19):**

> Los resultados disponibles **no permiten concluir** ni que un observable individual sea **suficiente** ni que un conjunto mínimo sea **necesario**.

H7-MIN permanece **abierta**: ni debilitada ni reforzada artificialmente.

Cobertura por régimen (§6) — solo para informar la síntesis, no para cerrar H7-MIN:

| Régimen | ¿Marca clara en ambas configs? |
|---------|--------------------------------|
| R1 | O1 sí (C1); O2 misma etiqueta C7, ops más débil |
| R2 | No claro (O4 plano; n≈1) |
| R3 | O3 parcial (alto ≠ pico predicho) |
| R4 | O1 parcial; O3/O4 anti-predicción |

---

## 4. Enlaces

| Artefacto | Path |
|-----------|------|
| Modelo de régimen | [regime_model.md](regime_model.md) |
| Caracterización | [observable_characterization.md](observable_characterization.md) |
| Paso 0 | [paso0_property_justification.md](paso0_property_justification.md) |
| Protocolo | [16-gap5-v2-observable-selection.md](../../diagnostics/16-gap5-v2-observable-selection.md) |
