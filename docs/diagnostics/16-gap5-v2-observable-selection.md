# GAP-5 v2 — Selección del observable de régimen (preregistración)

**Estado:** **CONGELADA** (v1.0) — tag `gap5-v2-observable-preregistration-frozen`  
**Fecha congelación:** 2026-07-18  
**Prerequisito:** [15-gap5-passive-outcome.md](15-gap5-passive-outcome.md) (v1 **CERRADA**)  
**Paso 0 congelado:** [paso0_property_justification.md](../benchmarks/gap5_v2_observable_selection/paso0_property_justification.md)  
**PoC controlador / RMSE / NHC activo / discusión de políticas:** **prohibidos** en toda GAP-5 v2

---

## 0. Arquitectura de fases (congelada como principio)

```
diagnóstico
      ↓
propiedad del filtro
      ↓
observable (operacionalización causal)
      ↓                    ↘
validación del observable   controlador (fase posterior)
      ↓                    ↘
caracterización              validación del controlador (O/D, RMSE…)
```

**Regla:** si el observable es incorrecto, ningún ajuste de umbrales o constantes de tiempo salvará el diseño. El controlador será casi una **consecuencia** de la elección del observable — no al revés.

### 0.1 Independencia observable ↔ controlador (principio general)

> **La validación de un observable es independiente de la utilidad del controlador construido sobre él.**

Un observable puede:

- representar **correctamente** una propiedad interna del filtro, y
- ser **inútil** como señal de control en tiempo real (memoria, latencia, dwell, palanca inadecuada),

**sin que eso invalide el observable.** Son preguntas científicas distintas.

| Pregunta | Fase | Ejemplo de error a evitar |
|----------|------|---------------------------|
| ¿El observable identifica el régimen? | GAP-5 v2 | — |
| ¿La operacionalización preserva la propiedad online? | Passive post-v2 | Confundir Γ offline (válido en F1) con Γ̄ (operacionalización refutada) |
| ¿El controlador mejora O1–O3? | GAP-5 v3+ | «Λ_N no mejoró RMSE → Λ_N era mal observable» |

**Corolario:** un observable **válido** puede no producir **nunca** una transición de controlador — y eso no es fallo del observable. Γ_inst ilustró la propiedad (consumo de P); Γ̄ falló como operacionalización de control; ambas cosas pueden ser ciertas a la vez.

**Prohibido en fases posteriores:** usar RMSE, accepts o éxito del PoC como retroactiva falsación del observable sin repetir la caracterización v2.

### 0.2 Regla estricta — solo observables (toda GAP-5 v2)

> **Durante toda GAP-5 v2 queda prohibido discutir políticas de control.**

Solo se habla de **observables** (propiedades, operacionalizaciones, caracterización, invarianza).

| Permitido en v2 | Prohibido en v2 |
|-----------------|-----------------|
| ¿Qué observa O3? | «O3 serviría para bajar NHC cada 5 ticks» |
| ¿Conserva significado en C-PoC? | Umbrales, dwell, histeresis |
| ¿Refuta la hipótesis del Paso 0? | RMSE, accepts, PoC B0/B1/P0 |
| Conjunto mínimo de observables (H7-MIN) | Palanca de actuación NHC |

Esta restricción evita mezclar dos niveles de abstracción que v1 demostró ser distintos. Parece artificial; es **disciplina metodológica**.

### 0.3 Arco de preguntas (cambio de nivel en v2)

| Fase | Pregunta | Naturaleza |
|------|----------|------------|
| GAP-3 | ¿Qué está comprimiendo la covarianza? | Mecanismo |
| GAP-4 | ¿Qué papel juega realmente P_pv? | Mecanismo |
| GAP-5 v1 | ¿Podemos controlar el régimen con Γ̄? | Ingeniería → **no** (operacionalización refutada) |
| **GAP-5 v2** | **¿Qué propiedad interna define un régimen?** | **Modelización / identificación** |

GAP-5 v2 ya no es pregunta de ingeniería («¿qué parche?») sino de **identificación de sistema dinámico**: ¿qué estado interno merece observarse?

**Cambio de nivel del proyecto:** la pregunta ya no es «¿por qué falla el EKF?» sino «**¿qué propiedad del filtro representa el cambio de régimen?**». No se depura una implementación; se construye un modelo explicativo del sistema.

---

## 1. Pregunta única de la fase

> **¿Qué propiedad del filtro queremos observar para decidir que el régimen ha cambiado?**

**No es pregunta de esta fase:**

| Fuera de alcance | Por qué |
|------------------|---------|
| «¿Cuál es el **mejor** observable?» | Induce ranking; ver §2 |
| Cómo controlar | Requiere propiedad + operacionalización validadas |
| Qué umbral | Instancia controlador, no observable |
| Qué política NHC | GAP-5 v3+ (post-observable) |
| RMSE / accepts / NIS como criterio | Confunde selección de observable con éxito del filtro |

---

## 2. Hipótesis científica (v2) — sin ranking

> **H6-OBS:** Existe al menos una **propiedad interna del EKF** (medible en replay sin modificar el filtro) cuya observación en tiempo real permite **identificar** los cambios de régimen ya caracterizados mecanicistamente en GAP-3/GAP-4, con **significado estable** bajo las configuraciones de filtro del experimento (§6).

**Formulación explícitamente rechazada:**

> ~~«Existe un observable que separa mejor los regímenes.»~~

Eso convertiría v2 en una competición tipo leaderboard. Los candidatos **no miden el mismo fenómeno**; son **observadores de propiedades distintas** del EKF.

**No es hipótesis de v2:**

- que Γ̄ (EWMA τ=1 s) sea la propiedad correcta — **refutado** en v1;
- que exista ya una política adaptativa que mejore RMSE — H5-PoC, fase posterior;
- que un umbral concreto funcione — instancia controlador.

**Criterio de falsación de H6-OBS:** ninguna propiedad candidata (§4) mantiene interpretabilidad mecanicista **e** invarianza de significado entre C-F1 y C-PoC (§7) suficiente para identificar R1–R4 (§6).

### 2.1 Hipótesis exploratoria — conjunto mínimo (H7-MIN, no H6-OBS)

> **H7-MIN (exploratoria):** Ningún observable aislado describe completamente el régimen; existe un **conjunto mínimo** de propiedades complementarias.

Ejemplo **hipotético** (no preregistrado como verdad):

| Rol | Propiedad candidata |
|-----|---------------------|
| Estructural | ‖P_pv‖ / P_vv |
| Consistencia | Λ_N |
| Temporal | dΛ_N/dt |

Si el benchmark v2 apunta a H7-MIN, el outcome sería **definir el estado interno del régimen** (vector de observables), no «el ganador». Eso es más interesante metodológicamente que un ranking. H7-MIN se evalúa **después** de caracterizar cada Oi; no sustituye H6-OBS.

---

## 3. Lección congelada de v1 (no reabrir)

> **El observable elegido (Γ̄ operacionalizado) no preserva el fenómeno mecanicista que motivó el controlador.**

Corolario v2: el fallo de Γ no fue solo sensibilidad — fue **pérdida de significado** al cambiar config (offline 19.7 en C-F1 vs 0.13 en C-PoC). **Invarianza entre configuraciones** es criterio de selección tan importante como sensibilidad.

**Prohibido:** retune post-hoc (umbral 12→9). Ver [15-gap5-passive-outcome.md](15-gap5-passive-outcome.md).

---

## 4. Paso 0 — Catálogo de estados internos (congelado pre-benchmark)

**Antes de correr un solo script**, el catálogo completo vive en:

**[`docs/benchmarks/gap5_v2_observable_selection/paso0_property_justification.md`](../benchmarks/gap5_v2_observable_selection/paso0_property_justification.md)**

Incluye por candidato: qué observa, **qué no observa**, escala temporal, dependencias, hipótesis y falsabilidad. La columna «qué no observa» evita pedir a un observable lo que, por definición, no puede medir (p.ej. Γ no sabe nada del nominal; Λ_N no sabe nada de la estructura de P).

**Regla Paso 0:** ningún candidato entra al benchmark sin fila **completa** en el catálogo. O6 solo con preregistración explícita pre-benchmark (post-tag: requiere v1.1 + nuevo tag).

**Espíritu GAP-3:** cada fila expone al candidato a falsación **antes** de ejecutar. Refutar O3 no refuta O4 — observan físicas distintas.

---

## 5. Paso 1 — Caracterización experimental (no ranking)

Tras Paso 0 congelado, ejecutar benchmark **passive / audit only**. Por cada candidato Oi, responder preguntas de **caracterización**:

| Pregunta | Qué evalúa |
|----------|------------|
| **C1** ¿Detecta R1 (burst) con latencia acotada? | Sensibilidad al evento mecanicista |
| **C2** ¿Permanece estable / interpretable dentro de R2? | No confundir pico con régimen sostenido |
| **C3** ¿Conserva **significado** entre C-F1 y C-PoC? | **Invarianza** (criterio central post-v1) |
| **C4** ¿Tiene memoria excesiva para el soporte de R1 (~0.4 s)? | Operacionalización vs fenómeno bursty |
| **C5** ¿Necesita conocer el futuro? | Causalidad para control en tiempo real |
| **C6** ¿Es puramente local (por tick / ventana causal)? | Implementabilidad online |

**Prohibido como criterio de selección primario:**

- «¿Detecta R1 **antes** que los demás?» — un controlador puede compensar latencia; **no** puede compensar que el observable cambie de significado entre configs.
- Comparar candidatos como si midieran la misma magnitud.
- Elegir el pico más alto sin interpretabilidad.

---

## 6. Regímenes de referencia (ground truth mecanicista)

| Régimen | Ventana / marcador | Evidencia previa | Propiedad(es) que lo caracterizan |
|---------|-------------------|------------------|-----------------------------------|
| **R0** | Pre-fix#2, t ≈ 1–5 s | P_vv bajo; NHC nominal | Baseline |
| **R1** | Burst fix#2→#3, Δt ≈ **0.39 s** | F1: Γ_offline ≈ 19.7 | Consumo P / sobreobservación |
| **R2** | Post-fix#3 → fix#4 | F1.2: Λ_N crece | Consistencia nominal ↓ |
| **R3** | Post-fix#4 (1d / 1d′) | GAP-4: P_pv gate | Acoplamiento pos→vel |
| **R4** | Crucero largo, t ≫ 30 s | Passive v1: mesetas | Cuasi-estacionario |

Mapeo **propiedad → régimen** es parte del Paso 0; el benchmark verifica si el observable asignado cumple ese mapeo.

---

## 7. Configuraciones de filtro — invarianza como criterio

Todo candidato se caracteriza en **ambas**:

| Config | Motivo |
|--------|--------|
| **C-F1** | pos-only — puente F1/GAP-3 |
| **C-PoC** | pos_vel + p_pv none — config PoC futura ([14-adaptive-nhc-protocol.md](14-adaptive-nhc-protocol.md) §4.2) |

**Pregunta central (no secundaria):**

> **¿Qué observable conserva su significado entre configuraciones?**

Un observable muy sensible en C-F1 pero cuyo significado mecanicista colapsa en C-PoC **no es seleccionable** para el PoC preregistrado — aunque «gane» en separación R1 vs R4 en C-F1 solo.

Passive v1: Γ perdió significado (19.7 → 0.13 offline). Eso es **lección**, no excusa para omitir C-PoC.

---

## 8. Criterios formales (sin RMSE)

| ID | Criterio | Operacionalización | Peso |
|----|----------|-------------------|------|
| **I1 — Interpretabilidad** | Enlace explícito propiedad → cadena predict→P→update (Paso 0) | Cualitativo + referencia GAP-3/4 | **Alto** |
| **I2 — Invarianza config** | Orden de magnitud, signo y **significado** mecanicista consistentes C-F1 ↔ C-PoC | C3 | **Alto** |
| **I3 — Localidad** | Causal, por tick o ventana causal; C5=C6 | C6 | **Alto** |
| **S1 — Identificación R1** | Señal distinguible en ventana burst | C1 | Medio |
| **S2 — Comportamiento R2/R4** | Estable o monótono según propiedad esperada | C2 | Medio |
| **S3 — Memoria acotada** | Soporte temporal compatible con R1 (~0.4 s) si la propiedad es bursty | C4 | Medio |

**Función de selección (orientativa, no numérica rígida):**

```
score ≈ interpretabilidad × invariancia × localidad
```

Sensibilidad (S*) modula pero **no domina**. Un controlador puede compensar latencia; **no** puede compensar invarianza rota.

**Prohibido como criterio primario:** RMSE, accepts, drift, «primer pico».

---

## 9. Criterio de selección (ganador)

**No se elige** el observable que detecte antes el burst.

**Se elige** la propiedad cuya observación maximiza **interpretabilidad × invarianza × localidad** y permite identificar R1–R4 con operacionalización causal validable en passive (fase posterior, análoga v1).

| Resultado | Acción |
|-----------|--------|
| Una propiedad cumple I1–I3 y S1–S3 en ambas configs | **Congelar** propiedad + operacionalización → GAP-5 v3 (controlador) |
| Varias cumplen I1–I3 | Documentar trade-offs por **propiedad distinta**, no por «score» único; no combinar post-hoc sin O6 preregistrado |
| Ninguna cumple I2 (invarianza) | **Refutar H6-OBS** para el conjunto candidato; reconsiderar propiedad o config PoC |
| Solo cumple en C-F1 | Reconciliar config PoC con mecanismo **antes** de controlador |

**Veredicto v2 positivo** ⟺ propiedad seleccionada + Paso 0 completo + informe caracterización JSON + (futuro) passive de operacionalización **sin controlador**.

---

## 10. Diseño experimental (post-Paso 0)

**Modificar:** scripts audit offline sobre logs existentes (`cov_step_audit.csv`, `gnss_nis_audit.csv`, …).

**No modificar:** EKF, NHC, lazo `--adaptive-nhc active`.

**Salida prevista:**

```
docs/benchmarks/gap5_v2_observable_selection/
  paso0_property_justification.md    # tabla §4 congelada
  observable_characterization.json   # respuestas C1–C6, I1–S3 por Oi × config
  observable_characterization.md
  figures/                           # series temporales por régimen
```

Script previsto (post-tag): `tools/audit_gap5_v2_observable_selection.py`

---

## 11. Secuencia y prohibiciones

```
15-gap5-passive-outcome.md (v1 CERRADA)
        │
        ▼
  16-gap5-v2-observable-selection.md (CONGELAR → tag)
        │
        ▼
  Paso 0: tabla §4 completa (sin scripts)
        │
        ▼
  audit_gap5_v2_observable_selection.py
        │
        ▼
  Informe caracterización (no ranking)
        │
        ▼
  Passive operacionalización (sin controlador)
        │
        ▼
  (futuro) GAP-5 v3 — controlador sobre propiedad congelada
```

**Prohibiciones:**

1. Retunear umbrales v1.
2. PoC activo antes de v3 preregistrado.
3. RMSE/accepts en selección de observable.
4. Declarar «ganador» mid-run o por gráfico único.
5. Lenguaje de «mejor observable» / leaderboard.
6. **Inferir validez del observable desde éxito/fallo del controlador** (§0.1).
7. **Discutir políticas de control, umbrales o palanca NHC** en documentación, issues o informes v2 (§0.2).

---

## 12. Entregable post-v2 — síntesis metodológica (§17, no protocolo)

**Condición estricta:** **no escribir** [`17-lessons-ekf-regime-identification.md`](17-lessons-ekf-regime-identification.md) hasta que GAP-5 v2 haya **terminado completamente**:

1. Informe `observable_characterization.json` congelado.
2. Veredicto H6-OBS / H7-MIN documentado.
3. (Si aplica) passive de operacionalización del observable seleccionado — **sin controlador**.

Escribir la síntesis antes de (1–3) reinterpreta experimentos para encajar una narrativa. Debe ser **destilación posterior**, no guía que condicione el benchmark.

**Título propuesto:** *Lessons learned: from debugging an EKF to identifying its internal regimes*

**Esquema de contenido (no redactar hasta cierre v2):**

- Diagnóstico offline ≠ señal de control online.
- Política después del observable, no antes.
- Invarianza config > sensibilidad máxima.
- Observable y controlador: objetos distintos, validación distinta.
- Observable válido puede no controlar nunca nada.
- Identificación de régimen ≠ depuración.

---

## 13. Congelación

| Campo | Valor |
|-------|-------|
| Versión | **1.0** |
| Estado | **CONGELADA** |
| Tag Git | `gap5-v2-observable-preregistration-frozen` |
| Hipótesis principal | H6-OBS (§2) |
| Hipótesis exploratoria | H7-MIN (§2.1) |
| Paso 0 | Catálogo congelado en `paso0_property_justification.md` |
| Regla §0.2 | Solo observables — prohibido discutir control en v2 |
| §17 | Prohibido redactar hasta cierre completo v2 |

---

## Changelog

| Versión | Fecha | Notas |
|---------|-------|-------|
| 0.1 | 2026-07-18 | Apertura fase v2; arquitectura diagnóstico→observable→controlador |
| 0.2 | 2026-07-18 | H6-OBS reformulada (propiedad, no ranking); Paso 0 conceptual; caracterización C1–C6; invarianza config |
| **0.3** | **2026-07-18** | §0.1 observable≠controlador; Paso 0 + falsabilidad; §12 síntesis post-v2 |
| **1.0** | **2026-07-18** | **CONGELADA** — §0.2 solo observables; catálogo Paso 0; H7-MIN; tag `gap5-v2-observable-preregistration-frozen` |
