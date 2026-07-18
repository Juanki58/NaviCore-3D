# GAP-5 v2 / H6 — Modelo de régimen (`regime_model.md`)

**Protocolo:** [16-gap5-v2-observable-selection.md](../../diagnostics/16-gap5-v2-observable-selection.md) v1.2 §9.2  
**Tag:** `gap5-v2-observable-preregistration-v1.2`  
**Fecha:** 2026-07-18  
**Entradas:** [`observable_characterization.json`](observable_characterization.json) · [`H6_ARTIFACT_CONSISTENCY_REVIEW.md`](H6_ARTIFACT_CONSISTENCY_REVIEW.md) (D19) · Paso 0  

**Nivel:** este documento trata la **representación** del EKF (propiedad → observable → modelo de régimen), no el mecanismo físico ya cerrado en GAP-3/4 ni un controlador.

**Orden de escritura:** descripción → limitaciones → hipótesis → cardinalidad → implicaciones v3 → **OQ1 solo al final**.

---

## 1. Qué pregunta respondía H6

**H6-OBS (formal):** ¿Existe al menos una propiedad interna del EKF, medible en replay sin modificar el filtro, cuya observación permite identificar los cambios de régimen caracterizados en GAP-3/GAP-4, con significado estable bajo C-F1 y C-PoC?

**H7-MIN (exploratoria):** ¿Ningún observable aislado basta y existe un conjunto mínimo complementario?

H6 se ejecutó como **caracterización**, no como optimización (D18). La justificación de abrirla fue insuficiencia de evidencia previa para discriminar candidatos (D17), no inercia de protocolo.

---

## 2. Qué resultados se obtuvieron

### 2.1 Resultados del benchmark (ya sólidos en D19)

1. **Discriminación:** el protocolo distingue familias de candidatos. Escenario A («todos iguales») **descartado**.  
2. **Colinealidad parcial O1–O2:** mismo vector C7 en ambas configs. Resultado **metodológico** sobre poder discriminativo C1–C7; no identifica O1 con O2 como la misma magnitud física.

### 2.2 Perfiles C7 observados (evidencia, no elección)

| Oi | Propiedad | Perfil C-F1 (R0→R4) | Perfil C-PoC | C3 (invarianza ordinal) |
|----|-----------|---------------------|--------------|-------------------------|
| O1 | Γ_inst (consumo) | bajo, alto, alto, bajo, bajo | bajo, alto, alto, bajo, meseta | Sí |
| O2 | Γ̄ (EWMA) | ≡ O1 | ≡ O1 | Sí |
| O3 | ‖P_pv‖/P_vv | bajo, meseta, meseta, alto, pico | bajo, meseta, bajo, alto, pico | Sí |
| O4 | Λ_N | meseta×3, bajo, pico | idéntico | Sí |
| O5 | dΛ_N/dt | N/A, bajo, alto, pico, alto | idéntico | Sí |

### 2.3 Estados R0–R4 (definición operativa §6)

Sin cambiar marcadores: R0 pre-fix#2; R1 burst fix#2→#3; R2 post-fix#3→#4; R3 post-fix#4; R4 crucero (t≳30 s). Ventanas temporales tomadas del JSON de caracterización (mismas en C-F1 y C-PoC en este dataset).

### 2.4 Observables por régimen (mapa emergente — provisional)

| Régimen | Qué pide el mecanismo (§6) | Qué alimenta el artefacto con claridad | Qué no alimenta con claridad |
|---------|----------------------------|----------------------------------------|------------------------------|
| **R0** | Baseline | O1 bajo (compatible) | O4/O5 frágiles (L-datos); O5 N/A |
| **R1** | Consumo / sobreobservación | **O1** (C1 verdadero; elevación numérica clara en ambas configs) | O2 misma etiqueta C7 pero ops más débil (C4 en C-F1); etiqueta «alto»≠«pico» (L-ops) |
| **R2** | Consistencia nominal ↓ | — | **O4 no eleva**; O1 no separa R2 de R1; O5 n≈1 |
| **R3** | Acoplamiento pos→vel | **O3** elevado (alto) en ambas configs | No alcanza la forma «pico» predicha en Paso 0 |
| **R4** | Cuasi-estacionario | **O1** bajo/meseta (compatible con meseta de consumo) | O3 y O4 en **pico** (anti-predicción del mapa orientativo) |

Este mapa **describe** qué evidencia C7 existe. No declara aún un controlador ni un score.

### 2.5 Invarianza del mapa

- C3 (acuerdo ordinal entre configs en regímenes comparables) es **verdadero** para O1–O5 en el JSON.  
- Eso **no** implica que el mapa §2.4 identifique R1–R4 de punta a punta: hay **hueco en R2** y **tensión en R4** para O3/O4.  
- El modelo emergente es, por tanto, **parcialmente invariante en las piezas que sí alimenta** (sobre todo R1←O1; R3←O3 con matiz), no un identificador completo de régimen en ambas configs.

### 2.6 Fuera de alcance declarado

- Controlador adaptativo, umbrales Γ̄/τ, política NHC, intervención P_pv, retune Q/R/Joseph  
- G-ext como brazo de caracterización H6  
- Remapeo Norte→longitudinal  
- O6 / combinaciones no preregistradas  
- Declarar «ganador» por pico temprano en R1  

---

## 3. Qué hipótesis quedaron en qué estado (Paso 0)

Vocabulario D19: Evaluada / Refutada / No refutada / Inconclusa. **Ninguna fila significa «confirmada».**

| Oi | Evaluada | Estado | Implicación para el modelo |
|----|:--------:|--------|----------------------------|
| O1 | Sí | **No refutada** | Sigue siendo el ancla más clara para **R1** (consumo) bajo C-F1 y C-PoC |
| O2 | Sí | **Refutada** (ops/memoria) | No entra al modelo como eje distinto de O1; colinealidad C7 + C4 |
| O3 | Sí | **Inconclusa** | Candidata a alimentar **R3**; no fijar aún como definición de R3–R4 |
| O4 | Sí | **Refutada*** | No ancla **R2** con la fuerza preregistrada; *rebajada por L-datos |
| O5 | Sí | **Inconclusa** | No fija transiciones R2 ni meseta R4 |

Predicciones de **forma** del mapa §5.2 (pico/↓/meseta) quedaron en varios casos contestadas; eso tensiona el mapa orientativo, no borra B1/B2 ni el núcleo no refutado de O1.

---

## 4. Cardinalidad (uno / vector / ninguno)

Aplicando la frase D19 sin endurecerla:

> Los resultados disponibles **no permiten concluir** ni que un observable individual sea **suficiente** ni que un conjunto mínimo sea **necesario**.

| Pregunta §9.2 | Respuesta desde artefactos |
|---------------|----------------------------|
| ¿Un observable basta para R0–R4? | **No afirmable** |
| ¿H7-MIN necesaria? | **No afirmable** |
| ¿Ninguno del catálogo aporta? | **No** — O1 aporta R1; O3 aporta señal distinta en R3 |
| ¿O1 y O2 ambos en un vector? | **No** — como máximo uno de {O1,O2}; O2 ops refutada |
| ¿Cuál de los 4 desenlaces pre-H6? | **Ninguno cerrado en exclusivo.** Lo más cercano: identificación **incompleta** del régimen con piezas no colineales (O1 vs O3 vs familia Λ), sin licencia aún para H7-MIN ni para mono-Oi |

**Cardinalidad declarada:** *indeterminada — modelo parcial de alimentación por régimen, no vector mínimo congelado ni propiedad única congelada.*

---

## 5. Limitaciones que el modelo arrastra

### L-datos

- Baja densidad GNSS → O4/O5 con `n≈1` en R0–R2  
- O5·R0 = N/A  

### L-ops

- Binding temporal → O1·R1 etiquetado «alto» en lugar de «pico»  
- Discretización ordinal C7  

La síntesis **no** reinterpreta ordinales «a ojo» para forzar el mapa §5.2.

---

## 6. Qué permanece abierto (antes de OQ1)

| ID | Estado tras este documento |
|----|----------------------------|
| **OQ2 / H7-MIN** | **Abierta** (ni suficiente mono-Oi ni necesidad de conjunto mínimo demostradas) |
| **OQ3** | Acotada: C3 ordinal OK para O1–O5 en este brazo; **abierta** la invarianza del *modelo completo* R1–R4 |
| **OQ4** | **Parcialmente respondida** por §2.4–§4 (mapa provisional + cardinalidad indeterminada); no congelada como arquitectura de controlador |
| **OQ5–OQ6** | Sin abrir (v3+) |
| **R2 en representación** | Hueco explícito: ninguna ops del catálogo identifica con claridad la caída de consistencia nominal bajo L-datos actuales |

---

## 7. Implicaciones para GAP-5 v3

1. **No abrir controlador** hasta que exista una propiedad o conjunto **congelado** con dominio de validez explícito. Este documento **no** congela esa entrada.  
2. Si v3 se preparara solo con lo más sólido hoy: la única pieza **no refutada** y mecanicistamente anclada a un régimen es **R1 ← O1 (Γ_inst)**. Eso **no** autoriza un controlador de «régimen completo».  
3. O2 (Γ̄ v1) permanece **fuera** como operacionalización de control (K12 + refutación ops H6).  
4. Cualquier propuesta de combinar O1+O3+… requiere **O6 preregistrado** (v1.1+), no combinación post-hoc.  
5. El hueco R2 / densidad GNSS es candidato natural a trabajo de **datos o operacionalización**, no a retune de umbrales mid-flight. Ideas → [`IDEAS_DURING_H6.md`](IDEAS_DURING_H6.md).  
6. H6 cambió de nivel: de conocimiento del EKF a conocimiento de su **representación**. v3 solo tiene sentido cuando esa representación deje de ser parcial en el sentido de §4.

---

## 8. Contenido mínimo §9.2 (resumen tabular)

| Campo | Valor |
|-------|-------|
| **Estados** | R0–R4 (§6), ventanas en JSON |
| **Observables por régimen** | R1←O1 (claro); R3←O3 (provisional); R4←O1 (parcial); R2←∅ (hueco); O2 excluida como eje distinto |
| **Cardinalidad** | Indeterminada (ni uno suficiente ni mínimo necesario demostrados) |
| **Invarianza** | Piezas parciales sí (C3); modelo completo R1–R4 no |
| **Fuera de alcance** | §2.6 |

---

## 9. Decisión sobre OQ1 (última)

**Condición de cierre (acordada pre-síntesis):** OQ1 queda cerrada si y solo si este documento puede afirmar, apoyándose en `observable_characterization.json`, **cuál** es la propiedad (o conjunto mínimo H7-MIN) que define el régimen **y por qué** las alternativas quedan descartadas o limitadas en su dominio de validez.

**Evaluación:**

- Se puede afirmar un **mapa parcial** (§2.4) y descartar O2 como eje distinto.  
- **No** se puede afirmar una propiedad única que defina R0–R4.  
- **No** se puede afirmar un conjunto mínimo necesario (H7-MIN).  
- Alternativas O3/O4/O5 quedan **limitadas o inconclusas**, no todas descartadas con el mismo rigor.

**Por tanto: OQ1 permanece parcialmente abierta.**

El resultado de H6 sigue siendo científicamente válido: discriminación experimental (B1), colinealidad metodológica O1–O2 (B2), y un modelo de régimen **parcial** que conecta mecanismo → representación sin fingir cierre prematuro.

---

## 10. Contribución de la fase (cierre D20)

> La contribución principal de H6 no es identificar un controlador, sino transformar un conjunto de candidatos independientes en un modelo parcial del espacio de regímenes del filtro.

En la cronología del programa:

| Fase | Pregunta respondida |
|------|---------------------|
| GAP-3 | Qué **mecanismo** existe |
| GAP-4 | Qué **estructura** interna participa |
| GAP-5 v1 | Qué **operacionalización** no preserva el fenómeno |
| G-ext | Qué parte del mecanismo **generaliza** |
| **H6** | Cómo **empieza a organizarse** el espacio de regímenes |

Cadena conceptual:

```
Fenómeno
    ↓
Mecanismo (GAP-3)
    ↓
Estructura (GAP-4)
    ↓
Operacionalización (GAP-5 v1)
    ↓
Generalidad (G-ext)
    ↓
Modelo parcial de régimen (H6 / D20)
    ↓
Controlador (GAP-5 v3)  ← aún no abierto
```

GAP-5 v3, cuando se abra, no partirá de una intuición sobre qué controlar, sino de este modelo explícito —aunque incompleto— del régimen interno.
