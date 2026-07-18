# Critical Evidence Review — Intento de desmontar EVIDENCE_REVIEW

**Tipo:** revisión adversaria. **No** genera datos ni nuevas hipótesis.  
**Fecha:** 2026-07-18  
**Objetivo:** buscar K demasiado fuertes, inferencias solo-G1, OQ ya respondidas.  
**Documento bajo ataque:** [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md)

---

## 1. Ataques a la cobertura G-ext (¿sobreafirmamos?)

| Claim en Evidence Review | Ataque | Veredicto crítico |
|--------------------------|--------|-------------------|
| K1 G-ext = «Parcial» (floor NHC) | K1 afirma **modulación por frecuencia** N. Un floor con N=1 **no** es dose–response. | **Ataque válido.** Corregir a: G-ext **no soporta K1**; solo es **consistente** con compresión NHC (fenómeno hermano, no la afirmación K1). |
| K5 G-ext = «Parcial» (floor) | K5 afirma cliff **bursty** (top3). Floor ≠ burst. Scan G-ext fue *uniform*. | **Ataque válido.** G-ext **no soporta K5**; no lo refuta (config/shell distintos al F1 pos-only). |
| K8 G-ext = «Sí» (crece P_pv) | K8 es «legítimo, no bug» (álgebra GAP-4). Crecer P_pv en G-ext es **consistencia**, no re-prueba «no bug». | **Ataque válido.** G-ext = **consistente con** K8, no confirmación independiente de legitimidad. |
| K4 G-ext = «Parcial / tensión» | STATE formula K4 como «North-axis». Tras G-ext, leer K4 como universal es **demasiado fuerte**. | **Ataque válido sobre la lectura.** El hallazgo F1.1 en G1 se mantiene; la **formulación sin scope** es el problema. Añadir scope note en STATE (abajo). |
| K14 / K15 | ¿Circular (G-ext se auto-confirma)? | **Ataque débil.** K14/K15 son claims *sobre* G-ext vs G1; la evidencia es el run. No son tautologías vacías. |
| «Generalidad del bloqueo» | ¿Mezcla K1/K5/K8 con K14? | **Matizar:** lo generalizado es el **núcleo INTERPRETATION** (P_vv↓, P_pv↑, Λ alta, no-reenganche), no cada K1–K7. |

---

## 2. ¿K con evidencia más débil de lo que parece?

| K | Debilidad | ¿Amenaza el programa? |
|---|-----------|------------------------|
| **K4** | Eje Norte no se reproduce en G-ext; STATE suena universal | Sí, si se usa K4 para elegir O4=Λ_N *como eje Norte*. No, si O4 = consistencia / Λ del canal (Paso 0 ya habla de Λ_N sin exigir Norte geográfico). |
| **K6** | Solo F1 pos-only; magnitud no portable (K13) | Ya acotado; no cierra OQ1. |
| **K9** | Solo región multi-accept G1 | Ya non-claim; no usarlo como requisito de H6. |
| **K2, K3, K7, K10–K13** | Solo G1 / metodológico | Esperable; no invalidados. Debilidad = **falta de réplica**, no contradicción. |
| **K14** | Un solo trayecto externo (V=2, no V≫2) | Generalidad *incipiente*, no ley. Suficiente para motivar H6; no para cerrar H6. |

Ningún K «se cae» del todo. Varios estaban **sobre-etiquetados** en la columna G-ext de Evidence Review → corregidos en §5.

---

## 3. ¿Alguna OQ ya respondida?

| OQ | ¿Se puede cerrar solo con evidencia actual? | Por qué no / sí |
|----|---------------------------------------------|-----------------|
| **OQ1** | **No** | Saber que hace falta un observable interno ≠ caracterizar cuál (C1–C7, regime_model). Atajo «usar O3+O4 del Paso 0» **viola prerregistro**. |
| **OQ2** | **No** | H7-MIN solo tras caracterizar cada Oi. |
| **OQ3** | **No** (parcialmente iluminada) | K13 responde para **Γ**, no para O1–O5. |
| **OQ4** | **No** | Entregable post-benchmark. |
| **OQ5–OQ6** | **No** | v3+; H5-PoC no testeada. |
| **OQ7** | **No** | §11 no ejecutado; G-ext no entra en región fix#4. |

**Ataque «OQ1 ya está respondida porque K15»:** falla. K15 responde una pregunta distinta (*¿basta lo externo?*). OQ1 pregunta *qué interno*.

**Ataque «OQ1 ya está respondida porque Paso 0 lista O1–O5»:** falla. Catálogo ≠ caracterización experimental.

---

## 4. ¿H6 es inevitable o inercia del protocolo?

| Opción | Condición | ¿Aplica? |
|--------|-----------|----------|
| Cerrar H6 **sin** benchmark | OQ1 respondida por datos existentes | **No** — §3 |
| Ejecutar H6 exactamente como preregistrado | OQ1 abierta **y** pregunta no respondible con datos actuales | **Sí** |
| Inventar atajo (elegir O3/O4 sin C1–C7) | Tentación post G-ext | **Rechazado** — D15 / prerregistro |

**Conclusión:** H6 no es inevitable por existir el archivo 16.  
Es **justificado** porque, tras revisión adversaria, **sigue sin existir respuesta a OQ1** en el corpus.

La inercia «ejecutar porque está preregistrado» queda **rechazada** como motivo.  
El motivo válido: **pregunta abierta + no respondible con datos actuales + protocolo ya congelado**.

---

## 5. Correcciones a Evidence Review (aplicadas)

Tras este ataque, la columna G-ext debe leerse así:

| K | G-ext (corregido) |
|---|-------------------|
| K1 | **No** (dose–response). Consistencia con floor NHC aparte. |
| K5 | **No** (bursty). Floor ≠ burst. |
| K8 | **Consistente** (crece P_pv), no re-prueba «no bug». |
| K4 | **Tensión de formulación** — ver scope note STATE. |

El veredicto «OQ1 sigue abierta» **sobrevive** al ataque.

---

## 6. Decisión de fase

| Campo | Valor |
|-------|-------|
| Revisión crítica | **Completada** |
| OQ1 | **Sigue abierta** (confianza ↑ tras adversaria) |
| Pausa metodológica (fase revisión) | **Terminada** |
| Siguiente experimento autorizado | Benchmark H6 v1.2 **sin modificar el prerregistro** (D15/D16) |
| Prohibido en ese paso | Controlador, Γ̄/τ/umbrales, NHC adaptativo, P_pv, Joseph, Q/R |

---

## 7. Etapas del programa (solo marco)

1. Debugging → 2. Diagnóstico mecanicista → 3. Validación externa → **4. Modelización (H6)**  

Claridad > otro CSV improvisado.
