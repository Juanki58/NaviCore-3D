# Evidence Strength Audit — Peso de cada K y supervivencia de cada OQ

**Tipo:** auditoría científica (no resumen). **Sin** código, replay ni CSV nuevos.  
**Fecha:** 2026-07-18  
**Precedentes:** [EVIDENCE_REVIEW.md](EVIDENCE_REVIEW.md), [CRITICAL_EVIDENCE_REVIEW.md](CRITICAL_EVIDENCE_REVIEW.md)  
**Criterio de apertura H6:** §4

---

## 1. Plantilla por K

| Fuerza | Significado |
|--------|-------------|
| **Alta** | Evidencia directa clara; réplica o generalización independiente, o claim metodológico acotado sin contradicción |
| **Media** | Evidencia directa sólida en un dominio; réplica parcial / tensión / un solo trayecto |
| **Limitada al dominio** | Válida solo en región de estado o config concreta; no proyectar fuera |
| **Metodológica** | Regla de procedimiento (logs, autoridad de campos), no mecanismo físico del vehículo |

**Evidencia independiente** = segundo dataset o segunda config que sostiene *la misma afirmación K*, no solo un fenómeno hermano.

### Tabla K1–K15

| K | Evidencia directa | Evidencia independiente | Evidencia en contra / límite | Fuerza | Alcance (no cambiar el K; delimitar lectura) |
|---|-------------------|-------------------------|------------------------------|--------|-----------------------------------------------|
| **K1** | F1 dose–response (G1-lineage, pos-only) | **Ninguna** (G-ext floor ≠ dose) | — | **Media** | Solo claim de *frecuencia* NHC; no leer floor G-ext como réplica |
| **K2** | Constraint matrix NHC ON/OFF | Ninguna en G-ext | — | **Media** | Un diseño experimental, un trayecto |
| **K3** | F1 + F1.1 | Ninguna en G-ext | — | **Media** | Fuerte *dentro* de ese diseño; no re-probado fuera |
| **K4** | F1.1 (contrib_N / Λ_N, formulación Norte) | G-ext: Λ elevada **sí**; Norte dominante **no** | Tensión eje Norte (INTERPRETATION) | **Media** (gate); **Limitada** si se lee “Norte universal” | Scope note en STATE; gate stress ≠ Norte geográfico universal |
| **K5** | F1.2 + autopsia fix#2→#3 | Ninguna (G-ext no reproduce burst top3) | Bajo shell G1/pos_vel el patrón burst F1 no se ve igual | **Media** | Burst F1-config; no ley de todo shell |
| **K6** | F1 baseline Γ≈19.7 | Ninguna; PoC da Γ≈0.13 (K13) | Magnitud no portable entre configs | **Limitada al dominio** | Solo F1 pos-only gap |
| **K7** | GAP-3.14 Joseph ~31% | Ninguna en G-ext (sin fix#2 análogo) | — | **Limitada al dominio** | Evento fix#2 G1-lineage |
| **K8** | GAP-4 autopsy (no bug) | G-ext: crece P_pv (**consistencia**, no re-prueba álgebra) | — | **Alta** (legitimidad en G1); réplica indep. **no** | “No bug” = GAP-4; G-ext no añade prueba de legitimidad |
| **K9** | Divergence tree / truth table fix#4 | **No alcanzado** en G-ext | — | **Limitada al dominio** | Solo región multi-accept / post-fix#4 |
| **K10** | Autopsy gps#32 | N/A (regla de método) | — | **Metodológica / Alta** en su clase | Autoridad de logs; no mecanismo de vehículo |
| **K11** | Passive f1-bridge Γ_inst≈Γ | Ninguna en G-ext | — | **Media** | Config F1-eq |
| **K12** | Passive Γ̄ inactivo / H-ops | Ninguna en G-ext (mismo diseño) | — | **Alta** (falsación ops v1) | Cierra Γ̄ v1; no cierra “todo observable” |
| **K13** | Passive f1-bridge vs PoC | Mismo CSV, dos configs (no 2º trayecto) | — | **Alta** para Γ | Responde invarianza de **Γ**; no de O1–O5 |
| **K14** | G-ext vs G1 shell | **Es** la evidencia independiente | No secuencia G1 completa | **Alta** para núcleo de bloqueo | V=2 trayectos; generalidad incipiente, no ley |
| **K15** | G-ext Phase A+B | Propia de G-ext; G1 no equivalente | — | **Alta** (en G-ext) | Desacople externo/interno; motiva OQ1 |

### Resumen de peso

| Fuerza | Ks |
|--------|-----|
| Alta (en su alcance) | K8 (legitimidad), K10, K12, K13 (Γ), K14, K15 |
| Media | K1–K5, K11 |
| Limitada al dominio | K6, K7, K9; lectura “Norte universal” de K4 |

---

## 2. Supervivencia de OQ1–OQ7

Pregunta de filtro: **¿Cuál no puede responderse con la evidencia existente?**

| OQ | ¿Respondible ya? | Qué dice la evidencia | Supervivencia |
|----|------------------|----------------------|---------------|
| **OQ1** | **No** | K12–K15 afilan; no hay discriminación C1–C7 entre O1–O5 ni `regime_model` | **Sobrevive — completamente abierta** |
| **OQ2** | **No** | H7-MIN solo tras caracterizar cada Oi | **Sobrevive** |
| **OQ3** | **Parcialmente** | K13: Γ **no** preserva significado C-F1↔C-PoC. Resto de Oi: **sin dato** | **Reformular** (abajo) — no cerrar |
| **OQ4** | **No** | Entregable post-caracterización | **Sobrevive** |
| **OQ5** | **Cota negativa parcial** | K12: operacionalización Γ̄ EWMA τ=1 s **falla**. Propiedad aún no elegida → OQ5 completa **no** contestable | **Reformular** (abajo) — no cerrar |
| **OQ6** | **No** | H5-PoC activo no ejecutado | **Sobrevive** (v3+) |
| **OQ7** | **No** | §11 no ejecutado; G-ext fuera de región fix#4 | **Sobrevive** |

### Reformulaciones (trazabilidad; no borrado silencioso)

**OQ3 — estado tras auditoría**

| Antes (énfasis) | Ahora (explícito) |
|-----------------|-------------------|
| “Which observable preserves meaning across C-F1 and C-PoC?” | **(a) Cerrado para Γ:** no (K13). **(b) Abierto para O1, O3, O4, O5 del Paso 0:** sin caracterización cruzada config. |

**OQ5 — estado tras auditoría**

| Antes | Ahora |
|-------|-------|
| Operacionalización causal online de la propiedad elegida | **Cota:** la instancia Γ̄ v1 **no** es esa operacionalización (K12). **Abierto:** cualquier operacionalización de la propiedad que H6 elija (requiere H6 primero). |

Ninguna OQ se elimina. El espacio se **reduce**: no reabrir invarianza de Γ; no re-probar Γ̄ v1 como respuesta a OQ5.

---

## 3. Criterio literal para abrir H6

Regla:

> Sólo ejecutar el benchmark si al final de la revisión podéis escribir **literalmente**:
>
> **"No existe evidencia suficiente para discriminar entre los observables candidatos mediante los experimentos ya realizados."**

### ¿Podemos escribirla honestamente?

| Chequeo | Resultado |
|---------|-----------|
| ¿Existe ranking C1–C7 de O1–O5? | **No** |
| ¿Existe `observable_characterization.json`? | **No** |
| ¿Paso 0 basta como discriminación experimental? | **No** (catálogo ≠ benchmark) |
| ¿K12 discrimina entre candidatos? | **Solo elimina O2/Γ̄ ops** — no ordena O1/O3/O4/O5 |
| ¿G-ext discrimina observables? | **No** — motiva observables internos; no los compara |

**Frase (congelada como justificación D17):**

> **No existe evidencia suficiente para discriminar entre los observables candidatos mediante los experimentos ya realizados.**

Por tanto el benchmark H6 está **plenamente justificado** — no por cronología ni porque el protocolo exista, sino porque esa frase es verdadera.

---

## 4. Decisión

| Ítem | Valor |
|------|-------|
| Auditoría de fuerza K | Completada (§1) |
| Supervivencia OQ | OQ1 completamente abierta; OQ3/OQ5 acotadas; resto abiertas |
| Frase criterio H6 | **Afirmable honestamente** (§3) |
| Pausa metodológica (revisión) | **Cerrada formalmente** |
| Ejecución H6 | Autorizada cuando se decida correrla — **sin** modificar prerregistro v1.2 |
| Motivo documentable en 6 meses | “Demostramos que la evidencia existente no podía responder OQ1 / discriminar candidatos” — no “porque tocaba” |

---

## 5. Prohibido al ejecutar H6

Controlador · retocar Γ̄/τ/umbrales · NHC adaptativo · intervenir P_pv · Joseph · Q/R — nivel posterior (OQ5–OQ7 / v3+).
