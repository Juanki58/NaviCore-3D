# Consistency Audit — Solo detección / correcciones de trazabilidad

**Tipo:** editorial. **No** añade conocimiento científico.  
**Fecha:** 2026-07-18  
**Alcance:** docs/diagnostics (+ G-ext baseline docs enlazados)

---

## 1. Enlaces markdown rotos

| Estado | Ítem |
|--------|------|
| **Corregido** | `STATE_OF_KNOWLEDGE.md` → fuentes `12/13/15-*.md` apuntaban sin `../` (14 ocurrencias) |
| **Abierto (esperado)** | `16-gap5-v2-observable-selection.md` → `17-lessons-ekf-regime-identification.md` — documento diferido por D10; enlace placeholder |
| **OK (muestreo)** | Enlaces OQ↔INTERPRETATION, RESEARCH_MAP↔G-ext, METRICS↔STATUS |

Re-scan tras corrección: solo queda el placeholder §17.

---

## 2. Tags / versiones de prerregistro

| Tag | Rol |
|-----|-----|
| `gap5-v2-observable-preregistration-v1.2` | **Canónico actual** (OQ1, RESEARCH_MAP, doc 16 header) |
| `gap5-v2-observable-preregistration-v1.1` | Histórico |
| `gap5-v2-observable-preregistration-frozen` | Tag de congelación v1.0; aún citado en `14-adaptive-nhc-protocol.md` L5 y `paso0_property_justification.md` L4 |

**Hallazgo:** tres nombres de tag para la misma línea v2. No son hipótesis distintas; son versiones. Un lector puede creer que “frozen” ≠ “v1.2”.  
**Acción editorial recomendada (no hecha aquí salvo listar):** en docs que citen solo `…-frozen`, añadir “superseded by / current: v1.2”.

Otros tags estables: `gap4-diagnostic-complete`, `gap5-preregistration-frozen`.

---

## 3. Nombres de hipótesis (ambigüedad, no error de contenido)

| Etiqueta | Dónde | Nota de consistencia |
|----------|-------|----------------------|
| **H-ops** | GAP-5 v1 / RESEARCH_MAP | Operacionalización Γ̄ — **refutada** |
| **H5-PoC** | Doc 14/15; **OQ5 y OQ6** | En OQ5 = operacionalización causal online; en OQ6 = ¿política mejora O1–O3? — **misma etiqueta, dos preguntas** |
| **H6-OBS** | Doc 16 §2 (formal) vs OQ1 (lectura afilada G-ext) | Documentado como no-conflicto: formal intacta; OQ1 es motivación experimental |
| **H7-MIN** | Doc 16 / OQ2 | Exploratoria; no sustituye H6-OBS |
| **K4** (“North-axis innovation”) vs G-ext (“North dominance not reproduced”) | STATE vs INTERPRETATION | No es ID mal citado: tensión **ya** acotada en non-claims G-ext / D12 — no reescribir K4 aquí |

---

## 4. IDs citados erróneamente

| Chequeo | Resultado |
|---------|-----------|
| K1–K15 presentes y únicos en STATE | OK |
| OQ1–OQ7 presentes y únicos en OPEN | OK |
| ¿K12 citado como K11 en docs de referencia? | No encontrado en `reference/` |
| RESEARCH_METRICS Y=11 vs lista explícita | Alineado con archivo (conteo editorial) |

---

## 5. Protocolos vs estado operativo

| Doc | Dice | Consistente con D13 / STATUS |
|-----|------|------------------------------|
| README diagnostics “pregunta operativa” | Pausa antes H6 | Sí (actualizado) |
| README raíz “GAP-5 v2 active research” | Antes decía active; roadmap ahora “pause” | Verificar si quedan frases “active” sueltas |
| Doc 16 | CONGELADA prereg; benchmark pending | Sí |
| Doc 14 L5 | v2 “CONGELADA” + tag `…-frozen` | Estado OK; tag desfasado vs v1.2 |

---

## 6. Figuras / artefactos huérfanos (muestreo)

No se hizo borrado. Candidatos a inventario detallado → [REDUNDANCY_INVENTORY.md](REDUNDANCY_INVENTORY.md):

- `G1_intervention/test_crash*.csv`, `test_nis5.csv` — apariencia de depuración
- Múltiples brazos intervention ya cerrados vs baseline G1

---

## 7. Acciones tomadas en esta pasada

1. Corregidos paths relativos en `STATE_OF_KNOWLEDGE.md`.  
2. Resto: **solo listado** (este archivo + redundancias).  
3. **No** se reescribieron hipótesis ni se unificó H5-PoC en OQ5/OQ6 (sería rediseño de etiquetas).
