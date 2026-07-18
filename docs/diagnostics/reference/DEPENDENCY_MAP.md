# Dependency Map — Trazabilidad K / fases / OQ

**Tipo:** editorial — comprobar que no hay huecos obvios entre nodos ya congelados.  
**No** propone aristas nuevas de conocimiento.  
**Fecha:** 2026-07-18

---

## 1. Cadena de fases

```mermaid
flowchart TD
  G1[GAP-1 body mount CLOSED]
  G2[GAP-2 dynamics CLOSED]
  G3[GAP-3 mechanism CLOSED]
  F1[F1 dose-response REFUTED]
  F11[F1.1 gate anatomy REFUTED hyp]
  F12[F1.2 decimation REFUTED hyp]
  G4[GAP-4 P_pv CLOSED]
  G5v1[GAP-5 v1 H-ops REFUTED ops]
  GEXT[G-ext partial confirm CLOSED]
  G5v2[GAP-5 v2 preregistered PAUSED]

  G1 --> G2 --> G3
  G3 --> F1 --> F11 --> F12 --> G4
  G4 --> G5v1
  G4 --> GEXT
  G5v1 --> GEXT
  GEXT --> G5v2
```

---

## 2. Conocimiento consolidado → fases

```mermaid
flowchart LR
  subgraph GAP3[GAP-3 / F1*]
    K1[K1 NHC freq / Pvv]
    K2[K2 NHC vs accepts]
    K3[K3 k_vel no restores accepts]
    K4[K4 North / Lambda gate]
    K5[K5 bursty cliff]
    K6[K6 Gamma offline NHC]
    K7[K7 Joseph partial]
  end
  subgraph GAP4[GAP-4]
    K8[K8 Ppv legitimate]
    K9[K9 fix4 bifurcation]
    K10[K10 cos from logs]
  end
  subgraph GAP5v1[GAP-5 v1]
    K11[K11 Gamma_inst tracks]
    K12[K12 Gamma_bar fails ops]
    K13[K13 Gamma not invariant PoC]
  end
  subgraph GEXT[G-ext]
    K14[K14 lockout core]
    K15[K15 clean GNSS vs reject]
  end

  F1 --> K1
  F1 --> K3
  F11 --> K3
  F11 --> K4
  F12 --> K5
  G3 --> K6
  G3 --> K7
  G4 --> K8
  G4 --> K9
  G4 --> K10
  G5v1 --> K11
  G5v1 --> K12
  G5v1 --> K13
  GEXT --> K14
  GEXT --> K15
```

---

## 3. Refutaciones operativas → preguntas abiertas

```mermaid
flowchart TD
  K3[K3 restore k_vel fails]
  K8[K8 Ppv not bug]
  K12[K12 Gamma_bar ops fails]
  K13[K13 Gamma not invariant]
  K14[K14 lockout generalizes]
  K15[K15 external vs internal]

  K12 --> OQ1[OQ1 / H6-OBS property]
  K13 --> OQ3[OQ3 invariance C-F1 C-PoC]
  K15 --> OQ1
  K14 --> OQ1
  OQ1 --> OQ2[OQ2 H7-MIN]
  OQ1 --> OQ4[OQ4 regime_model]
  OQ1 -.->|after freeze| OQ5[OQ5 ops online H5-PoC]
  OQ4 -.->|after freeze| OQ6[OQ6 policy vs B0]
  K9 --> OQ7[OQ7 Ppv intervention §11]
  K8 --> OQ7
```

---

## 4. Decisiones que bloquean reaperturas

| Decisión | Protege |
|----------|---------|
| D2 | No retune R/Q/K como vía primaria |
| D10 | No escribir §17 lessons hasta cierre v2 |
| D12 | No “Norte→longitudinal”; no “G-ext=G1 completo” |
| D13 | No abrir benchmark H6 todavía |

---

## 5. Huecos / tensiones (solo observación)

| Ítem | ¿Hueco? |
|------|---------|
| GAP-1/2 → GAP-3 | Encadenado en doc 09/10; no falta arista documental |
| K9 ↔ G-ext | G-ext **no** alimenta K9 (non-claim explícito) — arista deliberadamente ausente |
| H5-PoC en OQ5 y OQ6 | Misma etiqueta, dos OQ — ambigüedad de nombre (ver CONSISTENCY_AUDIT), no falta de nodo |
| OQ7 vs GAP-5 v2 | Separados por diseño (no mezclar brazos) |

**Conclusión editorial:** no aparece un K huérfano sin fase, ni un OQ de v2 sin precursor en K12/K13/K14/K15. La pausa D13 es el único “siguiente nodo” no ejecutado.
