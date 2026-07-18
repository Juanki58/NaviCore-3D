# Observable characterization (H6) — artefactos numéricos

**Síntesis / modelo de régimen: diferida** — no hay ganador en este archivo.

Protocolo v1.2 · bindings D18 · script `tools/audit_gap5_v2_observable_selection.py`

## C-F1

| Oi | R0 | R1 | R2 | R3 | R4 | C1 | C3 |
|----|----|----|----|----|----|----|----|
| O1 | bajo | alto | alto | bajo | bajo | True | True |
| O2 | bajo | alto | alto | bajo | bajo | True | True |
| O3 | bajo | meseta | meseta | alto | pico | True | True |
| O4 | meseta | meseta | meseta | bajo | pico | False | True |
| O5 | N/A | bajo | alto | pico | alto | False | True |

## C-PoC

| Oi | R0 | R1 | R2 | R3 | R4 | C1 | C3 |
|----|----|----|----|----|----|----|----|
| O1 | bajo | alto | alto | bajo | meseta | True | True |
| O2 | bajo | alto | alto | bajo | meseta | True | True |
| O3 | bajo | meseta | bajo | alto | pico | True | True |
| O4 | meseta | meseta | meseta | bajo | pico | False | True |
| O5 | N/A | bajo | alto | pico | alto | False | True |

## C3 invariance summary

```json
{
  "O1": {
    "C3_meaning_preserved": true,
    "same_regimes": 3,
    "compared_regimes": 4
  },
  "O2": {
    "C3_meaning_preserved": true,
    "same_regimes": 3,
    "compared_regimes": 4
  },
  "O3": {
    "C3_meaning_preserved": true,
    "same_regimes": 3,
    "compared_regimes": 4
  },
  "O4": {
    "C3_meaning_preserved": true,
    "same_regimes": 4,
    "compared_regimes": 4
  },
  "O5": {
    "C3_meaning_preserved": true,
    "same_regimes": 4,
    "compared_regimes": 4
  }
}
```

Parking lot ideas: `IDEAS_DURING_H6.md`.
