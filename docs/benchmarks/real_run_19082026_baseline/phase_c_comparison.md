# G-ext Phase C — Comparación mecanicista vs G1

**Same-story count:** 6/6
**Validacion externa fuerte (>=5/6):** SI

| Propiedad | G1 | G-ext | ¿Misma historia? |
|-----------|----|-------|------------------|
| Burst NHC / NHC floor | ✓ | ✓ | ✓ |
| Compresión P_vv | ✓ | ✓ | ✓ |
| Crecimiento P_pv | ✓ | ✓ | ✓ |
| Innovación Norte dominante | ✓ | ✓ | ✓ |
| Evolución Λ_N (rejects elevados) | ✓ | ✓ | ✓ |
| Rechazos GNSS (mayoría reject) | ✓ | ✓ | ✓ |

## Conteos GNSS

- G1: accepts=8 rejects=323 events=331
- G-ext: accepts=1 rejects=680 events=681

## Detalle clave

- G1 P_vv floor NHC: 0.03090602532
- G-ext P_vv floor NHC: 0.02384707332
- G1 Lambda_N median rejects: 56.309548667461975
- G-ext Lambda_N median rejects: 267.15727805498364
- G1 inter-GNSS max top3: 0.43921704141153295 pattern=uniform
- G-ext inter-GNSS max top3: 0.21816790656447738 pattern=uniform

## Nota

No se evalua RMSE ni deriva. Solo propiedades mecanicistas.
El burst clasico F1 (pos-only, gamma~19.7) no se re-mide aqui;
bajo shell G1 se puntua scan inter-GNSS + floor P_vv por NHC.

