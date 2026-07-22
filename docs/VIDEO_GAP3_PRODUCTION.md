# Vídeo GAP-3 — pack de producción

**Estado:** guion + stills listos · **falta tu grabación** (Unity local o edición con stills)  
**Guion VO:** [`VIDEO_SCRIPT_GAP3_NHC.md`](VIDEO_SCRIPT_GAP3_NHC.md)  
**Números:** [`nhc_experiments/manifest.json`](nhc_experiments/manifest.json)  
**Política:** [`NHC_OPS_POLICY.md`](NHC_OPS_POLICY.md)

## Qué ya está en el repo

| Artefacto | Uso |
|-----------|-----|
| `docs/video_gap3/stills/00_title_card.png` | Apertura 0–3 s |
| `docs/video_gap3/stills/01_exit_drift_bars.png` | Resultado 25–70 s (A vs B_always) |
| `docs/video_gap3/stills/02_policy_card.png` | Cierre / takeaway |
| `docs/video_gap3/stills/overlays.txt` | Textos para subtítulos / TextMeshPro |
| `tools/plot_gap3_video_assets.py` | Regenerar stills tras nuevo manifiesto |

```powershell
python tools\plot_gap3_video_assets.py
```

## Dos caminos de grabación

### A — Rápido (sin Unity, hoy)

**Guía de principiante (sigue esta):** [`VIDEO_GAP3_HOWTO.md`](VIDEO_GAP3_HOWTO.md) — CapCut + 3 PNG + VO leída en voz alta.

Resumen: timeline title → bars → policy → export 1080p30 → YouTube no listado → link en README.

### B — Instrumento (Unity + Cesium, local)

`ekf_explorer/` es **local-only** (binarios grandes). Protocolo: [`diagnostics/19-ekf-explorer-protocol.md`](diagnostics/19-ekf-explorer-protocol.md).

1. Abrir sesión con traza super-tunnel / NHC arms si el session pack lo permite.  
2. HUD: modo `HYBRID`→`DEAD_RECKONING`; contraste A vs `B_always`.  
3. Insertar stills del repo como lower-thirds (números del manifiesto, no inventar).  
4. Misma VO / duración.

## Checklist de rodaje

- [ ] VO grabada (guion sin claims mil-grade / RF spoof)  
- [ ] Números en pantalla = manifiesto (493 / 1408)  
- [ ] Mencionar reproduce: `NaviCore3D_Sim.exe --nhc-experiments`  
- [ ] End card: repo URL + “NHC off or gap-triggered”  
- [ ] Link añadido a README Visibility cuando el vídeo esté publicado

## No hacer

- No decir “NHC siempre ayuda”.  
- No usar RF spoof en la demo.  
- No mostrar `ALWAYS` como default de producto.
