# Cómo hacer el vídeo GAP-3

## Opción 0 — Ya está hecho (recomendado)

MP4 generados automáticamente:

| Idioma | Archivo |
|--------|---------|
| ES | `C:\NaviCore-3D\docs\video_gap3\NaviCore_GAP3_NHC.mp4` |
| EN | `C:\NaviCore-3D\docs\video_gap3\NaviCore_GAP3_NHC_en.mp4` |

(~70 s, stills del manifiesto + voz TTS)

1. Ábrelos y comprueba que suenan bien.  
2. Súbelos a [YouTube](https://www.youtube.com/upload) → visibilidad **No listado** (puedes hacer dos vídeos o un playlist).  
3. Pega las URLs aquí en el chat o en README Visibility.

Regenerar:

```powershell
python tools\render_gap3_video.py          # ES + EN
python tools\render_gap3_video.py --lang en
```

---

## Opción 1 — CapCut a mano (si quieres tu voz)

No hace falta Cesium ni saber editar. Solo: **3 imágenes del repo + tu voz + CapCut**.

Carpeta de imágenes (Explorador de Windows):

`C:\NaviCore-3D\docs\video_gap3\stills\`

| Archivo | Qué es |
|---------|--------|
| `00_title_card.png` | Portada |
| `01_exit_drift_bars.png` | Gráfico 493 m vs 1408 m |
| `02_policy_card.png` | Política OFF / gap-triggered |
| `overlays.txt` | Frases cortas (opcional, subtítulos) |

---

## 1. Instalar CapCut (5 min)

1. Abre el Microsoft Store o ve a [capcut.com](https://www.capcut.com/) → **Desktop**.  
2. Instala **CapCut**.  
3. Abre CapCut → **Nuevo proyecto** → relación **16:9** → resolución **1080p**.

*(Si prefieres el móvil: misma app CapCut en el teléfono; copia las 3 PNG al teléfono.)*

---

## 2. Montar la línea de tiempo (10 min)

Arrastra las imágenes **en este orden** a la pista de vídeo:

| Orden | Archivo | Duración sugerida |
|------:|---------|-------------------|
| 1 | `00_title_card.png` | **8 s** |
| 2 | `01_exit_drift_bars.png` | **45 s** |
| 3 | `02_policy_card.png` | **25 s** |
| 4 | (opcional) otra vez `00_title_card.png` o pantalla negra | **15 s** para el cierre / URL |

Cómo poner duración en CapCut:

1. Clic en la imagen en la timeline.  
2. Arriba o en el panel: **Duración** → escribe los segundos.  
3. Repite para cada clip.

Total ≈ **90–95 s** (está bien; el guion admite hasta 120 s).

---

## 3. Grabar la voz (10–15 min)

### Opción fácil: grabar dentro de CapCut

1. Menú **Audio** → **Grabar voz en off** (o “Voiceover”).  
2. Pon el cabezal al inicio.  
3. Pulsa grabar y **lee en voz alta** el texto de abajo (despacio, como si explicaras a un colega).  
4. Si te equivocas, borra ese tramo y vuelve a grabar solo esa frase.

### Opción alternativa: móvil

1. Abre la app **Grabadora** del móvil.  
2. Lee el texto completo.  
3. Pasa el `.m4a`/`.mp3` al PC e **impórtalo** a CapCut en la pista de audio.  
4. Ajusta para que coincida con las imágenes (arrastra el audio).

### Texto para leer (cópialo tal cual)

*(Pausa breve entre párrafos.)*

**[imagen título — primeros ~8 s]**  
En navegación inercial sin GPS, mucha gente pone Non-Holonomic Constraints a tope. Nosotros medimos qué pasa.

**[imagen barras — ~45 s]**  
Mismo escenario sintético super-túnel, mismo IMU, misma salida. Solo cambia la política NHC.  
Con NHC apagado, el error a la salida del túnel es unos cuatrocientos noventa y tres metros, y al recuperar el GPS se reancla.  
Con NHC siempre activo, el error sube a unos mil cuatrocientos ocho metros. El filtro se cree demasiado la velocidad del vehículo y el coast empeora.  
Incluso el mejor brazo de tuning sigue peor que apagar NHC.

**[imagen política — ~25 s]**  
Por eso en producción no vendemos NHC always-on. Política: NHC off, o gap-triggered en el core v2 — no siempre.

**[cierre — últimos ~15 s]**  
NaviCore-3D: resiliencia GNSS con evidencia publicada. Monte Carlo, matriz NHC, tooling Allan. MIT, zero-heap, auditable.  
GitHub: Juanki58 slash NaviCore-3D. Reproduce con NaviCore3D Sim, guión nhc-experiments.

---

## 4. Detalles opcionales (5 min)

- **Subtítulos:** Texto → añade frases de `overlays.txt` (493 m / 1408 m).  
- **Zoom suave:** clic en la imagen → Animación → “Ken Burns” / zoom lento (queda más vivo).  
- **Música:** solo si es muy baja; la voz manda. Sin música también está perfecto.  
- **No digas:** anti-jam militar, RF spoof, “NHC nunca sirve”.

---

## 5. Exportar

1. **Exportar** (arriba a la derecha).  
2. Resolución **1080p**, fps **30**, formato **MP4**.  
3. Guarda p.ej. `C:\Users\juanc\Videos\NaviCore_GAP3_NHC.mp4`.

---

## 6. Publicar (elige una)

### YouTube (recomendado para link en README)

1. [youtube.com/upload](https://www.youtube.com/upload)  
2. Sube el MP4.  
3. Visibilidad: **No listado** (unlisted) al principio está bien.  
4. Título sugerido: `NaviCore-3D GAP-3: NHC always-on empeora el coast (493 m vs 1408 m)`  
5. Descripción: enlace al repo + “números de docs/nhc_experiments/manifest.json”.  
6. Copia la URL.

### LinkedIn

Sube el mismo MP4 como post (o pega el link de YouTube). Texto corto: el hallazgo en una frase + link al repo.

---

## 7. Dejarlo visible en el repo

Cuando tengas la URL, dímela o edita el README § Roadmap / Visibility y pega el enlace.  
Eso cierra C1 según [`EVIDENCE_CLOSEOUT.md`](EVIDENCE_CLOSEOUT.md): el vídeo no cuenta si solo vive en tu disco.

---

## Si te atascas

| Problema | Qué hacer |
|----------|-----------|
| No encuentro las PNG | Explorador → `C:\NaviCore-3D\docs\video_gap3\stills\` |
| CapCut pide cuenta | Puedes crear una gratis; o usa **Clipchamp** (viene en Windows 11): mismos 3 PNG + voz. |
| Me da vergüenza la voz | Graba solo audio con el móvil, una toma mala vale; el mensaje son los números. |
| Quiero Unity | Eso es el camino B — más tarde; **no bloquees** el vídeo por eso. |

**Éxito = MP4 de ~2 min + URL pública/no listada.** No hace falta que sea bonito; hace falta que exista.
