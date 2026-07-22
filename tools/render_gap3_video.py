#!/usr/bin/env python3
"""Render GAP-3 explainer MP4 from banked stills + Spanish TTS (edge-tts).

No CapCut/Unity required. Output: docs/video_gap3/NaviCore_GAP3_NHC.mp4
"""
from __future__ import annotations

import asyncio
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg
from PIL import Image

REPO = Path(__file__).resolve().parents[1]
STILLS = REPO / "docs" / "video_gap3" / "stills"
OUT_DIR = REPO / "docs" / "video_gap3"
WORK = OUT_DIR / "_render_work"
OUT_MP4 = OUT_DIR / "NaviCore_GAP3_NHC.mp4"

# Spanish neural voice (Microsoft Edge TTS)
VOICE = "es-ES-AlvaroNeural"

# Full VO matching HOWTO / script (synthetic bank numbers from manifest)
VO_SCRIPT = """
En navegación inercial sin GPS, mucha gente pone Non-Holonomic Constraints a tope.
Nosotros medimos qué pasa.

Mismo escenario sintético super-túnel, mismo IMU, misma salida.
Solo cambia la política NHC.

Con NHC apagado, el error a la salida del túnel es unos cuatrocientos noventa y tres metros,
y al recuperar el GPS se reancla.

Con NHC always-on, el error sube a unos mil cuatrocientos ocho metros.
El filtro se cree demasiado la velocidad del vehículo y el coast empeora.
Incluso el mejor brazo de tuning sigue peor que apagar NHC.

Por eso en producción no vendemos NHC always-on.
Política: NHC off, o gap-triggered en el core v2 — no siempre.

NaviCore-3D: resiliencia GNSS con evidencia publicada.
Monte Carlo, matriz NHC, tooling Allan. MIT, zero-heap, auditable.
GitHub: Juanki58 / NaviCore-3D.
Reproduce con NaviCore3D Sim, opción nhc-experiments.
""".strip()

# Visual beats (fractions of total audio length)
BEATS = [
    ("00_title_card.png", 0.12),
    ("01_exit_drift_bars.png", 0.50),
    ("02_policy_card.png", 0.23),
    ("00_title_card.png", 0.15),  # end card reuse
]


def fit_1080(src: Path, dst: Path) -> None:
    img = Image.open(src).convert("RGB")
    canvas = Image.new("RGB", (1920, 1080), (14, 17, 22))
    img.thumbnail((1920, 1080), Image.Resampling.LANCZOS)
    x = (1920 - img.width) // 2
    y = (1080 - img.height) // 2
    canvas.paste(img, (x, y))
    canvas.save(dst, "PNG")


async def synth_voice(mp3: Path) -> None:
    import edge_tts

    communicate = edge_tts.Communicate(VO_SCRIPT, VOICE)
    await communicate.save(str(mp3))


def ffprobe_duration(ffmpeg: str, media: Path) -> float:
    # imageio-ffmpeg ships ffmpeg only; parse duration via ffmpeg -i stderr
    p = subprocess.run(
        [ffmpeg, "-i", str(media)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    err = p.stderr or ""
    for part in err.replace(",", " ").split():
        # look for Duration: HH:MM:SS.xx
        pass
    # Robust parse
    import re

    m = re.search(r"Duration:\s*(\d+):(\d+):(\d+\.\d+)", err)
    if not m:
        raise RuntimeError(f"Could not parse duration from {media}\n{err[-800:]}")
    h, mi, s = int(m.group(1)), int(m.group(2)), float(m.group(3))
    return h * 3600 + mi * 60 + s


def build_concat_list(durations: list[tuple[Path, float]], list_path: Path) -> None:
    lines: list[str] = []
    for path, dur in durations:
        # ffmpeg concat demuxer wants forward slashes on Windows too
        p = path.resolve().as_posix()
        lines.append(f"file '{p}'")
        lines.append(f"duration {dur:.3f}")
    # last file must be repeated without duration for concat demuxer
    last = durations[-1][0].resolve().as_posix()
    lines.append(f"file '{last}'")
    list_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    WORK.mkdir(parents=True, exist_ok=True)

    for name, _ in BEATS:
        src = STILLS / name
        if not src.is_file():
            print(f"Missing still: {src}", file=sys.stderr)
            return 1

    print("1/4 TTS Spanish voice…")
    mp3 = WORK / "vo_es.mp3"
    asyncio.run(synth_voice(mp3))

    print("2/4 Fit stills to 1080p…")
    fitted: list[Path] = []
    for i, (name, _) in enumerate(BEATS):
        dst = WORK / f"frame_{i:02d}.png"
        fit_1080(STILLS / name, dst)
        fitted.append(dst)

    audio_s = ffprobe_duration(ffmpeg, mp3)
    print(f"   VO duration: {audio_s:.1f} s")
    fracs = [f for _, f in BEATS]
    s = sum(fracs)
    fracs = [f / s for f in fracs]
    durations = [(fitted[i], max(1.0, audio_s * fracs[i])) for i in range(len(fitted))]
    # nudge so sum ~= audio
    drift = audio_s - sum(d for _, d in durations)
    path0, d0 = durations[-1]
    durations[-1] = (path0, d0 + drift)

    concat = WORK / "concat.txt"
    build_concat_list(durations, concat)

    print("3/4 Encode MP4…")
    silent = WORK / "video_silent.mp4"
    cmd1 = [
        ffmpeg,
        "-y",
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(concat),
        "-vf",
        "fps=30,format=yuv420p",
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        str(silent),
    ]
    r1 = subprocess.run(cmd1, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r1.returncode != 0:
        print(r1.stderr[-2000:], file=sys.stderr)
        return 1

    print("4/4 Mux voice…")
    cmd2 = [
        ffmpeg,
        "-y",
        "-i",
        str(silent),
        "-i",
        str(mp3),
        "-c:v",
        "copy",
        "-c:a",
        "aac",
        "-b:a",
        "192k",
        "-shortest",
        str(OUT_MP4),
    ]
    r2 = subprocess.run(cmd2, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r2.returncode != 0:
        print(r2.stderr[-2000:], file=sys.stderr)
        return 1

    size_mb = OUT_MP4.stat().st_size / (1024 * 1024)
    print(f"OK -> {OUT_MP4} ({size_mb:.1f} MB, ~{audio_s:.0f}s)")
    print("Next: upload to YouTube (unlisted) and paste the URL into README Visibility.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
