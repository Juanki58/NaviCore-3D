#!/usr/bin/env python3
"""Render GAP-3 explainer MP4 from banked stills + TTS (edge-tts).

No CapCut/Unity required.
  ES: docs/video_gap3/NaviCore_GAP3_NHC.mp4
  EN: docs/video_gap3/NaviCore_GAP3_NHC_en.mp4
"""
from __future__ import annotations

import argparse
import asyncio
import re
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg
from PIL import Image

REPO = Path(__file__).resolve().parents[1]
STILLS = REPO / "docs" / "video_gap3" / "stills"
OUT_DIR = REPO / "docs" / "video_gap3"
WORK = OUT_DIR / "_render_work"

LANGS = {
    "es": {
        "voice": "es-ES-AlvaroNeural",
        "out": OUT_DIR / "NaviCore_GAP3_NHC.mp4",
        "script": """
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
""".strip(),
    },
    "en": {
        "voice": "en-US-GuyNeural",
        "out": OUT_DIR / "NaviCore_GAP3_NHC_en.mp4",
        "script": """
In GPS-denied inertial navigation, a lot of people turn Non-Holonomic Constraints all the way up.
We measured what happens.

Same synthetic super-tunnel scenario, same IMU, same exit.
Only the NHC policy changes.

With NHC off, tunnel-exit drift is about four hundred ninety-three metres,
and when GNSS returns, it re-anchors.

With NHC always-on, exit drift jumps to about one thousand four hundred eight metres.
The filter trusts vehicle velocity too much, and coasting gets worse.
Even the best tuning arm still loses to turning NHC off.

That is why we do not ship NHC always-on in production.
Policy: NHC off, or gap-triggered in the v2 core — not always-on.

NaviCore-3D: GNSS resilience with published evidence.
Monte Carlo, NHC matrix, Allan tooling. MIT, zero-heap, auditable.
GitHub: Juanki58 / NaviCore-3D.
Reproduce with NaviCore3D Sim, flag nhc-experiments.
""".strip(),
    },
}

BEATS = [
    ("00_title_card.png", 0.12),
    ("01_exit_drift_bars.png", 0.50),
    ("02_policy_card.png", 0.23),
    ("00_title_card.png", 0.15),
]


def fit_1080(src: Path, dst: Path) -> None:
    img = Image.open(src).convert("RGB")
    canvas = Image.new("RGB", (1920, 1080), (14, 17, 22))
    img.thumbnail((1920, 1080), Image.Resampling.LANCZOS)
    x = (1920 - img.width) // 2
    y = (1080 - img.height) // 2
    canvas.paste(img, (x, y))
    canvas.save(dst, "PNG")


async def synth_voice(mp3: Path, text: str, voice: str) -> None:
    import edge_tts

    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(str(mp3))


def ffprobe_duration(ffmpeg: str, media: Path) -> float:
    p = subprocess.run(
        [ffmpeg, "-i", str(media)],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    err = p.stderr or ""
    m = re.search(r"Duration:\s*(\d+):(\d+):(\d+\.\d+)", err)
    if not m:
        raise RuntimeError(f"Could not parse duration from {media}\n{err[-800:]}")
    h, mi, s = int(m.group(1)), int(m.group(2)), float(m.group(3))
    return h * 3600 + mi * 60 + s


def build_concat_list(durations: list[tuple[Path, float]], list_path: Path) -> None:
    lines: list[str] = []
    for path, dur in durations:
        p = path.resolve().as_posix()
        lines.append(f"file '{p}'")
        lines.append(f"duration {dur:.3f}")
    last = durations[-1][0].resolve().as_posix()
    lines.append(f"file '{last}'")
    list_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def render_one(lang: str) -> int:
    cfg = LANGS[lang]
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    work = WORK / lang
    work.mkdir(parents=True, exist_ok=True)

    for name, _ in BEATS:
        src = STILLS / name
        if not src.is_file():
            print(f"Missing still: {src}", file=sys.stderr)
            return 1

    print(f"[{lang}] 1/4 TTS…")
    mp3 = work / "vo.mp3"
    asyncio.run(synth_voice(mp3, cfg["script"], cfg["voice"]))

    print(f"[{lang}] 2/4 Fit stills…")
    fitted: list[Path] = []
    for i, (name, _) in enumerate(BEATS):
        dst = work / f"frame_{i:02d}.png"
        fit_1080(STILLS / name, dst)
        fitted.append(dst)

    audio_s = ffprobe_duration(ffmpeg, mp3)
    print(f"[{lang}]    VO duration: {audio_s:.1f} s")
    fracs = [f for _, f in BEATS]
    s = sum(fracs)
    fracs = [f / s for f in fracs]
    durations = [(fitted[i], max(1.0, audio_s * fracs[i])) for i in range(len(fitted))]
    drift = audio_s - sum(d for _, d in durations)
    path0, d0 = durations[-1]
    durations[-1] = (path0, d0 + drift)

    concat = work / "concat.txt"
    build_concat_list(durations, concat)

    print(f"[{lang}] 3/4 Encode…")
    silent = work / "video_silent.mp4"
    r1 = subprocess.run(
        [
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
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if r1.returncode != 0:
        print(r1.stderr[-2000:], file=sys.stderr)
        return 1

    out_mp4: Path = cfg["out"]
    print(f"[{lang}] 4/4 Mux…")
    r2 = subprocess.run(
        [
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
            str(out_mp4),
        ],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    if r2.returncode != 0:
        print(r2.stderr[-2000:], file=sys.stderr)
        return 1

    size_mb = out_mp4.stat().st_size / (1024 * 1024)
    print(f"[{lang}] OK -> {out_mp4} ({size_mb:.1f} MB, ~{audio_s:.0f}s)")
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Render GAP-3 stills+TTS video")
    ap.add_argument(
        "--lang",
        choices=("es", "en", "both"),
        default="both",
        help="Language (default: both)",
    )
    args = ap.parse_args(argv)
    langs = ["es", "en"] if args.lang == "both" else [args.lang]
    rc = 0
    for lang in langs:
        r = render_one(lang)
        if r != 0:
            rc = r
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
