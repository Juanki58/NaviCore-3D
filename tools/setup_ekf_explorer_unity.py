#!/usr/bin/env python3
"""Wait for Unity Editor install, then create/open EKF Explorer project."""

from __future__ import annotations

import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PROJECT = REPO / "ekf_explorer" / "UnityProject"
ASSETS_READY = REPO / "ekf_explorer" / "AssetsReady" / "NaviCore" / "Scripts"
SESSIONS = REPO / "docs" / "ekf_explorer" / "sessions"
STREAM = PROJECT / "Assets" / "StreamingAssets" / "Sessions"

HUB = Path(r"C:\Program Files\Unity Hub\Unity Hub.exe")
EDITOR_ROOT = Path(r"C:\Program Files\Unity\Hub\Editor")


def find_unity_exe() -> Path | None:
    if not EDITOR_ROOT.is_dir():
        return None
    candidates = sorted(EDITOR_ROOT.glob("*/Editor/Unity.exe"), reverse=True)
    return candidates[0] if candidates else None


def sync_assets() -> None:
    scripts_dst = PROJECT / "Assets" / "NaviCore" / "Scripts"
    scripts_dst.mkdir(parents=True, exist_ok=True)
    if ASSETS_READY.is_dir():
        for p in ASSETS_READY.iterdir():
            shutil.copy2(p, scripts_dst / p.name)
    STREAM.mkdir(parents=True, exist_ok=True)
    if SESSIONS.is_dir():
        for d in SESSIONS.iterdir():
            if d.is_dir():
                dest = STREAM / d.name
                if dest.exists():
                    shutil.rmtree(dest)
                shutil.copytree(d, dest)


def create_project(unity: Path) -> int:
    PROJECT.mkdir(parents=True, exist_ok=True)
    # If ProjectSettings already partially present, just open once to import
    cmd = [
        str(unity),
        "-batchmode",
        "-nographics",
        "-quit",
        "-projectPath",
        str(PROJECT),
        "-logFile",
        str(REPO / "ekf_explorer" / "unity_create_project.log"),
    ]
    # First-time: use -createProject if no Library
    if not (PROJECT / "Library").exists() and not (PROJECT / "ProjectSettings" / "ProjectVersion.txt").exists():
        cmd = [
            str(unity),
            "-batchmode",
            "-nographics",
            "-quit",
            "-createProject",
            str(PROJECT),
            "-logFile",
            str(REPO / "ekf_explorer" / "unity_create_project.log"),
        ]
    print("RUN:", " ".join(cmd))
    return subprocess.call(cmd)


def main() -> int:
    sync_assets()
    unity = find_unity_exe()
    if unity is None:
        print("Unity.exe not found under", EDITOR_ROOT)
        print("Hub still downloading? Check ekf_explorer/unity_editor_install.log")
        print("When Installs shows an editor, re-run: python tools/setup_ekf_explorer_unity.py")
        return 2
    print("Using", unity)
    # Ensure our manifest survives createProject — write after create if needed
    rc = create_project(unity)
    sync_assets()
    # Re-assert Cesium manifest
    manifest = PROJECT / "Packages" / "manifest.json"
    if manifest.is_file():
        data = json.loads(manifest.read_text(encoding="utf-8"))
    else:
        data = {"dependencies": {}}
    data.setdefault("dependencies", {})
    data["dependencies"]["com.cesium.unity"] = "1.24.0"
    data["scopedRegistries"] = [
        {
            "name": "Cesium",
            "url": "https://unity.pkg.cesium.com",
            "scopes": ["com.cesium.unity"],
        }
    ]
    manifest.parent.mkdir(parents=True, exist_ok=True)
    manifest.write_text(json.dumps(data, indent=2), encoding="utf-8")
    print("Wrote Cesium scoped registry into", manifest)
    print("Open in Hub: Add → add project from disk →", PROJECT)
    print("Then menu: NaviCore → EKF Explorer → Create Offline Scene")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
