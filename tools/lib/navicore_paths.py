"""Put all tools/<category>/ dirs on sys.path for cross-script imports."""

from __future__ import annotations

import sys
from pathlib import Path


def ensure_tools_path(repo_root: Path | str) -> Path:
    root = Path(repo_root).resolve()
    tools = root / "tools"
    # Prefer lib first for shared codecs / geodesy.
    ordered = ["lib"]
    if tools.is_dir():
        ordered.extend(
            sorted(
                p.name
                for p in tools.iterdir()
                if p.is_dir() and p.name not in {"lib", "__pycache__"}
            )
        )
    for name in reversed(ordered):
        path = tools / name
        if path.is_dir():
            text = str(path)
            if text not in sys.path:
                sys.path.insert(0, text)
    return root
