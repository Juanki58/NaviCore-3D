#!/usr/bin/env python3
"""Run static analysis / optional coverage helpers for NaviCore-3D.

Examples:
  python tools/run_static_analysis.py --cppcheck
  python tools/run_static_analysis.py --clang-tidy
  python tools/run_static_analysis.py --all
  python tools/run_static_analysis.py --coverage-report

Artefacts land in docs/benchmarks/static_analysis/
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
OUT_DIR = REPO / "docs" / "benchmarks" / "static_analysis"
CORE_DIR = REPO / "src" / "core"
PC_DIR = REPO / "src" / "targets" / "generic_pc"

CORE_GLOBS = [
    str(CORE_DIR / "*.cpp"),
    str(CORE_DIR / "*.hpp"),
    str(CORE_DIR / "*.h"),
]


def which(name: str) -> str | None:
    found = shutil.which(name)
    if found:
        return found
    # Windows: pip --user Scripts often not on PATH
    if os.name == "nt" and name.lower() in ("gcovr", "gcovr.exe"):
        local = Path.home() / "AppData" / "Local" / "Packages"
        if local.exists():
            for candidate in local.glob(
                "PythonSoftwareFoundation.Python.*/LocalCache/local-packages/Python*/Scripts/gcovr.exe"
            ):
                return str(candidate)
        user_scripts = Path.home() / "AppData" / "Roaming" / "Python"
        for candidate in user_scripts.glob("Python*/Scripts/gcovr.exe"):
            return str(candidate)
    return None


def stamp() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def run(cmd: list[str], log_path: Path) -> int:
    print("[*]", " ".join(cmd))
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, encoding="utf-8", errors="replace")
    log_path.write_text(
        f"$ {' '.join(cmd)}\nexit={proc.returncode}\n\n--- stdout ---\n{proc.stdout}\n--- stderr ---\n{proc.stderr}\n",
        encoding="utf-8",
    )
    if proc.stdout:
        print(proc.stdout[-4000:])
    if proc.stderr:
        print(proc.stderr[-4000:], file=sys.stderr)
    return proc.returncode


def run_cppcheck(tag: str) -> dict:
    exe = which("cppcheck")
    if not exe:
        return {"tool": "cppcheck", "status": "skipped", "reason": "cppcheck not on PATH"}

    xml_out = OUT_DIR / f"cppcheck_{tag}.xml"
    txt_out = OUT_DIR / f"cppcheck_{tag}.txt"
    cmd = [
        exe,
        "--enable=all",
        "--std=c++17",
        "--force",
        "--inline-suppr",
        "--error-exitcode=0",
        f"--suppress=missingIncludeSystem",
        f"--suppress=unmatchedSuppression",
        f"--suppress=unusedFunction",
        f"-I{CORE_DIR}",
        f"-I{CORE_DIR / 'interfaces'}",
        f"-I{PC_DIR}",
        str(CORE_DIR),
        f"--output-file={xml_out}",
        "--xml",
        "--xml-version=2",
    ]
    # Also human-readable: cppcheck writes XML to --output-file; capture console via second pass
    code = run(cmd, txt_out.with_suffix(".runlog.txt"))
    # Count issues from XML if present
    issues = 0
    error_ids: dict[str, int] = {}
    if xml_out.exists():
        text = xml_out.read_text(encoding="utf-8", errors="replace")
        issues = text.count("<error ")
        for line in text.splitlines():
            if 'id="' in line and "<error " in line:
                try:
                    eid = line.split('id="', 1)[1].split('"', 1)[0]
                    error_ids[eid] = error_ids.get(eid, 0) + 1
                except IndexError:
                    pass

    top = sorted(error_ids.items(), key=lambda kv: (-kv[1], kv[0]))[:15]
    summary = {
        "tool": "cppcheck",
        "status": "ok" if code == 0 else "failed",
        "exit_code": code,
        "xml": str(xml_out.relative_to(REPO)).replace("\\", "/"),
        "issue_count": issues,
        "top_ids": top,
        "scope": "src/core",
        "flags": "--enable=all --std=c++17",
    }
    (OUT_DIR / f"cppcheck_{tag}_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def run_clang_tidy(tag: str) -> dict:
    exe = which("clang-tidy")
    if not exe:
        return {
            "tool": "clang-tidy",
            "status": "skipped",
            "reason": "clang-tidy not on PATH (install LLVM; checks: cppcoreguidelines-*, bugprone-*)",
        }

    cpp_files = sorted(CORE_DIR.glob("*.cpp"))
    log = OUT_DIR / f"clang_tidy_{tag}.txt"
    # Compilation database optional; fall back to -- for single-file compile flags
    compile_flags = [
        "--",
        "-std=c++17",
        f"-I{CORE_DIR}",
        f"-I{CORE_DIR / 'interfaces'}",
        f"-I{PC_DIR}",
    ]
    all_out: list[str] = []
    exit_code = 0
    for src in cpp_files:
        cmd = [exe, str(src), "-p", str(REPO / "build")] if (REPO / "build" / "compile_commands.json").exists() else [exe, str(src), *compile_flags]
        # Prefer explicit flags when no compile_commands
        if not (REPO / "build" / "compile_commands.json").exists():
            cmd = [exe, str(src), *compile_flags]
        else:
            cmd = [exe, "-p", str(REPO / "build"), str(src)]
        proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True, encoding="utf-8", errors="replace")
        all_out.append(f"===== {src.name} exit={proc.returncode} =====\n{proc.stdout}\n{proc.stderr}\n")
        if proc.returncode not in (0,):
            exit_code = proc.returncode
    log.write_text("\n".join(all_out), encoding="utf-8")
    warning_lines = sum(1 for line in "\n".join(all_out).splitlines() if "warning:" in line)
    summary = {
        "tool": "clang-tidy",
        "status": "ok" if exit_code == 0 else "completed_with_findings",
        "exit_code": exit_code,
        "log": str(log.relative_to(REPO)).replace("\\", "/"),
        "warning_lines": warning_lines,
        "files": len(cpp_files),
        "config": ".clang-tidy",
    }
    (OUT_DIR / f"clang_tidy_{tag}_summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    return summary


def write_markdown_report(tag: str, parts: list[dict]) -> Path:
    path = OUT_DIR / f"REPORT_{tag}.md"
    lines = [
        f"# Static analysis report — {tag}",
        "",
        f"Generated (UTC): {stamp()}",
        "",
        "Scope: `src/core/` (safety-critical ESKF / fusion / guards).",
        "Standard: [docs/SAFETY_CODING_STANDARD.md](../../SAFETY_CODING_STANDARD.md) (MISRA-inspired, not certified).",
        "",
        "| Tool | Status | Notes |",
        "|------|--------|-------|",
    ]
    for p in parts:
        tool = p.get("tool", "?")
        status = p.get("status", "?")
        if tool == "cppcheck":
            notes = f"issues≈{p.get('issue_count', '?')}; top={p.get('top_ids', [])[:5]}"
        elif tool == "clang-tidy":
            notes = p.get("reason") or f"warning_lines={p.get('warning_lines')}"
        else:
            notes = p.get("reason") or json.dumps({k: v for k, v in p.items() if k not in ('tool', 'status')})[:120]
        lines.append(f"| {tool} | **{status}** | {notes} |")
    lines.extend(
        [
            "",
            "## Reproduce",
            "",
            "```powershell",
            "python tools\\run_static_analysis.py --all",
            "```",
            "",
            "Sanitizers (separate build tree):",
            "",
            "```powershell",
            "cmake -S . -B build_asan -G \"MinGW Makefiles\" -DCMAKE_BUILD_TYPE=Debug -DNAVICORE_ENABLE_SANITIZERS=ON",
            "cmake --build build_asan --target navicore_regression_test",
            ".\\build_asan\\navicore_regression_test.exe",
            "```",
            "",
            "Coverage:",
            "",
            "```powershell",
            "python tools\\run_static_analysis.py --coverage-build",
            "```",
            "",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    # Stable "latest" pointer
    latest = OUT_DIR / "REPORT_LATEST.md"
    latest.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    return path


def coverage_build() -> dict:
    """Configure+build+run regression with gcov flags; summarize .gcda if gcov available."""
    build_dir = REPO / "build_coverage"
    build_dir.mkdir(exist_ok=True)
    cmake = which("cmake")
    if not cmake:
        return {"tool": "coverage", "status": "skipped", "reason": "cmake missing"}

    conf = [
        cmake,
        "-S",
        str(REPO),
        "-B",
        str(build_dir),
        "-G",
        "MinGW Makefiles",
        "-DCMAKE_BUILD_TYPE=Debug",
        "-DNAVICORE_ENABLE_COVERAGE=ON",
        "-DNAVICORE_ENABLE_SANITIZERS=OFF",
    ]
    log = OUT_DIR / f"coverage_{stamp()}.runlog.txt"
    # Use fixed name for latest
    log = OUT_DIR / "coverage_build.runlog.txt"
    code = run(conf, log)
    if code != 0:
        return {"tool": "coverage", "status": "failed", "reason": "cmake configure failed", "log": str(log)}

    code = run(
        [cmake, "--build", str(build_dir), "--target", "navicore_regression_test", "-j", "8"],
        OUT_DIR / "coverage_build_compile.runlog.txt",
    )
    if code != 0:
        return {"tool": "coverage", "status": "failed", "reason": "build failed"}

    exe = build_dir / ("navicore_regression_test.exe" if os.name == "nt" else "navicore_regression_test")
    code = run([str(exe)], OUT_DIR / "coverage_regression.runlog.txt")

    gcov_files = list(build_dir.rglob("*.gcda"))
    summary = {
        "tool": "coverage",
        "status": "ok" if gcov_files else "built_but_no_gcda",
        "regression_exit": code,
        "gcda_count": len(gcov_files),
        "note": "Install gcovr or lcov for HTML %; .gcda present after regression run",
        "build_dir": "build_coverage",
    }
    gcovr = which("gcovr")
    if gcovr and gcov_files:
        html_dir = OUT_DIR / "coverage_html"
        html_dir.mkdir(exist_ok=True)
        run(
            [
                gcovr,
                "--root",
                str(REPO),
                "--filter",
                str(CORE_DIR).replace("\\", "/"),
                "--html-details",
                str(html_dir / "index.html"),
                "--txt",
                str(OUT_DIR / "coverage_core.txt"),
                str(build_dir),
            ],
            OUT_DIR / "gcovr.runlog.txt",
        )
        summary["html"] = "docs/benchmarks/static_analysis/coverage_html/index.html"
        summary["txt"] = "docs/benchmarks/static_analysis/coverage_core.txt"
    (OUT_DIR / "coverage_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cppcheck", action="store_true")
    parser.add_argument("--clang-tidy", action="store_true")
    parser.add_argument("--coverage-build", action="store_true")
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--tag", default=None, help="Artefact tag (default: UTC stamp)")
    args = parser.parse_args()

    do_cpp = args.cppcheck or args.all
    do_tidy = args.clang_tidy or args.all
    do_cov = args.coverage_build
    if not (do_cpp or do_tidy or do_cov):
        do_cpp = True  # default cheap pass

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    tag = args.tag or stamp()
    parts: list[dict] = []

    if do_cpp:
        parts.append(run_cppcheck(tag))
    if do_tidy:
        parts.append(run_clang_tidy(tag))
    if do_cov:
        parts.append(coverage_build())

    report = write_markdown_report(tag, parts)
    print(f"[*] Report -> {report}")
    # Non-zero only if a requested tool hard-failed (not skipped)
    for p in parts:
        if p.get("status") == "failed":
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
