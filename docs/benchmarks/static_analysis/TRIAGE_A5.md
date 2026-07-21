# A5 triage — cppcheck / clang-tidy (2026-07-21)

Scope: `src/core/`. Full run: `docs/benchmarks/static_analysis/REPORT_triage_a5.md` + `cppcheck_triage_a5.xml`.

## Disposition summary

| Finding id | Count | Disposition |
|------------|------:|-------------|
| `normalCheckLevelMaxBranches` | 12 | **Accept** — informational; use `--check-level=exhaustive` only in deep audits |
| `memsetClassFloat` | 9 | **Accept (documented)** — intentional zeroing of POD filters with floats; IEEE zero bit pattern on our targets |
| `compareValueOutOfTypeRangeError` / `unsignedPositive` | 3+3 | **Fixed** — redundant `r/c >= INS_ERR_POS_N` (always true for `uint8_t` when POS_N=0) |
| `dangerousTypeCast` (command_ingestor) | 1 | **Fixed** — `reinterpret_cast` + `static_cast` |
| `passedByValue` / `constParameter*` / style | few | **Defer** — micro-opts; no safety impact |
| clang-tidy CI | — | **Soft fail kept** (`continue-on-error`) until guideline noise is whitelisted in `.clang-tidy`; cppcheck + ASan remain hard gates |

## Fixed in this pass

- `ins_ekf.cpp`: position-index range checks without `>= 0` / `>= INS_ERR_POS_N`
- `command_ingestor.cpp`: C++ casts for packet checksum walk

## Reproduce

```powershell
python tools\run_static_analysis.py --cppcheck --tag triage_a5
```

A5 status: **triaged + actionable style bugs fixed**; residual noise classified. Not “zero findings” (not the goal).
