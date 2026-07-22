# NHC operational policy (GAP-3 freeze)

**Status:** shipped in code · CI-guarded  
**Source finding:** NHC experiment matrix — [`nhc_experiments/manifest.json`](nhc_experiments/manifest.json)  
**Header:** [`src/core/nhc_ops_policy.hpp`](../src/core/nhc_ops_policy.hpp)

## Why

On the preregistered super-tunnel bank, **NHC every tick (`B_always`) worsened exit drift** vs NHC-off (1408 m vs 493 m). Naive “always on” is not a free integrity win.

## Policies

| Enum | Production-safe? | Behaviour |
|------|------------------|-----------|
| `NHC_OPS_OFF` | **Yes** (default) | No NHC updates — matches `ins_ekf_init` |
| `NHC_OPS_GAP_TRIGGERED` | **Yes** | NHC only when GNSS accept gap ≥ threshold (v2-style) |
| `NHC_OPS_ALWAYS` | **No** | Every eligible tick — lab / A-B only |

Default: `NAVICORE_NHC_OPS_POLICY_DEFAULT` → `NHC_OPS_OFF`.

## Tests

- `tests/unit/test_nhc_ops_policy.cpp` — default, production-safe set, tick gating, init arm
- RapidCheck integrity properties in `test_properties_rapidcheck.cpp` (teleport / small nudge)

```powershell
.\build\navicore_unit_tests.exe "[nhc_ops]"
.\build\navicore_unit_tests.exe "[rapidcheck][integrity]"
```

## Integrator rule

Do **not** ship products with `NHC_OPS_ALWAYS` as the default. Prefer off, or gap-triggered via `ins_ekf_v2_maybe_update_nhc` / `nhc_ops_should_update`.
