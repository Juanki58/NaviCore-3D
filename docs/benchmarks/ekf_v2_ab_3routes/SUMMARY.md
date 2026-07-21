# EKF v1 vs v2 - A/B NHC-off (3 routes)

| Route | Core | Accept rate | Final drift H [m] | Residual @60s [m] |
|-------|------|-------------|-------------------|-------------------|
| REF_19082026 | v1 | 0.0264 | 158140.031 | None |
| REF_19082026 | v2 | 1.0000 | 35.288 | None |
| REF_19082026 | verdict | accept_up=True drift_down=True | pass=True | |
| ALT_16072026 | v1 | 0.0242 | 856262.25 | None |
| ALT_16072026 | v2 | 1.0000 | 38.002 | None |
| ALT_16072026 | verdict | accept_up=True drift_down=True | pass=True | |
| JUL17_20260717 | v1 | 0.1063 | 332215.656 | None |
| JUL17_20260717 | v2 | 1.0000 | 110.295 | None |
| JUL17_20260717 | verdict | accept_up=True drift_down=True | pass=True | |

**Overall:** PASS - see SUMMARY.json
