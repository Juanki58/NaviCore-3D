# Evidence closeout rule

**Rule:** a hardware/lab campaign is **not done** when a CSV lands in `docs/`.  
It is done when the **README Evidence scorecard** (or the named Evidence subsection) shows the result in a table a stranger can find in ≤30 s.

Same failure mode we already fixed for MC/NHC/Allan tooling: real work invisible because it lived only in artefact folders.

## Applies to (minimum three)

| Campaign | Artefacts | README must update |
|----------|-----------|--------------------|
| **Allan fit** (B6 / S3) | `docs/imu_static_log.csv` + PNG | Evidence § Allan — ARW/BI/RRW IEEE + DUT/fs/duration |
| **Field outage Pico** (B1) | `docs/benchmarks/field_outage/<date>/` | Evidence — drift @ 30/60/120 s + mode notes |
| **Fault injection bank** (B5) | `docs/benchmarks/fault_injection/<date>/` | Evidence — pass/fail table per physical fault |

Optional but same rule: **PPK2** (B3) → mA/mW table in README Power section.

## Checklist (every campaign)

1. Commit CSV / NOTES / logs under the campaign folder.  
2. **Paste a short results table into README Evidence** (or Power for PPK2).  
3. Flip roadmap row to **Hecho** with link to both artefact **and** README anchor.  
4. Push — invisible work does not count.

Host-only smokes (e.g. `20260722_host`) may stay under benchmarks until the **physical** bank run; when the physical campaign closes, README gets the bank table.
