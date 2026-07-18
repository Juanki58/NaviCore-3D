# Experimento H0 / H2 / H3

Montaje: `calibration/imu_mount.json` (Rodrigues) en las tres corridas.

| Metrica | H0 | H2 | H3 |
|---------|----|----|-----|
| RMSE horizontal (m) | 2351.1 | 2302.2 | 2322.0 |
| Error H final (m) | 4586.7 | 4518.8 | 4565.5 |
| NIS medio en marcha | 197011.0 | 186328.9 | 181971.7 |
| RMSE Roll (deg) | 6.55 | 6.53 | 6.53 |
| RMSE Pitch (deg) | 4.39 | 4.44 | 4.44 |
| RMSE Yaw (deg) | 102.47 | 103.28 | 103.28 |
| GNSS aceptadas (%) | 3.0% | 3.0% | 6.0% |
| GNSS rechazadas (n) | 321 | 321 | 311 |

Grafico trayectorias: `docs\benchmarks\h3_trajectory_comparison.png`
Informe JSON: `docs\benchmarks\h3_experiment_report.json`
H3 diagnostics: `docs\benchmarks\h3_diagnostics.csv`
